/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Core routing algorithm based on CRoute:
 *
 *     CRoute: A Fast High-quality Timing-driven Connection-based FPGA Router
 *     Dries Vercruyce, Elias Vansteenkiste and Dirk Stroobandt
 *     DOI 10.1109/FCCM.2019.00017 [PDF on SciHub]
 *
 *  Modified for the nextpnr Arch API and data structures; optimised for
 *  real-world FPGA architectures in particular ECP5 and Xilinx UltraScale+
 *
 */

#include "router2.h"

#include <algorithm>
#include <boost/container/flat_map.hpp>
#include <chrono>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <queue>
#include <set>

#include "log.h"
#include "nextpnr.h"
#include "nextpnr_assertions.h"
#include "router1.h"
#include "timing.h"
#include "util.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
struct Router2
{
    // std::pair<int, int> is a bit too confusing, so:
    struct NetResourceData
    {
        int value = 0;
        int count = 0;
    };

    struct PerArcData
    {
        WireId sink_wire;
        BoundingBox bb;
        bool routed = false, pre_routed = false;
    };

    // As we allow overlap at first; the nextpnr bind functions can't be used
    // as the primary relation between arcs and wires/pips
    struct PerNetData
    {
        WireId src_wire;
        dict<WireId, std::pair<PipId, int>> wires;
        std::vector<std::vector<PerArcData>> arcs;
        // an arc of this net once needed the unbounded search: the source's
        // exit lies outside the box (a BSCAN, whose pins leave the CFG_CENTER
        // tile far from where it sits), so the box only wastes the other arcs' time
        bool bb_useless = false;
        dict<GroupId, NetResourceData> resources;
        BoundingBox bb;
        // Coordinates of the center of the net, used for the weight-to-average
        int cx, cy, hpwl;
        int total_route_us = 0;
        float max_crit = 0;
        delay_t worst_slack = std::numeric_limits<delay_t>::max();
        int fail_count = 0;
        // The box as set up, and how many times it has been grown since.  See
        // the expansion in update_congestion().
        BoundingBox base_bb;
        // The pins' own box, before any margin -- what budget mode sizes from.
        BoundingBox pin_bb;
        int bb_expansions = 0;
    };

    struct WireScore
    {
        float delay;
        float cost;
        float togo_cost;
        float total() const { return cost + togo_cost; }
    };

    struct PerWireData
    {
        // nextpnr
        WireId w;
        // Historical congestion cost
        int curr_cong = 0;
        float hist_cong_cost = 1.0;
        // This wire has to be used for this net
        int reserved_net = -1;
        static constexpr int RESERVED_UNAVAILABLE = -2;
        // The notional location of the wire, to guarantee thread safety
        int16_t x = 0, y = 0;
        // Visit data
        PipId pip_fwd, pip_bwd;
        static constexpr float NOT_VISITED = std::numeric_limits<float>::max();
        float cost_fwd = NOT_VISITED, cost_bwd = NOT_VISITED;
    };

    struct PerResourceData
    {
        GroupId key;
        // Historical congestion cost
        dict<int, int> value_count;
        float hist_cong_cost = 1.0;
    };

    Context *ctx;
    Router2Cfg cfg;

    Router2(Context *ctx, const Router2Cfg &cfg) : ctx(ctx), cfg(cfg), tmg(ctx)
    {
        tmg.setup_only = false;
        tmg.with_clock_skew = true;
        tmg.setup();
    }

    // Use 'udata' for fast net lookups and indexing
    std::vector<NetInfo *> nets_by_udata;
    std::vector<PerNetData> nets;

    bool timing_driven, timing_driven_ripup;
    TimingAnalyser tmg;

    void setup_nets()
    {
        // Populate per-net and per-arc structures at start of routing
        nets.resize(ctx->nets.size());
        nets_by_udata.resize(ctx->nets.size());
        size_t i = 0;
        for (auto &net : ctx->nets) {
            NetInfo *ni = net.second.get();
            ni->udata = i;
            nets_by_udata.at(i) = ni;
            nets.at(i).arcs.resize(ni->users.capacity());

            // Start net bounding box at overall min/max
            nets.at(i).bb.x0 = std::numeric_limits<int>::max();
            nets.at(i).bb.x1 = std::numeric_limits<int>::min();
            nets.at(i).bb.y0 = std::numeric_limits<int>::max();
            nets.at(i).bb.y1 = std::numeric_limits<int>::min();
            nets.at(i).cx = 0;
            nets.at(i).cy = 0;

            if (ni->driver.cell != nullptr) {
                Loc drv_loc = ni->driver.cell->getLocation();
                nets.at(i).cx += drv_loc.x;
                nets.at(i).cy += drv_loc.y;
            }

            for (auto usr : ni->users.enumerate()) {
                WireId src_wire = ctx->getNetinfoSourceWire(ni);
                for (auto &dst_wire : ctx->getNetinfoSinkWires(ni, usr.value)) {
                    nets.at(i).src_wire = src_wire;
                    if (ni->driver.cell == nullptr)
                        src_wire = dst_wire;
                    if (ni->driver.cell == nullptr && dst_wire == WireId())
                        continue;
                    if (src_wire == WireId())
                        log_error("No wire found for port %s on source cell %s.\n", ctx->nameOf(ni->driver.port),
                                  ctx->nameOf(ni->driver.cell));
                    if (dst_wire == WireId())
                        log_error("No wire found for port %s on destination cell %s.\n", ctx->nameOf(usr.value.port),
                                  ctx->nameOf(usr.value.cell));
                    nets.at(i).arcs.at(usr.index.idx()).emplace_back();
                    auto &ad = nets.at(i).arcs.at(usr.index.idx()).back();
                    ad.sink_wire = dst_wire;
                    // Set bounding box for this arc
                    ad.bb = ctx->getRouteBoundingBox(src_wire, dst_wire);
                    // Expand net bounding box to include this arc
                    nets.at(i).bb.x0 = std::min(nets.at(i).bb.x0, ad.bb.x0);
                    nets.at(i).bb.x1 = std::max(nets.at(i).bb.x1, ad.bb.x1);
                    nets.at(i).bb.y0 = std::min(nets.at(i).bb.y0, ad.bb.y0);
                    nets.at(i).bb.y1 = std::max(nets.at(i).bb.y1, ad.bb.y1);
                }
                // Add location to centroid sum
                Loc usr_loc = usr.value.cell->getLocation();
                nets.at(i).cx += usr_loc.x;
                nets.at(i).cy += usr_loc.y;
            }
            nets.at(i).hpwl = std::max(
                    std::abs(nets.at(i).bb.y1 - nets.at(i).bb.y0) + std::abs(nets.at(i).bb.x1 - nets.at(i).bb.x0), 1);
            nets.at(i).cx /= int(ni->users.entries() + 1);
            nets.at(i).cy /= int(ni->users.entries() + 1);
            if (ctx->debug)
                log_info("%s: bb=(%d, %d)->(%d, %d) c=(%d, %d) hpwl=%d\n", ctx->nameOf(ni), nets.at(i).bb.x0,
                         nets.at(i).bb.y0, nets.at(i).bb.x1, nets.at(i).bb.y1, nets.at(i).cx, nets.at(i).cy,
                         nets.at(i).hpwl);
            nets.at(i).pin_bb = nets.at(i).bb;
            nets.at(i).bb.x0 = std::max(nets.at(i).bb.x0 - cfg.bb_margin_x, 0);
            nets.at(i).bb.y0 = std::max(nets.at(i).bb.y0 - cfg.bb_margin_y, 0);
            nets.at(i).bb.x1 = std::min(nets.at(i).bb.x1 + cfg.bb_margin_x, ctx->getGridDimX());
            nets.at(i).bb.y1 = std::min(nets.at(i).bb.y1 + cfg.bb_margin_y, ctx->getGridDimY());
            nets.at(i).base_bb = nets.at(i).bb;
            i++;
        }
    }

    dict<WireId, int> wire_to_idx;
    std::vector<PerWireData> flat_wires;

    PerWireData &wire_data(WireId w) { return flat_wires[wire_to_idx.at(w)]; }

    void setup_wires()
    {
        // Set up per-wire structures, so that MT parts don't have to do any memory allocation
        // This is possibly quite wasteful and not cache-optimal; further consideration necessary
        for (auto wire : ctx->getWires()) {
            PerWireData pwd;
            pwd.w = wire;
            NetInfo *bound = ctx->getBoundWireNet(wire);
            if (bound != nullptr) {
                auto iter = bound->wires.find(wire);
                if (iter != bound->wires.end()) {
                    auto &nd = nets.at(bound->udata);
                    nd.wires[wire] = std::make_pair(bound->wires.at(wire).pip, 0);
                    pwd.curr_cong = 1;
                    const bool wire_is_reserved_for_this_net = bound->wires.at(wire).strength >= STRENGTH_PLACER;
                    if (wire_is_reserved_for_this_net) {
                        // Reserved for its net: no other net may enter, and the
                        // net itself may only use it through its bound pip
                        // (checked on expansion), but the net's remaining sinks
                        // can branch off it -- a locked tree (a pre-routed net,
                        // a routed clock) would otherwise be a dead end for a
                        // sink added since, whose only way out of the source is
                        // through a locked wire.
                        pwd.reserved_net = bound->udata;
                    }
                }
            }

            BoundingBox wire_loc = ctx->getRouteBoundingBox(wire, wire);
            pwd.x = (wire_loc.x0 + wire_loc.x1) / 2;
            pwd.y = (wire_loc.y0 + wire_loc.y1) / 2;

            wire_to_idx[wire] = int(flat_wires.size());
            flat_wires.push_back(pwd);
        }

        for (auto &net_pair : ctx->nets) {
            auto *net = net_pair.second.get();
            auto &nd = nets.at(net->udata);
            for (auto usr : net->users.enumerate()) {
                auto &ad = nd.arcs.at(usr.index.idx());
                for (size_t phys_pin = 0; phys_pin < ad.size(); phys_pin++) {
                    if (check_arc_routing(net, usr.index, phys_pin)) {
                        record_prerouted_net(net, usr.index, phys_pin);
                    }
                }
            }
        }
    }

    dict<GroupId, int> resource_to_idx;
    dict<WireId, int> wire_to_resource;
    std::vector<PerResourceData> flat_resources;

    PerResourceData &resource_data(GroupId r) { return flat_resources[resource_to_idx.at(r)]; }

    void setup_resources()
    {
        for (auto resource_key : ctx->getGroups()) {
            if (!ctx->isGroupResource(resource_key))
                continue;

            auto entry = resource_to_idx.find(resource_key);

            if (entry == resource_to_idx.end()) {
                auto data = PerResourceData{};
                data.key = resource_key;
                resource_to_idx.insert({resource_key, flat_resources.size()});
                flat_resources.push_back(data);

                entry = resource_to_idx.find(resource_key);
            }
            for (auto pip : ctx->getGroupPips(resource_key)) {
                auto dest_wire = ctx->getPipDstWire(pip);
                if (wire_to_resource.count(dest_wire) == 0)
                    wire_to_resource.insert({dest_wire, entry->second});
            }
        }
    }

    struct QueuedWire
    {

        explicit QueuedWire(int wire = -1, WireScore score = WireScore{}, int randtag = 0)
                : wire(wire), score(score), randtag(randtag) {};

        int wire;
        WireScore score;
        int randtag = 0;

        struct Greater
        {
            bool operator()(const QueuedWire &lhs, const QueuedWire &rhs) const noexcept
            {
                float lhs_score = lhs.score.cost + lhs.score.togo_cost,
                      rhs_score = rhs.score.cost + rhs.score.togo_cost;
                return lhs_score == rhs_score ? lhs.randtag > rhs.randtag : lhs_score > rhs_score;
            }
        };
    };

    bool hit_test_pip(BoundingBox &bb, Loc l) { return l.x >= bb.x0 && l.x <= bb.x1 && l.y >= bb.y0 && l.y <= bb.y1; }

    double curr_cong_weight, hist_cong_weight, estimate_weight;

    struct ThreadContext
    {
        // Nets to route
        std::vector<NetInfo *> route_nets;
        // Nets that failed routing
        std::vector<NetInfo *> failed_nets;

        std::vector<std::pair<store_index<PortRef>, size_t>> route_arcs;

        std::priority_queue<QueuedWire, std::vector<QueuedWire>, QueuedWire::Greater> fwd_queue, bwd_queue;
        // Special case where one net has multiple logical arcs to the same physical sink
        pool<WireId> processed_sinks;

        std::vector<int> dirty_wires;

        // Thread bounding box
        BoundingBox bb;

        DeterministicRNG rng;

        // Used to add existing routing to the heap
        pool<WireId> in_wire_by_loc;
        dict<std::pair<int, int>, pool<WireId>> wire_by_loc;
    };

    bool thread_test_wire(ThreadContext &t, PerWireData &w)
    {
        return w.x >= t.bb.x0 && w.x <= t.bb.x1 && w.y >= t.bb.y0 && w.y <= t.bb.y1;
    }

    enum ArcRouteResult
    {
        ARC_SUCCESS,
        ARC_RETRY_WITHOUT_BB,
        ARC_FATAL,
    };

// Define to make sure we don't print in a multithreaded context
#define ARC_LOG_ERR(...)                                                                                               \
    do {                                                                                                               \
        if (is_mt)                                                                                                     \
            return ARC_FATAL;                                                                                          \
        else                                                                                                           \
            log_error(__VA_ARGS__);                                                                                    \
    } while (0)
#define ROUTE_LOG_DBG(...)                                                                                             \
    do {                                                                                                               \
        if (!is_mt && ctx->debug)                                                                                      \
            log(__VA_ARGS__);                                                                                          \
    } while (0)

    void bind_pip_internal(PerNetData &net, store_index<PortRef> user, int wire, PipId pip)
    {
        auto &wd = flat_wires.at(wire);
        auto wire_found = net.wires.find(wd.w);
        if (wire_found == net.wires.end()) {
            // Not yet used for any arcs of this net, add to list
            net.wires.emplace(wd.w, std::make_pair(pip, 1));
            // Increase bound count of wire by 1
            ++wd.curr_cong;
        } else {
            // Already used for at least one other arc of this net
            // Don't allow two uphill PIPs for the same net and wire
            NPNR_ASSERT(wire_found->second.first == pip);
            // Increase the count of bound arcs
            ++wire_found->second.second;
        }

        if (pip == PipId())
            return;

        auto resource_key = ctx->getResourceKeyForPip(pip);
        if (resource_key == GroupId())
            return;

        auto &rd = resource_data(resource_key);
        auto resource_value = ctx->getResourceValueForPip(pip);

        auto resource_found = net.resources.find(resource_key);

        if (resource_found == net.resources.end()) {
            net.resources.emplace(resource_key, NetResourceData{resource_value, 1});
        } else {
            // net resource value must agree with arc resource value
            NPNR_ASSERT(resource_found->second.value == resource_value);

            ++resource_found->second.count;
        }

        auto resource_value_count = rd.value_count.find(resource_value);
        if (resource_value_count == rd.value_count.end()) {
            rd.value_count.insert({resource_value, 1});
        } else {
            ++resource_value_count->second;
        }
    }

    void unbind_pip_internal(PerNetData &net, store_index<PortRef> user, WireId wire)
    {
        auto &wd = wire_data(wire);
        auto &wire_found = net.wires.at(wd.w);
        auto pip = wire_found.first;

        --wire_found.second;
        if (wire_found.second == 0) {
            // No remaining arcs of this net bound to this wire
            --wd.curr_cong;
            net.wires.erase(wd.w);
        }

        if (pip == PipId())
            return;

        auto resource_key = ctx->getResourceKeyForPip(pip);
        if (resource_key == GroupId())
            return;

        auto &rd = resource_data(resource_key);
        auto resource_value = ctx->getResourceValueForPip(pip);
        auto resource_found = net.resources.at(resource_key);

        --resource_found.count;
        --rd.value_count.at(resource_value);
        if (resource_found.count == 0) {
            net.resources.erase(resource_key);
        }
        if (rd.value_count.at(resource_value) == 0) {
            rd.value_count.erase(resource_value);
        }
    }

    void ripup_arc(NetInfo *net, store_index<PortRef> user, size_t phys_pin)
    {
        auto &nd = nets.at(net->udata);
        auto &ad = nd.arcs.at(user.idx()).at(phys_pin);
        if (!ad.routed)
            return;
        WireId src = nets.at(net->udata).src_wire;
        WireId cursor = ad.sink_wire;
        while (cursor != src &&
               (net->constant_value == IdString() || ctx->getWireConstantValue(cursor) != net->constant_value)) {
            PipId pip = nd.wires.at(cursor).first;
            unbind_pip_internal(nd, user, cursor);
            cursor = ctx->getPipSrcWire(pip);
        }
        ad.routed = false;
        // Once we've ripped up the arc, the routing may no longer be the same as before...
        // (this should only happen if it was not strongly bound)
        ad.pre_routed = false;
    }

    float score_wire_for_arc(NetInfo *net, store_index<PortRef> user, size_t phys_pin, WireId wire, PipId pip,
                             float crit_weight)
    {
        auto &wd = wire_data(wire);
        auto &nd = nets.at(net->udata);
        float base_cost = cfg.get_base_cost(ctx, wire, pip, crit_weight);
        int overuse = wd.curr_cong;
        float hist_cost = 1.0f + crit_weight * (wd.hist_cong_cost - 1.0f);
        float bias_cost = 0;
        float resource_hist_cost = 0.0f;
        float resource_present_cost = 0.0f;
        int source_uses = 0;
        if (nd.wires.count(wire)) {
            overuse -= 1;
            source_uses = nd.wires.at(wire).second;
        }
        float present_cost = 1.0f + overuse * curr_cong_weight * crit_weight;
        if (pip != PipId()) {
            Loc pl = ctx->getPipLocation(pip);
            bias_cost = cfg.bias_cost_factor * (base_cost / int(net->users.entries())) *
                        ((std::abs(pl.x - nd.cx) + std::abs(pl.y - nd.cy)) / float(nd.hpwl));
        }

        auto resource_key = wire_to_resource.find(wire);
        if (resource_key != wire_to_resource.end()) {
            auto &rd = flat_resources.at(resource_key->second);
            resource_hist_cost = 1.0f + crit_weight * (rd.hist_cong_cost - 1.0f);
            resource_present_cost = 1.0f + rd.value_count.size() * curr_cong_weight * crit_weight;
        }

        // Density: cost a wire for sitting in a crowded tile, even when that
        // tile is perfectly legal.  Everything above this line is about
        // LEGALITY -- present_cost counts nets illegally sharing a wire, and
        // goes to zero the moment routing is legal.  This term does not, so
        // it is what lets the smoothing pass redistribute traffic that the
        // router has no other reason to move.  Inactive unless the pass is
        // running, so the normal route is bit-for-bit unchanged.
        float smooth_cost = 1.0f;
        bool smoothing_prices_this_pip = smoothing_active && smooth_target > 0 && pip != PipId();
        if (smoothing_prices_this_pip) {
            Loc pl = ctx->getPipLocation(pip);
            auto it = tile_occ.find(tile_key(pl.x, pl.y));
            bool pip_tile_is_hot = it != tile_occ.end() && it->second > smooth_target;
            if (pip_tile_is_hot)
                smooth_cost = 1.0f + cfg.smooth_weight *
                                     (float(it->second - smooth_target) / float(smooth_target));
        }

        return smooth_cost * (base_cost * hist_cost * present_cost / (1 + (source_uses * crit_weight)) + bias_cost +
                              base_cost * resource_hist_cost * resource_present_cost / (1 + crit_weight));
    }

    float get_togo_cost(NetInfo *net, store_index<PortRef> user, int wire, WireId src_sink, bool bwd, float crit_weight)
    {
        auto &nd = nets.at(net->udata);
        auto &wd = flat_wires[wire];
        int source_uses = 0;
        if (nd.wires.count(wd.w)) {
            source_uses = nd.wires.at(wd.w).second;
        }
        // FIXME: timing/wirelength balance?
        delay_t est_delay = ctx->estimateDelay(bwd ? src_sink : wd.w, bwd ? wd.w : src_sink);
        return (ctx->getDelayNS(est_delay) / (1 + source_uses * crit_weight)) + cfg.ipin_cost_adder;
    }

    bool check_arc_routing(NetInfo *net, store_index<PortRef> usr, size_t phys_pin)
    {
        auto &nd = nets.at(net->udata);
        auto &ad = nd.arcs.at(usr.idx()).at(phys_pin);
        WireId src_wire = nets.at(net->udata).src_wire;
        WireId cursor = ad.sink_wire;
        while (nd.wires.count(cursor)) {
            auto &wd = wire_data(cursor);
            if (wd.curr_cong != 1)
                return false;
            auto &uh = nd.wires.at(cursor).first;
            if (uh == PipId())
                break;
            auto resource_key = ctx->getResourceKeyForPip(uh);
            if (resource_key != GroupId()) {
                auto &rd = resource_data(resource_key);
                if (rd.value_count.size() > 1)
                    break;
            }
            cursor = ctx->getPipSrcWire(uh);
        }
        return (cursor == src_wire) ||
               (net->constant_value != IdString() && ctx->getWireConstantValue(cursor) == net->constant_value);
    }

    void record_prerouted_net(NetInfo *net, store_index<PortRef> usr, size_t phys_pin)
    {
        auto &nd = nets.at(net->udata);
        auto &ad = nd.arcs.at(usr.idx()).at(phys_pin);
        ad.routed = true;
        ad.pre_routed = true;

        WireId src = nets.at(net->udata).src_wire;
        WireId cursor = ad.sink_wire;
        while (cursor != src) {
            // Mirror check_arc_routing's termination: the routed tree can end
            // at a wire that is not src_wire -- a wire with no driving pip
            // (a route root, or a GND/VCC constant source for a constant net),
            // for which check_arc_routing still reports the arc as routed.
            // Without these guards getPipSrcWire(PipId()) indexes tile -1 and
            // asserts, which is why re-routing an already-routed design (a
            // second router2() invocation, or a loaded pre-routed design) fell
            // over here.
            if (!nd.wires.count(cursor))
                break;
            PipId pip = nd.wires.at(cursor).first;
            if (pip == PipId())
                break;
            size_t wire_idx = wire_to_idx.at(cursor);
            bind_pip_internal(nd, usr, wire_idx, pip);
            cursor = ctx->getPipSrcWire(pip);
        }
    }

    // Returns true if a wire contains no sink port
    bool is_wire_unusable(WireId wire, const NetInfo *net, const pool<WireId> &sink_wires, int iter_count = 0)
    {
        if (iter_count > 7)
            return false; // heuristic to assume we've hit general routing
        if (wire_data(wire).reserved_net != -1 && wire_data(wire).reserved_net != net->udata)
            return true; // reserved for another net
        if (sink_wires.count(wire))
            return false;
        for (auto p : ctx->getPipsDownhill(wire))
            if (ctx->checkPipAvail(p)) {
                if (!is_wire_unusable(ctx->getPipDstWire(p), net, sink_wires, iter_count + 1))
                    return false;
            }
        return true;
    }

    // Returns true if a wire contains no source ports or driving pips
    bool is_wire_undriveable(WireId wire, const NetInfo *net, int iter_count = 0)
    {
        // This is specifically designed to handle a particularly icky case that the current router struggles with in
        // the nexus device,
        // C -> C lut input only
        // C; D; or F from another lut -> D lut input
        // D or M -> M ff input
        // without careful reservation of C for C lut input and D for D lut input, there is fighting for D between FF
        // and LUT
        if (iter_count > 7)
            return false; // heuristic to assume we've hit general routing
        if (wire_data(wire).reserved_net == PerWireData::RESERVED_UNAVAILABLE)
            return true;
        if (wire_data(wire).reserved_net != -1 && wire_data(wire).reserved_net != net->udata)
            return true; // reserved for another net
        for (auto bp : ctx->getWireBelPins(wire))
            if ((net->driver.cell == nullptr || bp.bel == net->driver.cell->bel) &&
                ctx->getBelPinType(bp.bel, bp.pin) != PORT_IN)
                return false;
        for (auto p : ctx->getPipsUphill(wire))
            if (ctx->checkPipAvail(p)) {
                if (!is_wire_undriveable(ctx->getPipSrcWire(p), net, iter_count + 1))
                    return false;
            }
        return true;
    }

    // Find all the sink wires that must be used to route a given arc
    bool reserve_driver_wires_for_arc(NetInfo *net, const pool<WireId> &sink_wires)
    {
        bool did_something = false;

        WireId src = ctx->getNetinfoSourceWire(net);
        if (!sink_wires.count(src) && !sink_wires.empty()) {
            WireId cursor = src;
            bool done = false;

            if (ctx->debug) {
                log("reserving wires for net %s\n", ctx->nameOf(net));
                log("   with driver wire %s\n", ctx->nameOfWire(src));
            }

            while (!done) {
                auto &wd = wire_data(cursor);
                if (ctx->debug)
                    log("      %s (driver output)\n", ctx->nameOfWire(cursor));
                if (wd.reserved_net == PerWireData::RESERVED_UNAVAILABLE)
                    break;
                did_something |= (wd.reserved_net != net->udata);
                if (wd.reserved_net != -1 && wd.reserved_net != net->udata)
                    log_error("attempting to reserve driver output path wire '%s' for nets '%s' and '%s'\n",
                              ctx->nameOfWire(cursor), ctx->nameOf(nets_by_udata.at(wd.reserved_net)),
                              ctx->nameOf(net));
                wd.reserved_net = net->udata;
                WireId next_cursor;
                for (auto dh : ctx->getPipsDownhill(cursor)) {
                    WireId w = ctx->getPipDstWire(dh);
                    if (is_wire_unusable(w, net, sink_wires))
                        continue;
                    if (next_cursor != WireId() || sink_wires.count(w)) {
                        done = true;
                        break;
                    }
                    next_cursor = w;
                }
                if (next_cursor == WireId())
                    break;
                cursor = next_cursor;
            }
        }
        return did_something;
    }

    // Find all the sink wires that must be used to route a given arc
    bool reserve_sink_wires_for_arc(NetInfo *net, store_index<PortRef> i)
    {
        bool did_something = false;

        WireId src = ctx->getNetinfoSourceWire(net);
        auto &usr = net->users.at(i);
        auto &nd = nets.at(net->udata);
        for (auto &ad : nd.arcs.at(i.idx())) {
            if (ad.pre_routed)
                continue;
            WireId cursor = ad.sink_wire;
            bool done = false;
            if (ctx->debug) {
                log("reserving wires for arc %d (%s.%s) of net %s\n", i.idx(), ctx->nameOf(usr.cell),
                    ctx->nameOf(usr.port), ctx->nameOf(net));
                log("   with sink wire %s\n", ctx->nameOfWire(ad.sink_wire));
            }
            while (!done) {
                auto &wd = wire_data(cursor);
                if (ctx->debug)
                    log("      %s (sink input)\n", ctx->nameOfWire(cursor));
                if (wd.reserved_net == PerWireData::RESERVED_UNAVAILABLE)
                    break;
                did_something |= (wd.reserved_net != net->udata);
                if (wd.reserved_net != -1 && wd.reserved_net != net->udata)
                    log_error("attempting to reserve sink input path wire '%s' for nets '%s' and '%s'\n",
                              ctx->nameOfWire(cursor), ctx->nameOf(nets_by_udata.at(wd.reserved_net)),
                              ctx->nameOf(net));
                wd.reserved_net = net->udata;
                if (cursor == src)
                    break;
                WireId next_cursor;
                for (auto uh : ctx->getPipsUphill(cursor)) {
                    WireId w = ctx->getPipSrcWire(uh);
                    if (is_wire_undriveable(w, net))
                        continue;
                    if (next_cursor != WireId()) {
                        done = true;
                        break;
                    }
                    next_cursor = w;
                }
                if (next_cursor == WireId())
                    break;
                cursor = next_cursor;
            }
        }
        return did_something;
    }

    void find_all_reserved_wires()
    {
        // Run iteratively, as reserving wires for one net might limit choices for another
        bool did_something = false;
        do {
            did_something = false;
            for (auto net : nets_by_udata) {
                auto &nd = nets.at(net->udata);
                WireId src = ctx->getNetinfoSourceWire(net);
                if (src == WireId())
                    continue;
                pool<WireId> sink_wires;
                bool all_pre_routed = true;
                for (auto usr : net->users.enumerate()) {
                    for (auto &sink : nd.arcs.at(usr.index.idx())) {
                        if (!sink.pre_routed)
                            all_pre_routed = false;
                    }
                    for (auto sink_wire : ctx->getNetinfoSinkWires(net, usr.value))
                        sink_wires.insert(sink_wire);
                }
                if (all_pre_routed)
                    continue;
                did_something |= reserve_driver_wires_for_arc(net, sink_wires);
                for (auto usr : net->users.enumerate())
                    did_something |= reserve_sink_wires_for_arc(net, usr.index);
            }
        } while (did_something);
    }

    void reset_wires(ThreadContext &t)
    {
        for (auto w : t.dirty_wires) {
            flat_wires[w].pip_fwd = PipId();
            flat_wires[w].pip_bwd = PipId();
            flat_wires[w].cost_fwd = PerWireData::NOT_VISITED;
            flat_wires[w].cost_bwd = PerWireData::NOT_VISITED;
        }
        t.dirty_wires.clear();
    }

    // These nets have very-high-fanout pips and special rules must be followed (only working backwards) to avoid
    // crippling perf
    bool is_dedi_const_net(const NetInfo *net) { return net->constant_value != IdString(); }

    void update_wire_by_loc(ThreadContext &t, NetInfo *net, store_index<PortRef> i, size_t phys_pin, bool is_mt)
    {
        if (is_dedi_const_net(net))
            return;
        auto &nd = nets.at(net->udata);
        auto &ad = nd.arcs.at(i.idx()).at(phys_pin);
        WireId cursor = ad.sink_wire;
        if (!nd.wires.count(cursor))
            return;
        while (cursor != nd.src_wire) {
            if (!t.in_wire_by_loc.count(cursor)) {
                t.in_wire_by_loc.insert(cursor);
                for (auto dh : ctx->getPipsDownhill(cursor)) {
                    Loc dh_loc = ctx->getPipLocation(dh);
                    t.wire_by_loc[std::make_pair(dh_loc.x, dh_loc.y)].insert(cursor);
                }
            }
            cursor = ctx->getPipSrcWire(nd.wires.at(cursor).first);
        }
    }

    // Functions for marking wires as visited, and checking if they have already been visited
    void set_visited_fwd(ThreadContext &t, int wire, PipId pip, float cost)
    {
        auto &wd = flat_wires.at(wire);
        if (wd.cost_fwd == PerWireData::NOT_VISITED && wd.cost_bwd == PerWireData::NOT_VISITED)
            t.dirty_wires.push_back(wire);
        wd.pip_fwd = pip;
        wd.cost_fwd = cost;
    }
    void set_visited_bwd(ThreadContext &t, int wire, PipId pip, float cost)
    {
        auto &wd = flat_wires.at(wire);
        if (wd.cost_fwd == PerWireData::NOT_VISITED && wd.cost_bwd == PerWireData::NOT_VISITED)
            t.dirty_wires.push_back(wire);
        wd.pip_bwd = pip;
        wd.cost_bwd = cost;
    }

    bool was_visited_fwd(int wire, float cost)
    {
        return flat_wires.at(wire).cost_fwd != PerWireData::NOT_VISITED && flat_wires.at(wire).cost_fwd <= cost;
    }
    bool was_visited_bwd(int wire, float cost)
    {
        return flat_wires.at(wire).cost_bwd != PerWireData::NOT_VISITED && flat_wires.at(wire).cost_bwd <= cost;
    }

    float get_arc_crit(NetInfo *net, store_index<PortRef> i)
    {
        if (!timing_driven)
            return 0;
        return tmg.get_criticality(CellPortKey(net->users.at(i)));
    }

    bool arc_failed_slack(NetInfo *net, store_index<PortRef> usr_idx)
    {
        return timing_driven_ripup &&
               (tmg.get_setup_slack(CellPortKey(net->users.at(usr_idx))) < (2 * ctx->getDelayEpsilon()));
    }

    ArcRouteResult route_arc(ThreadContext &t, NetInfo *net, store_index<PortRef> i, size_t phys_pin, bool is_mt,
                             bool is_bb = true)
    {
        // Do some initial lookups and checks
        auto arc_start = std::chrono::high_resolution_clock::now();
        auto &nd = nets[net->udata];
        auto &ad = nd.arcs.at(i.idx()).at(phys_pin);
        auto &usr = net->users.at(i);
        bool const_mode = is_dedi_const_net(net);
        ROUTE_LOG_DBG("Routing arc %d of net '%s' (%d, %d) -> (%d, %d)\n", i.idx(), ctx->nameOf(net), ad.bb.x0,
                      ad.bb.y0, ad.bb.x1, ad.bb.y1);
        WireId src_wire = ctx->getNetinfoSourceWire(net), dst_wire = ctx->getNetinfoSinkWire(net, usr, phys_pin);
        if (src_wire == WireId() && !const_mode)
            ARC_LOG_ERR("No wire found for port %s on source cell %s.\n", ctx->nameOf(net->driver.port),
                        ctx->nameOf(net->driver.cell));
        if (dst_wire == WireId())
            ARC_LOG_ERR("No wire found for port %s on destination cell %s.\n", ctx->nameOf(usr.port),
                        ctx->nameOf(usr.cell));
        int src_wire_idx = const_mode ? -1 : wire_to_idx.at(src_wire);
        int dst_wire_idx = wire_to_idx.at(dst_wire);
        // Calculate a timing weight based on criticality
        float crit = get_arc_crit(net, i);
        float crit_weight = std::max<float>(0.05f, (1.0f - std::pow(crit, 2)));
        ROUTE_LOG_DBG("     crit=%.3f crit_weight=%.3f\n", crit, crit_weight);
        // Check if arc was already done _in this iteration_
        if (t.processed_sinks.count(dst_wire))
            return ARC_SUCCESS;

        // We have two modes:
        //     0. starting within a small range of existing routing
        //     1. expanding from all routing
        int mode = 0;
        if (net->users.entries() < 4 || nd.wires.empty() || (crit > 0.95))
            mode = 1;

        // This records the point where forwards and backwards routing met
        int midpoint_wire = -1;
        float best_midpoint_cost = 0;
        int explored = 1;

        for (; mode < 2; mode++) {
            // Clear out the queues
            if (!t.fwd_queue.empty()) {
                std::priority_queue<QueuedWire, std::vector<QueuedWire>, QueuedWire::Greater> new_queue;
                t.fwd_queue.swap(new_queue);
            }
            if (!t.bwd_queue.empty()) {
                std::priority_queue<QueuedWire, std::vector<QueuedWire>, QueuedWire::Greater> new_queue;
                t.bwd_queue.swap(new_queue);
            }
            // Unvisit any previously visited wires
            reset_wires(t);

            ROUTE_LOG_DBG("src_wire = %s -> dst_wire = %s\n", ctx->nameOfWire(src_wire), ctx->nameOfWire(dst_wire));

            // Add 'forward' direction startpoints to queue
            auto seed_queue_fwd = [&](WireId wire) {
                WireScore base_score;
                base_score.delay = 0;
                base_score.cost = 0;
                int wire_idx = wire_to_idx.at(wire);
                base_score.togo_cost = get_togo_cost(net, i, wire_idx, dst_wire, false, crit_weight);
                t.fwd_queue.push(QueuedWire(wire_idx, base_score));
                set_visited_fwd(t, wire_idx, PipId(), 0.0);
            };
            auto &dst_data = flat_wires.at(dst_wire_idx);
            // Look for nearby existing routing
            for (int dy = -cfg.bb_margin_y; dy <= cfg.bb_margin_y; dy++)
                for (int dx = -cfg.bb_margin_x; dx <= cfg.bb_margin_x; dx++) {
                    auto fnd = t.wire_by_loc.find(std::make_pair(dst_data.x + dx, dst_data.y + dy));
                    if (fnd == t.wire_by_loc.end())
                        continue;
                    for (WireId wire : fnd->second) {
                        ROUTE_LOG_DBG("   seeding with %s\n", ctx->nameOfWire(wire));
                        seed_queue_fwd(wire);
                    }
                }

            if (mode == 0 && t.fwd_queue.size() < 4)
                continue;
            if (!const_mode) {
                if (mode == 1) {
                    // Seed forwards with the source wire, if less than 8 existing wires added
                    seed_queue_fwd(src_wire);
                } else {
                    set_visited_fwd(t, src_wire_idx, PipId(), 0.0);
                }
            }
            auto seed_queue_bwd = [&](WireId wire) {
                WireScore base_score;
                base_score.delay = 0;
                base_score.cost = 0;
                int wire_idx = wire_to_idx.at(wire);
                base_score.togo_cost = get_togo_cost(net, i, wire_idx, src_wire, true, crit_weight);
                t.bwd_queue.push(QueuedWire(wire_idx, base_score));
                set_visited_bwd(t, wire_idx, PipId(), 0.0);
            };

            // Seed backwards with the dest wire
            seed_queue_bwd(dst_wire);

            int toexplore = 25000 * std::max(1, (ad.bb.x1 - ad.bb.x0) + (ad.bb.y1 - ad.bb.y0));
            int iter = 0;

            // Mode 0 required both queues to be live
            while (((mode == 0) ? (!t.fwd_queue.empty() && !t.bwd_queue.empty())
                                : (!t.fwd_queue.empty() || !t.bwd_queue.empty())) &&
                   ((!is_bb && midpoint_wire == -1) || iter < toexplore)) {
                ++iter;
                const bool time_to_report_progress = (iter % 5000000) == 0;
                if (time_to_report_progress)
                    log_warning("route_arc: net %s, %s -> %s: %d iterations (fwd %zu, bwd %zu, bb %d)\n", ctx->nameOf(net),
                                ctx->nameOfWire(src_wire), ctx->nameOfWire(dst_wire), iter, t.fwd_queue.size(),
                                t.bwd_queue.size(), int(is_bb));
                if (!t.fwd_queue.empty() && !const_mode) {
                    // Explore forwards
                    auto curr = t.fwd_queue.top();
                    t.fwd_queue.pop();
                    ++explored;
                    if (was_visited_bwd(curr.wire, std::numeric_limits<float>::max())) {
                        // Meet in the middle; done
                        midpoint_wire = curr.wire;
                        break;
                    }
                    auto &curr_data = flat_wires.at(curr.wire);
                    for (PipId dh : ctx->getPipsDownhill(curr_data.w)) {
                        // Skip pips outside of box in bounding-box mode
                        if (is_bb && !hit_test_pip(nd.bb, ctx->getPipLocation(dh)))
                            continue;
                        if (!ctx->checkPipAvailForNet(dh, net))
                            continue;
                        WireId next = ctx->getPipDstWire(dh);
                        int next_idx = wire_to_idx.at(next);
                        WireScore next_score;
                        next_score.delay = curr.score.delay + cfg.get_base_cost(ctx, next, dh, crit_weight);
                        next_score.cost = curr.score.cost + score_wire_for_arc(net, i, phys_pin, next, dh, crit_weight);
                        next_score.togo_cost =
                                cfg.estimate_weight * get_togo_cost(net, i, next_idx, dst_wire, false, crit_weight);
                        if (was_visited_fwd(next_idx, next_score.delay)) {
                            // Don't expand the same node twice.
                            continue;
                        }
                        auto &nwd = flat_wires.at(next_idx);
                        if (nwd.reserved_net == PerWireData::RESERVED_UNAVAILABLE)
                            continue;
                        // Reserved for another net
                        if (nwd.reserved_net != -1 && nwd.reserved_net != net->udata)
                            continue;
                        // Don't allow the same wire to be bound to the same net with a different driving pip
                        auto fnd_wire = nd.wires.find(next);
                        if (fnd_wire != nd.wires.end() && fnd_wire->second.first != dh)
                            continue;
                        // Don't allow the same resource to be bound to the same net with a different value
                        auto resource_key = ctx->getResourceKeyForPip(dh);
                        if (resource_key != GroupId()) {
                            auto fnd_resource = nd.resources.find(resource_key);
                            if (fnd_resource != nd.resources.end() &&
                                fnd_resource->second.value != ctx->getResourceValueForPip(dh))
                                continue;
                        }
                        if (!thread_test_wire(t, nwd))
                            continue; // thread safety issue
                        set_visited_fwd(t, next_idx, dh, next_score.delay);
                        t.fwd_queue.push(QueuedWire(next_idx, next_score, t.rng.rng()));
                    }
                }
                if (!t.bwd_queue.empty()) {
                    // Explore backwards
                    auto curr = t.bwd_queue.top();
                    t.bwd_queue.pop();
                    ++explored;
                    auto &curr_data = flat_wires.at(curr.wire);
                    if (const_mode && ctx->getWireConstantValue(curr_data.w) == net->constant_value) {
                        if (midpoint_wire == -1) {
                            midpoint_wire = curr.wire;
                            best_midpoint_cost = curr.score.cost;
                            if (curr_cong_weight >= 10) {
                                // try harder at this point to prevent infinite iterations when constants conflict
                                toexplore = iter + std::min(200, int(curr.score.cost));
                            } else {
                                break;
                            }
                        } else if (curr.score.cost < best_midpoint_cost) {
                            midpoint_wire = curr.wire;
                            best_midpoint_cost = curr.score.cost;
                        }
                    } else {
                        if (was_visited_fwd(curr.wire, std::numeric_limits<float>::max())) {
                            // Meet in the middle; done
                            midpoint_wire = curr.wire;
                            break;
                        }
                    }
                    // Don't allow the same wire to be bound to the same net with a different driving pip
                    PipId bound_pip;
                    auto fnd_wire = nd.wires.find(curr_data.w);
                    if (fnd_wire != nd.wires.end())
                        bound_pip = fnd_wire->second.first;

                    for (PipId uh : ctx->getPipsUphill(curr_data.w)) {
                        if (bound_pip != PipId() && bound_pip != uh)
                            continue;
                        if (is_bb && !hit_test_pip(nd.bb, ctx->getPipLocation(uh)))
                            continue;
                        if (!ctx->checkPipAvailForNet(uh, net))
                            continue;
                        WireId next = ctx->getPipSrcWire(uh);
                        int next_idx = wire_to_idx.at(next);
                        WireScore next_score;
                        next_score.delay = curr.score.delay + cfg.get_base_cost(ctx, next, uh, crit_weight);
                        next_score.cost = curr.score.cost + score_wire_for_arc(net, i, phys_pin, next, uh, crit_weight);
                        next_score.togo_cost = const_mode
                                                       ? 0
                                                       : cfg.estimate_weight * get_togo_cost(net, i, next_idx, src_wire,
                                                                                             true, crit_weight);
                        if (was_visited_bwd(next_idx, next_score.delay)) {
                            // Don't expand the same node twice.
                            continue;
                        }
                        auto &nwd = flat_wires.at(next_idx);
                        if (nwd.reserved_net == PerWireData::RESERVED_UNAVAILABLE)
                            continue;
                        // Reserved for another net
                        if (nwd.reserved_net != -1 && nwd.reserved_net != net->udata)
                            continue;
                        // Don't allow the same resource to be bound to the same net with a different value
                        auto resource_key = ctx->getResourceKeyForPip(uh);
                        if (resource_key != GroupId()) {
                            auto fnd_resource = nd.resources.find(resource_key);
                            if (fnd_resource != nd.resources.end() &&
                                fnd_resource->second.value != ctx->getResourceValueForPip(uh))
                                continue;
                        }
                        if (!thread_test_wire(t, nwd))
                            continue; // thread safety issue
                        set_visited_bwd(t, next_idx, uh, next_score.delay);
                        t.bwd_queue.push(QueuedWire(next_idx, next_score, t.rng.rng()));
                    }
                }
            }
            if (midpoint_wire != -1)
                break;
        }
        ArcRouteResult result = ARC_SUCCESS;
        if (midpoint_wire != -1) {
            ROUTE_LOG_DBG("   Routed (explored %d wires): ", explored);
            if (const_mode) {
                bind_pip_internal(nd, i, midpoint_wire, PipId());
            } else {
                int cursor_bwd = midpoint_wire;
                while (was_visited_fwd(cursor_bwd, std::numeric_limits<float>::max())) {
                    PipId pip = flat_wires.at(cursor_bwd).pip_fwd;
                    if (pip == PipId() && cursor_bwd != src_wire_idx)
                        break;
                    bind_pip_internal(nd, i, cursor_bwd, pip);
                    if (ctx->debug && !is_mt) {
                        auto &wd = flat_wires.at(cursor_bwd);
                        ROUTE_LOG_DBG("      fwd wire: %s (curr %d hist %f share %d)\n", ctx->nameOfWire(wd.w),
                                      wd.curr_cong - 1, wd.hist_cong_cost, nd.wires.at(wd.w).second);
                    }
                    if (pip == PipId()) {
                        break;
                    }
                    auto resource_key = ctx->getResourceKeyForPip(pip);
                    if (resource_key != GroupId()) {
                        ROUTE_LOG_DBG("         fwd pip: %s (%d, %d) %s = %d\n", ctx->nameOfPip(pip),
                                      ctx->getPipLocation(pip).x, ctx->getPipLocation(pip).y,
                                      ctx->nameOfGroup(resource_key), ctx->getResourceValueForPip(pip));
                    } else {
                        ROUTE_LOG_DBG("         fwd pip: %s (%d, %d)\n", ctx->nameOfPip(pip),
                                      ctx->getPipLocation(pip).x, ctx->getPipLocation(pip).y);
                    }
                    cursor_bwd = wire_to_idx.at(ctx->getPipSrcWire(pip));
                }

                while (cursor_bwd != src_wire_idx) {
                    // Tack onto existing routing
                    WireId bwd_w = flat_wires.at(cursor_bwd).w;
                    if (!nd.wires.count(bwd_w))
                        break;
                    auto &bound = nd.wires.at(bwd_w);
                    PipId pip = bound.first;
                    if (ctx->debug && !is_mt) {
                        auto &wd = flat_wires.at(cursor_bwd);
                        ROUTE_LOG_DBG("      ext wire: %s (curr %d hist %f share %d)\n", ctx->nameOfWire(wd.w),
                                      wd.curr_cong - 1, wd.hist_cong_cost, bound.second);
                    }
                    bind_pip_internal(nd, i, cursor_bwd, pip);
                    if (pip == PipId())
                        break;
                    cursor_bwd = wire_to_idx.at(ctx->getPipSrcWire(pip));
                }

                NPNR_ASSERT(cursor_bwd == src_wire_idx);
            }

            int cursor_fwd = midpoint_wire;
            while (was_visited_bwd(cursor_fwd, std::numeric_limits<float>::max())) {
                PipId pip = flat_wires.at(cursor_fwd).pip_bwd;
                if (pip == PipId()) {
                    break;
                }
                auto resource_key = ctx->getResourceKeyForPip(pip);
                if (resource_key != GroupId()) {
                    ROUTE_LOG_DBG("         bwd pip: %s (%d, %d) %s = %d\n", ctx->nameOfPip(pip),
                                  ctx->getPipLocation(pip).x, ctx->getPipLocation(pip).y,
                                  ctx->nameOfGroup(resource_key), ctx->getResourceValueForPip(pip));
                } else {
                    ROUTE_LOG_DBG("         bwd pip: %s (%d, %d)\n", ctx->nameOfPip(pip), ctx->getPipLocation(pip).x,
                                  ctx->getPipLocation(pip).y);
                }

                cursor_fwd = wire_to_idx.at(ctx->getPipDstWire(pip));
                bind_pip_internal(nd, i, cursor_fwd, pip);
                if (ctx->debug && !is_mt) {
                    auto &wd = flat_wires.at(cursor_fwd);
                    ROUTE_LOG_DBG("      bwd wire: %s (curr %d hist %f share %d)\n", ctx->nameOfWire(wd.w),
                                  wd.curr_cong - 1, wd.hist_cong_cost, nd.wires.at(wd.w).second);
                }
            }
            NPNR_ASSERT(cursor_fwd == dst_wire_idx);

            update_wire_by_loc(t, net, i, phys_pin, is_mt);
            t.processed_sinks.insert(dst_wire);
            ad.routed = true;
            auto arc_end = std::chrono::high_resolution_clock::now();
            ROUTE_LOG_DBG("Routing arc %d of net '%s' (is_bb = %d) took %02fs\n", i.idx(), ctx->nameOf(net), is_bb,
                          std::chrono::duration<float>(arc_end - arc_start).count());
        } else {
            auto arc_end = std::chrono::high_resolution_clock::now();
            ROUTE_LOG_DBG("Failed routing arc %d of net '%s' (is_bb = %d) took %02fs\n", i.idx(), ctx->nameOf(net),
                          is_bb, std::chrono::duration<float>(arc_end - arc_start).count());
            result = ARC_RETRY_WITHOUT_BB;
        }
        reset_wires(t);
        return result;
    }
#undef ARC_ERR

    bool route_net(ThreadContext &t, NetInfo *net, bool is_mt)
    {

#ifdef ARCH_ECP5
        if (net->is_global)
            return true;
#endif

        ROUTE_LOG_DBG("Routing net '%s'...\n", ctx->nameOf(net));

        auto rstart = std::chrono::high_resolution_clock::now();

        // Nothing to do if net is undriven
        if (net->driver.cell == nullptr)
            return true;

        bool have_failures = false;
        t.processed_sinks.clear();
        t.route_arcs.clear();
        t.wire_by_loc.clear();
        t.in_wire_by_loc.clear();
        auto &nd = nets.at(net->udata);
        bool failed_slack = false;
        for (auto usr : net->users.enumerate())
            failed_slack |= arc_failed_slack(net, usr.index);
        for (auto usr : net->users.enumerate()) {
            auto &ad = nd.arcs.at(usr.index.idx());
            for (size_t j = 0; j < ad.size(); j++) {
                // Ripup failed arcs to start with
                // Check if arc is already legally routed
                if (!failed_slack && check_arc_routing(net, usr.index, j)) {
                    update_wire_by_loc(t, net, usr.index, j, true);
                    continue;
                }

                // Ripup arc to start with
                ripup_arc(net, usr.index, j);
                t.route_arcs.emplace_back(usr.index, j);
            }
        }
        // Route most critical arc first
        std::stable_sort(t.route_arcs.begin(), t.route_arcs.end(),
                         [&](std::pair<store_index<PortRef>, size_t> a, std::pair<store_index<PortRef>, size_t> b) {
                             return get_arc_crit(net, a.first) > get_arc_crit(net, b.first);
                         });
        for (auto a : t.route_arcs) {
            const bool this_net_needs_no_bounding_box = nd.bb_useless && !is_mt;
            auto res1 = this_net_needs_no_bounding_box ? ARC_RETRY_WITHOUT_BB
                                                      : route_arc(t, net, a.first, a.second, is_mt, true);
            if (res1 == ARC_FATAL)
                return false; // Arc failed irrecoverably
            else if (res1 == ARC_RETRY_WITHOUT_BB) {
                if (is_mt) {
                    // Can't break out of bounding box in multi-threaded mode, so mark this arc as a failure
                    have_failures = true;
                } else {
                    // Attempt a re-route without the bounding box constraint
                    ROUTE_LOG_DBG("Rerouting arc %d.%d of net '%s' without bounding box, possible tricky routing...\n",
                                  a.first.idx(), int(a.second), ctx->nameOf(net));
                    auto res2 = route_arc(t, net, a.first, a.second, is_mt, false);
                    const bool the_unbounded_search_was_needed = (res2 == ARC_SUCCESS);
                    if (the_unbounded_search_was_needed)
                        nd.bb_useless = true;
                    // If this also fails, no choice but to give up
                    if (res2 != ARC_SUCCESS) {
                        if (ctx->debug) {
                            log_info("Pre-bound routing: \n");
                            for (auto &wire_pair : net->wires) {
                                log("        %s", ctx->nameOfWire(wire_pair.first));
                                if (wire_pair.second.pip != PipId())
                                    log(" %s", ctx->nameOfPip(wire_pair.second.pip));
                                log("\n");
                            }
                        }
                        log_error("Failed to route arc %d.%d of net '%s', from %s to %s.\n", a.first.idx(),
                                  int(a.second), ctx->nameOf(net), ctx->nameOfWire(ctx->getNetinfoSourceWire(net)),
                                  ctx->nameOfWire(ctx->getNetinfoSinkWire(net, net->users.at(a.first), a.second)));
                    }
                }
            }
        }
        if (cfg.perf_profile) {
            auto rend = std::chrono::high_resolution_clock::now();
            nets.at(net->udata).total_route_us +=
                    (std::chrono::duration_cast<std::chrono::microseconds>(rend - rstart).count());
        }
        return !have_failures;
    }
#undef ROUTE_LOG_DBG

    int total_wire_use = 0;
    int overused_wires = 0;
    int total_wire_overuse = 0;
    int total_resource_use = 0;
    int overused_resources = 0;
    int total_resource_overuse = 0;
    std::vector<int> route_queue;
    std::set<int> failed_nets;

    // Size a net's bounding box from its timing criticality, not from how many
    // times it has failed to route.  This is the inverse of grow-on-congestion:
    // a critical net (crit -> 1) gets the TIGHTEST box, which forces its route
    // short and leaves the negotiated-congestion loop to clear the short lanes
    // for it by ripping up the slack nets in the way; a slack net (crit -> 0)
    // gets the LOOSEST box, free to detour around congestion and stay off those
    // lanes.  The box thus encodes "how short must this be", which is a timing
    // property, rather than "how stuck is this", which drove the divergence.
    //
    // bb_expansions is added on top as a back-off: a critical net that its tight
    // box leaves genuinely unroutable earns a little room per failed iteration
    // (capped), and gets it back the moment it routes clean.  So the tightness
    // is a target, not a trap.
    void apply_budget_bb(PerNetData &nd)
    {
        // crit in [0,1]; loose in [0,1] is 0 for the most critical net.
        float loose = nd.max_crit >= 1.0f ? 0.0f : (1.0f - nd.max_crit);
        int span_x = cfg.bb_margin_x + int((cfg.bb_budget_max - cfg.bb_margin_x) * loose) + nd.bb_expansions;
        int span_y = cfg.bb_margin_y + int((cfg.bb_budget_max - cfg.bb_margin_y) * loose) + nd.bb_expansions;
        nd.bb.x0 = std::max(nd.pin_bb.x0 - span_x, 0);
        nd.bb.y0 = std::max(nd.pin_bb.y0 - span_y, 0);
        nd.bb.x1 = std::min(nd.pin_bb.x1 + span_x, ctx->getGridDimX());
        nd.bb.y1 = std::min(nd.pin_bb.y1 + span_y, ctx->getGridDimY());
    }

    void update_congestion()
    {
        total_wire_overuse = 0;
        overused_wires = 0;
        total_wire_use = 0;
        total_resource_overuse = 0;
        overused_resources = 0;
        total_resource_use = 0;
        failed_nets.clear();
        pool<WireId> already_updated_wires;
        pool<GroupId> already_updated_resources;
        for (size_t i = 0; i < nets.size(); i++) {
            auto &nd = nets.at(i);
            for (const auto &w : nd.wires) {
                ++total_wire_use;
                auto &wd = wire_data(w.first);
                if (wd.curr_cong > 1) {
                    if (already_updated_wires.count(w.first)) {
                        ++total_wire_overuse;
                    } else {
                        if (curr_cong_weight > 0)
                            wd.hist_cong_cost =
                                    std::min(1e9, wd.hist_cong_cost + (wd.curr_cong - 1) * hist_cong_weight);
                        already_updated_wires.insert(w.first);
                        ++overused_wires;
                    }
                    failed_nets.insert(i);
                }
            }
            for (const auto &r : nd.resources) {
                ++total_resource_use;
                auto &rd = resource_data(r.first);
                if (rd.value_count.size() > 1) {
                    if (already_updated_resources.count(r.first)) {
                        ++total_resource_overuse;
                    } else {
                        if (curr_cong_weight > 0)
                            rd.hist_cong_cost =
                                    std::min(1e9, rd.hist_cong_cost + (rd.value_count.size() - 1) * hist_cong_weight);
                        already_updated_resources.insert(r.first);
                        ++overused_resources;
                    }
                    failed_nets.insert(i);
                }
            }
        }
        for (int n : failed_nets) {
            auto &net_data = nets.at(n);
            ++net_data.fail_count;
            bool third_consecutive_failure = (net_data.fail_count % 3) == 0;
            if (third_consecutive_failure) {
                // Every three times a net fails to route, expand the bounding box to increase the search space.
                //
                // Unbounded, this is one tile per side every third overused
                // iteration, forever: a net that stays congested for 160
                // iterations ends up with a box 53 tiles bigger on every side
                // than it started, which on this fabric is the whole device.
                // That is the regime in which large designs DIVERGE -- overuse
                // falls to a floor around iteration 30 and then climbs, with
                // total wire use climbing alongside it, because a net with a
                // device-sized box and a present-cost weight that has grown
                // without limit takes an ever-longer detour, and the detours
                // collide with each other faster than the hotspots clear.  A
                // wider box in that regime is more room to make a worse
                // choice.  So with bbExpandMax set, stop growing after that
                // many expansions.
                bool bb_expansion_allowed = cfg.bb_expand_max == 0 || net_data.bb_expansions < cfg.bb_expand_max;
                if (cfg.bb_budget) {
                    // The box is rebuilt from criticality every iteration, so
                    // growing it here would be overwritten; accumulate the
                    // back-off margin instead and let apply_budget_bb() pick it
                    // up next iteration.  Capped so a stubborn net cannot walk
                    // its box across the device the way the unbounded path did.
                    int cap = cfg.bb_expand_max > 0 ? cfg.bb_expand_max : 12;
                    if (net_data.bb_expansions < cap)
                        ++net_data.bb_expansions;
                } else if (bb_expansion_allowed) {
                    ctx->expandBoundingBox(net_data.bb);
                    ++net_data.bb_expansions;
                }
            }
        }
        // ...and let the pressure decay.  A net that routes clean this
        // iteration no longer needs the box it earned while it was congested;
        // give it back its starting box, so that when a neighbour disturbs it
        // later it searches close to home first rather than across the device.
        // Only with bbExpandMax set or in budget mode, so the default path is
        // untouched.
        bool reset_bb_after_clean_route = cfg.bb_expand_max > 0 || cfg.bb_budget;
        if (reset_bb_after_clean_route) {
            for (size_t i = 0; i < nets.size(); i++) {
                auto &nd = nets.at(i);
                bool net_did_not_just_recover = nd.fail_count == 0 || failed_nets.count(int(i));
                if (net_did_not_just_recover)
                    continue;
                nd.fail_count = 0;
                nd.bb_expansions = 0;
                if (!cfg.bb_budget)
                    nd.bb = nd.base_bb; // budget mode rebuilds bb from crit next iteration
            }
        }
    }

    bool bind_and_check(NetInfo *net, store_index<PortRef> usr_idx, int phys_pin)
    {
#ifdef ARCH_ECP5
        if (net->is_global)
            return true;
#endif
        bool success = true;
        auto &nd = nets.at(net->udata);
        auto &ad = nd.arcs.at(usr_idx.idx()).at(phys_pin);
        auto &usr = net->users.at(usr_idx);
        WireId src = ctx->getNetinfoSourceWire(net);
        // Skip routes with no source
        if (src == WireId() && net->constant_value == IdString())
            return true;
        WireId dst = ctx->getNetinfoSinkWire(net, usr, phys_pin);
        if (dst == WireId())
            return true;

        // Skip routes where there is no routing (special cases)
        if (!ad.routed) {
            if ((src == dst) && ctx->getBoundWireNet(dst) != net)
                ctx->bindWire(src, net, STRENGTH_WEAK);
            if (ctx->debug) {
                log("Net %s not routed, not binding\n", ctx->nameOf(net));
            }
            return true;
        }

        WireId cursor = dst;

        std::vector<PipId> to_bind;

        while (cursor != src) {
            if (!ctx->checkWireAvail(cursor)) {
                NetInfo *bound_net = ctx->getBoundWireNet(cursor);
                if (bound_net != net) {
                    if (ctx->verbose) {
                        if (bound_net != nullptr) {
                            log_info("Failed to bind wire %s to net %s, bound to net %s\n", ctx->nameOfWire(cursor),
                                     net->name.c_str(ctx), bound_net->name.c_str(ctx));
                        } else {
                            log_info("Failed to bind wire %s to net %s, bound net nullptr\n", ctx->nameOfWire(cursor),
                                     net->name.c_str(ctx));
                        }
                    }
                    success = false;
                    break;
                }
            }
            if (!nd.wires.count(cursor)) {
                log("Failure details:\n");
                log("    Cursor: %s\n", ctx->nameOfWire(cursor));
                log_error("Internal error; incomplete route tree for arc %d of net %s.\n", usr_idx.idx(),
                          ctx->nameOf(net));
            }
            PipId p = nd.wires.at(cursor).first;
            if (ctx->checkPipAvailForNet(p, net)) {
                NetInfo *bound_net = ctx->getBoundPipNet(p);
                if (bound_net == nullptr) {
                    to_bind.push_back(p);
                }
            } else if (!ad.pre_routed ||
                       ctx->getBoundPipNet(p) != net) { // allow pre routing to break normal validity checking rules
                if (ctx->verbose) {
                    log_info("Failed to bind pip %s to net %s\n", ctx->nameOfPip(p), net->name.c_str(ctx));
                }
                success = false;
                break;
            }
            cursor = ctx->getPipSrcWire(p);
            if (net->constant_value != IdString() && ctx->getWireConstantValue(cursor) == net->constant_value) {
                src = cursor;
                break;
            }
        }

        if (success) {
            if (ctx->getBoundWireNet(src) == nullptr)
                ctx->bindWire(src, net, STRENGTH_WEAK);
            for (auto tb : to_bind)
                ctx->bindPip(tb, net, STRENGTH_WEAK);
        } else {
            ripup_arc(net, usr_idx, phys_pin);
            failed_nets.insert(net->udata);
        }
        return success;
    }

    int arch_fail = 0;
    bool bind_and_check_all()
    {
        // Make sure arch is internally consistent before we mess with it.
        ctx->check();

        bool success = true;
        std::vector<WireId> net_wires;
        for (auto net : nets_by_udata) {
#ifdef ARCH_ECP5
            if (net->is_global)
                continue;
#endif
            // Ripup wires and pips used by the net in nextpnr's structures
            net_wires.clear();
            for (auto &w : net->wires) {
                if (w.second.strength <= STRENGTH_STRONG) {
                    net_wires.push_back(w.first);
                } else if (ctx->debug) {
                    log("Net %s didn't rip up wire %s because strength was %d\n", ctx->nameOf(net),
                        ctx->nameOfWire(w.first), w.second.strength);
                }
            }
            for (auto w : net_wires)
                ctx->unbindWire(w);

            if (ctx->debug) {
                log("Ripped up %zu wires on net %s\n", net_wires.size(), ctx->nameOf(net));
            }

            // Bind the arcs using the routes we have discovered
            for (auto usr : net->users.enumerate()) {
                for (size_t phys_pin = 0; phys_pin < nets.at(net->udata).arcs.at(usr.index.idx()).size(); phys_pin++) {
                    if (!bind_and_check(net, usr.index, phys_pin)) {
                        ++arch_fail;
                        success = false;
                    }
                }
            }
        }

        // Check that the arch is still internally consistent!
        ctx->check();

        return success;
    }

    void write_congestion_by_wiretype_heatmap(std::ostream &out)
    {
        dict<IdString, std::vector<int>> cong_by_type;
        size_t max_cong = 0;
        // Build histogram
        for (auto &wd : flat_wires) {
            size_t val = wd.curr_cong;
            IdString type = ctx->getWireType(wd.w);
            max_cong = std::max(max_cong, val);
            if (cong_by_type[type].size() <= max_cong)
                cong_by_type[type].resize(max_cong + 1);
            cong_by_type[type].at(val) += 1;
        }
        // Write csv
        out << "type,";
        for (size_t i = 0; i <= max_cong; i++)
            out << "bound=" << i << ",";
        out << std::endl;
        for (auto &ty : cong_by_type) {
            out << ctx->nameOf(ty.first) << ",";
            for (int count : ty.second)
                out << count << ",";
            out << std::endl;
        }
    }

    void write_utilisation_by_wiretype_heatmap(std::ostream &out)
    {
        dict<IdString, int> util_by_type;
        for (auto &wd : flat_wires) {
            IdString type = ctx->getWireType(wd.w);
            if (wd.curr_cong > 0)
                util_by_type[type] += wd.curr_cong;
        }
        // Write csv
        for (auto &u : util_by_type)
            out << u.first.c_str(ctx) << "," << u.second << std::endl;
    }

    void write_congestion_by_coordinate_heatmap(std::ostream &out)
    {
        auto util_by_coord =
                std::vector<std::vector<int>>(ctx->getGridDimX() + 1, std::vector<int>(ctx->getGridDimY() + 1, 0));
        for (auto &wd : flat_wires)
            if (wd.curr_cong > 1)
                util_by_coord[wd.x][wd.y] += wd.curr_cong;
        // Write csv
        for (auto &x : util_by_coord) {
            for (auto y : x)
                out << y << ",";
            out << std::endl;
        }
    }

    void write_congestion_by_net_heatmap(std::ostream &out)
    {
        dict<IdString, int> congestion_by_net;
        for (size_t i = 0; i < nets_by_udata.size(); i++) {
            IdString name = nets_by_udata.at(i)->name;
            for (const auto &wire : nets.at(i).wires) {
                const auto &wd = flat_wires.at(wire_to_idx.at(wire.first));
                if (wd.curr_cong > 1)
                    congestion_by_net[name] += (wd.curr_cong - 1);
            }
        }
        // Write csv
        for (auto &u : congestion_by_net)
            out << u.first.c_str(ctx) << "," << u.second << std::endl;
    }

    int mid_x = 0, mid_y = 0;

    void partition_nets()
    {
        // Create a histogram of positions in X and Y positions
        std::map<int, int> cxs, cys;
        for (auto &n : nets) {
            if (n.cx != -1)
                ++cxs[n.cx];
            if (n.cy != -1)
                ++cys[n.cy];
        }
        // 4-way split for now
        int accum_x = 0, accum_y = 0;
        int halfway = int(nets.size()) / 2;
        for (auto &p : cxs) {
            if (accum_x < halfway && (accum_x + p.second) >= halfway)
                mid_x = p.first;
            accum_x += p.second;
        }
        for (auto &p : cys) {
            if (accum_y < halfway && (accum_y + p.second) >= halfway)
                mid_y = p.first;
            accum_y += p.second;
        }
        if (ctx->verbose) {
            log_info("    x splitpoint: %d\n", mid_x);
            log_info("    y splitpoint: %d\n", mid_y);
        }
        std::vector<int> bins(5, 0);
        for (auto &n : nets) {
            if (n.bb.x0 < mid_x && n.bb.x1 < mid_x && n.bb.y0 < mid_y && n.bb.y1 < mid_y)
                ++bins[0]; // TL
            else if (n.bb.x0 >= mid_x && n.bb.x1 >= mid_x && n.bb.y0 < mid_y && n.bb.y1 < mid_y)
                ++bins[1]; // TR
            else if (n.bb.x0 < mid_x && n.bb.x1 < mid_x && n.bb.y0 >= mid_y && n.bb.y1 >= mid_y)
                ++bins[2]; // BL
            else if (n.bb.x0 >= mid_x && n.bb.x1 >= mid_x && n.bb.y0 >= mid_y && n.bb.y1 >= mid_y)
                ++bins[3]; // BR
            else
                ++bins[4]; // cross-boundary
        }
        if (ctx->verbose)
            for (int i = 0; i < 5; i++)
                log_info("        bin %d N=%d\n", i, bins[i]);
    }

    void router_thread(ThreadContext &t, bool is_mt)
    {
        for (auto n : t.route_nets) {
            bool result = route_net(t, n, is_mt);
            if (!result)
                t.failed_nets.push_back(n);
        }
    }

    void do_route()
    {
        // Don't multithread if fewer than 200 nets (heuristic)
        if (route_queue.size() < 200) {
            ThreadContext st;
            st.rng.rngseed(ctx->rng64());
            st.bb = BoundingBox(0, 0, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
            for (size_t j = 0; j < route_queue.size(); j++) {
                route_net(st, nets_by_udata[route_queue[j]], false);
            }
            return;
        }
        const int Nq = 4, Nv = 2, Nh = 2;
        const int N = Nq + Nv + Nh;
        std::vector<ThreadContext> tcs(N + 1);
        for (auto &th : tcs) {
            th.rng.rngseed(ctx->rng64());
        }
        int le_x = mid_x;
        int rs_x = mid_x;
        int le_y = mid_y;
        int rs_y = mid_y;
        // Set up thread bounding boxes
        tcs.at(0).bb = BoundingBox(0, 0, mid_x, mid_y);
        tcs.at(1).bb = BoundingBox(mid_x + 1, 0, std::numeric_limits<int>::max(), le_y);
        tcs.at(2).bb = BoundingBox(0, mid_y + 1, mid_x, std::numeric_limits<int>::max());
        tcs.at(3).bb =
                BoundingBox(mid_x + 1, mid_y + 1, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());

        tcs.at(4).bb = BoundingBox(0, 0, std::numeric_limits<int>::max(), mid_y);
        tcs.at(5).bb = BoundingBox(0, mid_y + 1, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());

        tcs.at(6).bb = BoundingBox(0, 0, mid_x, std::numeric_limits<int>::max());
        tcs.at(7).bb = BoundingBox(mid_x + 1, 0, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());

        tcs.at(8).bb = BoundingBox(0, 0, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());

        for (auto n : route_queue) {
            auto &nd = nets.at(n);
            auto ni = nets_by_udata.at(n);
            int bin = N;
            // Quadrants
            if (nd.bb.x0 < le_x && nd.bb.x1 < le_x && nd.bb.y0 < le_y && nd.bb.y1 < le_y)
                bin = 0;
            else if (nd.bb.x0 >= rs_x && nd.bb.x1 >= rs_x && nd.bb.y0 < le_y && nd.bb.y1 < le_y)
                bin = 1;
            else if (nd.bb.x0 < le_x && nd.bb.x1 < le_x && nd.bb.y0 >= rs_y && nd.bb.y1 >= rs_y)
                bin = 2;
            else if (nd.bb.x0 >= rs_x && nd.bb.x1 >= rs_x && nd.bb.y0 >= rs_y && nd.bb.y1 >= rs_y)
                bin = 3;
            // Vertical split
            else if (nd.bb.y0 < le_y && nd.bb.y1 < le_y)
                bin = Nq + 0;
            else if (nd.bb.y0 >= rs_y && nd.bb.y1 >= rs_y)
                bin = Nq + 1;
            // Horizontal split
            else if (nd.bb.x0 < le_x && nd.bb.x1 < le_x)
                bin = Nq + Nv + 0;
            else if (nd.bb.x0 >= rs_x && nd.bb.x1 >= rs_x)
                bin = Nq + Nv + 1;
            tcs.at(bin).route_nets.push_back(ni);
        }
        if (ctx->verbose)
            log_info("%d/%d nets not multi-threadable\n", int(tcs.at(N).route_nets.size()), int(route_queue.size()));
#ifdef NPNR_DISABLE_THREADS
        // Singlethreaded routing - quadrants
        for (int i = 0; i < Nq; i++) {
            router_thread(tcs.at(i), /*is_mt=*/false);
        }
        // Vertical splits
        for (int i = Nq; i < Nq + Nv; i++) {
            router_thread(tcs.at(i), /*is_mt=*/false);
        }
        // Horizontal splits
        for (int i = Nq + Nv; i < Nq + Nv + Nh; i++) {
            router_thread(tcs.at(i), /*is_mt=*/false);
        }
#else
        // Multithreaded part of routing - quadrants
        std::vector<boost::thread> threads;
        for (int i = 0; i < Nq; i++) {
            threads.emplace_back([this, &tcs, i]() { router_thread(tcs.at(i), /*is_mt=*/true); });
        }
        for (auto &t : threads)
            t.join();
        threads.clear();
        // Vertical splits
        for (int i = Nq; i < Nq + Nv; i++) {
            threads.emplace_back([this, &tcs, i]() { router_thread(tcs.at(i), /*is_mt=*/true); });
        }
        for (auto &t : threads)
            t.join();
        threads.clear();
        // Horizontal splits
        for (int i = Nq + Nv; i < Nq + Nv + Nh; i++) {
            threads.emplace_back([this, &tcs, i]() { router_thread(tcs.at(i), /*is_mt=*/true); });
        }
        for (auto &t : threads)
            t.join();
        threads.clear();
#endif
        // Singlethreaded part of routing - nets that cross partitions
        // or don't fit within bounding box
        for (auto st_net : tcs.at(N).route_nets)
            route_net(tcs.at(N), st_net, false);
        // Failed nets
        for (int i = 0; i < N; i++)
            for (auto fail : tcs.at(i).failed_nets)
                route_net(tcs.at(N), fail, false);
    }

    delay_t get_route_delay(int net, store_index<PortRef> usr_idx, int phys_idx)
    {
        auto &nd = nets.at(net);
        if (nets_by_udata.at(net)->constant_value != IdString())
            return 0;
        auto &ad = nd.arcs.at(usr_idx.idx()).at(phys_idx);
        WireId cursor = ad.sink_wire;
        if (cursor == WireId() || nd.src_wire == WireId())
            return 0;
        delay_t delay = 0;
        while (true) {
            delay += ctx->getWireDelay(cursor).maxDelay();
            if (!nd.wires.count(cursor))
                break;
            auto &bound = nd.wires.at(cursor);
            if (bound.first == PipId())
                break;
            delay += ctx->getPipDelay(bound.first).maxDelay();
            cursor = ctx->getPipSrcWire(bound.first);
        }
        NPNR_ASSERT(cursor == nd.src_wire);
        return delay;
    }

    void update_route_delays()
    {
        for (int net : route_queue) {
            NetInfo *ni = nets_by_udata.at(net);
#ifdef ARCH_ECP5
            if (ni->is_global)
                continue;
#endif
            auto &nd = nets.at(net);
            for (auto usr : ni->users.enumerate()) {
                delay_t arc_delay = 0;
                for (int j = 0; j < int(nd.arcs.at(usr.index.idx()).size()); j++)
                    arc_delay = std::max(arc_delay, get_route_delay(net, usr.index, j));
                tmg.set_route_delay(CellPortKey(usr.value), DelayPair(arc_delay));
            }
        }
    }

    // ---- congestion smoothing ---------------------------------------------
    //
    // A post-legality pass.  See docs/routing-smoothing-pass.md for the
    // measurement that motivates it: on a VexRiscv-SMP SoC nextpnr spends
    // 1.97x Vivado's interconnect over 1.10x the tiles, so each tile carries
    // more rather than the design spreading wider, and that design misses
    // its clock by 25%.

    dict<int64_t, int> tile_occ;
    bool smoothing_active = false;
    int smooth_target = 0;

    static int64_t tile_key(int x, int y) { return int64_t(x) * 100000 + int64_t(y); }

    void build_tile_occupancy()
    {
        tile_occ.clear();
        for (auto &n : nets_by_udata) {
            if (n == nullptr)
                continue;
            auto &nd = nets.at(n->udata);
            for (auto &w : nd.wires) {
                PipId pip = w.second.first;
                if (pip == PipId())
                    continue;
                Loc pl = ctx->getPipLocation(pip);
                ++tile_occ[tile_key(pl.x, pl.y)];
            }
        }
    }

    // mean / percentile / max of tile occupancy, and the total pip count --
    // the same numbers the design document compares against Vivado, so the
    // pass can be judged by the metric that motivated it.
    std::tuple<float, int, int, int64_t> occupancy_stats(float pct)
    {
        std::vector<int> v;
        v.reserve(tile_occ.size());
        int64_t total = 0;
        for (auto &kv : tile_occ) {
            v.push_back(kv.second);
            total += kv.second;
        }
        if (v.empty())
            return {0.0f, 0, 0, 0};
        std::sort(v.begin(), v.end());
        size_t idx = std::min(v.size() - 1, size_t(pct * v.size()));
        return {float(total) / float(v.size()), v.at(idx), v.back(), total};
    }

    // A net's routing as the router's own structures hold it.  This is
    // everything a trial re-route can disturb, PROVIDED the Arch-level binding
    // has been released first -- see unbind_arch_routing().
    struct NetRouteSnapshot
    {
        dict<WireId, std::pair<PipId, int>> wires;
        std::vector<std::vector<std::pair<bool, bool>>> arcs; // routed, pre_routed
    };

    NetRouteSnapshot snapshot_net(NetInfo *net)
    {
        auto &nd = nets.at(net->udata);
        NetRouteSnapshot s;
        s.wires = nd.wires;
        s.arcs.resize(nd.arcs.size());
        for (size_t u = 0; u < nd.arcs.size(); u++) {
            s.arcs.at(u).reserve(nd.arcs.at(u).size());
            for (auto &ad : nd.arcs.at(u))
                s.arcs.at(u).emplace_back(ad.routed, ad.pre_routed);
        }
        return s;
    }

    void restore_net(NetInfo *net, const NetRouteSnapshot &s)
    {
        auto &nd = nets.at(net->udata);
        // Unwind whatever is bound now.  Ripping the arcs walks each route tree
        // back to the source; draining any residue afterwards keeps the
        // per-wire congestion and the per-resource counts falling together,
        // which is what unbind_pip_internal() exists to do.
        for (auto usr : net->users.enumerate())
            for (size_t j = 0; j < nd.arcs.at(usr.index.idx()).size(); j++)
                ripup_arc(net, usr.index, j);
        while (!nd.wires.empty()) {
            WireId w = nd.wires.begin()->first;
            unbind_pip_internal(nd, store_index<PortRef>(), w);
        }
        // Re-bind exactly the pips that were there, each as many times as it
        // had arcs sharing it, so curr_cong and the resource counts come back
        // to the values the snapshot was taken at.
        for (auto &w : s.wires)
            for (int k = 0; k < w.second.second; k++)
                bind_pip_internal(nd, store_index<PortRef>(), wire_to_idx.at(w.first), w.second.first);
        for (size_t u = 0; u < nd.arcs.size(); u++)
            for (size_t j = 0; j < nd.arcs.at(u).size(); j++) {
                nd.arcs.at(u).at(j).routed = s.arcs.at(u).at(j).first;
                nd.arcs.at(u).at(j).pre_routed = s.arcs.at(u).at(j).second;
            }
    }

    // How many of this net's pips sit in a tile above the round's target.
    // Measured against tile_occ as built at the start of the round, so a before
    // and an after reading of the same net are on the same yardstick.
    int net_hot_pips(const PerNetData &nd)
    {
        int hot = 0;
        for (auto &w : nd.wires) {
            PipId pip = w.second.first;
            if (pip == PipId())
                continue;
            Loc pl = ctx->getPipLocation(pip);
            auto it = tile_occ.find(tile_key(pl.x, pl.y));
            bool pip_tile_is_hot = it != tile_occ.end() && it->second > smooth_target;
            if (pip_tile_is_hot)
                ++hot;
        }
        return hot;
    }

    // Is this net driven by a clock buffer or clock generator?  Such nets
    // reach their sinks over dedicated global routing that route_net() cannot
    // rebuild, so smoothing must leave them alone.
    bool driven_by_clock_resource(const NetInfo *n) const
    {
        bool driver_is_placed = n->driver.cell != nullptr && n->driver.cell->bel != BelId();
        if (!driver_is_placed)
            return false;
        std::string bt = ctx->getBelType(n->driver.cell->bel).str(ctx);
        for (const char *clock_bel : {"BUFG", "BUFH", "BUFR", "BUFIO", "MMCM", "PLL"})
            if (bt.find(clock_bel) != std::string::npos)
                return true;
        return false;
    }

    dict<int, NetRouteSnapshot> snapshot_all()
    {
        dict<int, NetRouteSnapshot> s;
        for (auto net : nets_by_udata)
            if (net != nullptr)
                s[net->udata] = snapshot_net(net);
        return s;
    }

    void restore_all(const dict<int, NetRouteSnapshot> &s)
    {
        for (auto &kv : s)
            restore_net(nets_by_udata.at(kv.first), kv.second);
    }

    // Release the Arch-level binding.  bind_and_check_all() runs inside the main
    // loop, on the first iteration that comes out legal, so by the time this
    // pass gets to look at anything every wire in the design is bound to its
    // net in the Arch.  The search itself does not care -- it works off
    // PerWireData, which was fixed at setup -- but the re-bind at the end does:
    // bind_and_check_all() rips up and re-binds one net at a time, so a net
    // whose route moved can collide with a net not yet reached.  Clearing the
    // lot first removes that ordering hazard.
    int unbind_arch_routing()
    {
        int released = 0;
        std::vector<WireId> net_wires;
        for (auto net : nets_by_udata) {
            if (net == nullptr)
                continue;
#ifdef ARCH_ECP5
            if (net->is_global)
                continue;
#endif
            net_wires.clear();
            for (auto &w : net->wires)
                if (w.second.strength <= STRENGTH_STRONG)
                    net_wires.push_back(w.first);
            for (auto w : net_wires)
                ctx->unbindWire(w);
            released += int(net_wires.size());
        }
        return released;
    }

    void smooth_congestion(ThreadContext &st)
    {
        if (cfg.smooth_iters <= 0)
            return;
        build_tile_occupancy();
        auto [mean0, p0, max0, total0] = occupancy_stats(cfg.smooth_percentile);
        log_info("Smoothing congestion: %d tiles, mean %.1f, p%.0f %d, max %d, %d pips\n",
                 int(tile_occ.size()), mean0, cfg.smooth_percentile * 100, p0, max0, int(total0));

        // One transaction over the whole pass, on top of the per-net one: if
        // the Arch will not take the smoothed result, the routing that got here
        // goes back untouched.
        auto all_snap = snapshot_all();
        int arch_fail_before = arch_fail;
        // The ThreadContext handed to this pass comes from operator(), which
        // never set its bb -- BoundingBox default-constructs to (-1,-1,-1,-1),
        // and thread_test_wire() then rejects every wire in the fabric, so the
        // search cannot expand at all and only arcs whose two ends already meet
        // can route.  Open it to the whole device, as the single-threaded main
        // loop does for its own context.
        st.bb = BoundingBox(0, 0, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
        log_info("    released %d arch-bound wires for the duration of the pass\n", unbind_arch_routing());

        int stagnant = 0;
        for (int round = 1; round <= cfg.smooth_iters; round++) {
            build_tile_occupancy();
            auto [mean, p95, mx, total] = occupancy_stats(cfg.smooth_percentile);
            (void)mean; (void)mx; (void)total;
            smooth_target = p95;
            if (smooth_target <= 0)
                break;

            // Nets with pips in a hot tile, least critical first: slack is the
            // budget this pass spends, so spend it where there is some.
            dict<int, int> hot_pips; // net udata -> pips in hot tiles
            for (auto &n : nets_by_udata) {
                if (n == nullptr)
                    continue;
                auto &nd = nets.at(n->udata);
                // Never touch a clock.  It reaches its sinks over dedicated
                // global resources that a general re-route cannot reconstruct,
                // so ripping up 'clk' and asking route_net for it back fails
                // outright --
                //   ERROR: Failed to route arc 0.0 of net 'clk', from
                //   BUFGCTRL_X0Y5.O to SLICE_X0Y0.CLKINV_OUT
                // The snapshot below would put it back, but the attempt is
                // certain to fail and costs a full search to find that out.
                // NetInfo::is_global exists only on some arches (not this one)
                // and criticality does not flag clocks either, so the test is
                // the driver's bel type.  That is the whole of it: fanout is
                // NOT a proxy for this -- see smoothMaxFanout.
                bool fanout_too_high = int(n->users.entries()) > cfg.smooth_max_fanout;
                if (fanout_too_high)
                    continue;
                // Driven by a clock buffer?  Then it reaches its sinks over
                // dedicated global routing.  Fanout does not identify these --
                // the johnson example's clock has 25 sinks and still cannot be
                // rebuilt by route_net() -- so test the driver directly.
                bool is_clock_net = driven_by_clock_resource(n);
                if (is_clock_net)
                    continue;
                bool too_critical_to_move = nd.max_crit >= cfg.smooth_max_crit;
                if (too_critical_to_move)
                    continue;
                // Trivial local nets are not worth a search.  A net held in
                // a handful of pips is an intra-slice hop -- a flip-flop
                // output back into a LUT input in the same slice -- which
                // occupies no interconnect, so there is no density to move:
                //   Failed to route arc 0.0 of net 'core.prbs[19]',
                //   from SLICE_X1Y0.A5FF_Q to SLICE_X1Y0.A4
                bool too_small_to_move = int(nd.wires.size()) < cfg.smooth_min_wires;
                if (too_small_to_move)
                    continue;
                int hot = net_hot_pips(nd);
                if (hot > 0)
                    hot_pips[n->udata] = hot;
            }
            if (hot_pips.empty())
                break;

            // Why the peak does not move.  The mean comes down and the max
            // sits still, so say who owns the busiest tile and which filter
            // put each of its nets out of reach.
            if (round == 1) {
                int64_t peak_key = 0;
                int peak = 0;
                for (auto &kv : tile_occ)
                    if (kv.second > peak) {
                        peak = kv.second;
                        peak_key = kv.first;
                    }
                int n_clock = 0, n_fanout = 0, n_crit = 0, n_small = 0, n_eligible = 0, n_pips = 0;
                for (auto &n : nets_by_udata) {
                    if (n == nullptr)
                        continue;
                    auto &nd2 = nets.at(n->udata);
                    int here = 0;
                    for (auto &w : nd2.wires) {
                        if (w.second.first == PipId())
                            continue;
                        Loc pl = ctx->getPipLocation(w.second.first);
                        if (tile_key(pl.x, pl.y) == peak_key)
                            ++here;
                    }
                    if (here == 0)
                        continue;
                    n_pips += here;
                    bool clockish = driven_by_clock_resource(n);
                    bool fanout_too_high = int(n->users.entries()) > cfg.smooth_max_fanout;
                    bool too_critical_to_move = nd2.max_crit >= cfg.smooth_max_crit;
                    bool too_small_to_move = int(nd2.wires.size()) < cfg.smooth_min_wires;
                    if (clockish)
                        n_clock += here;
                    else if (fanout_too_high)
                        n_fanout += here;
                    else if (too_critical_to_move)
                        n_crit += here;
                    else if (too_small_to_move)
                        n_small += here;
                    else
                        n_eligible += here;
                }
                log_info("    busiest tile X%dY%d holds %d pips: %d eligible, %d clock-driven, %d high-fanout, "
                         "%d critical, %d too-small\n",
                         int(peak_key / 100000), int(peak_key % 100000), n_pips, n_eligible, n_clock, n_fanout,
                         n_crit, n_small);
            }

            std::vector<std::pair<int, int>> victims; // (hot pips, udata)
            for (auto &kv : hot_pips)
                victims.emplace_back(kv.second, kv.first);
            std::sort(victims.begin(), victims.end(), std::greater<std::pair<int, int>>());
            // Bounded work per round.
            size_t cap = std::min<size_t>(
                    victims.size(), std::max<size_t>(64, size_t(victims.size() * cfg.smooth_cap_frac)));

            // Each net below is a trial: ripped up, re-routed against the
            // density cost, and kept only if it comes back no worse.  A net the
            // search cannot rebuild reports that with log_error(), which prints
            // an ERROR line before it throws, so say in advance what those
            // lines mean -- they are moves this pass declined and undid, not
            // failures of the run.
            log_info("    round %d: trying %d nets; any ERROR below is a declined move, and is undone\n", round,
                     int(cap));
            smoothing_active = true;
            int rerouted = 0, rejected = 0, unroutable = 0;
            for (size_t i = 0; i < cap; i++) {
                NetInfo *net = nets_by_udata.at(victims.at(i).second);
                if (net == nullptr)
                    continue;
                auto &nd = nets.at(net->udata);
                int hot_before = net_hot_pips(nd);
                // Take the snapshot before touching anything.  A re-route that
                // fails, or that comes back no better, is then a move this pass
                // declines rather than damage it has to live with.
                NetRouteSnapshot snap = snapshot_net(net);
                // Force the rip-up.  route_net() keeps any arc that is already
                // legally routed, so without this it would look at a legal net
                // and do nothing -- the density cost would never be consulted.
                for (auto usr : net->users.enumerate()) {
                    auto &ad = nd.arcs.at(usr.index.idx());
                    for (size_t j = 0; j < ad.size(); j++) {
                        // An arc the placer pre-bound is allowed to break the
                        // normal availability rules (see bind_and_check), and
                        // ripup_arc() drops that privilege on the way out -- as
                        // its own comment says, the routing may then no longer
                        // be the same as before.  Leave those alone.
                        if (ad.at(j).pre_routed)
                            continue;
                        WireId sink = ad.at(j).sink_wire;
                        bool arc_has_no_endpoints = nd.src_wire == WireId() || sink == WireId();
                        if (arc_has_no_endpoints)
                            continue;
                        // Leave arcs that stay inside one tile alone: a
                        // flip-flop output back into a LUT input in the same
                        // slice occupies no interconnect, so there is nothing
                        // for smoothing to move, and the general search cannot
                        // rebuild that hop once it is ripped.  ad.bb does not
                        // identify them -- getRouteBoundingBox() adds a search
                        // margin, so even X79Y221 to X79Y221 comes back as a
                        // box with area -- but PerWireData carries the tile
                        // each wire is in, which answers it exactly.
                        const auto &swd = wire_data(nd.src_wire);
                        const auto &dwd = wire_data(sink);
                        bool arc_stays_in_one_tile = swd.x == dwd.x && swd.y == dwd.y;
                        if (arc_stays_in_one_tile)
                            continue;
                        ripup_arc(net, usr.index, j);
                    }
                }

                bool ok;
                try {
                    ok = route_net(st, net, false);
                } catch (log_execution_error_exception &) {
                    // An arc the search cannot rebuild is reported with
                    // log_error(), which throws rather than returning.  Nothing
                    // else hangs off that throw -- log_error_atexit is never
                    // installed -- so the ERROR line it printed is the only
                    // trace, and the move is simply refused.  The search's
                    // visit marks are cleared before st is used again.
                    reset_wires(st);
                    ok = false;
                }
                if (!ok) {
                    restore_net(net, snap);
                    ++unroutable;
                    continue;
                }
                bool reroute_made_hot_tiles_worse = net_hot_pips(nd) > hot_before;
                if (reroute_made_hot_tiles_worse) {
                    restore_net(net, snap);
                    ++rejected;
                    continue;
                }
                ++rerouted;
            }
            smoothing_active = false;

            // Legality is not negotiable after the fact: if the re-routes left
            // any overuse, fall back to the ordinary cost and route it out.
            update_congestion();
            int recover = 0;
            while (overused_wires > 0 && recover++ < 8) {
                route_queue.clear();
                for (auto &n : nets_by_udata)
                    if (n != nullptr)
                        route_queue.push_back(n->udata);
                for (auto n : route_queue)
                    route_net(st, nets_by_udata.at(n), false);
                update_congestion();
            }

            build_tile_occupancy();
            auto [m2, p2, x2, t2] = occupancy_stats(cfg.smooth_percentile);
            log_info("    round %d: kept %d nets (%d rejected, %d unroutable), mean %.1f, p%.0f %d, max %d, "
                     "%d pips%s\n",
                     round, rerouted, rejected, unroutable, m2, cfg.smooth_percentile * 100, p2, x2, int(t2),
                     overused_wires ? " (OVERUSE REMAINS)" : "");
            bool percentile_did_not_improve = p2 >= p95;
            if (percentile_did_not_improve) {
                if (++stagnant > cfg.smooth_stagnant)
                    break; // no longer improving
            } else {
                stagnant = 0;
            }
        }
        smooth_target = 0;

        // Put the Arch-level binding back.  bind_and_check() rips up any arc it
        // cannot bind and records the net as failed, so a refusal here leaves
        // the routing incomplete with no loop left to repair it -- revert the
        // whole pass instead.
        failed_nets.clear();
        bool arch_refused_binding = !bind_and_check_all() || !failed_nets.empty();
        if (arch_refused_binding) {
            log_info("    smoothing: the arch would not bind the smoothed result, reverting\n");
            restore_all(all_snap);
            failed_nets.clear();
            arch_fail = arch_fail_before;
            unbind_arch_routing();
            if (!bind_and_check_all())
                log_error("Internal error: the pre-smoothing routing no longer binds.\n");
        }
    }

    void operator()()
    {
        log_info("Running router2...\n");
        log_info("Setting up routing resources...\n");
        auto rstart = std::chrono::high_resolution_clock::now();
        setup_resources();
        setup_nets();
        setup_wires();
        find_all_reserved_wires();
        partition_nets();
        curr_cong_weight = cfg.init_curr_cong_weight;
        hist_cong_weight = cfg.hist_cong_weight;
        ThreadContext st;
        int iter = 1;

        std::unique_lock<Context> lock{*ctx};

        for (size_t i = 0; i < nets_by_udata.size(); i++)
            route_queue.push_back(i);

        timing_driven = ctx->setting<bool>("timing_driven");
        if (ctx->settings.count(ctx->id("router/tmg_ripup")))
            timing_driven_ripup = timing_driven && ctx->setting<bool>("router/tmg_ripup");
        else
            timing_driven_ripup = false;
        log_info("Running main router loop...\n");
        if (timing_driven)
            tmg.run(true);
        do {
            ctx->sorted_shuffle(route_queue);

            if (timing_driven && int(route_queue.size()) >= 30) {
                for (auto n : route_queue) {
                    NetInfo *ni = nets_by_udata.at(n);
                    auto &net = nets.at(n);
                    net.max_crit = 0;
                    net.worst_slack = std::numeric_limits<delay_t>::max();
                    for (auto &usr : ni->users) {
                        CellPortKey key(usr);
                        net.max_crit = std::max(net.max_crit, tmg.get_criticality(key));
                        net.worst_slack = std::min(net.worst_slack, delay_t(tmg.get_setup_slack(key)));
                    }
                    if (cfg.bb_budget)
                        apply_budget_bb(net);
                }
                if (cfg.slack_order) {
                    // Worst absolute slack first, rather than highest
                    // criticality.  Criticality is normalised within a clock
                    // domain, so in a design with several -- 100 MHz sys_clk,
                    // 125 MHz ethernet, 200 MHz idelay, 12 MHz SGMII -- a net
                    // at 0.9 in the slowest domain outranks one at 0.85 in the
                    // domain that is actually failing, and gets first pick of
                    // the routing resource it does not need.  Slack in
                    // picoseconds is comparable across domains; criticality is
                    // not.
                    //
                    // This ordering is only as good as the delay estimate
                    // behind it, and on the first iteration nothing is routed
                    // yet, so every number here comes from predictDelay().
                    // That is why this is worth doing now: see
                    // docs/measured-delay-matrix.md.
                    std::stable_sort(route_queue.begin(), route_queue.end(), [&](int na, int nb) {
                        return nets.at(na).worst_slack < nets.at(nb).worst_slack;
                    });
                } else {
                    std::stable_sort(route_queue.begin(), route_queue.end(),
                                     [&](int na, int nb) { return nets.at(na).max_crit > nets.at(nb).max_crit; });
                }
            }

            do_route();
            update_route_delays();
            route_queue.clear();
            update_congestion();

            if (!cfg.heatmap.empty()) {
                {
                    std::string filename(cfg.heatmap + "_congestion_by_wiretype_" + std::to_string(iter) + ".csv");
                    auto cong_map = open_ofstream_and_log_error(filename, "congestion-by-wiretype heatmap");
                    write_congestion_by_wiretype_heatmap(cong_map);
                    log_info("        wrote congestion-by-wiretype heatmap to %s.\n", filename.c_str());
                }
                {
                    std::string filename(cfg.heatmap + "_utilisation_by_wiretype_" + std::to_string(iter) + ".csv");
                    auto cong_map = open_ofstream_and_log_error(filename, "utilisation-by-wiretype heatmap");
                    write_utilisation_by_wiretype_heatmap(cong_map);
                    log_info("        wrote utilisation-by-wiretype heatmap to %s.\n", filename.c_str());
                }
                {
                    std::string filename(cfg.heatmap + "_congestion_by_coordinate_" + std::to_string(iter) + ".csv");
                    auto cong_map = open_ofstream_and_log_error(filename, "congestion-by-coordinate heatmap");
                    write_congestion_by_coordinate_heatmap(cong_map);
                    log_info("        wrote congestion-by-coordinate heatmap to %s.\n", filename.c_str());
                }
                {
                    std::string filename(cfg.heatmap + "_congestion_by_net_" + std::to_string(iter) + ".csv");
                    auto cong_map = open_ofstream_and_log_error(filename, "congestion-by-net heatmap");
                    write_congestion_by_net_heatmap(cong_map);
                    log_info("        wrote congestion-by-net heatmap to %s.\n", filename.c_str());
                }
            }
            int tmgfail = 0;
            if (timing_driven)
                tmg.run(false);
            if (timing_driven_ripup && iter < 1500) {
                for (size_t i = 0; i < nets_by_udata.size(); i++) {
                    NetInfo *ni = nets_by_udata.at(i);
                    for (auto usr : ni->users.enumerate()) {
                        if (arc_failed_slack(ni, usr.index)) {
                            failed_nets.insert(i);
                            ++tmgfail;
                        }
                    }
                }
            }
            if (overused_wires == 0 && overused_resources == 0 && tmgfail == 0) {
                // Try and actually bind nextpnr Arch API wires
                bind_and_check_all();
            }
            for (auto cn : failed_nets)
                route_queue.push_back(cn);
            std::string resource_str = total_resource_use == 0
                                               ? ""
                                               : stringf("resources=%d overused=%d overuse=%d ", total_resource_use,
                                                         overused_resources, total_resource_overuse);
            if (timing_driven_ripup)
                log_info("    iter=%d wires=%d overused=%d overuse=%d %stmgfail=%d "
                         "archfail=%s\n",
                         iter, total_wire_use, overused_wires, total_wire_overuse, resource_str.c_str(), tmgfail,
                         (overused_wires > 0 || tmgfail > 0) ? "NA" : std::to_string(arch_fail).c_str());
            else
                log_info("    iter=%d wires=%d overused=%d overuse=%d %sarchfail=%s\n", iter, total_wire_use,
                         overused_wires, total_wire_overuse, resource_str.c_str(),
                         (overused_wires > 0 || tmgfail > 0) ? "NA" : std::to_string(arch_fail).c_str());
            ++iter;
            if (curr_cong_weight < 1e9)
                curr_cong_weight += cfg.curr_cong_mult;
        } while (!failed_nets.empty());

        // Routing is legal here.  Only now is it meaningful to redistribute:
        // before this point the router is still negotiating overuse, and
        // pulling nets out of dense tiles would fight it.
        smooth_congestion(st);

        if (cfg.perf_profile) {
            std::vector<std::pair<int, IdString>> nets_by_runtime;
            for (auto &n : nets_by_udata) {
                nets_by_runtime.emplace_back(nets.at(n->udata).total_route_us, n->name);
            }
            std::sort(nets_by_runtime.begin(), nets_by_runtime.end(), std::greater<std::pair<int, IdString>>());
            log_info("1000 slowest nets by runtime:\n");
            for (int i = 0; i < std::min(int(nets_by_runtime.size()), 1000); i++) {
                log("        %80s %6d %.1fms\n", nets_by_runtime.at(i).second.c_str(ctx),
                    int(ctx->nets.at(nets_by_runtime.at(i).second)->users.entries()),
                    nets_by_runtime.at(i).first / 1000.0);
            }
        }

        // Check what we did was actually legal
        ctx->check();
        ctx->checkRoutedDesign();

        auto rend = std::chrono::high_resolution_clock::now();

        log_info("Checksum: 0x%08x\n", ctx->checksum());
        log_info("Router2 time %.02fs\n", std::chrono::duration<float>(rend - rstart).count());

        timing_analysis(ctx, true /* slack_histogram */, true /* print_fmax */, true /* print_path */,
                        true /* warn_on_failure */, true /* update_results */);

        lock.unlock();
    }
};
} // namespace

void router2(Context *ctx, const Router2Cfg &cfg)
{
    Router2 rt(ctx, cfg);
    rt.ctx = ctx;
    rt();
}

Router2Cfg::Router2Cfg(Context *ctx)
{
    backwards_max_iter = ctx->setting<int>("router2/bwdMaxIter", 20);
    global_backwards_max_iter = ctx->setting<int>("router2/glbBwdMaxIter", 200);
    bb_margin_x = ctx->setting<int>("router2/bbMargin/x", 3);
    bb_margin_y = ctx->setting<int>("router2/bbMargin/y", 3);
    // Cap on how many times a congested net's bounding box may grow, and with
    // it a reset of the box once the net routes clean.  0 keeps the unbounded
    // behaviour.  See the expansion in update_congestion().
    bb_expand_max = ctx->setting<int>("router2/bbExpandMax", 0);
    // Size boxes from criticality rather than growing them on congestion.  The
    // loosest margin (a fully-slack net) defaults to 60 tiles: room to detour
    // right around a congested region without the whole-device search cost that
    // sizing every slack net's box at the fabric edge would incur.
    bb_budget = ctx->setting<bool>("router2/bbBudget", false);
    bb_budget_max = ctx->setting<int>("router2/bbBudgetMax", 60);
    ipin_cost_adder = ctx->setting<float>("router2/ipinCostAdder", 0.0f);
    bias_cost_factor = ctx->setting<float>("router2/biasCostFactor", 0.25f);
    if (ctx->settings.count(ctx->id("router2/alt-weights"))) {
        init_curr_cong_weight = ctx->setting<float>("router2/initCurrCongWeight", 5.0f);
        hist_cong_weight = ctx->setting<float>("router2/histCongWeight", 0.5f);
        curr_cong_mult = ctx->setting<float>("router2/currCongWeightMult", 0.0f);
        estimate_weight = ctx->setting<float>("router2/estimateWeight", 1.0f);
    } else {
        init_curr_cong_weight = ctx->setting<float>("router2/initCurrCongWeight", 0.5f);
        hist_cong_weight = ctx->setting<float>("router2/histCongWeight", 1.0f);
        curr_cong_mult = ctx->setting<float>("router2/currCongWeightMult", 2.0f);
        estimate_weight = ctx->setting<float>("router2/estimateWeight", 1.25f);
    }
    {
        // Order the route queue by worst absolute slack instead of by
        // criticality.  See the comment at the sort site.
        slack_order = ctx->setting<bool>("router2/slackOrder", false);
    }
    // Smoothing is opt-in: --router2-smooth-iters N, or router2/smoothIters.
    smooth_iters = ctx->setting<int>("router2/smoothIters", 0);
    smooth_weight = ctx->setting<float>("router2/smoothWeight", 1.0f);
    // Which tiles count as hot.  0.95 targets the tail rather than the mean:
    // peak occupancy is what lengthens the paths that set fmax.
    smooth_percentile = ctx->setting<float>("router2/smoothPercentile", 0.95f);
    // Nets at or above this criticality are left alone.  They are already
    // where the timing-driven router put them, and moving them is how a
    // smoothing pass makes timing worse.
    smooth_max_crit = ctx->setting<float>("router2/smoothMaxCrit", 0.8f);
    // Fanout cap, off by default.  It used to be 32, on the theory that a
    // high-fanout net is a clock or a reset on dedicated resources.  That is
    // wrong for this flow: the only global buffer meaningfully supported is
    // CLKG, which is on dedicated routing and so does not appear in the tile
    // occupancy at all, and the driver-bel test below catches it directly.
    // Everything else with a large fanout is an ordinary signal on ordinary
    // interconnect -- and it is what the crowded tiles are made of.  On the
    // vc707 LiteX design the busiest tile holds 531 pips, of which a cap of 64
    // put 412 out of reach and left only 79 eligible; that is why the mean came
    // down and the peak never moved.  Uncapped, the same tile offers 491, and
    // the peak moves for the first time.
    smooth_max_fanout = ctx->setting<int>("router2/smoothMaxFanout", std::numeric_limits<int>::max());
    // How much of the candidate list to spend per round.  A quarter keeps a
    // round cheap; 1.0 takes every candidate, which is what "aggressive" means
    // here -- the selection, not the cost weight, is what bounds this pass.
    smooth_cap_frac = ctx->setting<float>("router2/smoothCapFrac", 0.25f);
    // Nets held in fewer wires than this are skipped as not worth a search.
    // The per-arc intra-tile test is what actually keeps unroutable hops out,
    // so this is a cost filter, not a safety one, and can go down to 2.
    smooth_min_wires = ctx->setting<int>("router2/smoothMinWires", 8);
    // How many rounds that fail to lower the percentile to tolerate before
    // giving up.  Redistribution can need a round that goes sideways before it
    // finds the one that helps.
    smooth_stagnant = ctx->setting<int>("router2/smoothStagnant", 0);
    perf_profile = ctx->setting<bool>("router2/perfProfile", false);
    if (ctx->settings.count(ctx->id("router2/heatmap")))
        heatmap = ctx->settings.at(ctx->id("router2/heatmap")).as_string();
    else
        heatmap = "";
}

NEXTPNR_NAMESPACE_END
