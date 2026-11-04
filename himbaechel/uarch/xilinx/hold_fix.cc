/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Hold-time fixing by feedthrough-LUT insertion (xc7).
 *
 *  A measured interconnect delay model lets the placer pack timing paths
 *  tighter (better setup/fmax), but nothing in the flow lengthens a path that
 *  is now TOO SHORT: a flop-to-flop or flop-to-BRAM-data arc whose min delay
 *  falls below the sink's hold requirement.  The timing engine already reports
 *  these (`ctx->timing_result.min_delay_violations`); on a flow that does not
 *  pass --timing-allow-fail they are fatal.  Vivado's phys_opt and OpenROAD's
 *  rsz repair_hold both fix this the same way: insert delay on the short path.
 *
 *  Here that delay is a feedthrough LUT -- a 6-LUT wired as an identity buffer
 *  (O6 = A1) spliced between the driver and the one violating sink.  It adds
 *  the LUT's own delay plus the routing to and from it; a sink that still
 *  violates after one buffer gets another on the next pass (the buffers chain,
 *  because each pass re-runs timing and re-targets the net now feeding the
 *  sink).  Only the violating sink is rerouted through the buffer; the net's
 *  other sinks are untouched.
 *
 *  The pass runs AFTER routing (in postRoute, before FASM): route -> analyse
 *  hold -> insert+place buffers -> reroute (router2 keeps already-routed arcs
 *  and routes only the new ones) -> re-analyse, up to a bounded number of
 *  passes.  Opt-in via -o hold-fix (an optional value sets the maximum number
 *  of passes, 8 by default).
 */

#include <queue>

#include "log.h"
#include "nextpnr.h"
#include "router2.h"
#include "timing.h"
#include "util.h"

#include "extra_data.h"
#include "himbaechel_helpers.h"
#include "xilinx.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
constexpr int DEFAULT_MAX_PASSES = 8;
// How far out (in tiles) to look for a free LUT bel to host a buffer.
constexpr int PLACE_SEARCH_RADIUS = 12;
// Hold deficits up to this are fixed by a routing detour (no cell); larger ones
// by a feedthrough LUT.  A detour adds pure interconnect delay, so it is the
// cheaper fix when a few extra hops suffice; beyond that a buffer is more
// compact than a long, congestion-prone scenic route.  Override with
// --xilinx-hold-detour-max <ns>.
constexpr double DEFAULT_DETOUR_MAX_NS = 0.5;
// Tiles of slack around the branch..sink box the detour search may roam.
constexpr int DETOUR_BBOX_MARGIN = 8;
} // namespace

// Centre tile of a wire (mirrors router2's setup_wires).
static void wire_xy(Context *ctx, WireId w, int &x, int &y)
{
    BoundingBox bb = ctx->getRouteBoundingBox(w, w);
    x = (bb.x0 + bb.x1) / 2;
    y = (bb.y0 + bb.y1) / 2;
}

// Lengthen the routing to ONE sink of `net` by at least `extra_delay`, adding no
// cell: rip that sink's private branch and re-bind it along a more circuitous
// path.  Best-effort and side-effect-free on failure -- if it cannot reach the
// sink with enough delay it restores the original routing exactly and returns
// false, so the caller can fall back to a feedthrough buffer.
static bool detour_arc(Context *ctx, NetInfo *net, WireId sink_wire, delay_t extra_delay)
{
    bool sink_is_routed = sink_wire != WireId() && net->wires.count(sink_wire);
    if (!sink_is_routed)
        return false;

    // Fan-out count of each wire in the net's routing tree.
    dict<WireId, int> child_count;
    for (auto &wp : net->wires) {
        PipId pip = wp.second.pip;
        if (pip != PipId())
            child_count[ctx->getPipSrcWire(pip)]++;
    }

    // Walk sink -> src to the branch point (a wire with >1 child, or the src),
    // saving this sink's private wires and their driving pips for rollback.
    std::vector<WireId> priv;
    std::vector<PipId> priv_pips;
    WireId cur = sink_wire, branch = WireId();
    delay_t branch_delay = 0;
    for (int guard = 0; guard < 100000; guard++) {
        auto it = net->wires.find(cur);
        bool reached_net_source = it == net->wires.end() || it->second.pip == PipId();
        if (reached_net_source) {
            branch = cur;
            break;
        }
        PipId pip = it->second.pip;
        priv.push_back(cur);
        priv_pips.push_back(pip);
        branch_delay += ctx->getPipDelay(pip).maxDelay();
        WireId parent = ctx->getPipSrcWire(pip);
        bool parent_is_branch_point = child_count.at(parent) > 1;
        if (parent_is_branch_point) {
            branch = parent;
            break;
        }
        cur = parent;
    }
    bool found_private_branch = branch != WireId() && !priv.empty();
    if (!found_private_branch)
        return false;
    delay_t target = branch_delay + extra_delay;

    // Search box: the branch..sink bounding box, expanded by a margin.
    int bx, by, sx, sy;
    wire_xy(ctx, branch, bx, by);
    wire_xy(ctx, sink_wire, sx, sy);
    int x0 = std::min(bx, sx) - DETOUR_BBOX_MARGIN, x1 = std::max(bx, sx) + DETOUR_BBOX_MARGIN;
    int y0 = std::min(by, sy) - DETOUR_BBOX_MARGIN, y1 = std::max(by, sy) + DETOUR_BBOX_MARGIN;
    auto in_box = [&](WireId w) {
        int x, y;
        wire_xy(ctx, w, x, y);
        return x >= x0 && x <= x1 && y >= y0 && y <= y1;
    };
    auto usable = [&](WireId w) {
        NetInfo *b = ctx->getBoundWireNet(w);
        return b == nullptr || b == net; // free, or already ours (the branch/tree)
    };

    // Dijkstra backwards from the sink over uphill pips: distT[w] = cheapest
    // delay from w down to the sink; next_pip[w] = the downhill pip to take.
    dict<WireId, delay_t> distT;
    dict<WireId, PipId> next_pip;
    struct QE
    {
        delay_t d;
        WireId w;
    };
    struct QCmp
    {
        bool operator()(const QE &a, const QE &b) const { return a.d > b.d; }
    };
    std::priority_queue<QE, std::vector<QE>, QCmp> pq;
    distT[sink_wire] = 0;
    pq.push({0, sink_wire});
    while (!pq.empty()) {
        QE e = pq.top();
        pq.pop();
        bool stale_queue_entry = e.d > distT.at(e.w);
        if (stale_queue_entry)
            continue;
        for (PipId up : ctx->getPipsUphill(e.w)) {
            WireId s = ctx->getPipSrcWire(up);
            // The pip must be available too, not just the wire: a LUT
            // route-through pip is only usable while its LUT bel is free,
            // and a detour through an occupied LUT writes a second INIT
            // over the cell's (the FASM then carries both).
            bool candidate_wire = usable(s) && in_box(s) && ctx->checkPipAvailForNet(up, net);
            if (!candidate_wire)
                continue;
            delay_t nd = e.d + ctx->getPipDelay(up).maxDelay();
            auto it = distT.find(s);
            bool shorter_path_to_sink = it == distT.end() || nd < it->second;
            if (shorter_path_to_sink) {
                distT[s] = nd;
                next_pip[s] = up; // downhill pip s -> e.w
                pq.push({nd, s});
            }
        }
    }
    if (!distT.count(branch))
        return false; // sink unreachable from branch within the box

    // Rip this sink's private branch (leaving the shared trunk intact).
    for (WireId w : priv)
        if (net->wires.count(w))
            ctx->unbindWire(w);

    auto rollback = [&]() {
        // Re-bind the saved private branch exactly as it was.
        for (int i = int(priv.size()) - 1; i >= 0; i--) {
            WireId w = priv[i];
            if (ctx->getBoundWireNet(w) == nullptr)
                ctx->bindPip(priv_pips[i], net, STRENGTH_STRONG);
        }
    };

    // Greedily wander downhill from the branch to accumulate delay, always
    // keeping the sink reachable (distT finite), then follow the shortest tail.
    std::vector<PipId> path;
    pool<WireId> used;
    used.insert(branch);
    cur = branch;
    delay_t acc = 0;
    for (int guard = 0; guard < 100000 && cur != sink_wire; guard++) {
        bool enough_delay_banked = acc + distT.at(cur) >= target;
        if (enough_delay_banked)
            break; // take the shortest tail below
        PipId best;
        WireId best_w;
        delay_t best_score = std::numeric_limits<delay_t>::min();
        for (PipId dh : ctx->getPipsDownhill(cur)) {
            WireId n = ctx->getPipDstWire(dh);
            bool can_extend_path = !used.count(n) && distT.count(n) && usable(n) && ctx->checkPipAvailForNet(dh, net);
            if (!can_extend_path)
                continue;
            delay_t total = acc + ctx->getPipDelay(dh).maxDelay() + distT.at(n);
            // Prefer the largest total not exceeding target; penalise overshoot.
            delay_t score = (total <= target) ? total : (2 * target - total);
            if (score > best_score) {
                best_score = score;
                best = dh;
                best_w = n;
            }
        }
        if (best == PipId())
            break; // dead end -- fall through to the shortest tail
        path.push_back(best);
        acc += ctx->getPipDelay(best).maxDelay();
        used.insert(best_w);
        cur = best_w;
    }
    // Shortest tail from cur to the sink.
    for (int guard = 0; guard < 100000 && cur != sink_wire; guard++) {
        auto it = next_pip.find(cur);
        if (it == next_pip.end())
            break;
        path.push_back(it->second);
        acc += ctx->getPipDelay(it->second).maxDelay();
        cur = ctx->getPipDstWire(it->second);
    }
    bool detour_reached_sink = cur == sink_wire;
    if (!detour_reached_sink) {
        rollback();
        return false;
    }

    // Bind the detour.  Any collision (a wire taken since Dijkstra) aborts and
    // restores the original branch.
    std::vector<PipId> bound;
    for (PipId p : path) {
        WireId d = ctx->getPipDstWire(p);
        bool wire_taken_since_search = ctx->getBoundWireNet(d) != nullptr || !ctx->checkPipAvailForNet(p, net);
        if (wire_taken_since_search) {
            for (int i = int(bound.size()) - 1; i >= 0; i--)
                ctx->unbindPip(bound[i]);
            rollback();
            return false;
        }
        ctx->bindPip(p, net, STRENGTH_LOCKED);
        bound.push_back(p);
    }
    return acc >= target;
}

// Try to place `buf` at a free SLICE_LUTX 6-LUT bel, searching outward from
// `origin`.  Binds and validates each candidate; a lone combinational LUT is
// legal in any otherwise-free 6-LUT bel, so this normally succeeds on the
// first ring.  Returns the chosen bel, or BelId() if none validated.
static BelId place_hold_buffer(XilinxImpl *impl, Context *ctx, CellInfo *buf, Loc origin)
{
    auto try_tile = [&](int x, int y) -> BelId {
        bool tile_on_grid = x >= 0 && y >= 0 && x < ctx->getGridDimX() && y < ctx->getGridDimY();
        if (!tile_on_grid)
            return BelId();
        for (BelId bel : ctx->getBelsByTile(x, y)) {
            if (ctx->getBelType(bel) != id_SLICE_LUTX)
                continue;
            Loc l = ctx->getBelLocation(bel);
            bool is_6lut_bel = (l.z & 0xF) == BEL_6LUT;
            if (!is_6lut_bel)
                continue;
            if (!ctx->checkBelAvail(bel))
                continue;
            // Require the whole tile's slice bels free before hosting a buffer.
            // A lone LUT shares physical input pins with its sibling 5-LUT
            // (A1-A6) and slice control wires (SRUSEDMUX etc.) with the rest of
            // the slice; a neighbouring cell -- or a second hold buffer -- with
            // different nets there collides at route setup ("attempting to
            // reserve sink input path wire ... for nets X and Y").  Demanding an
            // empty tile also stops two buffers landing in one tile, since the
            // first bind makes the tile non-empty for the next search.
            bool tile_free = true;
            for (BelId b2 : ctx->getBelsByTile(x, y)) {
                IdString bt = ctx->getBelType(b2);
                bool is_slice_bel = bt == id_SLICE_LUTX || bt == id_SLICE_FFX;
                bool slice_bel_occupied = is_slice_bel && !ctx->checkBelAvail(b2);
                if (slice_bel_occupied) {
                    tile_free = false;
                    break;
                }
            }
            if (!tile_free)
                continue;
            ctx->bindBel(bel, buf, STRENGTH_STRONG);
            if (impl->isBelLocationValid(bel))
                return bel;
            ctx->unbindBel(bel);
        }
        return BelId();
    };

    for (int r = 0; r <= PLACE_SEARCH_RADIUS; r++) {
        if (r == 0) {
            BelId b = try_tile(origin.x, origin.y);
            if (b != BelId())
                return b;
            continue;
        }
        for (int dx = -r; dx <= r; dx++) {
            int dy = r - std::abs(dx);
            BelId b = try_tile(origin.x + dx, origin.y + dy);
            if (b != BelId())
                return b;
            if (dy != 0) {
                b = try_tile(origin.x + dx, origin.y - dy);
                if (b != BelId())
                    return b;
            }
        }
    }
    return BelId();
}

void XilinxImpl::fixup_hold()
{
    const ArchArgs &args = ctx->args;
    if (!args.options.count("hold-fix"))
        return;
    int max_passes = DEFAULT_MAX_PASSES;
    {
        std::string v = args.options["hold-fix"].as<std::string>();
        if (!v.empty()) {
            try {
                max_passes = std::stoi(v);
            } catch (...) {
                max_passes = DEFAULT_MAX_PASSES;
            }
        }
    }

    double detour_max_ns = DEFAULT_DETOUR_MAX_NS;
    if (args.options.count("hold-detour-max"))
        detour_max_ns = args.options["hold-detour-max"].as<double>();
    delay_t detour_max = ctx->getDelayFromNS(detour_max_ns);
    delay_t detour_margin = ctx->getDelayFromNS(0.02); // clear the deficit, don't just meet it

    if (getenv("HOLDFIX_PROBE")) {
        // Does device wire iteration itself crash here, before any 2nd router2?
        // If so the corruption predates this pass entirely.
        long n = 0;
        for (auto w : ctx->getWires()) {
            (void)w;
            n++;
        }
        log_info("Hold-fix PROBE: iterated %ld wires OK.\n", n);
        return;
    }

    Router2Cfg rcfg(ctx);
    configureRouter2(rcfg);

    // Bisection aid: reroute the already-routed design without inserting
    // anything, to separate a router2 re-entrancy problem from an insertion bug.
    if (getenv("HOLDFIX_REROUTE_ONLY")) {
        log_info("Hold-fix: HOLDFIX_REROUTE_ONLY set -- rerouting with no insertion.\n");
        router2(ctx, rcfg);
        timing_analysis(ctx, true, true, false, true, true);
        return;
    }

    // createNet also records the name in net_aliases; erasing only the net would leave the alias behind, and the next buffer
    // given the same name (the count only moves on for the buffers that stay) would trip createNet's assertion.
    auto erase_net = [this](NetInfo *net) {
        const IdString name = net->name;
        ctx->nets.erase(name);
        ctx->net_aliases.erase(name);
    };

    int total_buffers = 0, total_detours = 0;
    for (int pass = 0; pass < max_passes; pass++) {
        const auto &violations = ctx->timing_result.min_delay_violations;
        if (violations.empty()) {
            if (pass == 0)
                log_info("Hold-fix: no hold violations to fix.\n");
            break;
        }

        // Collect the unique violating sinks this pass, with the extra delay
        // each needs.  For a min-delay violation the sum of the path segment
        // delays IS the (negative) hold slack, so the deficit is its magnitude.
        struct Target
        {
            IdString net, sink_cell, sink_port;
            delay_t extra;
        };
        std::vector<Target> targets;
        pool<std::string> seen;
        for (const auto &cp : violations) {
            const CriticalPath::Segment *last_routing = nullptr;
            delay_t sum = 0;
            for (const auto &seg : cp.segments) {
                sum += seg.delay;
                if (seg.type == CriticalPath::Segment::Type::ROUTING)
                    last_routing = &seg;
            }
            if (!last_routing)
                continue;
            std::string key = last_routing->to.first.str(ctx) + "/" + last_routing->to.second.str(ctx);
            if (seen.count(key))
                continue;
            seen.insert(key);
            delay_t extra = (sum < 0 ? -sum : delay_t(0)) + detour_margin;
            targets.push_back({last_routing->net, last_routing->to.first, last_routing->to.second, extra});
        }
        if (targets.empty())
            break;

        // 1. For small deficits, lengthen the routing with a detour (no cell).
        //    Anything above the threshold -- and any detour that could not reach
        //    its target -- falls through to a feedthrough LUT.
        std::vector<Target> ft;
        int detoured = 0;
        for (const auto &t : targets) {
            bool target_still_exists = ctx->nets.count(t.net) && ctx->cells.count(t.sink_cell);
            if (!target_still_exists)
                continue;
            NetInfo *net = ctx->nets.at(t.net).get();
            CellInfo *sink = ctx->cells.at(t.sink_cell).get();
            bool driven_and_placed = net->driver.cell != nullptr && sink->bel != BelId();
            if (!driven_and_placed)
                continue;
            bool stale_report = sink->getPort(t.sink_port) != net; // report vs current netlist
            if (stale_report)
                continue;
            bool small_enough_to_detour = t.extra <= detour_max;
            if (small_enough_to_detour) {
                PortRef pr;
                pr.cell = sink;
                pr.port = t.sink_port;
                WireId sink_wire = ctx->getNetinfoSinkWire(net, pr, 0);
                if (detour_arc(ctx, net, sink_wire, t.extra)) {
                    detoured++;
                    continue;
                }
            }
            ft.push_back(t);
        }

        // 2. Create identity-buffer cells and splice them onto the remaining
        //    violating sink arcs.  No bel binding yet -- tags must be
        //    (re)assigned first.
        struct Pending
        {
            CellInfo *buf;
            IdString sink_cell;
            Loc origin;
        };
        std::vector<Pending> pending;
        pool<IdString> touched_nets;
        int no_exit = 0;
        for (const auto &t : ft) {
            if (!ctx->nets.count(t.net))
                continue;
            NetInfo *net = ctx->nets.at(t.net).get();
            if (net->driver.cell == nullptr)
                continue;
            if (!ctx->cells.count(t.sink_cell))
                continue;
            CellInfo *sink = ctx->cells.at(t.sink_cell).get();
            if (sink->bel == BelId())
                continue;
            bool stale_report = sink->getPort(t.sink_port) != net; // report vs current netlist
            if (stale_report)
                continue;

            NetInfo *buf_out = ctx->createNet(ctx->idf("%s$holdbuf%d$net", net->name.c_str(ctx), total_buffers));
            CellInfo *buf = ctx->createCell(ctx->idf("%s$holdbuf%d", net->name.c_str(ctx), total_buffers), id_SLICE_LUTX);
            buf->addInput(id_A1);
            buf->addOutput(id_O6);
            // Present the cell as a packed LUT1 identity buffer, exactly as the
            // LUT packer would (X_ORIG_TYPE + per-pin X_ORIG_PORT_* attrs), so
            // the FASM writer's get_inputs()/get_lut_init() accept it and expand
            // the 2-bit logical INIT (O6 = A1) to the physical 64.  Without
            // these attrs the writer asserts "unsupported LUT-type cell".
            buf->params[id_INIT] = Property(2, 2); // LUT1 truth table: O = I0
            buf->attrs[id_X_ORIG_TYPE] = std::string("LUT1");
            buf->attrs[ctx->id("X_ORIG_PORT_A1")] = std::string("I0");
            buf->attrs[ctx->id("X_ORIG_PORT_O6")] = std::string("O");
            buf->connectPort(id_A1, net);       // buffer input  = original net
            buf->connectPort(id_O6, buf_out);   // buffer output = new net

            // Move just this sink from the original net onto the buffer output.
            sink->disconnectPort(t.sink_port);
            sink->connectPort(t.sink_port, buf_out);

            // The buffer is a fabric consumer of the source, which an O5, a
            // carry sum or a 5FF Q can only reach through its position's one
            // output mux.  If the source's slice has no exit left (a 5FF whose
            // Q already uses it), the buffer would make an unroutable site:
            // leave this arc alone rather than hand it to the router.
            const bool source_is_placed = net->driver.cell->bel != BelId();
            const bool source_slice_has_no_exit_for_a_buffer =
                    source_is_placed && !isBelLocationValid(net->driver.cell->bel);
            if (source_slice_has_no_exit_for_a_buffer) {
                const bool holdfix_verbose = getenv("HOLDFIX_VERBOSE") != nullptr;
                if (holdfix_verbose)
                    log_info("Hold-fix: %s -> %s.%s left: the source's slice has no output mux free for a buffer\n",
                             ctx->nameOf(net), ctx->nameOf(sink), t.sink_port.c_str(ctx));
                sink->disconnectPort(t.sink_port);
                sink->connectPort(t.sink_port, net);
                buf->disconnectPort(id_A1);
                buf->disconnectPort(id_O6);
                ctx->cells.erase(buf->name);
                erase_net(buf_out);
                no_exit++;
                continue;
            }

            pending.push_back({buf, t.sink_cell, ctx->getBelLocation(sink->bel)});
            touched_nets.insert(net->name);
            total_buffers++;
        }
        // 3. Place and reroute the feedthrough buffers (if any remained after
        //    detours).  Detours have already re-bound their routing directly.
        int placed = 0, failed = 0;
        pool<IdString> unbuffered_nets;   // nets whose buffer did not land
        if (!pending.empty()) {
            // Reindex and re-tag so get_tags() (used by bindBel ->
            // notifyBelChange and isBelLocationValid) sees the new cells.
            ctx->assignArchInfo();
            assign_cell_tags();

            // Place each buffer near its sink; on the rare failure, undo the
            // splice so the netlist stays consistent.
            for (auto &p : pending) {
                BelId bel = place_hold_buffer(this, ctx, p.buf, p.origin);
                if (bel != BelId()) {
                    placed++;
                    continue;
                }
                failed++;
                // reconnect the sink to the original input net, drop the buffer
                NetInfo *orig = p.buf->getPort(id_A1);
                NetInfo *buf_out = p.buf->getPort(id_O6);
                CellInfo *sink = ctx->cells.at(p.sink_cell).get();
                IdString sink_port;
                for (auto &pr : sink->ports)
                    if (pr.second.net == buf_out)
                        sink_port = pr.first;
                p.buf->disconnectPort(id_A1);
                p.buf->disconnectPort(id_O6);
                if (sink_port != IdString()) {
                    sink->disconnectPort(sink_port);
                    sink->connectPort(sink_port, orig);
                }
                ctx->cells.erase(p.buf->name);
                if (buf_out)
                    erase_net(buf_out);
                // The net is as it was, so it is not one of the touched: a
                // net whose buffer never landed must not be ripped up.
                const bool the_sink_was_reconnected_to_a_net = (orig != nullptr);
                if (the_sink_was_reconnected_to_a_net)
                    unbuffered_nets.insert(orig->name);
            }
            for (IdString nn : unbuffered_nets) {
                bool still_buffered = false;
                for (auto &q : pending) {
                    const bool buffer_survived =
                            ctx->cells.count(q.buf->name) != 0 && q.buf->getPort(id_A1) != nullptr;
                    const bool buffer_still_feeds_this_net = buffer_survived && q.buf->getPort(id_A1)->name == nn;
                    if (buffer_still_feeds_this_net)
                        still_buffered = true;
                }
                const bool no_buffer_is_left_on_this_net = !still_buffered;
                if (no_buffer_is_left_on_this_net)
                    touched_nets.erase(nn);
            }
            const bool some_buffers_had_no_bel = failed != 0;
            if (some_buffers_had_no_bel)
                log_warning("Hold-fix pass %d: %d buffer(s) had no free LUT bel nearby, skipped.\n", pass, failed);
        }
        const bool some_arcs_were_left_unbuffered = no_exit != 0;
        if (some_arcs_were_left_unbuffered)
            log_warning("Hold-fix pass %d: %d arc(s) left unbuffered, their source's slice has no output mux free.\n",
                        pass, no_exit);
        // Only if a buffer actually landed: with none placed the netlist is
        // exactly as it was, and rerouting would move routing that is
        // already good -- in a replay, routing that was meant to be frozen.
        const bool a_buffer_was_placed = placed > 0;
        const bool some_net_was_touched = !touched_nets.empty();
        const bool a_reroute_is_warranted = a_buffer_was_placed && some_net_was_touched;
        if (a_reroute_is_warranted) {

            // Rip up only the touched source nets (minimal perturbation).  Each
            // buffer's input is a new sink on its source net, so rerouting just
            // those nets picks up the buffer arc while every other net keeps its
            // routing -- what keeps the pass convergent.  A full rip-up instead
            // re-routes the whole design and, because a buffer shifts its source
            // net's routing to its other sinks, churns fresh hold violations and
            // diverges (6 -> 38).  router2 still rips a pre-routed net locally if
            // a buffer arc overuses one of its wires (check_arc_routing's
            // curr_cong test), so congestion resolves without a global rip-up;
            // empty-tile placement keeps the new arcs clear of unresolvable
            // reserved-wire collisions, which is what deadlocked an earlier try.
            // ...plus the nets driven from the same tile as a touched net's
            // source: a touched net's only exit (a LUT5's O5 has one, the
            // slice's output mux) may be held by a neighbour's first-pass
            // route, which the incremental reroute cannot take back.  Locked
            // routes (a replay's reference) stay.
            pool<int> src_tiles;
            for (IdString nn : touched_nets) {
                const bool net_still_exists = ctx->nets.count(nn) != 0;
                const bool source_is_placed = net_still_exists && ctx->nets.at(nn)->driver.cell != nullptr &&
                                              ctx->nets.at(nn)->driver.cell->bel != BelId();
                if (source_is_placed)
                    src_tiles.insert(ctx->nets.at(nn)->driver.cell->bel.tile);
            }
            pool<IdString> ripup(touched_nets.begin(), touched_nets.end());
            for (auto &n : ctx->nets) {
                NetInfo *ni = n.second.get();
                const bool net_is_not_driven_from_a_touched_source_tile =
                        ni->driver.cell == nullptr || ni->driver.cell->bel == BelId() ||
                        src_tiles.count(ni->driver.cell->bel.tile) == 0;
                if (net_is_not_driven_from_a_touched_source_tile)
                    continue;
                bool net_carries_a_locked_route = false;
                for (auto &w : ni->wires) {
                    const bool wire_is_locked = w.second.strength >= STRENGTH_LOCKED;
                    if (wire_is_locked)
                        net_carries_a_locked_route = true;
                }
                const bool net_is_free_to_rip_up = !net_carries_a_locked_route;
                if (net_is_free_to_rip_up)
                    ripup.insert(n.first);
            }
            for (IdString nn : ripup) {
                if (!ctx->nets.count(nn))
                    continue;
                NetInfo *net = ctx->nets.at(nn).get();
                std::vector<WireId> wires;
                wires.reserve(net->wires.size());
                for (auto &w : net->wires)
                    wires.push_back(w.first);
                for (WireId w : wires)
                    ctx->unbindWire(w);
            }

            // Reroute (incremental: already-routed arcs are kept).  Reindex+
            // retag after any undo erasures so get_tags() stays valid.
            ctx->assignArchInfo();
            assign_cell_tags();
            router2(ctx, rcfg);
        }

        // 4. Re-analyse.  Detours changed routing directly; feedthroughs via the
        //    reroute above.
        timing_analysis(ctx, true /*slack_histogram*/, true /*print_fmax*/, false /*print_path*/,
                        true /*warn_on_failure*/, true /*update_results*/);
        total_detours += detoured;
        log_info("Hold-fix pass %d: %d detour(s), %d feedthrough buffer(s); %zu hold violation(s) remain.\n", pass,
                 detoured, placed, ctx->timing_result.min_delay_violations.size());
        bool pass_changed_nothing = detoured == 0 && placed == 0;
        if (pass_changed_nothing)
            break;
    }

    bool anything_was_fixed = total_detours > 0 || total_buffers > 0;
    if (anything_was_fixed) {
        log_info("Hold-fix: %d detour(s) + %d feedthrough buffer(s) total; %zu hold violation(s) remain.\n",
                 total_detours, total_buffers, ctx->timing_result.min_delay_violations.size());

        // The timing analysis in Arch::route ran BEFORE this pass and flagged the
        // hold violations we have now fixed as a nonfatal error -- a sticky flag
        // that fails the run regardless of the fix.  Re-judge the finished design
        // so the exit status reflects the POST-fix state: a clean fix passes,
        // while any residual hold violation, or a genuine setup failure, still
        // fails.  This is what lets hold-fix run without --timing-allow-fail and
        // still surface real timing failures.  The fmax is computed too:
        // update_results replaces ctx->timing_result, which --report reads, and
        // without it the report of a fixed design has an empty fmax.
        had_nonfatal_error = false;
        timing_analysis(ctx, false /*histogram*/, true /*fmax*/, false /*path*/, true /*warn_on_failure*/,
                        true /*update_results*/);
    }
}

NEXTPNR_NAMESPACE_END
