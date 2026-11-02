/*
 *  nextpnr -- Next Generation Place and Route
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
 */

// Block RAM placement by read-mux group.
//
// A deep memory is many block RAMs whose data outputs meet in read
// multiplexers: a 2.5 MB ROM of 640 one-bit-wide RAMB36s is 32 groups of 20,
// one per data bit, each feeding a 20-way mux.  Those mux paths are the
// memory's critical paths, so a group wants to sit together, close to its
// mux, and the groups should crowd around the logic rather than out to the
// edges of the die.  The heap placer legalises block RAMs column by column
// instead: on that ROM each bit's 20 block RAMs ended up spread over the full
// height of the die, edge columns filled first, and the far ones read wrongly
// on the board.  Vivado's placement of the same netlist keeps each group to
// 60% of that span and fills columns outward from the logic; with its block
// RAM sites, and nextpnr placing everything else, the design worked.
//
// So, after the heap placement: group the block RAMs by the cone their data
// outputs drive, take each group's consumers' centroid from the placement,
// regrow all the groups at once from their centroids -- one site per group
// per round, the cheapest free site by estimated delay to the centroid plus
// the clock tree's extra insertion delay there -- lock them, and place
// everything else again around them.  It runs by default on the dies whose
// clock delays clock_tree.cc has measured, since its cost is calibrated
// there; "-o bram-place=force" runs it on any die, "-o bram-place=off" skips
// it.

#include <queue>

#include "log.h"
#include "nextpnr.h"
#include "placer_heap.h"
#include "util.h"
#include "xilinx.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
// How far the cone of a block RAM's data outputs is followed, in cells; the
// fanout beyond which a net is not followed (a reset or an enable, not read
// data); and the size at which a group is complete and joins no other
const int cone_depth = 6;
const int cone_max_fanout = 32;
const int complete_group = 16;
// Fewer block RAMs than this are left where the heap placer put them
const int min_brams = 16;

struct BramGroup
{
    std::vector<CellInfo *> members;
    float tx = 0, ty = 0; // the consumers' centroid
    std::vector<BelId> candidates;
    size_t next = 0;
    size_t placed = 0;
};

int find_root(std::vector<int> &parent, int i)
{
    while (parent[i] != i)
        i = parent[i] = parent[parent[i]];
    return i;
}
} // namespace

bool XilinxImpl::is_movable_bram(const CellInfo *ci) const
{
    const bool is_bram = ci->type.in(id_RAMB36E1_RAMB36E1, id_RAMB18E1_RAMB18E1);
    const bool placed_by_the_placer = ci->bel != BelId() && ci->belStrength <= STRENGTH_STRONG;
    const bool in_a_cascade = ci->cluster != ClusterId();
    return is_bram && placed_by_the_placer && !in_a_cascade;
}

bool XilinxImpl::place_brams_by_group()
{
    std::string mode =
            ctx->args.options.count("bram-place") ? ctx->args.options.at("bram-place").as<std::string>() : "on";
    const bool mode_is_off = mode == "off";
    if (mode_is_off)
        return false;
    // The cost of a site is its estimated delay plus the clock tree's extra
    // insertion delay there, and the second placement pass costs run time and
    // often Fmax; both are measured on the xc7vx485t only.  Elsewhere the skew
    // term would be zero and the pass is untested, so it is left out unless
    // forced, as the clock model is.
    const bool unmeasured_and_not_forced = mode != "force" && !clock_rates_measured();
    if (unmeasured_and_not_forced) {
        log_info("bram-place: no clock delays measured for %s, block RAMs keep the heap placement "
                 "(-o bram-place=force to run it)\n",
                 chipdb_die.c_str());
        return false;
    }

    std::vector<CellInfo *> brams;
    dict<IdString, int> index_of;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool movable = is_movable_bram(ci);
        if (movable) {
            index_of[ci->name] = brams.size();
            brams.push_back(ci);
        }
    }
    const bool too_few_brams = int(brams.size()) < min_brams;
    if (too_few_brams)
        return false;

    // Group: block RAMs whose data-output cones meet.  The cones grow a level
    // at a time for all block RAMs together, so groups join in the order their
    // muxes combine them; two groups that are both complete already (a whole
    // ROM data bit each, say) are not joined by a later, wider level of logic.
    std::vector<int> parent(brams.size()), group_size(brams.size(), 1);
    for (size_t i = 0; i < brams.size(); i++)
        parent[i] = i;
    auto join = [&](int a, int b) {
        a = find_root(parent, a);
        b = find_root(parent, b);
        const bool both_complete = group_size[a] >= complete_group && group_size[b] >= complete_group;
        const bool same_or_both_complete = a == b || both_complete;
        if (same_or_both_complete)
            return;
        parent[a] = b;
        group_size[b] += group_size[a];
    };
    dict<IdString, int> reached_from; // cone cell -> the first block RAM to reach it
    std::vector<std::vector<CellInfo *>> frontier(brams.size()), registers(brams.size()), logic(brams.size());
    auto users_of_outputs = [&](CellInfo *ci, bool data_outputs_only, std::vector<CellInfo *> &out) {
        for (auto &port : ci->ports) {
            const bool not_a_driven_output = port.second.type != PORT_OUT || port.second.net == nullptr;
            if (not_a_driven_output)
                continue;
            const bool is_data_output = port.first.str(ctx).rfind("DO", 0) == 0;
            const bool skip_non_data_output = data_outputs_only && !is_data_output;
            if (skip_non_data_output)
                continue;
            NetInfo *net = port.second.net;
            const bool fanout_too_wide = net->users.entries() > cone_max_fanout;
            if (fanout_too_wide)
                continue;
            for (auto &usr : net->users) {
                const bool user_is_placed = usr.cell != nullptr && usr.cell->bel != BelId();
                if (user_is_placed)
                    out.push_back(usr.cell);
            }
        }
    };
    for (size_t i = 0; i < brams.size(); i++)
        users_of_outputs(brams[i], true, frontier[i]);
    for (int depth = 1; depth <= cone_depth; depth++) {
        for (size_t i = 0; i < brams.size(); i++) {
            std::vector<CellInfo *> next;
            for (auto *ci : frontier[i]) {
                const bool is_block_ram = ci->type.in(id_RAMB36E1_RAMB36E1, id_RAMB18E1_RAMB18E1);
                if (is_block_ram)
                    continue;
                auto other = reached_from.find(ci->name);
                const bool already_reached = other != reached_from.end();
                if (already_reached) {
                    // reached already, by another block RAM (whose cone it now
                    // continues) or by another path from this one
                    join(i, other->second);
                    continue;
                }
                reached_from[ci->name] = i;
                const bool continues_combinationally = ci->type.in(id_SLICE_LUTX, id_SELMUX2_1, id_CARRY4);
                (continues_combinationally ? logic : registers)[i].push_back(ci);
                if (continues_combinationally)
                    users_of_outputs(ci, false, next);
            }
            frontier[i] = std::move(next);
        }
    }
    // A group's anchor is where its cones end: the registers they reach.  The
    // mux logic on the way followed the block RAMs in the first placement, so
    // is only the anchor when no register is reached.
    std::vector<std::vector<CellInfo *>> &consumers = registers;
    for (size_t i = 0; i < brams.size(); i++) {
        const bool no_register_reached = consumers[i].empty();
        if (no_register_reached)
            consumers[i] = logic[i];
    }

    dict<int, int> group_of_root;
    std::vector<BramGroup> groups;
    for (size_t i = 0; i < brams.size(); i++) {
        int root = find_root(parent, i);
        const bool root_is_new = !group_of_root.count(root);
        if (root_is_new) {
            group_of_root[root] = groups.size();
            groups.emplace_back();
        }
        groups.at(group_of_root.at(root)).members.push_back(brams[i]);
    }
    for (auto &g : groups) {
        float sx = 0, sy = 0;
        int n = 0;
        for (auto *ci : g.members) {
            for (auto *c : consumers.at(index_of.at(ci->name))) {
                Loc l = ctx->getBelLocation(c->bel);
                sx += l.x;
                sy += l.y;
                n++;
            }
        }
        // nothing reads it (yet): keep it near where it is
        const bool nothing_reads_it = n == 0;
        if (nothing_reads_it) {
            for (auto *ci : g.members) {
                Loc l = ctx->getBelLocation(ci->bel);
                sx += l.x;
                sy += l.y;
                n++;
            }
        }
        g.tx = sx / n;
        g.ty = sy / n;
    }

    // The cost of a site to a group: estimated delay from the site to the
    // centroid, plus the clock arriving later at the site than at flip-flops
    // by the centroid
    auto site_cost = [&](const BramGroup &g, BelId bel) {
        Loc l = ctx->getBelLocation(bel);
        delay_t route = delay_matrix_lookup(int(g.tx + 0.5f) - l.x, int(g.ty + 0.5f) - l.y);
        const bool route_unknown = route < 0;
        if (route_unknown)
            route = 30 * std::abs(l.x - g.tx) + 20 * std::abs(l.y - g.ty);
        delay_t skew = clock_tree_delay(bel) - clock_tree_delay_at(int(g.tx + 0.5f), int(g.ty + 0.5f), false);
        return route + std::max<delay_t>(0, skew);
    };

    // Lift every movable block RAM, then regrow the groups together
    for (auto *ci : brams)
        ctx->unbindBel(ci->bel);
    dict<IdString, std::vector<BelId>> bels_for_type;
    for (auto *ci : brams) {
        const bool type_already_listed = bels_for_type.count(ci->type);
        if (type_already_listed)
            continue;
        auto &v = bels_for_type[ci->type];
        for (auto bel : ctx->getBels()) {
            const bool bel_matches_type = ctx->isValidBelForCellType(ci->type, bel);
            if (bel_matches_type)
                v.push_back(bel);
        }
    }
    for (auto &g : groups) {
        // a group of mixed types takes the sites of its first member's type,
        // and its other members fall back to the nearest free site of theirs
        g.candidates = bels_for_type.at(g.members.front()->type);
        std::vector<std::pair<delay_t, BelId>> costed;
        for (auto bel : g.candidates)
            costed.emplace_back(site_cost(g, bel), bel);
        std::sort(costed.begin(), costed.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
        g.candidates.clear();
        for (auto &c : costed)
            g.candidates.push_back(c.second);
    }
    std::sort(groups.begin(), groups.end(),
              [](const BramGroup &a, const BramGroup &b) { return a.members.size() > b.members.size(); });

    auto try_bind = [&](CellInfo *ci, BelId bel) {
        const bool site_taken_or_wrong_type = !ctx->checkBelAvail(bel) || !ctx->isValidBelForCellType(ci->type, bel);
        if (site_taken_or_wrong_type)
            return false;
        ctx->bindBel(bel, ci, STRENGTH_USER);
        const bool location_invalid = !ctx->isBelLocationValid(bel);
        if (location_invalid) {
            ctx->unbindBel(bel);
            return false;
        }
        return true;
    };
    bool growing = true;
    while (growing) {
        growing = false;
        for (auto &g : groups) {
            const bool group_is_full = g.placed == g.members.size();
            if (group_is_full)
                continue;
            growing = true;
            CellInfo *ci = g.members.at(g.placed);
            bool bound = false;
            while (!bound && g.next < g.candidates.size())
                bound = try_bind(ci, g.candidates.at(g.next++));
            if (!bound) { // the group's own type ran out of sites: any free one of this cell's type
                for (auto bel : bels_for_type.at(ci->type)) {
                    // the initializer assigns bound from try_bind; the if tests it, one site at a time
                    const bool bound_this_site = (bound = try_bind(ci, bel));
                    if (bound_this_site)
                        break;
                }
            }
            if (!bound)
                log_error("bram-place: no free site for block RAM '%s'\n", ctx->nameOf(ci));
            g.placed++;
        }
    }

    int largest = 0;
    float span_sum = 0;
    for (auto &g : groups) {
        largest = std::max<int>(largest, g.members.size());
        int x0 = INT_MAX, x1 = INT_MIN, y0 = INT_MAX, y1 = INT_MIN;
        for (auto *ci : g.members) {
            Loc l = ctx->getBelLocation(ci->bel);
            x0 = std::min(x0, l.x), x1 = std::max(x1, l.x), y0 = std::min(y0, l.y), y1 = std::max(y1, l.y);
        }
        span_sum += (x1 - x0) + (y1 - y0);
    }
    log_info("bram-place: %d block RAMs in %d read-mux groups (largest %d), mean group span %.1f tiles\n",
             int(brams.size()), int(groups.size()), largest, span_sum / groups.size());
    return true;
}

void XilinxImpl::replace_around_brams()
{
    const bool groups_not_placed = !place_brams_by_group();
    if (groups_not_placed)
        return;
    // Everything else the placer put down is placed again around them
    int lifted = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool placed_by_the_placer = ci->bel != BelId() && ci->belStrength <= STRENGTH_STRONG;
        if (placed_by_the_placer) {
            ctx->unbindBel(ci->bel);
            lifted++;
        }
    }
    log_info("bram-place: placing %d cells again around the block RAMs\n", lifted);
    PlacerHeapCfg cfg(ctx);
    configurePlacerHeap(cfg);
    cfg.ioBufTypes.insert(ctx->id("GENERIC_IOB"));
    const bool second_pass_failed = !placer_heap(ctx, cfg);
    if (second_pass_failed)
        log_error("bram-place: the second placement failed\n");
}

NEXTPNR_NAMESPACE_END
