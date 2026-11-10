/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2024  The Project Trellis Authors.
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

// A measured interconnect delay table, indexed by tile offset.
//
// The hand-tuned formula this supplements is separable and piecewise linear:
//
//     30*min(dx,18) + 10*max(dx-18,0) + 60*min(dy,6) + 20*max(dy-6,0) + 300
//
// Being separable, it charges a diagonal the sum of both legs; being linear, it
// charges two tiles twice what it charges one.  The fabric does neither.
// BENTQUAD and the bent doubles exist precisely so that a diagonal is reached
// in ONE hop, and a SINGLE, a DOUBLE and a QUAD are each one hop whatever
// distance they span -- the routing resource classes are quantised where the
// formula is smooth.  The formula's one structural insight is the knee at
// dx=18 / dy=6, which stands in for the long lines.
//
// Rather than guess a better shape, measure it.  One Dijkstra over the real
// routing graph, using the per-pip delays the chipdb already carries from the
// interchange database, records the cheapest route to every tile in a window
// around a source.  Averaging a few sources smooths out where in the clock
// region the source happens to sit.  This is VPR's place_delay_matrix idea.
//
// The table feeds estimateDelay() (the router's A* guidance) and predictDelay()
// (which reaches the placer as criticality), so a single measurement improves
// placement and routing guidance together.

#include "xilinx.h"

#include <algorithm>
#include <fstream>
#include <queue>

NEXTPNR_NAMESPACE_BEGIN

namespace {
struct QueuedWire
{
    WireId wire;
    delay_t delay;
    bool operator<(const QueuedWire &other) const { return delay > other.delay; } // min-heap
};
} // namespace

// Walk downhill from one source wire, recording the cheapest delay at which a
// logic-tile input pin in each surrounding tile is reached.  Returns the number
// of offsets this source managed to fill.
int XilinxImpl::measure_from(WireId src, int sx, int sy, std::vector<delay_t> &out) const
{
    const int span = dm_window;
    // Expanding a little beyond the window lets a route leave and come back,
    // which is exactly what a bent wire or a long line does.
    const int margin = span + 6;

    dict<WireId, delay_t> best;
    std::priority_queue<QueuedWire> queue;
    queue.push(QueuedWire{src, 0});
    best[src] = 0;

    int filled = 0, popped = 0;
    while (!queue.empty()) {
        auto qw = queue.top();
        queue.pop();
        auto found = best.find(qw.wire);
        bool stale_heap_entry = found == best.end() || found->second < qw.delay;
        if (stale_heap_entry)
            continue;
        if (++popped > dm_max_explore)
            break;

        int wx, wy;
        tile_xy(ctx->chip_info, qw.wire.tile, wx, wy);
        int dx = wx - sx, dy = wy - sy;
        bool outside_search_window = std::abs(dx) > margin || std::abs(dy) > margin;
        if (outside_search_window)
            continue; // don't expand

        // Reaching a logic-tile input pin is what we are timing: the pin
        // access is part of the cost, the same way it is for a real arc.
        bool inside_matrix = std::abs(dx) <= span && std::abs(dy) <= span;
        if (inside_matrix) {
            for (auto bp : ctx->getWireBelPins(qw.wire)) {
                bool is_logic_input_pin = is_logic_tile(bp.bel) && ctx->getBelPinType(bp.bel, bp.pin) == PORT_IN;
                if (!is_logic_input_pin)
                    continue;
                size_t idx = dm_index(dx, dy);
                bool entry_unmeasured = out.at(idx) < 0;
                bool faster_than_recorded = entry_unmeasured || qw.delay < out.at(idx);
                if (faster_than_recorded) {
                    if (entry_unmeasured)
                        ++filled;
                    out.at(idx) = qw.delay;
                }
                break;
            }
        }

        for (auto pip : ctx->getPipsDownhill(qw.wire)) {
            WireId dst = ctx->getPipDstWire(pip);
            // As router2 costs a step: the pip and the node it enters.
            delay_t nd = qw.delay + ctx->getPipDelay(pip).maxDelay() + ctx->getWireDelay(dst).maxDelay();
            auto prev = best.find(dst);
            bool already_reached_as_fast = prev != best.end() && prev->second <= nd;
            if (already_reached_as_fast)
                continue;
            best[dst] = nd;
            queue.push(QueuedWire{dst, nd});
        }
    }
    return filled;
}

// Pick source wires: logic-bel outputs spread around the middle of the device,
// far enough apart that they do not all sit at the same place within a clock
// region, which is what makes the vertical numbers vary.
std::vector<std::pair<WireId, Loc>> XilinxImpl::pick_delay_sources(int count) const
{
    int max_x = 0, max_y = 0;
    for (auto bel : ctx->getBels()) {
        Loc l = ctx->getBelLocation(bel);
        max_x = std::max(max_x, l.x);
        max_y = std::max(max_y, l.y);
    }

    std::vector<std::pair<WireId, Loc>> out;
    // Offsets around the centre, deliberately not multiples of the clock
    // region height so the samples land at different heights within one.
    const std::vector<std::pair<int, int>> probes = {{0, 0}, {-13, 7}, {11, -9}, {-7, -17}, {17, 13}};
    for (auto &off : probes) {
        if (int(out.size()) >= count)
            break;
        int tx = max_x / 2 + off.first, ty = max_y / 2 + off.second;
        BelId best_bel;
        int best_dist = std::numeric_limits<int>::max();
        for (auto bel : ctx->getBels()) {
            if (!is_logic_tile(bel))
                continue;
            Loc l = ctx->getBelLocation(bel);
            int d = std::abs(l.x - tx) + std::abs(l.y - ty);
            if (d < best_dist) {
                best_dist = d;
                best_bel = bel;
            }
            if (d == 0)
                break;
        }
        if (best_bel == BelId())
            continue;
        // The bel's output pin, then down into general routing.
        WireId src;
        for (auto pin : ctx->getBelPins(best_bel)) {
            if (ctx->getBelPinType(best_bel, pin) != PORT_OUT)
                continue;
            WireId w = ctx->getBelPinWire(best_bel, pin);
            if (w == WireId())
                continue;
            bool has_downhill = false;
            for (auto pip : ctx->getPipsDownhill(w)) {
                (void)pip;
                has_downhill = true;
                break;
            }
            if (has_downhill) {
                src = w;
                break;
            }
        }
        if (src == WireId())
            continue;
        Loc sl = ctx->getBelLocation(best_bel);
        bool dup = false;
        for (auto &e : out)
            if (e.first == src)
                dup = true;
        if (!dup)
            out.emplace_back(src, sl);
    }
    return out;
}

void XilinxImpl::build_delay_matrix()
{
    dm_window = ctx->setting<int>("xilinx/delayMatrixWindow", 24);
    dm_max_explore = ctx->setting<int>("xilinx/delayMatrixExplore", 4000000);
    int nsources = ctx->setting<int>("xilinx/delayMatrixSources", 3);

    std::string cache = ctx->settings.count(ctx->id("xilinx/delayMatrixFile"))
                                ? ctx->settings.at(ctx->id("xilinx/delayMatrixFile")).as_string()
                                : std::string();
    bool loaded_from_cache = !cache.empty() && load_delay_matrix(cache);
    if (loaded_from_cache)
        return;

    int side = 2 * dm_window + 1;
    std::vector<delay_t> sum(side * side, 0), acc(side * side, -1);
    std::vector<int> hits(side * side, 0);

    auto sources = pick_delay_sources(nsources);
    if (sources.empty()) {
        log_info("Delay matrix: no logic-bel source found, keeping the tuned formula.\n");
        return;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    for (auto &s : sources) {
        std::fill(acc.begin(), acc.end(), delay_t(-1));
        int filled = measure_from(s.first, s.second.x, s.second.y, acc);
        for (size_t i = 0; i < acc.size(); i++) {
            if (acc.at(i) < 0)
                continue;
            sum.at(i) += acc.at(i);
            ++hits.at(i);
        }
        log_info("    source at X%dY%d reached %d of %d offsets\n", s.second.x, s.second.y, filled, side * side);
    }

    dm_delay.assign(side * side, -1);
    int covered = 0;
    for (size_t i = 0; i < dm_delay.size(); i++) {
        if (hits.at(i) == 0)
            continue;
        dm_delay.at(i) = sum.at(i) / hits.at(i);
        ++covered;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    log_info("Delay matrix: %d of %d offsets measured from %d sources in %.2fs (window %d).\n", covered, side * side,
             int(sources.size()), std::chrono::duration<float>(t1 - t0).count(), dm_window);
    if (covered < side) {
        log_info("Delay matrix: too sparse to be useful, keeping the tuned formula.\n");
        dm_delay.clear();
        return;
    }

    // Most uncovered offsets are structural: on this fabric only about one
    // column in three holds CLBs, so a third of the table can never be
    // measured directly.  Leaving those to the tuned formula would mix two
    // incompatible scales in one function -- the measurements are real
    // picoseconds from the chipdb, the formula is tuned arbitrary units -- and
    // an estimate that switches units from offset to offset is worse than
    // either scale used consistently.  So fill the holes from the nearest
    // measured neighbours instead.
    {
        std::vector<delay_t> filled_tab = dm_delay;
        int holes = 0;
        for (int y = -dm_window; y <= dm_window; y++) {
            for (int x = -dm_window; x <= dm_window; x++) {
                if (dm_delay.at(dm_index(x, y)) >= 0)
                    continue;
                ++holes;
                for (int r = 1; r <= dm_window; r++) {
                    int64_t acc_d = 0;
                    int n = 0;
                    for (int yy = std::max(-dm_window, y - r); yy <= std::min(dm_window, y + r); yy++)
                        for (int xx = std::max(-dm_window, x - r); xx <= std::min(dm_window, x + r); xx++) {
                            bool on_ring = std::abs(xx - x) == r || std::abs(yy - y) == r;
                            if (!on_ring)
                                continue;
                            delay_t v = dm_delay.at(dm_index(xx, yy));
                            if (v >= 0) {
                                acc_d += v;
                                ++n;
                            }
                        }
                    if (n > 0) {
                        filled_tab.at(dm_index(x, y)) = delay_t(acc_d / n);
                        break;
                    }
                }
            }
        }
        dm_delay = std::move(filled_tab);
        log_info("Delay matrix: interpolated %d structural holes.\n", holes);
    }

    dm_valid = true;
    compute_edge_rates();
    if (!cache.empty())
        save_delay_matrix(cache);
}

// The marginal delay per tile at the window edge, one rate per axis, averaged
// over the outer band of the measured table.  This is what long connections
// actually cost per tile out there -- on this fabric only about 8 ps, because
// the long lines make distant tiles cheap -- and it is derived from the
// measurements, not from the old hand-tuned formula, whose out-of-window slope
// was wrong by 3x.  Recomputed identically whether the matrix was built or
// loaded from cache, so the cache round-trip cannot drop it (the bug that made
// a cached run and a --delay-matrix=build run price long connections
// differently, and made a proof run disagree with CI on the same netlist).
void XilinxImpl::compute_edge_rates()
{
    const int band = 4;
    if (dm_window <= band) {
        dm_rate_x = dm_rate_y = 0.0f;
        return;
    }
    double sx = 0, sy = 0;
    int nx = 0, ny = 0;
    for (int y = -dm_window; y <= dm_window; y++) {
        delay_t outer = dm_delay.at(dm_index(dm_window, y));
        delay_t inner = dm_delay.at(dm_index(dm_window - band, y));
        bool both_measured = outer >= 0 && inner >= 0;
        if (both_measured) {
            sx += double(outer - inner) / band;
            ++nx;
        }
        delay_t outer_n = dm_delay.at(dm_index(-dm_window, y));
        delay_t inner_n = dm_delay.at(dm_index(-(dm_window - band), y));
        bool both_measured_n = outer_n >= 0 && inner_n >= 0;
        if (both_measured_n) {
            sx += double(outer_n - inner_n) / band;
            ++nx;
        }
    }
    for (int x = -dm_window; x <= dm_window; x++) {
        delay_t outer = dm_delay.at(dm_index(x, dm_window));
        delay_t inner = dm_delay.at(dm_index(x, dm_window - band));
        bool both_measured = outer >= 0 && inner >= 0;
        if (both_measured) {
            sy += double(outer - inner) / band;
            ++ny;
        }
        delay_t outer_n = dm_delay.at(dm_index(x, -dm_window));
        delay_t inner_n = dm_delay.at(dm_index(x, -(dm_window - band)));
        bool both_measured_n = outer_n >= 0 && inner_n >= 0;
        if (both_measured_n) {
            sy += double(outer_n - inner_n) / band;
            ++ny;
        }
    }
    // Never let the extrapolation slope go non-positive: a further tile must
    // not look free or cheaper.  Floor at a small positive rate.
    dm_rate_x = nx ? std::max(1.0f, float(sx / nx)) : 8.0f;
    dm_rate_y = ny ? std::max(1.0f, float(sy / ny)) : 8.0f;
    log_info("Delay matrix: out-of-window rates %.1f ps/tile (x), %.1f ps/tile (y).\n", dm_rate_x, dm_rate_y);
}

// Look the offset up.  In window: the measured value.  Out of window: the
// measured edge value plus the measured marginal rate times the overshoot, per
// axis -- no formula.  Returns -1 only when there is no matrix at all.
delay_t XilinxImpl::delay_matrix_lookup(int dx, int dy) const
{
    if (!dm_valid)
        return -1;
    int ex = std::max(-dm_window, std::min(dm_window, dx));
    int ey = std::max(-dm_window, std::min(dm_window, dy));
    delay_t base = dm_delay.at(dm_index(ex, ey));
    if (base < 0)
        return -1; // an unmeasured, uninterpolated hole (should not happen post-fill)
    int over_x = std::abs(dx) - std::abs(ex);
    int over_y = std::abs(dy) - std::abs(ey);
    return base + delay_t(over_x * dm_rate_x + over_y * dm_rate_y);
}

bool XilinxImpl::load_delay_matrix(const std::string &path)
{
    std::ifstream in(path);
    if (!in)
        return false;
    int window = 0;
    bool window_is_valid = (in >> window) && window > 0;
    if (!window_is_valid)
        return false;
    int side = 2 * window + 1;
    std::vector<delay_t> vals;
    vals.reserve(side * side);
    long long v;
    while (in >> v)
        vals.push_back(delay_t(v));
    if (int(vals.size()) != side * side) {
        log_info("Delay matrix: %s has %d entries, expected %d; ignoring it.\n", path.c_str(), int(vals.size()),
                 side * side);
        return false;
    }
    dm_window = window;
    dm_delay = std::move(vals);
    dm_valid = true;
    compute_edge_rates();
    log_info("Delay matrix: loaded %d offsets from %s.\n", side * side, path.c_str());
    return true;
}

void XilinxImpl::save_delay_matrix(const std::string &path) const
{
    std::ofstream out(path);
    if (!out) {
        log_warning("Delay matrix: could not write %s.\n", path.c_str());
        return;
    }
    out << dm_window << "\n";
    int side = 2 * dm_window + 1;
    for (int y = 0; y < side; y++) {
        for (int x = 0; x < side; x++)
            out << dm_delay.at(y * side + x) << (x + 1 == side ? '\n' : ' ');
    }
    log_info("Delay matrix: wrote %s.\n", path.c_str());
}

NEXTPNR_NAMESPACE_END
