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

// The global clock tree's insertion delay.
//
// A BUFG drives the clock spine, which runs up and down the die from its
// centre; each clock region's HROW taps the spine and drives the region's
// HCLK rows left and right, which feed the leaves.  prjxray gives the spine
// and HCLK nodes no resistance, so the routed delay of a global clock is the
// same at every sink (762 ps here) and nextpnr sees no skew at all.  The
// silicon has plenty: on an xc7vx485t, Vivado's clock delays grow by about
// 170 ps for each clock region row away from the BUFG's rows and 1.7 ps for
// each tile column away from the spine, and a block RAM's clock pin is 430 ps
// later than a flip-flop's.  On a design whose ROM block RAMs ended up at the
// top and bottom of the die that was the 0.4-0.9 ns which nextpnr reported as
// met and the board did not.
//
// The model adds that delay to each sink of a BUFGCTRL-driven net, routed or
// not, and turns on clock-skew analysis for every timing analyser, so the
// placer and router see it as well as the final report.  The region rows and
// the spine come from the chipdb's CLK_HROW and CLK_BUFG tiles, but the rates
// are measured per die, and a die without a measurement keeps the flat delay:
// another part's rates would be no better than none, and would move timing
// in either direction.  "-o clock-tree=off" keeps the flat delay everywhere;
// "-o clock-tree=force" uses the xc7vx485t's rates on an uncalibrated die,
// for measuring one.

#include "log.h"
#include "nextpnr.h"
#include "xilinx.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
// Per die: Vivado's clock delays (the destination clock delay of flip-flops
// and block RAMs in every clock region of a placed design), and the routed
// delay nextpnr gives every global clock sink, for unrouted estimates
const XilinxImpl::ClockTreeRates clock_tree_rates[] = {
        // xc7vx485t-2, a VexRiscv LiteX SoC placed by Vivado 2020.1
        {"xc7vx485t", 170, 17, 430, 762},
};
} // namespace

bool XilinxImpl::clock_rates_measured() const
{
    for (const auto &r : clock_tree_rates) {
        const bool die_is_measured = chipdb_die == r.die;
        if (die_is_measured)
            return true;
    }
    return false;
}

void XilinxImpl::init_clock_tree()
{
    std::string mode =
            ctx->args.options.count("clock-tree") ? ctx->args.options.at("clock-tree").as<std::string>() : "on";
    const bool mode_is_off = mode == "off";
    if (mode_is_off)
        return;
    const ClockTreeRates *rates = nullptr;
    for (const auto &r : clock_tree_rates) {
        const bool die_is_measured = chipdb_die == r.die;
        if (die_is_measured)
            rates = &r;
    }
    const bool unmeasured_and_not_forced = rates == nullptr && mode != "force";
    if (unmeasured_and_not_forced) {
        log_info("clock-tree: no clock delays measured for %s, global clocks keep their routed delay\n",
                 chipdb_die.c_str());
        return;
    }
    const bool borrowing_measured_die = rates == nullptr;
    if (borrowing_measured_die) {
        rates = &clock_tree_rates[0];
        log_warning("clock-tree: no clock delays measured for %s, using %s's\n", chipdb_die.c_str(), rates->die);
    }
    ct_rates = *rates;
    std::vector<int> bufg_rows;
    for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
        const std::string prefix = IdString(tile_extra_data(tile)->name_prefix).str(ctx);
        int x, y;
        tile_xy(ctx->chip_info, tile, x, y);
        const bool is_hrow = prefix.rfind("CLK_HROW_", 0) == 0;
        const bool is_bufg = prefix.rfind("CLK_BUFG_TOP", 0) == 0 || prefix.rfind("CLK_BUFG_BOT", 0) == 0;
        if (is_hrow) {
            ct_region_rows.push_back(y);
            ct_spine_x = x;
        }
        if (is_bufg)
            bufg_rows.push_back(y);
    }
    std::sort(ct_region_rows.begin(), ct_region_rows.end());
    ct_region_rows.erase(std::unique(ct_region_rows.begin(), ct_region_rows.end()), ct_region_rows.end());
    const bool device_has_a_clock_tree = !ct_region_rows.empty() && !bufg_rows.empty() && ct_spine_x >= 0;
    if (!device_has_a_clock_tree) {
        log_info("clock-tree: no CLK_HROW/CLK_BUFG tiles, global clocks keep their routed delay\n");
        return;
    }
    int sum = 0;
    for (int y : bufg_rows)
        sum += y;
    ct_centre_y = float(sum) / bufg_rows.size();
    for (auto bel : ctx->getBels()) {
        const std::string type = ctx->getBelType(bel).str(ctx);
        const bool is_bufgctrl = type.rfind("BUFGCTRL", 0) == 0;
        if (is_bufgctrl)
            ct_bufg_bel_types.insert(ctx->getBelType(bel));
    }
    ct_enabled = true;
    ctx->settings[ctx->id("timing/clockSkew")] = Property(1, 1);
    log_info("clock-tree: %s, %d clock region rows, spine at x=%d, BUFGs at y=%.1f\n", ct_rates.die,
             int(ct_region_rows.size()), ct_spine_x, ct_centre_y);
}

bool XilinxImpl::is_global_clock_bel(BelId bel) const
{
    return bel != BelId() && ct_bufg_bel_types.count(ctx->getBelType(bel));
}

delay_t XilinxImpl::clock_tree_delay_at(int x, int y, bool bram_clock_pin) const
{
    if (!ct_enabled)
        return 0;
    // the sink's region is the one whose HROW row is nearest; its distance
    // from the BUFGs is the number of region rows between them.  The BUFGs
    // sit between two region rows, and those two share the base delay: the
    // first step is the row beyond them.  Vivado on the xc7vx485t (the LiteX
    // SoC's sys_clk, BUFGCTRL_X0Y16 between CLOCKREGION Y3 and Y4; the
    // earliest block RAM clock pin of each row) gives Y0..Y6 = 1759, 1587,
    // 1415, 1243 | 1230, 1390, 1550 ps -- equal either side of the BUFGs,
    // then 160-172 ps a row.  Counting the sink's own row as well would add
    // the same 170 ps to every sink, a constant and no skew.
    int region_row = ct_region_rows.front();
    for (int r : ct_region_rows) {
        const bool nearer = std::abs(r - y) < std::abs(region_row - y);
        if (nearer)
            region_row = r;
    }
    const float region_offset = region_row - ct_centre_y;
    int rows_away = 0;
    for (int r : ct_region_rows) {
        const float offset = r - ct_centre_y;
        const bool same_side = (offset > 0) == (region_offset > 0);
        const bool between = same_side && std::abs(offset) < std::abs(region_offset);
        if (between)
            rows_away++;
    }
    delay_t delay = ct_rates.ps_per_region_row * rows_away + ct_rates.ps_per_10_columns * std::abs(x - ct_spine_x) / 10;
    if (bram_clock_pin)
        delay += ct_rates.bram_clock_pin_ps;
    return delay;
}

delay_t XilinxImpl::clock_tree_delay(BelId sink) const
{
    const bool tree_off_or_no_sink = !ct_enabled || sink == BelId();
    if (tree_off_or_no_sink)
        return 0;
    int x, y;
    tile_xy(ctx->chip_info, sink.tile, x, y);
    return clock_tree_delay_at(x, y, is_bram_tile(sink));
}

bool XilinxImpl::getArcDelayOverride(const NetInfo *net_info, const PortRef &sink, DelayQuad &delay) const
{
    if (!ct_enabled)
        return false;
    const CellInfo *driver = net_info->driver.cell;
    const bool driven_by_a_bufg = driver != nullptr && is_global_clock_bel(driver->bel);
    const bool not_a_bufg_sink = !driven_by_a_bufg || sink.cell == nullptr;
    if (not_a_bufg_sink)
        return false;
    // Each of the port's sink wires as Context::getNetinfoRouteDelayQuad
    // takes it -- the routed delay where it is routed, the prediction where
    // not -- with the insertion delay on top; the prediction (predictDelay,
    // for a BUFG source) carries the insertion delay already
    const delay_t insertion = clock_tree_delay(sink.cell->bel);
    WireId src_wire = ctx->getNetinfoSourceWire(net_info);
    const bool source_unrouted = src_wire == WireId();
    if (source_unrouted)
        return false;
    bool found = false;
    DelayQuad result(std::numeric_limits<delay_t>::max(), std::numeric_limits<delay_t>::lowest());
    for (auto dst_wire : ctx->getNetinfoSinkWires(net_info, sink)) {
        WireId cursor = dst_wire;
        DelayQuad d{0};
        while (cursor != WireId() && cursor != src_wire) {
            auto it = net_info->wires.find(cursor);
            const bool hop_unrouted = it == net_info->wires.end() || it->second.pip == PipId();
            if (hop_unrouted)
                break;
            d = d + ctx->getPipDelay(it->second.pip) + ctx->getWireDelay(cursor);
            cursor = ctx->getPipSrcWire(it->second.pip);
        }
        const bool arc_is_routed = cursor == src_wire;
        if (arc_is_routed)
            d = d + ctx->getWireDelay(src_wire) + DelayQuad(insertion);
        else
            d = DelayQuad(ctx->predictArcDelay(net_info, sink));
        // rise and fall kept apart, as Context::getNetinfoRouteDelayQuad does
        result.rise.min_delay = std::min(result.rise.min_delay, d.rise.min_delay);
        result.rise.max_delay = std::max(result.rise.max_delay, d.rise.max_delay);
        result.fall.min_delay = std::min(result.fall.min_delay, d.fall.min_delay);
        result.fall.max_delay = std::max(result.fall.max_delay, d.fall.max_delay);
        found = true;
    }
    if (!found)
        return false;
    delay = result;
    return true;
}

delay_t XilinxImpl::predict_clock_tree_delay(BelId src_bel, BelId dst_bel) const
{
    const bool tree_off_or_not_global = !ct_enabled || !is_global_clock_bel(src_bel);
    if (tree_off_or_not_global)
        return -1;
    // The source's own site adds nothing: every BUFGCTRL of a 7-series die
    // sits in the one CLK_BUFG column at the centre the rates were measured
    // from (ct_centre_y is the mean of those rows), and the regional buffers
    // (BUFH, BUFR, BUFIO) are not global clock bels, so their sinks keep the
    // routed delay.
    return ct_rates.flat_route_ps + clock_tree_delay(dst_bel);
}

NEXTPNR_NAMESPACE_END
