/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-2023  Myrtle Shah <gatecat@ds0.me>
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

#include <algorithm>
#include <boost/optional.hpp>
#include <iterator>
#include <queue>
#include <unordered_set>
#include "chain_utils.h"
#include "design_utils.h"
#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

BelId XilinxPacker::find_bel_with_short_route(WireId source, IdString beltype, IdString belpin)
{
    if (source == WireId())
        return BelId();
    const size_t max_visit = 1000000; // effort/runtime tradeoff
    pool<WireId> visited;
    std::queue<WireId> visit;
    visit.push(source);
    while (!visit.empty() && visited.size() < max_visit) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor))
            if (bp.pin == belpin && ctx->getBelType(bp.bel) == beltype && ctx->checkBelAvail(bp.bel))
                return bp.bel;
        for (auto pip : ctx->getPipsDownhill(cursor)) {
            WireId dst = ctx->getPipDstWire(pip);

            IdString intent = ctx->getWireType(dst);
            // log_info("%s %s\n", ctx->nameOfWire(dst), ctx->nameOf(intent));
            // The wire types route_clocks() refuses.  Without PINBOUNCE a clock
            // with no dedicated way to a free bel walks the FAN bounce chain of
            // the fabric down to a CLK_BUFG_IMUX, and the bel found that way is
            // locked "based on dedicated routing" on a site its clock cannot
            // reach through the clock network.
            const bool wire_is_not_clock_routing =
                    intent.in(id_NODE_DOUBLE, id_NODE_HLONG, id_NODE_HQUAD, id_NODE_VLONG, id_NODE_VQUAD,
                              id_NODE_SINGLE, id_NODE_CLE_OUTPUT, id_NODE_OPTDELAY, id_BENTQUAD, id_DOUBLE, id_HLONG,
                              id_HQUAD, id_OPTDELAY, id_SINGLE, id_VLONG, id_VLONG12, id_VQUAD, id_PINBOUNCE);
            if (wire_is_not_clock_routing)
                continue;

            if (visited.count(dst))
                continue;
            visit.push(dst);
            visited.insert(dst);
        }
    }
    return BelId();
}

bool XilinxPacker::try_preplace(CellInfo *cell, IdString port)
{
    if (cell->attrs.count(id_BEL) || cell->bel != BelId())
        return false;
    NetInfo *n = cell->getPort(port);
    if (n == nullptr || n->driver.cell == nullptr)
        return false;
    CellInfo *drv = n->driver.cell;
    BelId drv_bel = drv->bel;
    if (drv_bel == BelId())
        return false;
    WireId drv_wire = ctx->getBelPinWire(drv_bel, n->driver.port);
    if (drv_wire == WireId())
        return false;
    BelId tgt = find_bel_with_short_route(drv_wire, cell->type, port);
    if (tgt != BelId()) {
        ctx->bindBel(tgt, cell, STRENGTH_LOCKED);
        log_info("    Constrained %s '%s' to bel '%s' based on dedicated routing\n", cell->type.c_str(ctx),
                 ctx->nameOf(cell), ctx->nameOfBel(tgt));
        return true;
    } else {
        return false;
    }
}

void XilinxPacker::preplace_unique(CellInfo *cell)
{
    const bool the_design_already_placed_this_cell = cell->attrs.count(id_BEL) != 0 || cell->bel != BelId();
    if (the_design_already_placed_this_cell)
        return;
    // a site another cell of this type is pinned to (BEL attribute, not yet
    // bound) is not available either
    pool<BelId> claimed;
    for (auto &other : ctx->cells) {
        const bool another_cell_of_this_type_is_pinned =
                other.second->type == cell->type && other.second->attrs.count(id_BEL) != 0;
        if (another_cell_of_this_type_is_pinned)
            claimed.insert(ctx->getBelByNameStr(other.second->attrs.at(id_BEL).as_string()));
    }
    for (auto bel : ctx->getBels()) {
        const bool bel_is_claimed_by_a_pinned_cell = claimed.count(bel) != 0;
        if (bel_is_claimed_by_a_pinned_cell)
            continue;
        const bool bel_is_free_and_of_this_type =
                ctx->checkBelAvail(bel) && ctx->getBelType(bel) == cell->type;
        if (bel_is_free_and_of_this_type) {
            ctx->bindBel(bel, cell, STRENGTH_LOCKED);
            return;
        }
    }
}

// Undo a global buffer sitting between an input pad and a PLL/MMCM reference
// input.
//
// yosys' clkbufmap inserts a BUFG on any clock net, INCLUDING the one feeding
// PLLE2_ADV.CLKIN1.  That is fatal rather than merely wasteful: with a BUFG in
// the way the PLL's reference is a global-buffer output, so the dedicated
// CCIO -> HCLK_CMT_MUX_PLLE2_CLKIN1 connection is not on any path the router
// could take.  The router then has to drag the buffered clock back into the
// CMT through a BUFH, and the PLL never locks -- HW-measured on the Sonata with
// picosoc/top_pll_debug.v: the raw 25 MHz reached the fabric and nrst was
// released, but LOCKED stayed low.  Vivado drives CLKIN1 straight from the pad
// and uses buffers only for fabric distribution.
//
// It also fixes placement for free.  preplace_clocking() finds a PLL site by
// BFS from the driver wire (find_bel_with_short_route); starting from a BUFG
// output every PLL looks equally "dedicated", so it picked one two clock
// regions from the pad.  Starting from the pad it finds the right site
// unaided -- one cause behind both symptoms.
//
// Only bypass when the buffer is fed by an input pad, since that is exactly
// when a dedicated path exists, and keep the buffer if anything else uses it.
void XC7Packer::bypass_pll_input_buffers()
{
    const pool<IdString> gbufs{id_BUFG, id_BUFGCE, id_BUFGCTRL, id_BUFH, id_BUFHCE};
    const pool<IdString> pads{id_IOB33_INBUF_EN, id_IOB18_INBUF_DCIEN};
    int bypassed = 0, removed = 0;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!ci->type.in(id_PLLE2_ADV, id_PLLE2_BASE, id_MMCME2_ADV, id_MMCME2_BASE))
            continue;
        for (IdString port : {id_CLKIN1, id_CLKIN2}) {
            NetInfo *ref = ci->getPort(port);
            bool reference_is_driven = ref != nullptr && ref->driver.cell != nullptr;
            if (!reference_is_driven)
                continue;
            CellInfo *buf = ref->driver.cell;
            if (!gbufs.count(buf->type))
                continue;
            // BUFG/BUFH use I; a BUFGCTRL that has already been converted uses I0
            NetInfo *src = buf->getPort(id_I);
            if (src == nullptr)
                src = buf->getPort(id_I0);
            bool buffer_input_is_driven = src != nullptr && src->driver.cell != nullptr;
            if (!buffer_input_is_driven)
                continue;
            if (!pads.count(src->driver.cell->type))
                continue;

            bool buffer_is_constrained = buf->attrs.count(id_LOC) || buf->attrs.count(id_BEL) || buf->bel != BelId();
            if (buffer_is_constrained)
                continue;

            ci->disconnectPort(port);
            ci->connectPort(port, src);
            ++bypassed;
            log_info("    %s.%s: bypassed %s '%s' to take the dedicated route from pad '%s'\n",
                     ci->name.c_str(ctx), port.c_str(ctx), buf->type.c_str(ctx), buf->name.c_str(ctx),
                     src->driver.cell->name.c_str(ctx));

            if (ref->users.empty()) {
                IdString bufname = buf->name;
                for (auto &p : buf->ports)
                    if (p.second.net != nullptr)
                        buf->disconnectPort(p.first);
                ctx->cells.erase(bufname);
                ++removed;
            }
        }
    }
    if (bypassed > 0)
        log_info("    bypassed %d gratuitous clock buffer(s) into CMT reference inputs, removed %d\n", bypassed,
                 removed);
}

// Vivado ties IGNORE1 of a BUFG or BUFGCE high: the unused I1 is never
// waited for. pack_constants() has tied it low, to VCC through the site
// inverter; clearing the inverter here, instead of tying the pin in
// prepare_clocking(), keeps the users of the VCC net in their order, and with
// them the routing of every design.
void XC7Packer::raise_bufg_ignore1()
{
    for (CellInfo *ci : bufgs_ignoring_i1)
        ci->params[id_IS_IGNORE1_INVERTED] = Property(0);
}

// BUFGMUX, BUFGMUX_1 and BUFGMUX_CTRL on a BUFGCTRL, as Vivado builds them:
// BUFGMUX and BUFGMUX_1 switch on the clock enables (CE1 = S, CE0 = S through
// the site inverter, S0 = S1 = 1), BUFGMUX_CTRL on the selects (S1 = S, S0 = S
// inverted, CE0 = CE1 = 1). IGNORE0/IGNORE1 are low, or high with CLK_SEL_TYPE
// "ASYNC". BUFGMUX_1 idles high (INIT_OUT).
void XC7Packer::bufgmux_to_bufgctrl(CellInfo *ci)
{
    const std::string type = ci->type.str(ctx);
    for (IdString port : {id_I0, id_I1, id_S}) {
        const bool port_has_no_net = ci->getPort(port) == nullptr;
        if (port_has_no_net)
            log_error("%s '%s': port %s is not connected\n", type.c_str(), ci->name.c_str(ctx), port.c_str(ctx));
    }
    IdString clk_sel_type = ctx->id("CLK_SEL_TYPE");
    const std::string sel = str_or_default(ci->params, clk_sel_type, "SYNC");
    const bool sel_is_async = sel == "ASYNC";
    const bool sel_is_unknown = !sel_is_async && sel != "SYNC";
    if (sel_is_unknown)
        log_error("%s '%s': CLK_SEL_TYPE must be \"SYNC\" or \"ASYNC\", not \"%s\"\n", type.c_str(),
                  ci->name.c_str(ctx), sel.c_str());
    ci->params.erase(clk_sel_type);

    NetInfo *s = ci->getPort(id_S);
    ci->type = id_BUFGCTRL;
    const bool switches_on_selects = type == "BUFGMUX_CTRL";
    if (switches_on_selects) {
        ci->renamePort(id_S, id_S1);
        ci->addInput(id_S0);
        ci->connectPort(id_S0, s);
        ci->params[id_IS_S0_INVERTED] = Property(1);
        tie_port(ci, "CE0", true, true);
        tie_port(ci, "CE1", true, true);
    } else {
        ci->renamePort(id_S, id_CE1);
        ci->addInput(id_CE0);
        ci->connectPort(id_CE0, s);
        ci->params[id_IS_CE0_INVERTED] = Property(1);
        tie_port(ci, "S0", true, true);
        tie_port(ci, "S1", true, true);
    }
    tie_port(ci, "IGNORE0", sel_is_async, true);
    tie_port(ci, "IGNORE1", sel_is_async, true);
    const bool idles_high = type == "BUFGMUX_1";
    if (idles_high)
        ci->params[id_INIT_OUT] = Property(1);
}

void XC7Packer::prepare_clocking()
{
    log_info("Preparing clocking...\n");
    bypass_pll_input_buffers();
    dict<IdString, IdString> upgrade;
    upgrade[id_MMCME2_BASE] = id_MMCME2_ADV;
    upgrade[id_PLLE2_BASE] = id_PLLE2_ADV;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        bool is_horizontal_clock_buffer = ci->type == id_BUFH || ci->type == id_BUFHCE;
        if (upgrade.count(ci->type)) {
            IdString new_type = upgrade.at(ci->type);
            ci->type = new_type;
        } else if (ci->type == id_BUFG) {
            ci->type = id_BUFGCTRL;
            ci->renamePort(id_I, id_I0);
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", false, true);
            bufgs_ignoring_i1.push_back(ci);
        } else if (ci->type == id_BUFGCE) {
            ci->type = id_BUFGCTRL;
            ci->renamePort(id_I, id_I0);
            ci->renamePort(id_CE, id_CE0);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", false, true);
            bufgs_ignoring_i1.push_back(ci);
        } else if (is_horizontal_clock_buffer) {
            // BUFH is the legacy non-CE spelling; both map to the BUFHCE bel
            // with CE tied active (port of nextpnr-xilinx pack_clocking_xc7.cc)
            ci->type = id_BUFHCE_BUFHCE;
            bool ce_is_connected = ci->ports.count(id_CE) && ci->getPort(id_CE) != nullptr;
            if (ce_is_connected)
                ci->disconnectPort(id_CE);
            tie_port(ci, "CE", true, true);
        } else if (ci->type == id_BUFR) {
            // BUFR pins (I/CE/CLR/O) match the BUFR_BUFR bel one-to-one
            ci->type = id_BUFR_BUFR;
        } else if (ci->type == id_BUFIO) {
            // BUFIO is the undivided I/O clock buffer: a BUFIO_BUFIO bel with
            // just I and O, no CE/CLR to tie off.  Without this branch the cell
            // reaches the placer still typed BUFIO, no bel of that type exists,
            // and the run dies with "no Bels remaining of type 'BUFIO'" while
            // the BUFIO_BUFIO sites sit unused.  (Port of nextpnr-xilinx #157.)
            ci->type = id_BUFIO_BUFIO;
        } else {
            // The BUFGMUX names have no constid: compare the string, as a
            // ctx->id() here would intern them for every design.
            const std::string &type = ci->type.str(ctx);
            const bool is_bufgmux = type == "BUFGMUX" || type == "BUFGMUX_1" || type == "BUFGMUX_CTRL";
            if (is_bufgmux)
                bufgmux_to_bufgctrl(ci);
        }
    }
}

void XC7Packer::pack_plls()
{
    log_info("Packing PLLs...\n");

    auto set_default = [](CellInfo *ci, IdString param, const Property &value) {
        if (!ci->params.count(param))
            ci->params[param] = value;
    };

    dict<IdString, XFormRule> pll_rules;
    pll_rules[id_MMCME2_ADV].new_type = id_MMCME2_ADV_MMCME2_ADV;
    pll_rules[id_PLLE2_ADV].new_type = id_PLLE2_ADV_PLLE2_ADV;
    generic_xform(pll_rules);
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_MMCM_MMCM_TOP) {
            // Fixup parameters
            for (int i = 1; i <= 2; i++)
                set_default(ci, ctx->idf("CLKIN%d_PERIOD", i), Property("0.0"));
            for (int i = 0; i <= 6; i++) {
                set_default(ci, ctx->idf("CLKOUT%d_CASCADE", i), Property("FALSE"));
                set_default(ci, ctx->idf("CLKOUT%d_DIVIDE", i), Property(1));
                set_default(ci, ctx->idf("CLKOUT%d_DUTY_CYCLE", i), Property("0.5"));
                set_default(ci, ctx->idf("CLKOUT%d_PHASE", i), Property(0));
                set_default(ci, ctx->idf("CLKOUT%d_USE_FINE_PS", i), Property("FALSE"));
            }
            set_default(ci, id_COMPENSATION, Property("INTERNAL"));

            // Fixup routing
            if (str_or_default(ci->params, id_COMPENSATION, "INTERNAL") == "INTERNAL") {
                ci->disconnectPort(id_CLKFBIN);
                ci->connectPort(id_CLKFBIN, ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get());
            }
        }
    }
}

void XC7Packer::pack_gbs()
{
    log_info("Packing global buffers...\n");
    dict<IdString, XFormRule> gb_rules;
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;

    generic_xform(gb_rules);

    // Make sure prerequisites are set up first
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_PS7_PS7)
            preplace_unique(ci);
        if (ci->type == id_PCIE_2_1_PCIE_2_1)
            preplace_unique(ci);
        if (ci->type.in(id_PSEUDO_GND, id_PSEUDO_VCC))
            preplace_unique(ci);
    }

    // A BUFIO/BUFR is a regional I/O clock buffer rather than a global one,
    // but this is where the clock buffers get their bels and it has to run
    // after pack_io() has placed the pads it reads.
    constrain_bufios();
}

void XC7Packer::preplace_clocking()
{
    // The XDC's LOCs on these cells first.  Each site picked below follows
    // from where the driver sits and from which sites are still free; a LOC
    // applied after that (prePlace) moves the driver, or evicts a cell from
    // the site it names, and leaves the loads chosen here on sites the clock
    // cannot reach through the clock network.  Only when there is such a LOC:
    // resolving one names every site of the device, and those names are
    // IdStrings the rest of the flow would otherwise create later.
    const pool<IdString> clocking_types{
            id_BUFGCTRL,    id_BUFG_BUFG,           id_BUFHCE_BUFHCE,        id_MMCM_MMCM_TOP,
            id_PLL_PLL_TOP, id_PLLE2_ADV_PLLE2_ADV, id_MMCME2_ADV_MMCME2_ADV};
    const bool a_clocking_cell_has_a_loc = std::any_of(ctx->cells.begin(), ctx->cells.end(), [&](const auto &cell) {
        return clocking_types.count(cell.second->type) && cell.second->attrs.count(id_LOC);
    });
    if (a_clocking_cell_has_a_loc)
        uarch->apply_loc_constraints(clocking_types);
    bool did_something = false;
    do {
        did_something = false;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel != BelId())
                continue;
            if (ci->type == id_BUFGCTRL)
                did_something |= try_preplace(ci, id_I0);
            else if (ci->type == id_BUFG_BUFG)
                did_something |= try_preplace(ci, id_I);
            else if (ci->type == id_BUFHCE_BUFHCE)
                did_something |= try_preplace(ci, id_I);
            else if (ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP, id_PLLE2_ADV_PLLE2_ADV, id_MMCME2_ADV_MMCME2_ADV))
                did_something |= try_preplace(ci, id_CLKIN1);
        }
    } while (did_something);
}

// A BUFIO is not placed wherever there is room: it is placed where the pad
// says.  Its I pin has no fabric input at all -- the only wire that reaches it
// is the I2IOCLK leg its own clock-capable pad drives into the HCLK_IOI tile --
// so of the four BUFIO_BUFIO bels of a tile exactly ONE is reachable from a
// given pad.  A pad-fed BUFR carries the identical constraint, for the same
// reason.  Ask the routing graph which bel the pad reaches -- the same pip BFS
// find_bel_with_short_route() runs for a BUFG -- and constrain the cell to it.
// A table of pad->site pairs would answer the same question for artix7 today
// and be wrong for the next family; the graph is per-part data and already
// knows.
void XC7Packer::constrain_bufios()
{
    const pool<IdString> inbuf_types{id_IOB33M_INBUF_EN, id_IOB33S_INBUF_EN, id_IOB33_INBUF_EN,
                                     id_IOB18_INBUF_DCIEN, id_IOB18M_INBUF_DCIEN};
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool is_regional_buffer = ci->type == id_BUFIO_BUFIO || ci->type == id_BUFR_BUFR;
        if (!is_regional_buffer)
            continue;

        NetInfo *clk = ci->getPort(id_I);
        const bool has_driven_input = clk != nullptr && clk->driver.cell != nullptr;
        if (!has_driven_input)
            continue;

        CellInfo *drv = clk->driver.cell;
        // Only a pad fixes the site.  A regional buffer driven by an MMCM/PLL
        // output, or by anything else, enters the tile through a different
        // DMUX leg and is left exactly as it was.  pack_io() has already given
        // every input buffer its bel by now, which is what makes the pad
        // knowable this early.
        const bool driven_by_input_buffer = inbuf_types.count(drv->type) > 0;
        const bool driver_is_placed = drv->bel != BelId();
        if (!driven_by_input_buffer || !driver_is_placed)
            continue;

        WireId drv_wire = ctx->getBelPinWire(drv->bel, clk->driver.port);
        const bool pad_has_an_output_wire = drv_wire != WireId();
        if (!pad_has_an_output_wire)
            continue;

        BelId dedicated = find_bel_with_short_route(drv_wire, ci->type, id_I);
        // No site reachable: the pad is not clock-capable.  Leave that to the
        // router, whose message names the two ends of the arc it could not
        // build; guessing a site here would only move the failure.
        const bool pad_reaches_a_bufio = dedicated != BelId();
        if (!pad_reaches_a_bufio)
            continue;

        // A site the user (or an earlier pass) already bound wins; leave it,
        // and its sinks, exactly as they were.
        const bool already_bound = ci->bel != BelId();
        if (already_bound)
            continue;

        ctx->bindBel(dedicated, ci, STRENGTH_LOCKED);
        log_info("    Constrained %s '%s' to bel '%s' (dedicated site of the pad at %s)\n", ci->type.c_str(ctx),
                 ctx->nameOf(ci), ctx->nameOfBel(dedicated), ctx->nameOfBel(drv->bel));
    }
}

// Binding the buffer to the right site fixes the arc into it.  The arc out of
// it is a separate problem: a regional buffer drives one clock region, and
// nothing tells the placer that the flops it clocks have to live there.  The
// placer does not cost global nets, so the flops follow whatever data pin they
// touch -- to an LED half a die away -- and the router then dies on the clock.
// Ask the graph where the clock actually arrives, exactly as the site search
// above does, and hand the placer that rectangle.  Deriving it from geometry
// would mean encoding how tall a clock region is per family; the routing graph
// is per-part data and already knows.
void XC7Packer::constrain_regional_clock_sinks(CellInfo *buf)
{
    NetInfo *clk = buf->getPort(id_O);
    const bool clk_has_sinks = clk != nullptr && !clk->users.empty();
    if (!clk_has_sinks)
        return;

    const bool buf_is_placed = buf->bel != BelId();
    if (!buf_is_placed)
        return;

    WireId src = ctx->getBelPinWire(buf->bel, id_O);
    const bool buf_has_an_output_wire = src != WireId();
    if (!buf_has_an_output_wire)
        return;

    // Same effort cap and the same layer-by-layer walk as
    // find_bel_with_short_route(); here we want every bel the clock reaches
    // rather than the nearest one, so the walk runs to exhaustion.
    const size_t max_visit = 1000000;
    pool<WireId> visited;
    visited.insert(src);
    std::vector<WireId> frontier{src};
    bool any = false;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    while (!frontier.empty() && visited.size() < max_visit) {
        for (WireId w : frontier) {
            for (auto bp : ctx->getWireBelPins(w)) {
                // Only where the clock can actually clock something.  Counting
                // every bel pin the walk touches returns the whole die: a wire
                // brushing some bel's data input says nothing about the clock
                // reaching its CLK, and the resulting rectangle was the whole
                // die -- no constraint at all.
                const bool pin_is_a_clock = bp.pin == id_CLK;
                if (!pin_is_a_clock)
                    continue;
                Loc l = ctx->getBelLocation(bp.bel);
                const bool first_clock_pin = !any;
                if (first_clock_pin) {
                    x0 = x1 = l.x;
                    y0 = y1 = l.y;
                    any = true;
                } else {
                    x0 = std::min(x0, l.x);
                    x1 = std::max(x1, l.x);
                    y0 = std::min(y0, l.y);
                    y1 = std::max(y1, l.y);
                }
            }
        }
        std::vector<WireId> next_frontier;
        for (WireId w : frontier)
            for (auto pip : ctx->getPipsDownhill(w)) {
                WireId dst = ctx->getPipDstWire(pip);
                const bool already_visited = visited.count(dst) > 0;
                if (already_visited)
                    continue;
                visited.insert(dst);
                next_frontier.push_back(dst);
            }
        frontier.swap(next_frontier);
    }
    const bool clock_reaches_a_bel = any;
    if (!clock_reaches_a_bel)
        return;

    IdString rname = ctx->id("clkregion_" + buf->name.str(ctx));
    ctx->createRectangularRegion(rname, x0, y0, x1, y1);
    int n = 0;
    for (auto &usr : clk->users) {
        // A cell already inside a placement cluster is positioned relative to
        // its root, so constrain the root and let the cluster follow it.
        CellInfo *tgt = usr.cell;
        const bool tgt_is_in_a_cluster = tgt->cluster != IdString();
        if (tgt_is_in_a_cluster)
            tgt = ctx->getClusterRootCell(tgt->cluster);
        const bool sink_unconstrained = tgt->region == nullptr;
        if (sink_unconstrained) {
            ctx->constrainCellToRegion(tgt->name, rname);
            ++n;
        }
    }
    log_info("    Constrained %d sink(s) of %s '%s' to its clock region x%d..%d y%d..%d\n", n, buf->type.c_str(ctx),
             ctx->nameOf(buf), x0, x1, y0, y1);
}

void XC7Packer::pack_clocking()
{
    pack_plls();
    pack_gbs();
    preplace_clocking();
}

void XilinxImpl::route_clocks()
{
    log_info("Routing global clocks...\n");
    // Special pass for faster routing of global clock psuedo-net
    for (auto &net : ctx->nets) {
        NetInfo *clk_net = net.second.get();
        if (!clk_net->driver.cell)
            continue;

        // check if we have a global clock net, skip otherwise
        bool driven_by_bufr = clk_net->driver.cell->type == id_BUFR_BUFR && clk_net->driver.port == id_O;
        bool feeds_only_a_bufr = clk_net->users.entries() == 1 &&
                                 (*clk_net->users.begin()).cell->type == id_BUFR_BUFR &&
                                 (*clk_net->users.begin()).port == id_I;
        bool is_global = false;
        if ((clk_net->driver.cell->type.in(id_BUFGCTRL, id_BUFCE_BUFG_PS, id_BUFCE_BUFCE, id_BUFGCE_DIV_BUFGCE_DIV)) &&
            clk_net->driver.port == id_O)
            is_global = true;
        else if (driven_by_bufr)
            is_global = true;
        else if (feeds_only_a_bufr)
            is_global = true;
        else if (clk_net->driver.cell->type.in(id_PLLE2_ADV_PLLE2_ADV, id_MMCME2_ADV_MMCME2_ADV) &&
                 clk_net->users.entries() == 1 &&
                 ((*clk_net->users.begin()).cell->type.in(id_BUFGCTRL, id_BUFCE_BUFCE, id_BUFGCE_DIV_BUFGCE_DIV)))
            is_global = true;
        else if (clk_net->users.entries() == 1 &&
                 (*clk_net->users.begin()).cell->type.in(id_PLLE2_ADV_PLLE2_ADV, id_MMCME2_ADV_MMCME2_ADV) &&
                 (*clk_net->users.begin()).port == id_CLKIN1)
            is_global = true;
        if (!is_global)
            continue;

        // A buffer output with no loads must stay unbound: router1 treats any
        // wire on a sinkless net as a stub, and nothing needs the source wire.
        const bool clock_has_no_sinks = clk_net->users.empty();
        if (clock_has_no_sinks) {
            log_info("    clock '%s' has no sinks; nothing to route\n", clk_net->name.c_str(ctx));
            continue;
        }

        log_info("    routing clock '%s'\n", clk_net->name.c_str(ctx));
        ctx->bindWire(ctx->getNetinfoSourceWire(clk_net), clk_net, STRENGTH_LOCKED);

        for (auto &usr : clk_net->users) {
            std::queue<WireId> visit;
            dict<WireId, PipId> backtrace;
            WireId dest = WireId();

            auto sink_wire = ctx->getNetinfoSinkWire(clk_net, usr, 0);
            if (ctx->debug) {
                auto sink_wire_name = "(uninitialized)";
                if (sink_wire != WireId())
                    sink_wire_name = ctx->nameOfWire(sink_wire);
                log_info("        routing arc to %s.%s (wire %s):\n", usr.cell->name.c_str(ctx), usr.port.c_str(ctx),
                         sink_wire_name);
            }

            visit.push(sink_wire);
            while (!visit.empty()) {
                WireId curr = visit.front();
                visit.pop();
                if (ctx->getBoundWireNet(curr) == clk_net) {
                    dest = curr;
                    break;
                }
                for (auto uh : ctx->getPipsUphill(curr)) {
                    if (!ctx->checkPipAvail(uh))
                        continue;
                    WireId src = ctx->getPipSrcWire(uh);
                    if (backtrace.count(src))
                        continue;
                    IdString intent = ctx->getWireType(src);
                    if (intent.in(id_NODE_DOUBLE, id_NODE_HLONG, id_NODE_HQUAD, id_NODE_VLONG, id_NODE_VQUAD,
                                  id_NODE_SINGLE, id_NODE_CLE_OUTPUT, id_NODE_OPTDELAY, id_BENTQUAD, id_DOUBLE,
                                  id_HLONG, id_HQUAD, id_OPTDELAY, id_SINGLE, id_VLONG, id_VLONG12, id_VQUAD,
                                  id_PINBOUNCE))
                        continue;
                    if (!ctx->checkWireAvail(src) && ctx->getBoundWireNet(src) != clk_net)
                        continue;
                    backtrace[src] = uh;
                    visit.push(src);
                }
            }
            if (dest == WireId()) {
                log_info("            failed to find a route using dedicated resources.\n");
                if (clk_net->users.entries() == 1 && (*clk_net->users.begin()).cell->type == id_PLLE2_ADV_PLLE2_ADV &&
                    (*clk_net->users.begin()).port == id_CLKIN1) {
                    // Due to some missing pips, currently special case more lenient solution
                    std::queue<WireId> empty;
                    std::swap(visit, empty);
                    backtrace.clear();
                    visit.push(sink_wire);
                    while (!visit.empty()) {
                        WireId curr = visit.front();
                        visit.pop();
                        if (ctx->getBoundWireNet(curr) == clk_net) {
                            dest = curr;
                            break;
                        }
                        for (auto uh : ctx->getPipsUphill(curr)) {
                            if (!ctx->checkPipAvail(uh))
                                continue;
                            WireId src = ctx->getPipSrcWire(uh);
                            if (backtrace.count(src))
                                continue;
                            if (!ctx->checkWireAvail(src) && ctx->getBoundWireNet(src) != clk_net)
                                continue;
                            backtrace[src] = uh;
                            visit.push(src);
                        }
                    }
                    if (dest == WireId())
                        continue;
                } else {
                    continue;
                }
            }
            while (backtrace.count(dest)) {
                auto uh = backtrace[dest];
                dest = ctx->getPipDstWire(uh);
                if (ctx->getBoundWireNet(dest) == clk_net) {
                    NPNR_ASSERT(clk_net->wires.at(dest).pip == uh);
                    break;
                }
                if (ctx->debug)
                    log_info("            bind pip %s --> %s\n", ctx->nameOfPip(uh), ctx->nameOfWire(dest));
                ctx->bindPip(uh, clk_net, STRENGTH_LOCKED);
            }
        }
    }
#if 0
    for (auto& net : nets) {
        NetInfo *ni = net.second.get();
        for (auto &usr : ni->users) {
            if (usr.cell->type != id_BUFGCTRL || usr.port != id_I0)
                continue;
            WireId dst = getCtx()->getNetinfoSinkWire(ni, usr, 0);
            std::queue<WireId> visit;
            visit.push(dst);
            int i = 0;
            while(!visit.empty() && i < 5000) {
                WireId curr = visit.front();
                visit.pop();
                log("  %s\n", nameOfWire(curr));
                for (auto pip : getPipsUphill(curr)) {
                    auto &pd = locInfo(pip).pip_data[pip.index];
                    log_info("    p %s sr %s (t %d s %d sv %d)\n", nameOfPip(pip), nameOfWire(getPipSrcWire(pip)), pd.flags, pd.site, pd.site_variant);
                    if (!checkPipAvail(pip)) {
                        log("      p unavail\n");
                        continue;
                    }
                    WireId src = getPipSrcWire(pip);
                    if (!checkWireAvail(src)) {
                        log("      w unavail (%s)\n", nameOf(getBoundWireNet(src)));
                        continue;
                    }
                    log_info("     p %s s %s\n", nameOfPip(pip), nameOfWire(src));
                    visit.push(src);
                }
                ++i;
            }
        }
    }
#endif
}

namespace {
double float_or_default(CellInfo *ci, IdString p, double def)
{
    if (!ci->params.count(p))
        return def;
    auto &prop = ci->params.at(p);
    if (prop.is_string)
        return std::stod(prop.as_string());
    else
        return prop.as_int64();
}
} // namespace

void XilinxPacker::generate_constraints()
{
    log_info("Generating derived timing constraints...\n");
    auto MHz = [&](delay_t a) { return 1000.0 / ctx->getDelayNS(a); };

    auto equals_epsilon = [](delay_t a, delay_t b) { return (std::abs(a - b) / std::max(double(b), 1.0)) < 1e-3; };
    auto equals_epsilon_pair = [&](DelayPair &a, DelayPair &b) {
        return equals_epsilon(a.min_delay, b.min_delay) && equals_epsilon(a.max_delay, b.max_delay);
    };
    auto equals_epsilon_constr = [&](ClockConstraint &a, ClockConstraint &b) {
        return equals_epsilon_pair(a.high, b.high) && equals_epsilon_pair(a.low, b.low) &&
               equals_epsilon_pair(a.period, b.period);
    };

    pool<IdString> user_constrained, changed_nets;
    for (auto &net : ctx->nets) {
        if (net.second->clkconstr != nullptr)
            user_constrained.insert(net.first);
        changed_nets.insert(net.first);
    }
    auto get_period = [&](CellInfo *ci, IdString port, delay_t &period) {
        if (!ci->ports.count(port))
            return false;
        NetInfo *from = ci->ports.at(port).net;
        if (from == nullptr || from->clkconstr == nullptr)
            return false;
        period = from->clkconstr->period.minDelay();
        return true;
    };

    auto simple_clk_contraint = [&](delay_t period) {
        auto constr = std::unique_ptr<ClockConstraint>(new ClockConstraint());
        constr->low = DelayPair(period / 2);
        constr->high = DelayPair(period / 2);
        constr->period = DelayPair(period);

        return constr;
    };

    auto set_constraint = [&](CellInfo *ci, IdString port, std::unique_ptr<ClockConstraint> constr) {
        if (!ci->ports.count(port))
            return;
        NetInfo *to = ci->ports.at(port).net;
        if (to == nullptr)
            return;
        if (to->clkconstr != nullptr) {
            if (!equals_epsilon_constr(*to->clkconstr, *constr) && user_constrained.count(to->name))
                log_warning("    Overriding derived constraint of %.1f MHz on net %s with user-specified constraint of "
                            "%.1f MHz.\n",
                            MHz(constr->period.min_delay), to->name.c_str(ctx), MHz(to->clkconstr->period.min_delay));
            return;
        }
        to->clkconstr = std::move(constr);
        log_info("    Derived frequency constraint of %.1f MHz for net %s\n", MHz(to->clkconstr->period.minDelay()),
                 to->name.c_str(ctx));
        changed_nets.insert(to->name);
    };

    auto copy_constraint = [&](CellInfo *ci, IdString fromPort, IdString toPort, double ratio = 1.0) {
        if (!ci->ports.count(fromPort) || !ci->ports.count(toPort))
            return;
        NetInfo *from = ci->ports.at(fromPort).net, *to = ci->ports.at(toPort).net;
        if (from == nullptr || from->clkconstr == nullptr || to == nullptr)
            return;
        if (to->clkconstr != nullptr) {
            if (!equals_epsilon(to->clkconstr->period.minDelay(),
                                delay_t(from->clkconstr->period.minDelay() / ratio)) &&
                user_constrained.count(to->name))
                log_warning("    Overriding derived constraint of %.1f MHz on net %s with user-specified constraint of "
                            "%.1f MHz.\n",
                            MHz(delay_t(from->clkconstr->period.minDelay() / ratio)), to->name.c_str(ctx),
                            MHz(to->clkconstr->period.minDelay()));
            return;
        }
        to->clkconstr = std::unique_ptr<ClockConstraint>(new ClockConstraint());
        to->clkconstr->low = DelayPair(ctx->getDelayFromNS(ctx->getDelayNS(from->clkconstr->low.min_delay) / ratio));
        to->clkconstr->high = DelayPair(ctx->getDelayFromNS(ctx->getDelayNS(from->clkconstr->high.min_delay) / ratio));
        to->clkconstr->period =
                DelayPair(ctx->getDelayFromNS(ctx->getDelayNS(from->clkconstr->period.min_delay) / ratio));
        log_info("    Derived frequency constraint of %.1f MHz for net %s\n", MHz(to->clkconstr->period.minDelay()),
                 to->name.c_str(ctx));
        changed_nets.insert(to->name);
    };

    // Run in a loop while constraints are changing to deal with dependencies
    // Iteration limit avoids hanging in crazy loopback situation (self-fed PLLs or dividers, etc)
    int iter = 0;
    const int itermax = 5000;
    while (!changed_nets.empty() && iter < itermax) {
        ++iter;
        pool<IdString> changed_cells;
        for (auto net : changed_nets) {
            for (auto &user : ctx->nets.at(net)->users)
                if (user.port.in(id_CLKIN1, id_I0, id_I1, id_I, id_PAD))
                    changed_cells.insert(user.cell->name);
        }
        changed_nets.clear();
        for (auto cell : changed_cells) {
            CellInfo *ci = ctx->cells.at(cell).get();
            if (ci->type == id_BUFGCTRL) {
                copy_constraint(ci, id_I0, id_O, 1);
                copy_constraint(ci, id_I1, id_O, 1);
            } else if (ci->type.in(id_BUFHCE_BUFHCE, id_BUFIO_BUFIO, id_BUFMRCE)) {
                copy_constraint(ci, id_I, id_O, 1);
            } else if (ci->type == id_BUFR_BUFR) {
                std::string div = str_or_default(ci->params, ctx->id("BUFR_DIVIDE"), "BYPASS");
                double ratio = 1.0;
                if (div != "BYPASS") {
                    try {
                        ratio = 1.0 / std::stod(div);
                    } catch (...) {
                        log_warning("    BUFR '%s': unrecognised BUFR_DIVIDE '%s', assuming BYPASS for the "
                                    "constraint\n",
                                    ci->name.c_str(ctx), div.c_str());
                    }
                }
                copy_constraint(ci, id_I, id_O, ratio);
            } else if (ci->type.in(id_IOB33M_INBUF_EN, id_IOB33S_INBUF_EN, id_IOB33_INBUF_EN, id_IOB18_INBUF_DCIEN,
                                   id_IOB18M_INBUF_DCIEN)) {
                copy_constraint(ci, id_PAD, id_OUT, 1);
            } else if (ci->type.in(id_MMCME2_ADV_MMCME2_ADV, id_PLLE2_ADV_PLLE2_ADV)) {
                delay_t period_in;
                if (!get_period(ci, id_CLKIN1, period_in))
                    continue;
                log_info("    Input frequency of PLL '%s' is constrained to %.1f MHz\n", ci->name.c_str(ctx),
                         MHz(period_in));
                double period_in_div = period_in * int_or_default(ci->params, id_DIVCLK_DIVIDE, 1);

                // The feedback net is normally driven by the PLL itself, but
                // Vivado's clocking wizard always puts a global buffer between
                // CLKFBOUT and CLKFBIN (BUFG clkf_buf), so follow clock buffers
                // back to the PLL's own output port instead of assuming a direct
                // connection: without this the whole PLL is skipped and the
                // design is timed at the default frequency.
                const NetInfo *clkfb = ci->getPort(id_CLKFBIN);
                if (clkfb == nullptr || clkfb->driver.cell == nullptr)
                    continue;
                CellInfo *fb_driver = clkfb->driver.cell;
                IdString fb_port = clkfb->driver.port;
                for (int hop = 0; fb_driver != ci && hop < 8; hop++) {
                    if (!fb_driver->type.in(id_BUFGCTRL, id_BUFG, id_BUFG_BUFG, id_BUFH, id_BUFHCE, id_BUFHCE_BUFHCE,
                                            id_BUFIO, id_BUFIO_BUFIO, id_BUFMRCE))
                        break;
                    NetInfo *upstream =
                            fb_driver->ports.count(id_I0) ? fb_driver->getPort(id_I0) : fb_driver->getPort(id_I);
                    if (upstream == nullptr && fb_driver->ports.count(id_I1))
                        upstream = fb_driver->getPort(id_I1);
                    if (upstream == nullptr || upstream->driver.cell == nullptr)
                        break;
                    fb_driver = upstream->driver.cell;
                    fb_port = upstream->driver.port;
                }
                if (fb_driver != ci)
                    continue;
                const std::string &clkfb_port = fb_port.str(ctx);
                double feedback_div = 0;
                if (clkfb_port == "CLKFBOUT") {
                    feedback_div = float_or_default(
                            ci, ci->type == id_MMCME2_ADV_MMCME2_ADV ? id_CLKFBOUT_MULT_F : id_CLKFBOUT_MULT, 1);
                } else {
                    if (clkfb_port.substr(0, 6) != "CLKOUT")
                        continue;
                    feedback_div = float_or_default(
                            ci,
                            ctx->idf("CLKOUT%s_DIVIDE%s", clkfb_port.substr(6).c_str(),
                                     (ci->type == id_MMCME2_ADV_MMCME2_ADV && clkfb_port.substr(6) == "0") ? "_F" : ""),
                            1);
                }

                double vco_period = period_in_div / feedback_div;
                double vco_freq = MHz(vco_period);
                log_info("    Derived VCO frequency %.1f MHz for PLL '%s'\n", vco_freq, ci->name.c_str(ctx));

                for (int i = 0; i <= 6; i++) {
                    auto port = ctx->idf("CLKOUT%d", i);
                    if (!ci->getPort(port))
                        continue;
                    set_constraint(
                            ci, port,
                            simple_clk_contraint(
                                    vco_period *
                                    float_or_default(
                                            ci,
                                            ctx->idf("CLKOUT%d_DIVIDE%s", i,
                                                     (ci->type == id_MMCME2_ADV_MMCME2_ADV && i == 0) ? "_F" : ""),
                                            1)));
                }
                // The MMCM's secondary (inverted) outputs CLKOUT0B..3B run at
                // their primary output's rate.
                if (ci->type == id_MMCME2_ADV_MMCME2_ADV) {
                    for (int i = 0; i <= 3; i++) {
                        auto port = ctx->idf("CLKOUT%dB", i);
                        if (!ci->getPort(port))
                            continue;
                        set_constraint(ci, port,
                                       simple_clk_contraint(vco_period *
                                                            float_or_default(
                                                                    ci,
                                                                    ctx->idf("CLKOUT%d_DIVIDE%s", i,
                                                                             (i == 0) ? "_F" : ""),
                                                                    1)));
                    }
                }
                // CLKFBOUT (and CLKFBOUTB on the MMCM) run at the VCO rate.
                set_constraint(ci, ctx->id("CLKFBOUT"), simple_clk_contraint(vco_period));
                if (ci->type == id_MMCME2_ADV_MMCME2_ADV)
                    set_constraint(ci, ctx->id("CLKFBOUTB"), simple_clk_contraint(vco_period));
            }
            // IBUFDS_GTE2 is the reference-clock buffer of a GTP/GTX pad.
            // A create_clock on that pad constrains the net on I (or IB);
            // the fabric clock is the buffer's O, or ODIV2 at half the rate.
            // The port name is taken from the cell so this does not intern a
            // new id on a design that has no such buffer.
            const bool is_ibufds_gte2 = ci->type == id_IBUFDS_GTE2;
            if (is_ibufds_gte2) {
                copy_constraint(ci, id_I, id_O, 1.0);
                copy_constraint(ci, id_IB, id_O, 1.0);
                IdString odiv2;
                bool have_odiv2 = false;
                for (auto const &p : ci->ports) {
                    const bool port_is_odiv2 = p.first.str(ctx) == "ODIV2";
                    if (port_is_odiv2) {
                        odiv2 = p.first;
                        have_odiv2 = true;
                    }
                }
                if (have_odiv2) {
                    // ratio is output/input frequency: 0.5 doubles the period.
                    copy_constraint(ci, id_I, odiv2, 0.5);
                    copy_constraint(ci, id_IB, odiv2, 0.5);
                }
            }
        }
    }

    // A clock with no constraint is timed at the default target and the
    // report says PASS at that frequency. Say so, once per net, while the
    // flip-flops still carry their clock on port C (pack_ffs runs later).
    const float target_mhz = ctx->setting<float>("target_freq") / 1e6f;
    pool<IdString> warned;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool is_fabric_ff =
                ci->type.in(id_FDRE, id_FDRE_1, id_FDCE, id_FDCE_1, id_FDPE, id_FDPE_1, id_FDSE, id_FDSE_1);
        if (!is_fabric_ff)
            continue;
        NetInfo *clk = ci->getPort(id_C);
        const bool clk_absent = clk == nullptr;
        if (clk_absent)
            continue;
        const bool already_constrained = clk->clkconstr != nullptr;
        if (already_constrained)
            continue;
        const bool already_warned = warned.count(clk->name) != 0;
        if (already_warned)
            continue;
        warned.insert(clk->name);
        log_warning("Clock net '%s' has no timing constraint and will be timed at the default %.2f MHz.\n",
                    clk->name.c_str(ctx), target_mhz);
    }
}
NEXTPNR_NAMESPACE_END
