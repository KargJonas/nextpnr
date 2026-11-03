/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2023  Myrtle Shah <gatecat@ds0.me>
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

#include "pack.h"
#include <algorithm>
#include <boost/optional.hpp>
#include <iterator>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "chain_utils.h"
#include "design_utils.h"
#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pins.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

// Process the contents of packed_cells
void XilinxPacker::flush_cells()
{
    for (auto pcell : packed_cells) {
        for (auto &port : ctx->cells[pcell]->ports) {
            ctx->cells[pcell]->disconnectPort(port.first);
        }
        ctx->cells.erase(pcell);
    }
    packed_cells.clear();
}

void XilinxPacker::xform_cell(const dict<IdString, XFormRule> &rules, CellInfo *ci)
{
    auto &rule = rules.at(ci->type);
    ci->attrs[id_X_ORIG_TYPE] = ci->type.str(ctx);
    ci->type = rule.new_type;
    std::vector<IdString> orig_port_names;
    for (auto &port : ci->ports)
        orig_port_names.push_back(port.first);

    for (auto pname : orig_port_names) {
        if (rule.port_multixform.count(pname)) {
            auto old_port = ci->ports.at(pname);
            ci->disconnectPort(pname);
            ci->ports.erase(pname);
            for (auto new_name : rule.port_multixform.at(pname)) {
                ci->ports[new_name].name = new_name;
                ci->ports[new_name].type = old_port.type;
                ci->connectPort(new_name, old_port.net);
                ci->attrs[ctx->id("X_ORIG_PORT_" + new_name.str(ctx))] = pname.str(ctx);
            }
        } else {
            IdString new_name;
            if (rule.port_xform.count(pname)) {
                new_name = rule.port_xform.at(pname);
            } else {
                std::string stripped_name;
                for (auto c : pname.str(ctx))
                    if (c != '[' && c != ']')
                        stripped_name += c;
                new_name = ctx->id(stripped_name);
            }
            if (new_name != pname) {
                ci->renamePort(pname, new_name);
            }
            ci->attrs[ctx->id("X_ORIG_PORT_" + new_name.str(ctx))] = pname.str(ctx);
        }
    }

    std::vector<IdString> xform_params;
    for (auto &param : ci->params)
        if (rule.param_xform.count(param.first))
            xform_params.push_back(param.first);
    for (auto param : xform_params)
        ci->params[rule.param_xform.at(param)] = ci->params[param];

    for (auto &attr : rule.set_attrs)
        ci->attrs[attr.first] = attr.second;

    for (auto &param : rule.set_params)
        ci->params[param.first] = param.second;
}

void XilinxPacker::generic_xform(const dict<IdString, XFormRule> &rules, bool print_summary)
{
    std::map<std::string, int> cell_count;
    std::map<std::string, int> new_types;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (rules.count(ci->type)) {
            cell_count[ci->type.str(ctx)]++;
            xform_cell(rules, ci);
            new_types[ci->type.str(ctx)]++;
        }
    }
    if (print_summary) {
        for (auto &nt : new_types) {
            log_info("    Created %d %s cells from:\n", nt.second, nt.first.c_str());
            for (auto &cc : cell_count) {
                if (rules.at(ctx->id(cc.first)).new_type != ctx->id(nt.first))
                    continue;
                log_info("        %6dx %s\n", cc.second, cc.first.c_str());
            }
        }
    }
}

CellInfo *XilinxPacker::feed_through_lut(NetInfo *net, const std::vector<PortRef> &feed_users)
{
    // Named after the pin fed, not a counter, so the name is the same in a
    // build of the same netlist with cells added (-o preplaced pins by name)
    const PortRef &first = feed_users.front();
    NetInfo *feedthru_net = ctx->createNet(ctx->idf("%s$legal$%s$%s", net->name.c_str(ctx), first.cell->name.c_str(ctx),
                                                    first.port.c_str(ctx)));
    CellInfo *lut = create_lut(stringf("%s$LUT$%s$%s", net->name.c_str(ctx), first.cell->name.c_str(ctx),
                                       first.port.c_str(ctx)),
                               {net}, feedthru_net, Property(2));

    for (auto &usr : feed_users) {
        usr.cell->disconnectPort(usr.port);
        usr.cell->connectPort(usr.port, feedthru_net);
    }

    return lut;
}

CellInfo *XilinxPacker::feed_through_muxf(NetInfo *net, IdString type, const std::vector<PortRef> &feed_users)
{
    const PortRef &first = feed_users.front();
    NetInfo *feedthru_net = ctx->createNet(ctx->idf("%s$legal$%s$%s", net->name.c_str(ctx), first.cell->name.c_str(ctx),
                                                    first.port.c_str(ctx)));
    CellInfo *mux = create_cell(type, ctx->idf("%s$MUX$%s$%s", net->name.c_str(ctx), first.cell->name.c_str(ctx),
                                               first.port.c_str(ctx)));
    mux->connectPort(id_I0, net);
    mux->connectPort(id_O, feedthru_net);
    mux->connectPort(id_S, ctx->nets[ctx->id("$PACKER_GND_NET")].get());

    for (auto &usr : feed_users) {
        usr.cell->disconnectPort(usr.port);
        usr.cell->connectPort(usr.port, feedthru_net);
    }

    return mux;
}

IdString XilinxPacker::int_name(IdString base, const std::string &postfix, bool is_hierarchy)
{
    return ctx->id(base.str(ctx) + (is_hierarchy ? "$subcell$" : "$intcell$") + postfix);
}

NetInfo *XilinxPacker::create_internal_net(IdString base, const std::string &postfix, bool is_hierarchy)
{
    IdString name = ctx->id(base.str(ctx) + (is_hierarchy ? "$subnet$" : "$intnet$") + postfix);
    return ctx->createNet(name);
}

void XilinxPacker::pack_luts()
{
    log_info("Packing LUTs..\n");

    dict<IdString, XFormRule> lut_rules;
    for (int k = 1; k <= 6; k++) {
        IdString lut = ctx->id("LUT" + std::to_string(k));
        lut_rules[lut].new_type = id_SLICE_LUTX;
        for (int i = 0; i < k; i++)
            lut_rules[lut].port_xform[ctx->id("I" + std::to_string(i))] = ctx->id("A" + std::to_string(i + 1));
        lut_rules[lut].port_xform[id_O] = id_O6;
    }
    lut_rules[id_LUT6_2] = XFormRule(lut_rules[id_LUT6]);
    generic_xform(lut_rules, true);
}

// A LUT6_2 drives two outputs (O5 and O6) and so needs two bels -- the 5LUT and
// the 6LUT of one slice -- but a nextpnr cell occupies exactly one bel.
// Inheriting the LUT6 transform rule only renames a port called "O", and LUT6_2
// has no such port, so O5 kept no bel pin and routing aborted with
// "No wire found for port O5".
//
// Split each LUT6_2 into two ordinary LUT cells, one per output. When both
// halves are five-input functions, constrain_lut6_2_pairs() puts them back on
// the 6LUT and 5LUT of a single site, so the LUT6_2 still costs one LUT.
std::vector<std::pair<IdString, IdString>> XilinxPacker::split_lut6_2()
{
    std::vector<std::pair<IdString, IdString>> pairs;
    std::vector<CellInfo *> to_split;
    for (auto &cell : ctx->cells)
        if (cell.second->type == id_LUT6_2)
            to_split.push_back(cell.second.get());
    if (to_split.empty())
        return pairs;

    for (CellInfo *ci : to_split) {
        NetInfo *o5 = ci->getPort(id_O5);
        NetInfo *o6 = ci->getPort(id_O6);
        NetInfo *i5 = ci->getPort(id_I5);
        Property init = get_or_default(ci->params, id_INIT, Property()).extract(0, 64);

        const bool both_outputs_used = (o5 != nullptr) && (o6 != nullptr);
        // A BEL pin names one physical resource for what is about to become two
        // independent cells. That is fine while only one output is used -- the
        // lone surviving half inherits it below -- but with both driven there
        // is no rule for which half should get it.
        const bool bel_constrained_with_both_outputs_used = (ci->attrs.count(id_BEL) != 0) && both_outputs_used;
        if (bel_constrained_with_both_outputs_used)
            log_error("LUT6_2 cell '%s' has a BEL constraint but drives both O5 and O6; splitting it in two "
                      "would leave that constraint ambiguous\n",
                      ci->name.c_str(ctx));

        // One LUT<n_in> cell driving `out`, its INIT the 2^n_in-bit slice of
        // the LUT6_2 INIT starting at bit `lo`.
        auto make_half = [&](const char *suffix, NetInfo *out, int n_in, int lo) {
            if (out == nullptr)
                return;
            IdString half_name = ctx->id(ci->name.str(ctx) + suffix);
            const bool half_name_collides = (ctx->cells.count(half_name) != 0);
            if (half_name_collides)
                log_error("splitting LUT6_2 cell '%s' would create cell '%s', which already exists\n",
                          ci->name.c_str(ctx), half_name.c_str(ctx));
            CellInfo *half = create_cell(ctx->idf("LUT%d", n_in), half_name);
            for (int i = 0; i < n_in; i++) {
                IdString p = ctx->idf("I%d", i);
                NetInfo *in = ci->getPort(p);
                const bool input_is_unused = (in == nullptr);
                if (input_is_unused)
                    continue;
                const bool half_lacks_this_input = (half->ports.count(p) == 0);
                if (half_lacks_this_input)
                    half->addInput(p);
                half->connectPort(p, in);
            }
            const bool half_lacks_an_output = (half->ports.count(id_O) == 0);
            if (half_lacks_an_output)
                half->addOutput(id_O);
            half->connectPort(id_O, out);
            half->params[id_INIT] = init.extract(lo, 1 << n_in);
            // A region is a bounding box and applies to both halves equally.
            half->region = ci->region;
            const bool bel_constrained = (ci->attrs.count(id_BEL) != 0);
            if (bel_constrained)
                half->attrs[id_BEL] = ci->attrs.at(id_BEL);
        };

        if (o5 != nullptr)
            ci->disconnectPort(id_O5);
        if (o6 != nullptr)
            ci->disconnectPort(id_O6);

        // O5 is always a five-input function of I0..I4, taken from INIT[31:0].
        make_half("$LUT5", o5, 5, 0);

        // O6 depends on I5. A constant I5 folds away and keeps O6 five-input,
        // so that both halves draw the same I0..I4 nets and can share a site.
        const bool i5_is_gnd = (i5 != nullptr) && (i5->name == ctx->id("$PACKER_GND_NET"));
        const bool i5_is_vcc = (i5 != nullptr) && (i5->name == ctx->id("$PACKER_VCC_NET"));
        const bool i5_is_constant = (i5 == nullptr) || i5_is_gnd || i5_is_vcc;
        // An unused or grounded I5 leaves O6 a five-input function of I0..I4;
        // a tied-high one takes the upper half of the INIT.
        const bool o6_is_a_five_input_function = (i5 == nullptr) || i5_is_gnd;
        if (o6_is_a_five_input_function)
            make_half("$LUT6", o6, 5, 0);
        else if (i5_is_vcc)
            make_half("$LUT6", o6, 5, 32);
        else
            make_half("$LUT6", o6, 6, 0);

        // Only a five-input O6 can share the site with the O5 half: a genuine
        // six-input O6 needs A6, which a 5LUT bel does not have.
        const bool the_two_halves_can_share_a_site = both_outputs_used && i5_is_constant;
        if (the_two_halves_can_share_a_site)
            pairs.emplace_back(ctx->id(ci->name.str(ctx) + "$LUT6"), ctx->id(ci->name.str(ctx) + "$LUT5"));

        packed_cells.insert(ci->name);
    }

    flush_cells();
    log_info("    split %d LUT6_2 cell(s) into LUT5/LUT6 pairs\n", int(to_split.size()));
    return pairs;
}

// Constrain each (lut6, lut5) pair onto the 6LUT and 5LUT of one site, so that
// a LUT6_2 still costs a single physical LUT.
//
// Runs after pack_luts(): that maps every LUT output to O6, but a 5LUT bel has
// only O5, so a 5LUT-constrained cell would have no legal site and placement
// would abort. Renaming the O5 half's output first is what makes the
// constraint placeable at all.
void XilinxPacker::constrain_lut6_2_pairs(const std::vector<std::pair<IdString, IdString>> &pairs)
{
    for (auto &p : pairs) {
        const bool pair_survived_packing = (ctx->cells.count(p.first) != 0) && (ctx->cells.count(p.second) != 0);
        if (!pair_survived_packing)
            continue;
        CellInfo *lut6 = ctx->cells.at(p.first).get();
        CellInfo *lut5 = ctx->cells.at(p.second).get();
        const bool pair_is_luts = (lut6->type == id_SLICE_LUTX) && (lut5->type == id_SLICE_LUTX);
        if (!pair_is_luts)
            continue;
        // Either half may already carry a constraint from an earlier pass, such
        // as constrain_muxf_tree(). Overwriting constr_parent would desync it
        // from the constr_children list it still sits in, so leave such a pair
        // unpaired rather than corrupt an existing constraint.
        // himbaechel names a cluster on its root and points the children at it,
        // so "already constrained" means a foreign cluster name, or a z/child
        // of its own.
        const bool lut6_already_constrained = (lut6->cluster != IdString() && lut6->cluster != lut6->name) ||
                                             lut6->constr_abs_z || !lut6->constr_children.empty();
        const bool lut5_already_constrained = (lut5->cluster != IdString() && lut5->cluster != lut5->name) ||
                                             lut5->constr_abs_z || !lut5->constr_children.empty();
        const bool pair_is_already_constrained = lut6_already_constrained || lut5_already_constrained;
        if (pair_is_already_constrained)
            continue;

        lut5->renamePort(id_O6, id_O5);
        lut5->attrs.erase(ctx->id("X_ORIG_PORT_O6"));
        lut5->attrs[ctx->id("X_ORIG_PORT_O5")] = std::string("O");

        // Both halves carry the same nets on A1..A5, which is what lets them
        // share the fractured LUT's input sitewires.
        const bool lut6_has_no_cluster_name = (lut6->cluster == IdString());
        if (lut6_has_no_cluster_name)
            lut6->cluster = lut6->name;
        lut5->cluster = lut6->name;
        lut6->constr_children.push_back(lut5);
        lut5->constr_x = 0;
        lut5->constr_y = 0;
        lut5->constr_abs_z = false;
        lut5->constr_z = BEL_5LUT - BEL_6LUT;
    }
}

void XilinxPacker::pack_ffs()
{
    log_info("Packing flipflops..\n");

    dict<IdString, XFormRule> ff_rules;
    ff_rules[id_FDCE].new_type = id_SLICE_FFX;
    ff_rules[id_FDCE].port_xform[id_C] = id_CK;
    ff_rules[id_FDCE].port_xform[id_CLR] = id_SR;
    // ff_rules[id_FDCE].param_xform[id_IS_CLR_INVERTED] = id_IS_SR_INVERTED;

    ff_rules[id_FDPE].new_type = id_SLICE_FFX;
    ff_rules[id_FDPE].port_xform[id_C] = id_CK;
    ff_rules[id_FDPE].port_xform[id_PRE] = id_SR;
    // ff_rules[id_FDPE].param_xform[id_IS_PRE_INVERTED] = id_IS_SR_INVERTED;

    ff_rules[id_FDRE].new_type = id_SLICE_FFX;
    ff_rules[id_FDRE].port_xform[id_C] = id_CK;
    ff_rules[id_FDRE].port_xform[id_R] = id_SR;
    ff_rules[id_FDRE].set_attrs.emplace_back(id_X_FFSYNC, "1");
    // ff_rules[id_FDRE].param_xform[id_IS_R_INVERTED] = id_IS_SR_INVERTED;

    ff_rules[id_FDSE].new_type = id_SLICE_FFX;
    ff_rules[id_FDSE].port_xform[id_C] = id_CK;
    ff_rules[id_FDSE].port_xform[id_S] = id_SR;
    ff_rules[id_FDSE].set_attrs.emplace_back(id_X_FFSYNC, "1");
    // ff_rules[id_FDSE].param_xform[id_IS_S_INVERTED] = id_IS_SR_INVERTED;

    ff_rules[id_FDCE_1] = XFormRule(ff_rules[id_FDCE]);
    ff_rules[id_FDCE_1].set_params.emplace_back(id_IS_C_INVERTED, 1);

    ff_rules[id_FDPE_1] = XFormRule(ff_rules[id_FDPE]);
    ff_rules[id_FDPE_1].set_params.emplace_back(id_IS_C_INVERTED, 1);

    ff_rules[id_FDRE_1] = XFormRule(ff_rules[id_FDRE]);
    ff_rules[id_FDRE_1].set_params.emplace_back(id_IS_C_INVERTED, 1);

    ff_rules[id_FDSE_1] = XFormRule(ff_rules[id_FDSE]);
    ff_rules[id_FDSE_1].set_params.emplace_back(id_IS_C_INVERTED, 1);

    generic_xform(ff_rules, true);
}

bool XilinxPacker::can_add_ff_to_cluster(const CellInfo *lut, const CellInfo *ff)
{
    bool found_wclk = false;
    bool found_ff = false;
    const NetInfo *clk = nullptr, *ce = nullptr, *sr = nullptr;
    bool is_clkinv = false, is_srinv = false, is_latch = false, is_ffsync = false;
    bool wclk_inv = false;
    CellInfo *base = ctx->cells.at(lut->cluster).get();

    auto process_cell = [&](const CellInfo *cell) {
        if (cell->constr_x != lut->constr_x || cell->constr_y != lut->constr_y)
            return;
        if (cell->type == id_SLICE_LUTX && cell->getPort(id_CLK)) {
            found_wclk = true;
            clk = cell->getPort(id_CLK);
            wclk_inv = bool_or_default(cell->params, id_IS_WCLK_INVERTED, false);
        }
        if (cell->type == id_SLICE_FFX) {
            found_ff = true;
            clk = cell->getPort(id_CK);
            ce = cell->getPort(id_CE);
            sr = cell->getPort(id_SR);
            is_clkinv = int_or_default(cell->params, id_IS_C_INVERTED, 0) == 1;
            is_srinv = bool_or_default(cell->params, id_IS_R_INVERTED, false) ||
                       bool_or_default(cell->params, id_IS_S_INVERTED, false) ||
                       bool_or_default(cell->params, id_IS_CLR_INVERTED, false) ||
                       bool_or_default(cell->params, id_IS_PRE_INVERTED, false);
            is_latch = cell->attrs.count(id_X_FF_AS_LATCH);
            is_ffsync = cell->attrs.count(id_X_FFSYNC);
        }
    };

    process_cell(base);
    for (auto child : base->constr_children)
        process_cell(child);

    if (!found_wclk && !found_ff)
        return true;
    if (ff->getPort(id_CK) != clk)
        return false;
    const bool ff_clock_inverted = int_or_default(ff->params, id_IS_C_INVERTED, 0) == 1;
    const bool ff_edge_is_not_wclk_edge = found_wclk && ff_clock_inverted != wclk_inv;
    if (ff_edge_is_not_wclk_edge)
        return false;
    if (!found_ff)
        return true;
    if (ff->getPort(id_CE) != ce)
        return false;
    if (ff->getPort(id_SR) != sr)
        return false;

    if (ff_clock_inverted != is_clkinv)
        return false;

    if ((bool_or_default(ff->params, id_IS_R_INVERTED, false) || bool_or_default(ff->params, id_IS_S_INVERTED, false) ||
         bool_or_default(ff->params, id_IS_CLR_INVERTED, false) ||
         bool_or_default(ff->params, id_IS_PRE_INVERTED, false)) != is_srinv)
        return false;

    if (ff->attrs.count(id_X_FF_AS_LATCH) != is_latch)
        return false;
    if (ff->attrs.count(id_X_FFSYNC) != is_ffsync)
        return false;
    return true;
}

void XilinxPacker::pack_lutffs()
{
    int pairs = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->cluster != ClusterId() || !ci->constr_children.empty())
            continue;
        if (ci->type != id_SLICE_FFX)
            continue;
        NetInfo *d = ci->getPort(id_D);
        const bool d_is_undriven = (d == nullptr || d->driver.cell == nullptr);
        if (d_is_undriven)
            continue; // an undriven D (yosys's 'x') is not a pairing
        if (d->driver.cell->type == id_SLICE_LUTX && d->driver.port == id_O6) {
            CellInfo *lut = d->driver.cell;
            if (lut->cluster != ClusterId() || !lut->constr_children.empty()) {
                if (d->users.entries() != 1)
                    continue; // fanout might not be packable
                if (!can_add_ff_to_cluster(lut, ci))
                    continue;
                CellInfo *base = ctx->cells.at(lut->cluster).get();
                base->constr_children.push_back(ci);
                ci->cluster = lut->cluster;
                ci->constr_x = lut->constr_x;
                ci->constr_y = lut->constr_y;
                ci->constr_abs_z = lut->constr_abs_z;
                ci->constr_z = lut->constr_z + (BEL_FF - BEL_6LUT);
                ++pairs;
            } else {
                // a shift register or memory LUT writes on its own clock, which the flipflop in the same
                // half-slice must share, edge included
                NetInfo *lut_clk = lut->getPort(id_CLK);
                const bool lut_is_clocked = lut_clk != nullptr;
                const bool ff_clock_is_not_lut_clock = lut_is_clocked && ci->getPort(id_CK) != lut_clk;
                const bool ff_clock_inverted = int_or_default(ci->params, id_IS_C_INVERTED, 0) == 1;
                const bool lut_clock_inverted = bool_or_default(lut->params, id_IS_WCLK_INVERTED, false);
                const bool ff_edge_is_not_lut_edge = lut_is_clocked && ff_clock_inverted != lut_clock_inverted;
                if (ff_clock_is_not_lut_clock || ff_edge_is_not_lut_edge)
                    continue;
                lut->constr_children.push_back(ci);
                lut->cluster = lut->name;
                ci->cluster = lut->name;
                ci->constr_x = 0;
                ci->constr_y = 0;
                ci->constr_z = (BEL_FF - BEL_6LUT);
                ++pairs;
            }
        } else if (d->driver.cell->type == id_CARRY4) {
            CellInfo *carry = d->driver.cell;
            if (carry->cluster != ClusterId() || !carry->constr_children.empty()) {
                auto port = d->driver.port.str(ctx);
                if (port.size() != 2 || port[0] != 'O')
                    continue;
                int z = std::stoi(port.substr(1));
                if (d->users.entries() != 1)
                    continue; // fanout might not be packable
                if (!can_add_ff_to_cluster(carry, ci))
                    continue;
                CellInfo *base = ctx->cells.at(carry->cluster).get();
                base->constr_children.push_back(ci);
                ci->cluster = carry->cluster;
                ci->constr_x = carry->constr_x;
                ci->constr_y = carry->constr_y;
                ci->constr_abs_z = carry->constr_abs_z;
                ci->constr_z = carry->constr_z + ((BEL_FF | (z << 4)) - BEL_CARRY4);
                ++pairs;
            }
        }
    }
    log_info("Constrained %d LUTFF pairs.\n", pairs);
}

bool XilinxPacker::is_constrained(const CellInfo *cell) { return cell->cluster != ClusterId(); }

void XilinxPacker::legalise_muxf_tree(CellInfo *curr, std::vector<CellInfo *> &mux_roots, pool<IdString> &seen_leaves)
{
    if (curr->type.str(ctx).substr(0, 3) == "LUT")
        return;
    for (IdString p : {id_I0, id_I1}) {
        NetInfo *pn = curr->getPort(p);
        if (pn == nullptr || pn->driver.cell == nullptr)
            continue;
        if (curr->type == id_MUXF7) {
            if (pn->driver.cell->type.str(ctx).substr(0, 3) != "LUT" || is_constrained(pn->driver.cell) ||
                seen_leaves.count(pn->driver.cell->name)) {
                PortRef pr;
                pr.cell = curr;
                pr.port = p;
                feed_through_lut(pn, {pr});
                continue;
            }
            seen_leaves.insert(pn->driver.cell->name);
        } else {
            IdString next_type;
            if (curr->type == id_MUXF9)
                next_type = id_MUXF8;
            else if (curr->type == id_MUXF8)
                next_type = id_MUXF7;
            else
                NPNR_ASSERT_FALSE("bad mux type");
            if (pn->driver.cell->type != next_type || is_constrained(pn->driver.cell) ||
                seen_leaves.count(pn->driver.cell->name) || bool_or_default(pn->driver.cell->attrs, id_MUX_TREE_ROOT)) {
                PortRef pr;
                pr.cell = curr;
                pr.port = p;
                feed_through_muxf(pn, next_type, {pr});
                continue;
            }
            seen_leaves.insert(pn->driver.cell->name);
        }
        legalise_muxf_tree(pn->driver.cell, mux_roots, seen_leaves);
    }
}

void XilinxPacker::constrain_muxf_tree(CellInfo *curr, CellInfo *base, int zoffset)
{
    if (curr->type == id_SLICE_LUTX && (curr->constr_abs_z || curr->cluster != ClusterId()))
        return;

    int base_z = 0;
    if (base->type == id_MUXF7)
        base_z = BEL_F7MUX;
    else if (base->type == id_MUXF8)
        base_z = BEL_F8MUX;
    else if (base->type == id_MUXF9)
        base_z = BEL_F9MUX;
    else if (base->constr_abs_z)
        base_z = base->constr_z;
    else
        NPNR_ASSERT_FALSE("unexpected mux base type");
    int curr_z = zoffset * 16;
    int input_spacing = 0;
    if (curr->type == id_MUXF7) {
        curr_z += BEL_F7MUX;
        input_spacing = 1;
    } else if (curr->type == id_MUXF8) {
        curr_z += BEL_F8MUX;
        input_spacing = 2;
    } else if (curr->type == id_MUXF9) {
        curr_z += BEL_F9MUX;
        input_spacing = 4;
    } else
        curr_z += BEL_6LUT;
    if (curr != base) {
        curr->constr_x = 0;
        curr->constr_y = 0;
        curr->constr_z = curr_z - base_z;
        curr->constr_abs_z = false;
        curr->cluster = base->name;
        base->constr_children.push_back(curr);
    }
    if (curr->type.in(id_MUXF7, id_MUXF8, id_MUXF9)) {
        NetInfo *i0 = curr->getPort(id_I0), *i1 = curr->getPort(id_I1);
        if (i0 != nullptr && i0->driver.cell != nullptr)
            constrain_muxf_tree(i0->driver.cell, base, zoffset + input_spacing);
        if (i1 != nullptr && i1->driver.cell != nullptr)
            constrain_muxf_tree(i1->driver.cell, base, zoffset);
    }
}

void XilinxPacker::pack_muxfs()
{
    log_info("Packing MUX[789]s..\n");
    std::vector<CellInfo *> mux_roots;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        ci->attrs.erase(id_MUX_TREE_ROOT);
        if (ci->type == id_MUXF9) {
            log_error("MUXF9 is not supported on xc7!\n");
        } else if (ci->type == id_MUXF8) {
            NetInfo *o = ci->getPort(id_O);
            if (o == nullptr || o->users.entries() != 1 || (*o->users.begin()).cell->type != id_MUXF9 ||
                is_constrained((*o->users.begin()).cell) || (*o->users.begin()).port == id_S)
                mux_roots.push_back(ci);
        } else if (ci->type == id_MUXF7) {
            NetInfo *o = ci->getPort(id_O);
            if (o == nullptr || o->users.entries() != 1 || (*o->users.begin()).cell->type != id_MUXF8 ||
                is_constrained((*o->users.begin()).cell) || (*o->users.begin()).port == id_S)
                mux_roots.push_back(ci);
        }
    }
    for (auto root : mux_roots)
        root->attrs[id_MUX_TREE_ROOT] = 1;
    pool<IdString> seen_leaves;
    for (auto root : mux_roots)
        legalise_muxf_tree(root, mux_roots, seen_leaves);
    for (auto root : mux_roots) {
        root->cluster = root->name;
        constrain_muxf_tree(root, root, 0);
    }
}

void XilinxPacker::finalise_muxfs()
{
    dict<IdString, XFormRule> muxf_rules;
    muxf_rules[id_MUXF9].new_type = id_F9MUX;
    muxf_rules[id_MUXF9].port_xform[id_I0] = id_0;
    muxf_rules[id_MUXF9].port_xform[id_I1] = id_1;
    muxf_rules[id_MUXF9].port_xform[id_S] = id_S0;
    muxf_rules[id_MUXF9].port_xform[id_O] = id_OUT;
    muxf_rules[id_MUXF8].new_type = id_SELMUX2_1;
    muxf_rules[id_MUXF8].port_xform = muxf_rules[id_MUXF9].port_xform;
    muxf_rules[id_MUXF7].new_type = id_SELMUX2_1;
    muxf_rules[id_MUXF7].port_xform = muxf_rules[id_MUXF9].port_xform;
    generic_xform(muxf_rules, true);
}

void XilinxPacker::pack_srls()
{
    dict<IdString, XFormRule> srl_rules;
    srl_rules[id_SRL16E].new_type = id_SLICE_LUTX;
    srl_rules[id_SRL16E].port_xform[id_CLK] = id_CLK;
    srl_rules[id_SRL16E].port_xform[id_CE] = id_WE;
    srl_rules[id_SRL16E].port_xform[id_D] = id_DI2;
    srl_rules[id_SRL16E].port_xform[id_Q] = id_O6;
    srl_rules[id_SRL16E].param_xform[id_IS_CLK_INVERTED] = id_IS_WCLK_INVERTED;
    srl_rules[id_SRL16E].set_attrs.emplace_back(id_X_LUT_AS_SRL, "1");

    srl_rules[id_SRLC32E].new_type = id_SLICE_LUTX;
    srl_rules[id_SRLC32E].port_xform[id_CLK] = id_CLK;
    srl_rules[id_SRLC32E].port_xform[id_CE] = id_WE;
    srl_rules[id_SRLC32E].port_xform[id_D] = id_DI1;
    srl_rules[id_SRLC32E].port_xform[id_Q] = id_O6;
    // Cascade shiftout: SRLC32E.Q31 is the dedicated MC31 output that feeds
    // the next SRL's DI mux (xDI1MUX <- (x+1)MC31).  Map it to the MC31 port
    // so the cascade routes through the dedicated in-slice path rather than
    // being dropped.  (Port of nextpnr-xilinx pack.cc.)
    srl_rules[id_SRLC32E].port_xform[id_Q31] = id_MC31;
    srl_rules[id_SRLC32E].param_xform[id_IS_CLK_INVERTED] = id_IS_WCLK_INVERTED;
    srl_rules[id_SRLC32E].set_attrs.emplace_back(id_X_LUT_AS_SRL, "1");
    generic_xform(srl_rules, true);
    // Fixup SRL inputs
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_SLICE_LUTX)
            continue;
        std::string orig_type = str_or_default(ci->attrs, id_X_ORIG_TYPE);
        if (orig_type == "SRL16E") {
            for (int i = 3; i >= 0; i--) {
                ci->renamePort(ctx->id("A" + std::to_string(i)), ctx->id("A" + std::to_string(i + 2)));
            }
            for (auto tp : {id_A1, id_A6}) {
                ci->ports[tp].name = tp;
                ci->ports[tp].type = PORT_IN;
                ci->connectPort(tp, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
            }
        } else if (orig_type == "SRLC32E") {
            for (int i = 4; i >= 0; i--) {
                ci->renamePort(ctx->id("A" + std::to_string(i)), ctx->id("A" + std::to_string(i + 2)));
            }
            for (auto tp : {id_A1}) {
                ci->ports[tp].name = tp;
                ci->ports[tp].type = PORT_IN;
                ci->connectPort(tp, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
            }
        }
    }
    constrain_srl_cascades();
}

// SRLC32E cascades (shift registers deeper than 32, chained through Q31).
// MC31 of the B, C and D LUTs only feeds the downward DI1 cascade
// (xDI1MUX <- (x+1)MC31). MC31 of the A LUT is the DOUTMUX input, so that
// one output can leave the slice. Two consequences, or the placer scatters
// the chain and the MC31 arc is physically unroutable:
//
//  - a cascade group of up to four SRLs must occupy D,C,B,A of ONE slice,
//    exactly like a carry chain (cluster, head at D);
//  - a Q31 link that cannot stay inside a slice leaves through DOUTMUX when
//    Q is also used, which means the cell has to sit on the A LUT. When Q
//    is unused the old rewrite stays: the link moves onto Q and the read
//    address is tied to 31.
//
// (Port of nextpnr-xilinx pack.cc constrain_srl_cascades(); the fork's
// constr_parent/constr_children grouping is re-expressed with the upstream
// cluster mechanism, exactly as pack_carry.cc does for carry chains: the
// head cell becomes the cluster root and the members its constr_children
// with absolute z placements D,C,B,A.)
void XilinxPacker::constrain_srl_cascades()
{
    auto is_srl32 = [&](const CellInfo *ci) {
        return ci->type == id_SLICE_LUTX && str_or_default(ci->attrs, id_X_ORIG_TYPE) == "SRLC32E";
    };
    NetInfo *vcc = ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get();
    auto reads_bit31 = [&](const CellInfo *ci) {
        for (auto a : {id_A2, id_A3, id_A4, id_A5, id_A6})
            if (ci->getPort(a) != vcc)
                return false;
        return true;
    };

    std::vector<CellInfo *> srls;
    std::unordered_map<CellInfo *, CellInfo *> next_srl, prev_srl;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!is_srl32(ci))
            continue;
        srls.push_back(ci);
        NetInfo *q31 = ci->getPort(id_MC31);
        bool q31_cascades_into_next_srl = q31 != nullptr && q31->users.entries() == 1 &&
                                          (*q31->users.begin()).port == id_DI1 &&
                                          is_srl32((*q31->users.begin()).cell);
        if (q31_cascades_into_next_srl) {
            next_srl[ci] = (*q31->users.begin()).cell;
            prev_srl[(*q31->users.begin()).cell] = ci;
        }
    }
    if (srls.empty())
        return;

    // Q31 links that must leave their slice, to be moved onto Q afterwards
    std::vector<CellInfo *> offslice;
    std::unordered_set<CellInfo *> visited;
    int clusters = 0;
    auto process_chain = [&](CellInfo *head) {
        if (prev_srl.count(head))
            return;
        std::vector<CellInfo *> chain;
        for (CellInfo *cur = head; cur != nullptr;) {
            chain.push_back(cur);
            visited.insert(cur);
            auto fnd = next_srl.find(cur);
            cur = (fnd == next_srl.end()) ? nullptr : fnd->second;
        }
        for (size_t g = 0; g < chain.size(); g += 4) {
            size_t glen = std::min<size_t>(4, chain.size() - g);
            if (glen > 1) {
                // Imported (BEL-pinned) chains were already placed legally;
                // adding constraints on top would fight the pin (cf. the
                // pinned-root lesson in pack_carries).  Likewise leave cells
                // an earlier pass (e.g. constrain_muxf_tree) already put in
                // a cluster alone: overwriting the cluster membership would
                // desync it from that cluster root's constr_children list.
                bool pinned = false;
                for (size_t i = 0; i < glen; i++)
                    pinned |= bool(chain[g + i]->attrs.count(id_BEL)) || chain[g + i]->cluster != ClusterId() ||
                              chain[g + i]->constr_abs_z || !chain[g + i]->constr_children.empty();
                if (!pinned) {
                    CellInfo *base = chain[g];
                    base->cluster = base->name;
                    base->constr_abs_z = true;
                    base->constr_z = (3 << 4) | BEL_6LUT; // head at D6LUT
                    for (size_t i = 1; i < glen; i++) {
                        CellInfo *m = chain[g + i];
                        m->cluster = base->name;
                        base->constr_children.push_back(m);
                        m->constr_x = 0;
                        m->constr_y = 0;
                        m->constr_abs_z = true;
                        m->constr_z = ((3 - int(i)) << 4) | BEL_6LUT; // then C, B, A
                    }
                    clusters++;
                }
            }
            CellInfo *tail = chain[g + glen - 1];
            if (next_srl.count(tail))
                offslice.push_back(tail);
        }
    };
    for (auto head : srls)
        process_chain(head);
    // A chain with no head is a pure Q31 cycle: some link has to go through
    // the fabric, and any of them may -- break the cycle at an arbitrary
    // element and cluster the rest as one open chain.
    for (auto ci : srls) {
        bool in_unvisited_cycle = !visited.count(ci) && next_srl.count(ci);
        if (!in_unvisited_cycle)
            continue;
        log_warning("SRL cell '%s' is part of a pure Q31 cascade cycle; breaking the cycle at its Q31 link\n",
                    ci->name.c_str(ctx));
        CellInfo *cycle_next = next_srl.at(ci);
        prev_srl.erase(cycle_next);
        next_srl.erase(ci);
        process_chain(cycle_next);
    }

    // Cells whose Q31 net is not one of the in-slice cascade links kept
    // above: multi-fanout Q31, a non-SRL consumer, or a group boundary.
    for (auto ci : srls) {
        NetInfo *q31 = ci->getPort(id_MC31);
        bool q31_leaves_slice = q31 != nullptr && !next_srl.count(ci);
        if (!q31_leaves_slice)
            continue;
        bool not_yet_listed = std::find(offslice.begin(), offslice.end(), ci) == offslice.end();
        if (not_yet_listed)
            offslice.push_back(ci);
    }

    int rewired = 0;
    int fabric = 0;
    for (auto ci : offslice) {
        NetInfo *q31 = ci->getPort(id_MC31);
        if (q31 == nullptr)
            continue;
        if (q31->users.empty()) {
            ci->disconnectPort(id_MC31);
            continue;
        }
        NetInfo *q = ci->getPort(id_O6);
        const bool q_output_is_used = q != nullptr && !q->users.empty();
        if (q_output_is_used) {
            const bool address_already_31 = reads_bit31(ci);
            if (address_already_31) {
                // Q already reads bit 31, so it carries the very value Q31
                // does: fold the off-slice Q31 consumers into the Q net.
                std::vector<PortRef> users;
                for (auto &user : q31->users)
                    users.push_back(user);
                for (auto &user : users) {
                    user.cell->disconnectPort(user.port);
                    user.cell->connectPort(user.port, q);
                }
                ci->disconnectPort(id_MC31);
                rewired++;
                continue;
            }
            // B/C/D MC31 cannot leave the slice. A cell already locked there,
            // or already a member of another cluster, still cannot reach
            // DOUTMUX without a placer change.
            const bool locked_above_a = ci->constr_abs_z && ((ci->constr_z >> 4) != 0);
            if (locked_above_a) {
                log_error("SRL '%s': its Q31 cascade must go through the fabric, which only an A LUT can do, "
                          "but the cell is already locked above the A LUT and Q is in use.\n",
                          ci->name.c_str(ctx));
            }
            const bool already_at_a = ci->constr_abs_z && ((ci->constr_z >> 4) == 0);
            const bool needs_the_a_lut = !already_at_a;
            if (needs_the_a_lut) {
                const bool already_in_a_cluster = ci->cluster != ClusterId() || !ci->constr_children.empty();
                if (already_in_a_cluster) {
                    log_error("SRL '%s': its Q31 cascade must go through the fabric, which only an A LUT can do, "
                              "but the cell is already in another cluster and Q is in use.\n",
                              ci->name.c_str(ctx));
                }
                ci->cluster = ci->name;
                ci->constr_abs_z = true;
                ci->constr_z = (0 << 4) | BEL_6LUT;
            }
            fabric++;
            continue;
        }
        ci->disconnectPort(id_MC31);
        if (q != nullptr)
            ci->disconnectPort(id_O6);
        if (!ci->ports.count(id_O6)) {
            ci->ports[id_O6].name = id_O6;
            ci->ports[id_O6].type = PORT_OUT;
        }
        ci->connectPort(id_O6, q31);
        for (auto a : {id_A2, id_A3, id_A4, id_A5, id_A6}) {
            ci->disconnectPort(a);
            if (!ci->ports.count(a)) {
                ci->ports[a].name = a;
                ci->ports[a].type = PORT_IN;
            }
            ci->connectPort(a, vcc);
        }
        rewired++;
    }
    const bool report_cascades = clusters || rewired || fabric;
    if (report_cascades) {
        log_info("Constrained %d SRL cascade group(s) into single slices, moved %d Q31 link(s) to Q[31]\n", clusters,
                 rewired);
        const bool any_fabric_q31 = fabric != 0;
        if (any_fabric_q31)
            log_info("Locked %d SRL Q31 output(s) to the A LUT so they leave through DOUTMUX\n", fabric);
    }
}

void XilinxPacker::pack_constants()
{
    log_info("Packing constants..\n");
    if (tied_pins.empty())
        get_tied_pins(ctx, tied_pins);
    if (invertible_pins.empty())
        get_invertible_pins(ctx, invertible_pins);
    if (!ctx->cells.count(ctx->id("$PACKER_GND_DRV"))) {
        CellInfo *gnd_cell = ctx->createCell(ctx->id("$PACKER_GND_DRV"), id_PSEUDO_GND);
        gnd_cell->addOutput(id_Y);
        NetInfo *gnd_net = ctx->createNet(ctx->id("$PACKER_GND_NET"));
        gnd_net->constant_value = id_GND;
        gnd_cell->connectPort(id_Y, gnd_net);

        CellInfo *vcc_cell = ctx->createCell(ctx->id("$PACKER_VCC_DRV"), id_PSEUDO_VCC);
        vcc_cell->addOutput(id_Y);
        NetInfo *vcc_net = ctx->createNet(ctx->id("$PACKER_VCC_NET"));
        vcc_net->constant_value = id_VCC;
        vcc_cell->connectPort(id_Y, vcc_net);
    }
    NetInfo *gnd = ctx->nets[ctx->id("$PACKER_GND_NET")].get(), *vcc = ctx->nets[ctx->id("$PACKER_VCC_NET")].get();

    std::vector<IdString> dead_nets;

    std::vector<std::tuple<CellInfo *, IdString, bool>> const_ports;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!tied_pins.count(ci->type))
            continue;
        auto &tp = tied_pins.at(ci->type);
        for (auto port : tp) {
            if (cell.second->ports.count(port.first) && cell.second->ports.at(port.first).net != nullptr &&
                cell.second->ports.at(port.first).net->driver.cell != nullptr)
                continue;
            const_ports.emplace_back(ci, port.first, port.second);
        }
    }

    for (auto &net : ctx->nets) {
        NetInfo *ni = net.second.get();
        if (ni->driver.cell != nullptr && ni->driver.cell->type == id_GND) {
            IdString drv_cell = ni->driver.cell->name;
            for (auto &usr : ni->users) {
                const_ports.emplace_back(usr.cell, usr.port, false);
                usr.cell->ports.at(usr.port).net = nullptr;
            }
            dead_nets.push_back(net.first);
            ctx->cells.erase(drv_cell);
        } else if (ni->driver.cell != nullptr && ni->driver.cell->type == id_VCC) {
            IdString drv_cell = ni->driver.cell->name;
            for (auto &usr : ni->users) {
                const_ports.emplace_back(usr.cell, usr.port, true);
                usr.cell->ports.at(usr.port).net = nullptr;
            }
            dead_nets.push_back(net.first);
            ctx->cells.erase(drv_cell);
        }
    }

    for (auto port : const_ports) {
        CellInfo *ci;
        IdString pname;
        bool cval;
        std::tie(ci, pname, cval) = port;

        if (!ci->ports.count(pname)) {
            ci->addInput(pname);
        }
        if (ci->ports.at(pname).net != nullptr) {
            // Case where a port with a default tie value is previously connected to an undriven net
            NPNR_ASSERT(ci->ports.at(pname).net->driver.cell == nullptr);
            ci->disconnectPort(pname);
        }

        // Do NOT exempt the CMT blocks from this.  I tried that, reasoning that
        // Vivado ties PLLE2_ADV.PWRDWN to GND uninverted while we produce
        // VCC+inversion, and that the two are only equivalent if the silicon
        // implements the pin inversion as prjxray models it.  It regressed
        // johnson+PLL, the one PLL design HW-verified through this flow: its
        // .RST(1'b0)/.PWRDWN(1'b0) rely on exactly this conversion, and its
        // working bitstream carries ZINV_RST and ZINV_PWRDWN as a result.
        // Skipping the conversion drops both bits and the PLL stops running.
        if (!cval && invertible_pins.count(ci->type) && invertible_pins.at(ci->type).count(pname)) {
            // Invertible pins connected to zero are optimised to a connection to Vcc (which is easier to route)
            // and an inversion
            ci->params[ctx->idf("IS_%s_INVERTED", pname.c_str(ctx))] = Property(1);
            cval = true;
        }

        ci->connectPort(pname, cval ? vcc : gnd);
    }

    for (auto dn : dead_nets) {
        ctx->nets.erase(dn);
    }
}

void XilinxPacker::rename_net(IdString old, IdString newname)
{
    std::unique_ptr<NetInfo> ni;
    std::swap(ni, ctx->nets[old]);
    ctx->nets.erase(old);
    ni->name = newname;
    ctx->nets[newname] = std::move(ni);
}

void XilinxPacker::tie_port(CellInfo *ci, const std::string &port, bool value, bool inv)
{
    IdString p = ctx->id(port);
    if (!ci->ports.count(p)) {
        ci->addInput(p);
    }
    if (value || inv)
        ci->connectPort(p, ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get());
    else
        ci->connectPort(p, ctx->nets.at(ctx->id("$PACKER_GND_NET")).get());
    if (!value && inv)
        ci->params[ctx->idf("IS_%s_INVERTED", port.c_str())] = Property(1);
}

// Whether REGCLKB of a block RAM is left without a net: Vivado does so when
// RAM_MODE says SDP, also on cells its DRC rejects, and the SDP transformation
// in pack_bram() packs a cell as SDP when port B writes the full width (36
// bits on a RAMB18E1, 72 on a RAMB36E1), with or without RAM_MODE.  The
// transformation decides how the cell is packed; this decides whether REGCLKB
// gets a net, so it looks at both.
static bool bram_regclkb_without_net(const Context *ctx, const CellInfo *ci)
{
    const bool sdp_by_mode = str_or_default(ci->params, ctx->id("RAM_MODE"), "TDP") == "SDP";
    const int full_width = (ci->type == id_RAMB36E1) ? 72 : 36;
    const bool sdp_by_width = int_or_default(ci->params, id_WRITE_WIDTH_B, 0) == full_width;
    return sdp_by_mode || sdp_by_width;
}

void XC7Packer::pack_bram()
{
    log_info("Packing BRAM..\n");

    // Rules for normal TDP BRAM
    dict<IdString, XFormRule> bram_rules;
    bram_rules[id_RAMB18E1].new_type = id_RAMB18E1_RAMB18E1;
    bram_rules[id_RAMB18E1].port_multixform[ctx->id("WEA[0]")] = {id_WEA0, id_WEA1};
    bram_rules[id_RAMB18E1].port_multixform[ctx->id("WEA[1]")] = {id_WEA2, id_WEA3};
    bram_rules[id_RAMB36E1].new_type = id_RAMB36E1_RAMB36E1;

    // Some ports have upper/lower bel pins in 36-bit mode
    std::vector<std::pair<IdString, std::vector<std::string>>> ul_pins;
    get_bram36_ul_pins(ctx, ul_pins);
    for (auto &ul : ul_pins) {
        for (auto &bp : ul.second)
            bram_rules[id_RAMB36E1].port_multixform[ul.first].push_back(ctx->id(bp));
    }
    bram_rules[id_RAMB36E1].port_multixform[ctx->id("ADDRARDADDR[15]")].push_back(id_ADDRARDADDRL15);
    bram_rules[id_RAMB36E1].port_multixform[ctx->id("ADDRBWRADDR[15]")].push_back(id_ADDRBWRADDRL15);

    // Special rules for SDP rules, relating to WE connectivity
    dict<IdString, XFormRule> sdp_bram_rules = bram_rules;
    for (int i = 0; i < 4; i++) {
        // Connects to two WEBWE bel pins
        sdp_bram_rules[id_RAMB18E1].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWE%d", i * 2));
        sdp_bram_rules[id_RAMB18E1].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWE%d", i * 2 + 1));
        // Not used in SDP mode
        sdp_bram_rules[id_RAMB18E1].port_multixform[ctx->idf("WEA[%d]", i)] = {};
    }

    for (int i = 0; i < 8; i++) {
        sdp_bram_rules[id_RAMB36E1].port_multixform[ctx->idf("WEBWE[%d]", i)].clear();
        // Connects to two WEBWE bel pins
        sdp_bram_rules[id_RAMB36E1].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWEL%d", i));
        sdp_bram_rules[id_RAMB36E1].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWEU%d", i));
        // Not used in SDP mode
        sdp_bram_rules[id_RAMB36E1].port_multixform[ctx->idf("WEA[%d]", i)] = {};
    }

    // 72-bit BRAMs: drop upper bits of WEB in TDP mode
    for (int i = 4; i < 8; i++)
        bram_rules[id_RAMB36E1].port_multixform[ctx->idf("WEBWE[%d]", i)] = {};

    // Drop constant ties on the dedicated CASCADE inputs of a BRAM that is not
    // cascading.  Whether the cascade is used is decided by the A_INPUT/B_INPUT
    // PARAMETERS (see fasm.cc, which emits A_INPUT[0] only for "CASCADE"), so a
    // constant on the pin carries no information.  It is not harmless though:
    // CASCADEINA has no path from general routing -- it reaches only another
    // BRAM's CASCADEOUT -- so leaving the tie makes the router try to deliver
    // $PACKER_VCC_NET to it and fail:
    //   Failed to route arc of net '$PACKER_VCC_NET', from X0Y0/VCC to
    //   RAMB36_X0Y0.CASCADEINA
    // yosys leaves these pins unconnected, so this only bites netlists that
    // spell the ties out -- Vivado's do, which is why importing a Vivado
    // netlist hit it and the yosys-synthesised equivalent never did.
    int casc_seen = 0, casc_dropped = 0, casc_bram = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        bool is_bram = ci->type == id_RAMB18E1 || ci->type == id_RAMB36E1;
        if (!is_bram)
            continue;
        ++casc_bram;
        for (auto &pin : {std::make_pair("CASCADEINA", "A_INPUT"),
                          std::make_pair("CASCADEINB", "B_INPUT")}) {
            bool port_uses_cascade = str_or_default(ci->params, ctx->id(pin.second), "DIRECT") == "CASCADE";
            if (port_uses_cascade)
                continue;
            NetInfo *n = ci->getPort(ctx->id(pin.first));
            if (n == nullptr)
                continue;
            ++casc_seen;
            log_info("  cascade tie: %s.%s net '%s' driver '%s'\n", ctx->nameOf(ci), pin.first,
                     ctx->nameOf(n),
                     n->driver.cell == nullptr ? "<none>" : n->driver.cell->type.c_str(ctx));
            // only a constant tie: a real cascade net is left alone
            // The constant network's drivers are PSEUDO_VCC/PSEUDO_GND at this
            // point, not VCC/GND -- checking the latter matched nothing.
            bool driven_by_logic = n->driver.cell != nullptr && n->driver.cell->type != id_PSEUDO_VCC &&
                                   n->driver.cell->type != id_PSEUDO_GND;
            if (driven_by_logic)
                continue;
            ci->disconnectPort(ctx->id(pin.first));
            ++casc_dropped;
        }
    }
    log_info("BRAM cascade scan: %d BRAM(s), %d tie(s) seen, %d dropped\n", casc_bram, casc_seen,
             casc_dropped);

    // The output registers (DOA_REG/DOB_REG) are clocked through site pins of
    // their own, REGCLKARDRCLK and REGCLKB (L and U on a RAMB36), which the
    // primitive has no port for.  prjxray feeds them from rows 4 and 0 of the
    // INT tiles beside the BRAM (the data clocks use rows 3 and 1), so with
    // nothing connected no clock reaches those rows and the register never
    // loads: the output holds its initial value.  Vivado drives them from the
    // port clocks: REGCLKARDRCLK from CLKARDCLK when DOA_REG is set, REGCLKB
    // from CLKBWRCLK when DOB_REG is set, except in SDP, where the read word
    // is registered from REGCLKARDRCLK and REGCLKB is left without a net.
    // The logical names are mapped to the L/U bel pins by get_bram36_ul_pins.
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool is_bram = ci->type.in(id_RAMB18E1, id_RAMB36E1);
        if (!is_bram)
            continue;
        const bool port_a_output_registered = bool_or_default(ci->params, id_DOA_REG, false);
        const bool port_b_output_registered = bool_or_default(ci->params, id_DOB_REG, false);
        const bool no_output_register = !port_a_output_registered && !port_b_output_registered;
        if (no_output_register)
            continue;
        auto connect_register_clock = [&](const char *regclk, IdString data_clock) {
            NetInfo *clk = ci->getPort(data_clock);
            const bool data_clock_unconnected = clk == nullptr;
            if (data_clock_unconnected)
                return;
            IdString port = ctx->id(regclk);
            ci->addInput(port);
            ci->connectPort(port, clk);
        };
        if (port_a_output_registered)
            connect_register_clock("REGCLKARDRCLK", id_CLKARDCLK);
        const bool regclkb_without_net = bram_regclkb_without_net(ctx, ci);
        const bool regclkb_used = port_b_output_registered && !regclkb_without_net;
        if (regclkb_used)
            connect_register_clock("REGCLKB", id_CLKBWRCLK);
    }

    // Process SDP BRAM first
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if ((ci->type == id_RAMB18E1 && int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) == 36) ||
            (ci->type == id_RAMB36E1 && int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) == 72))
            xform_cell(sdp_bram_rules, ci);
    }

    // Rewrite byte enables according to data width
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_RAMB18E1, id_RAMB36E1)) {
            for (char port : {'A', 'B'}) {
                int write_width = int_or_default(ci->params, ctx->idf("WRITE_WIDTH_%c", port), 18);
                int we_width;
                if (ci->type == id_RAMB36E1)
                    we_width = 4;
                else
                    we_width = (port == 'B') ? 4 : 2;
                if (write_width >= (9 * we_width))
                    continue;
                int used_we_width = std::max(write_width / 9, 1);
                std::string we_prefix = (port == 'B') ? "WEBWE[" : "WEA[";
                // The byte enables in use, read before any of them is rewritten
                std::vector<NetInfo *> used_we;
                for (int i = 0; i < used_we_width; i++)
                    used_we.push_back(ci->getPort(ctx->id(we_prefix + std::to_string(i) + "]")));
                // Port B of an 18Kb block takes each byte enable on a pair of site pins, the way
                // Vivado places them: in x18, WEBWE[0] on WEBWE0 and WEBWE1, WEBWE[1] on WEBWE2 and
                // WEBWE3 (the pins are named after WEBWE[i] one to one). Port A gets its pairs from
                // the WEA[0]/WEA[1] rules, and a 36Kb block repeats them (WE[i % used], also Vivado's).
                const bool we_in_pairs = ci->type == id_RAMB18E1 && port == 'B';
                for (int i = 0; i < we_width; i++) {
                    int src = we_in_pairs ? (i * used_we_width / we_width) : (i % used_we_width);
                    const bool already_there = (src == i);
                    if (already_there)
                        continue;
                    IdString curr_we = ctx->id(we_prefix + std::to_string(i) + "]");
                    const bool port_missing = !ci->ports.count(curr_we);
                    if (port_missing) {
                        ci->ports[curr_we].type = PORT_IN;
                        ci->ports[curr_we].name = curr_we;
                    }
                    ci->disconnectPort(curr_we);
                    ci->connectPort(curr_we, used_we.at(src));
                }
            }
        }
    }

    generic_xform(bram_rules, false);

    // These pins have no logical mapping, so must be tied after transformation
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_RAMB18E1_RAMB18E1) {
            int wwa = int_or_default(ci->params, id_WRITE_WIDTH_A, 0);
            for (int i = ((wwa == 0) ? 0 : 2); i < 4; i++) {
                IdString port = ctx->idf("WEA%d", i);
                if (!ci->ports.count(port)) {
                    ci->ports[port].name = port;
                    ci->ports[port].type = PORT_IN;
                    ci->connectPort(port, ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                }
            }
            int wwb = int_or_default(ci->params, id_WRITE_WIDTH_B, 0);
            if (wwb != 36) {
                for (int i = 4; i < 8; i++) {
                    IdString port = ctx->id("WEBWE" + std::to_string(i));
                    if (!ci->ports.count(port)) {
                        ci->ports[port].name = port;
                        ci->ports[port].type = PORT_IN;
                        ci->connectPort(port, ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                    }
                }
            }
            for (auto p : {id_ADDRATIEHIGH0, id_ADDRATIEHIGH1, id_ADDRBTIEHIGH0, id_ADDRBTIEHIGH1}) {
                if (!ci->ports.count(p)) {
                    ci->ports[p].name = p;
                    ci->ports[p].type = PORT_IN;
                } else {
                    ci->disconnectPort(p);
                }
                ci->connectPort(p, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
            }
        } else if (ci->type == id_RAMB36E1_RAMB36E1) {
            for (auto &a15 : {std::make_pair(id_ADDRARDADDRL15, "RAM_EXTENSION_A"),
                              std::make_pair(id_ADDRBWRADDRL15, "RAM_EXTENSION_B")}) {
                IdString p = a15.first;
                // Address bit 15 is unused by a RAMB36E1 on its own, and tied
                // high.  In a cascaded pair (RAM_EXTENSION LOWER/UPPER) it is
                // what picks the LOWER or the UPPER block (UG473), so it keeps
                // its net: tying it high would fold the 64K memory onto 32K.
                std::string extension = str_or_default(ci->params, ctx->id(a15.second), "NONE");
                const bool port_is_cascaded = extension == "LOWER" || extension == "UPPER";
                const bool a15_has_a_net = ci->getPort(p) != nullptr;
                if (port_is_cascaded && a15_has_a_net)
                    continue;
                if (!ci->ports.count(p)) {
                    ci->ports[p].name = p;
                    ci->ports[p].type = PORT_IN;
                } else {
                    ci->disconnectPort(p);
                }
                ci->connectPort(p, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
            }
            if (int_or_default(ci->params, id_WRITE_WIDTH_A, 0) == 1) {
                ci->disconnectPort(id_DIADI1);
                ci->connectPort(id_DIADI1, ci->getPort(id_DIADI0));
                ci->attrs[id_X_ORIG_PORT_DIADI1] = std::string("DIADI[0]");
                ci->disconnectPort(id_DIPADIP0);
                ci->disconnectPort(id_DIPADIP1);
            }
            if (int_or_default(ci->params, id_WRITE_WIDTH_B, 0) == 1) {
                ci->disconnectPort(id_DIBDI1);
                ci->connectPort(id_DIBDI1, ci->getPort(id_DIBDI0));
                ci->attrs[id_X_ORIG_PORT_DIBDI1] = std::string("DIBDI[0]");
                ci->disconnectPort(id_DIPBDIP0);
                ci->disconnectPort(id_DIPBDIP1);
            }
            // A width-9 port stores its ninth (parity) bit in the lower half at
            // even addresses and in the upper half at odd ones, and the upper
            // half takes it from DIPxDIP1. As with DIxDI1 at width 1, the data
            // has to be presented on both pins.
            if (int_or_default(ci->params, id_WRITE_WIDTH_A, 0) == 9) {
                ci->disconnectPort(id_DIPADIP1);
                ci->connectPort(id_DIPADIP1, ci->getPort(id_DIPADIP0));
                ci->attrs[ctx->id("X_ORIG_PORT_DIPADIP1")] = std::string("DIPADIP[0]");
            }
            if (int_or_default(ci->params, id_WRITE_WIDTH_B, 0) == 9) {
                ci->disconnectPort(id_DIPBDIP1);
                ci->connectPort(id_DIPBDIP1, ci->getPort(id_DIPBDIP0));
                ci->attrs[ctx->id("X_ORIG_PORT_DIPBDIP1")] = std::string("DIPBDIP[0]");
            }
            if (int_or_default(ci->params, id_WRITE_WIDTH_B, 0) != 72) {
                for (std::string s : {"L", "U"}) {
                    for (int i = 4; i < 8; i++) {
                        IdString port = ctx->idf("WEBWE%s%d", s.c_str(), i);
                        if (!ci->ports.count(port)) {
                            ci->ports[port].name = port;
                            ci->ports[port].type = PORT_IN;
                            ci->connectPort(port, ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                        }
                    }
                }
            } else {
                // Tie WEA low
                for (std::string s : {"L", "U"}) {
                    for (int i = 0; i < 4; i++) {
                        IdString port = ctx->idf("WEA%s%d", s.c_str(), i);
                        if (!ci->ports.count(port)) {
                            ci->ports[port].name = port;
                            ci->ports[port].type = PORT_IN;
                            ci->connectPort(port, ctx->nets[ctx->id("$PACKER_GND_NET")].get());
                        }
                    }
                }
            }
        }
    }

    constrain_bram_cascades();
}

// RAMB36E1 blocks joined by the data cascade (RAM_EXTENSION LOWER/UPPER: the
// LOWER's CASCADEOUTA/B drive the UPPER's CASCADEINA/B) have no path through
// general routing: the chipdb wires a CASCADEOUT to the CASCADEIN of exactly
// one other RAMB36, the one in the BRAM tile above.  Placed apart, the router
// fails on the first cascade net:
//   Failed to route arc 0.0 of net '...CAS_A', from X19Y109/RAMB36_X0Y0.CASCADEOUTA
//   to X19Y119/RAMB36_X0Y0.CASCADEINA
// So each chain becomes a cluster rooted at its first block.  The step up is
// 5 rows, or 6 across a clock-region or BRKH row, so constr_y is only the
// placer's estimate: XilinxImpl::getClusterPlacement() puts every next block
// on the bel the chipdb reaches from the previous one.
void XC7Packer::constrain_bram_cascades()
{
    // The RAMB36E1 that ci's cascade outputs drive, or nullptr.
    auto cascade_sink = [&](CellInfo *ci) -> CellInfo * {
        CellInfo *sink = nullptr;
        for (char port : {'A', 'B'}) {
            NetInfo *casc = ci->getPort(ctx->idf("CASCADEOUT%c", port));
            const bool cascade_out_unused = casc == nullptr;
            if (cascade_out_unused)
                continue;
            IdString cascade_in = ctx->idf("CASCADEIN%c", port);
            for (auto &usr : casc->users) {
                const bool user_is_a_cascade_input = usr.cell->type == id_RAMB36E1_RAMB36E1 && usr.port == cascade_in;
                if (!user_is_a_cascade_input)
                    log_error("%s.CASCADEOUT%c drives %s.%s, but a RAMB36E1 cascade output only reaches the "
                              "CASCADEIN%c of another RAMB36E1\n",
                              ctx->nameOf(ci), port, ctx->nameOf(usr.cell), usr.port.c_str(ctx), port);
                const bool drives_a_second_block = sink != nullptr && sink != usr.cell;
                if (drives_a_second_block)
                    log_error("the cascade outputs of %s drive two blocks, %s and %s\n", ctx->nameOf(ci),
                              ctx->nameOf(sink), ctx->nameOf(usr.cell));
                sink = usr.cell;
            }
        }
        return sink;
    };

    std::vector<CellInfo *> rams;
    pool<IdString> cascaded_into;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool is_ramb36 = ci->type == id_RAMB36E1_RAMB36E1;
        if (!is_ramb36)
            continue;
        rams.push_back(ci);
        CellInfo *sink = cascade_sink(ci);
        const bool cascades_into_another_block = sink != nullptr;
        if (cascades_into_another_block)
            cascaded_into.insert(sink->name);
    }

    int chains = 0;
    for (CellInfo *root : rams) {
        const bool starts_a_chain = !cascaded_into.count(root->name) && cascade_sink(root) != nullptr;
        if (!starts_a_chain)
            continue;
        root->cluster = root->name;
        root->constr_z = BEL_RAM36;
        root->constr_abs_z = true;
        int dy = 0;
        for (CellInfo *ci = cascade_sink(root); ci != nullptr; ci = cascade_sink(ci)) {
            const bool already_in_a_chain = ci->cluster != ClusterId();
            if (already_in_a_chain)
                log_error("the RAMB36E1 cascade from %s reaches %s twice\n", ctx->nameOf(root), ctx->nameOf(ci));
            dy -= 5;
            ci->cluster = root->name;
            ci->constr_x = 0;
            ci->constr_y = dy;
            ci->constr_z = BEL_RAM36;
            ci->constr_abs_z = true;
            root->constr_children.push_back(ci);
        }
        ++chains;
    }
    const bool found_any_chain = chains > 0;
    if (found_any_chain)
        log_info("Found %d RAMB36E1 cascade chain(s)\n", chains);
}

void XilinxPacker::pack_inverters()
{
    // Fold an inverter driving a PLL/MMCM control pin into that pin's
    // IS_<pin>_INVERTED parameter, which is what Vivado does.
    //
    // Without this, RTL such as .RST(~nrst) leaves the inverter as fabric logic
    // and IS_RST_INVERTED clear, so the PLL's reset arrives over general
    // interconnect; Vivado instead puts RST straight on the nrst net and sets
    // the parameter (which is why its bitstreams carry ZINV_RST).  HW on the
    // Sonata: the unfolded form never locked.  Note the FASM writer's
    // convention -- ZINV_<pin> SET means the pin IS inverted -- so folding is
    // what legitimately produces that bit; do not try to produce it by
    // negating the write instead (tried; it inverts RST and the PLL then runs
    // only while the reset button is held).
    //
    // Deliberately limited to the CMT blocks and to whole-signal pins: the
    // generic invertible_pins list includes bussed pins like OPMODE[0], whose
    // parameter is a vector rather than IS_<pin>_INVERTED, and folding those
    // needs bit-level handling this does not attempt.
    std::vector<IdString> dead_invs;
    int folded = 0;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!ci->type.in(id_PLLE2_ADV, id_PLLE2_BASE, id_MMCME2_ADV, id_MMCME2_BASE))
            continue;
        for (IdString pin : {id_RST, id_PWRDWN, id_CLKINSEL}) {
            NetInfo *n = ci->getPort(pin);
            bool pin_is_driven = n != nullptr && n->driver.cell != nullptr;
            if (!pin_is_driven)
                continue;
            CellInfo *drv = n->driver.cell;
            if (drv->type != id_INV)
                continue;
            NetInfo *src = drv->getPort(id_I);
            if (src == nullptr)
                continue;
            IdString param = ctx->idf("IS_%s_INVERTED", pin.c_str(ctx));
            bool cur = bool_or_default(ci->params, param, false);
            ci->disconnectPort(pin);
            ci->connectPort(pin, src);
            ci->params[param] = Property(cur ? 0 : 1, 1);
            ++folded;
            log_info("    folded inverter '%s' into %s.%s (%s=%d)\n", drv->name.c_str(ctx), ci->name.c_str(ctx),
                     pin.c_str(ctx), param.c_str(ctx), cur ? 0 : 1);
            if (n->users.empty())
                dead_invs.push_back(drv->name);
        }
    }
    for (IdString dn : dead_invs) {
        CellInfo *drv = ctx->cells.at(dn).get();
        for (auto &p : drv->ports)
            if (p.second.net != nullptr)
                drv->disconnectPort(p.first);
        ctx->cells.erase(dn);
    }
    if (folded > 0)
        log_info("    folded %d inverter(s) into CMT control pins\n", folded);

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_INV) {
            ci->params[id_INIT] = Property(1, 2);
            ci->renamePort(id_I, id_I0);
            ci->type = id_LUT1;
        }
    }
}

void XilinxImpl::pack()
{
    const ArchArgs &args = ctx->args;
    if (args.options.count("xdc")) {
        parse_xdc(args.options["xdc"].as<std::string>());
    }
    // The reference placement first for the cells that exist before packing
    // (prePlace pins the packer's own afterwards), so the packer's single-site
    // primitives leave a pinned cell's site alone.
    apply_preplaced(false);

    XC7Packer packer(ctx, this);
    packer.pack_constants();
    packer.pack_inverters();
    packer.pack_io();
    packer.prepare_clocking();
    packer.pack_constants();
    packer.raise_bufg_ignore1();
    packer.pack_iologic();
    packer.pack_idelayctrl();
    packer.pack_cfg();
    packer.pack_clocking();
    packer.pack_gt();
    packer.generate_constraints();
    packer.pack_muxfs();
    packer.pack_carries();
    packer.pack_srls();
    // Split LUT6_2 cells before the LUT transform: each half's INIT is a slice
    // of the LUT6_2's own INIT, and the pair constraint has to run after
    // pack_luts() has turned the halves into SLICE_LUTX cells.
    auto lut6_2_pairs = packer.split_lut6_2();
    packer.pack_luts();
    packer.constrain_lut6_2_pairs(lut6_2_pairs);
    packer.pack_dram();
    packer.pack_bram();
    packer.pack_dsps();
    packer.pack_ffs();
    packer.finalise_muxfs();
    packer.pack_lutffs();

    // Now that LUT-FF clusters exist, keep the sinks of every placed regional
    // buffer (BUFIO/BUFR) inside the clock region that buffer drives.  The
    // placer positions a cluster by its root, so the region must land on the
    // root -- walked inside constrain_regional_clock_sinks -- not on the
    // pre-cluster FF, which is why this runs after pack_lutffs() rather than
    // in constrain_bufios() alongside the bel binding.
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        const bool is_regional_buffer = ci->type == id_BUFIO_BUFIO || ci->type == id_BUFR_BUFR;
        const bool buffer_is_placed = ci->bel != BelId();
        if (is_regional_buffer && buffer_is_placed)
            packer.constrain_regional_clock_sinks(ci);
    }
}
NEXTPNR_NAMESPACE_END
