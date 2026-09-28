/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *  Cell packing: IO, LUT, FF, distributed RAM, SRL, MUXF, CARRY4, constants.
 */

#include "slg47910.h"

#include <array>
#include <set>
#include <vector>

NEXTPNR_NAMESPACE_BEGIN

static int xilinx_lut_input_count(IdString type)
{
    if (type == id_INV)  return 1;
    if (type == id_LUT1) return 1;
    if (type == id_LUT2) return 2;
    if (type == id_LUT3) return 3;
    if (type == id_LUT4) return 4;
    if (type == id_LUT5) return 5;
    return 6;
}

// Raw LUT INIT: BC[0..2^n_inputs-1] is the truth table; higher BC bits in the
// slice are config (mode) bits, so any 64-bit expansion would corrupt them.
// Vendor-emitted designs only set the low-order BC bits that index actual
// LUT addresses, so emit the raw INIT and let the downstream pack stage write
// only the truth-table region.
static uint64_t raw_lut_init(const Property &init)
{
    return uint64_t(init.as_int64());
}

// Handle IO: remove nextpnr-inserted IO buffers, create SLG_IOB cells.
void SLG47910Impl::pack_io()
{
    log_info("slg47910: packing IO...\n");

    // Remove nextpnr auto-inserted buffers. The top-level port's net is the
    // same object as the ibuf/obuf net, so we just disconnect and remove.
    std::vector<IdString> to_remove;
    for (auto &[cname, ci] : ctx->cells) {
        if (!ci->type.in(ctx->id("$nextpnr_ibuf"), ctx->id("$nextpnr_obuf"), ctx->id("$nextpnr_iobuf")))
            continue;
        ci->disconnectPort(ctx->id("I"));
        ci->disconnectPort(ctx->id("O"));
        to_remove.push_back(cname);
    }
    for (auto &n : to_remove)
        ctx->cells.erase(n);
    log_info("slg47910: removed %zu nextpnr IO buffers\n", to_remove.size());

    // Create SLG_IOB / SLG_CLKIN cells for each top-level port.
    // Use directional ports so the router knows which direction to route:
    // input pads drive fabric nets (addOutput), output pads consume them (addInput).
    std::vector<std::pair<IdString, std::unique_ptr<CellInfo>>> new_cells;
    std::set<std::string> matched_io_constraints;
    std::set<std::string> matched_clock_constraints;
    int clock_cell_count = 0;

    for (auto &port : ctx->ports) {
        auto &pi = port.second;
        std::string port_name = port.first.str(ctx);
        auto io_it  = constrained_io_bels.find(port_name);
        auto clk_it = constrained_clock_bels.find(port_name);
        if ((io_it != constrained_io_bels.end() || clk_it != constrained_clock_bels.end()) && !pi.net)
            log_error("slg47910: constrained top-level port '%s' has no routable net\n", port_name.c_str());
        if (!pi.net) continue;

        if (clk_it != constrained_clock_bels.end()) {
            if (pi.type != PORT_IN)
                log_error("slg47910: dedicated clock port '%s' must be a top-level input\n",
                          port_name.c_str());
            const auto &cc = clk_it->second;
            auto cell = std::make_unique<CellInfo>(ctx, ctx->idf("$clk_%s", port.first.c_str(ctx)), id_SLG_CLKIN);
            cell->addOutput(id_PAD);
            cell->connectPort(id_PAD, pi.net);
            cell->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(cc.bel));
            cell->params[ctx->id("CLK_TOKEN")]  = Property(cc.token);
            cell->params[ctx->id("CLK_SIDE")]   = Property(cc.side);
            cell->params[ctx->id("CLK_DOMAIN")] = Property(cc.domain);
            matched_clock_constraints.insert(port_name);
            new_cells.emplace_back(cell->name, std::move(cell));
            clock_cell_count++;
            continue;
        }

        auto cell = std::make_unique<CellInfo>(ctx, ctx->idf("$io_%s", port.first.c_str(ctx)), id_SLG_IOB);
        if      (pi.type == PORT_IN)  cell->addOutput(id_PAD);
        else if (pi.type == PORT_OUT) cell->addInput(id_PAD);
        else                          cell->addInout(id_PAD);
        cell->connectPort(id_PAD, pi.net);
        if (io_it != constrained_io_bels.end()) {
            cell->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(io_it->second));
            matched_io_constraints.insert(port_name);
        }
        new_cells.emplace_back(cell->name, std::move(cell));
    }

    for (auto &[name, cell] : new_cells)
        ctx->cells[name] = std::move(cell);

    for (auto &[port_name, bel] : constrained_io_bels) {
        (void)bel;
        if (!matched_io_constraints.count(port_name))
            log_error("slg47910: pin constraint references unknown top-level port '%s'\n", port_name.c_str());
    }
    for (auto &[port_name, cc] : constrained_clock_bels) {
        (void)cc;
        if (!matched_clock_constraints.count(port_name))
            log_error("slg47910: clock constraint references unknown top-level port '%s'\n", port_name.c_str());
    }

    if (!matched_io_constraints.empty())
        log_info("slg47910: applied %zu IO BEL constraints\n", matched_io_constraints.size());
    if (!matched_clock_constraints.empty())
        log_info("slg47910: applied %zu dedicated clock-input constraints\n", matched_clock_constraints.size());
    log_info("slg47910: created %zu IOB cells and %d clock-input cells\n",
             new_cells.size() - clock_cell_count, clock_cell_count);
}

// Convert LUT1..LUT6 + INV → SLG_LUT.
void SLG47910Impl::pack_luts()
{
    log_info("slg47910: packing LUTs...\n");
    int count = 0;
    for (auto &[name, ci] : ctx->cells) {
        if (!ci->type.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4, id_LUT5, id_LUT6, id_INV))
            continue;

        int n_inputs = xilinx_lut_input_count(ci->type);

        bool is_inv = ci->type == id_INV;
        auto init_it = ci->params.find(id_INIT);
        ci->type = id_SLG_LUT;
        if (is_inv) {
            // INV: 1-input inverter truth table → INIT[0]=1, INIT[1]=0.
            ci->params[id_INIT] = Property(0x1ull, 64);
        } else if (init_it != ci->params.end()) {
            ci->params[id_INIT] = Property(raw_lut_init(init_it->second), 64);
        }

        // Phase 4 — LUT5/LUT6 wire split. Silicon distinguishes OX0
        // (LUT5_0 result, MUX_XQ bit 2 / MUX_X bit 2 / MUX_OXQ bit 2)
        // from X_RAW (LUT6 mux result, MUX_XQ bit 5 only). LUT cells
        // with ≤ 5 inputs produce OX0 directly (the LUT6 mux passes
        // OX0 unchanged when I5 is unused); cells with 6 inputs only
        // produce X_RAW correctly. emit.cc reads this tag to pick
        // bit 2 vs bit 5 in AQ1_SEL and to gate Phase 1 OAQ_SEL /
        // Phase 3 AMX_SEL paths that silicon routes through OX0.
        bool is_lut5 = (n_inputs <= 5) || is_inv;
        if (!is_lut5 && init_it != ci->params.end()) {
            // Even if the cell is declared as LUT6, the truth table may
            // be I5-independent (high 32 INIT bits == low 32); in that
            // case OX0 == X_RAW and the LUT5 path is silicon-correct.
            uint64_t raw = raw_lut_init(init_it->second);
            if ((raw >> 32) == (raw & 0xFFFFFFFFull))
                is_lut5 = true;
        }
        ci->params[ctx->id("LUT_MODE")] = is_lut5 ? Property("LUT5") : Property("LUT6");

        if (n_inputs == 1) {
            if      (ci->ports.count(ctx->id("A"))) ci->renamePort(ctx->id("A"), id_I0);
            else if (ci->ports.count(ctx->id("I"))) ci->renamePort(ctx->id("I"), id_I0);
        }
        for (auto &[pn, pi] : ci->ports) {
            if (!pn.in(id_I0, id_I1, id_I2, id_I3, id_I4, id_I5, id_O))
                log_warning("slg47910: LUT cell %s has unmapped port %s\n",
                            ci->name.c_str(ctx), pn.c_str(ctx));
        }
        count++;
    }
    log_info("slg47910: packed %d LUTs\n", count);
}

// Convert FDRE/FDSE/FDCE/FDPE → SLG_FF.
// IS_D_INVERTED is lowered by inserting an inverting LUT on the D path because
// the vendor fabric FFs have no hardware inversion support (RE probe: identical
// SDC/bitstream for IS_D_INVERTED=0 and IS_D_INVERTED=1 in the vendor flow).
void SLG47910Impl::pack_ffs()
{
    log_info("slg47910: packing FFs...\n");
    int count = 0, dinv_lowered = 0;
    std::vector<std::pair<IdString, std::unique_ptr<CellInfo>>> new_cells;

    for (auto &[name, ci] : ctx->cells) {
        if (!ci->type.in(id_FDRE, id_FDSE, id_FDCE, id_FDPE))
            continue;

        auto dinv_it = ci->params.find(id_IS_D_INVERTED);
        if (dinv_it != ci->params.end() && dinv_it->second.as_int64() != 0) {
            NetInfo *d_net = nullptr;
            auto d_it = ci->ports.find(id_D);
            if (d_it != ci->ports.end()) d_net = d_it->second.net;

            IdString inv_name = ctx->idf("%s$dinv", ci->name.c_str(ctx));
            auto inv_lut = std::make_unique<CellInfo>(ctx, inv_name, id_SLG_LUT);
            inv_lut->params[id_INIT] = Property(0x1ull, 64);

            IdString inv_net_name = ctx->idf("%s$dinv_o", ci->name.c_str(ctx));
            NetInfo *inv_net = ctx->createNet(inv_net_name);

            inv_lut->addInput(id_I0);
            inv_lut->addOutput(id_O);
            if (d_net) {
                ci->disconnectPort(id_D);
                inv_lut->connectPort(id_I0, d_net);
            }
            inv_lut->connectPort(id_O, inv_net);
            ci->connectPort(id_D, inv_net);

            ci->params.erase(dinv_it);
            new_cells.emplace_back(inv_lut->name, std::move(inv_lut));
            dinv_lowered++;
        }

        ci->params[ctx->id("FF_TYPE")] = Property(ci->type.str(ctx));
        ci->type = id_SLG_FF;

        // 8-FF model (Phase 1): bind the cell to a slot in {XQ, OXQ, ANY}.
        // Default ANY — the placer is free to use either FF bel. Users can
        // pin via (* SLG_FF_SLOT="XQ" *) / "OXQ" attribute carried through
        // from synthesis. isBelLocationValid enforces the slot when it is
        // bound to a specific bel.
        std::string slot = "ANY";
        auto attr_it = ci->attrs.find(ctx->id("SLG_FF_SLOT"));
        if (attr_it != ci->attrs.end()) {
            std::string raw = attr_it->second.as_string();
            if (raw == "XQ" || raw == "OXQ" || raw == "ANY")
                slot = raw;
        }
        ci->params[id_FF_SLOT] = Property(slot);

        if (ci->ports.count(ctx->id("C")))
            ci->renamePort(ctx->id("C"), id_CLK);

        // All SR-type control inputs unify onto port R for the shared bel pin.
        if (ci->ports.count(ctx->id("S")))   ci->renamePort(ctx->id("S"),   id_R);
        if (ci->ports.count(ctx->id("CLR"))) ci->renamePort(ctx->id("CLR"), id_R);
        if (ci->ports.count(ctx->id("PRE"))) ci->renamePort(ctx->id("PRE"), id_R);

        count++;
    }

    for (auto &[cname, cell] : new_cells)
        ctx->cells[cname] = std::move(cell);

    if (dinv_lowered)
        log_info("slg47910: packed %d FFs (%d IS_D_INVERTED lowered to LUT inverters)\n",
                 count, dinv_lowered);
    else
        log_info("slg47910: packed %d FFs\n", count);
}

// Cluster each SLG_LUT that drives exactly one SLG_FF into the same slice
// as its FF, so the placer uses the dedicated within-slice LUT.O → FF.D
// path instead of routing the data signal across CLBs through the SM
// fabric. Without this, single-FF designs (e.g. osc_probe) land with the
// LUT in one CLB and the FF in another, forcing inter-CLB routes that
// (a) stress the SM bit encoders (some long-range nodes are still
// unmappable in emit_route) and (b) cause $PACKER_VCC/GND/etc. distribution
// to traverse long routes that may also use unmappable nodes — net result
// being silicon-dead FFs even when the logical PnR looks correct.
//
// Conditions for clustering:
//   - both cells are SLG_LUT and SLG_FF (post pack_luts + pack_ffs)
//   - neither is already in a cluster (skips carry-mode LUTs)
//   - the LUT's output net has exactly one driver (the LUT) and exactly
//     one sink that is this FF's D pin
//
// Z-layout: LUTs at z_off+0..3, FFs at z_off+4..7. Relative cluster:
// LUT root at constr_z=0, FF child at constr_z=4. constr_abs_z=false so
// the placer can pick any slice A..D (and any site).
void SLG47910Impl::pack_lut_ff_clusters()
{
    log_info("slg47910: clustering single-fanout LUT→FF pairs...\n");
    int n_clustered = 0;

    // Debug placement override. force_ff_loc=<rbb_x>,<rbb_y> resolves to an L0
    // site via the placer-coord inversion of emit_sdc.site_to_rbb:
    //   sy = (rbb_x - 1) / 3, sx = (rbb_y - 2) / 2
    // and the resulting site's grid (x, y) is used to lock the cluster's LUT
    // root onto slice A. The FF child rides along via the existing relative
    // cluster constraints (LUT @ z=0, FF @ z=4).
    BelId forced_lut_bel, forced_ff_bel;
    if (force_ff_rbb_x >= 0 && force_ff_rbb_y >= 0) {
        if ((force_ff_rbb_x - 1) % 3 != 0 || (force_ff_rbb_y - 2) % 2 != 0)
            log_error("slg47910: force_ff_loc=%d,%d does not align with the RBB grid\n",
                      force_ff_rbb_x, force_ff_rbb_y);
        int target_sy = (force_ff_rbb_x - 1) / 3;
        int target_sx = (force_ff_rbb_y - 2) / 2;
        int target_x = -1, target_y = -1;
        for (auto &site : db.sites) {
            if (site.type != "L0") continue;
            if (site.sx == target_sx && site.sy == target_sy) {
                target_x = site.x;
                target_y = site.y;
                break;
            }
        }
        if (target_x < 0)
            log_error("slg47910: force_ff_loc=%d,%d resolves to L0 (sx=%d,sy=%d) which is not present in the chipdb\n",
                      force_ff_rbb_x, force_ff_rbb_y, target_sx, target_sy);
        forced_lut_bel = ctx->getBelByLocation(Loc(target_x, target_y, LUT_Z_BASE));
        forced_ff_bel  = ctx->getBelByLocation(Loc(target_x, target_y, FF_XQ_Z_BASE));
        if (forced_lut_bel == BelId() || forced_ff_bel == BelId())
            log_error("slg47910: force_ff_loc=%d,%d has no LUT/FF bel at grid (%d,%d)\n",
                      force_ff_rbb_x, force_ff_rbb_y, target_x, target_y);
        log_info("slg47910: force_ff_loc=%d,%d locked to grid (%d,%d) bels %s + %s\n",
                 force_ff_rbb_x, force_ff_rbb_y, target_x, target_y,
                 ctx->nameOfBel(forced_lut_bel), ctx->nameOfBel(forced_ff_bel));
    }

    for (auto &[name, ci] : ctx->cells) {
        CellInfo *ff = ci.get();
        if (ff->type != id_SLG_FF) continue;
        if (ff->cluster != ClusterId()) continue;

        auto d_it = ff->ports.find(id_D);
        if (d_it == ff->ports.end() || d_it->second.net == nullptr) continue;
        NetInfo *d_net = d_it->second.net;

        if (d_net->driver.cell == nullptr) continue;
        CellInfo *lut = d_net->driver.cell;
        if (lut->type != id_SLG_LUT) continue;

        // Phase 2.5: if the driver is a carry-mode LUT (already in a cluster
        // from pack_carry), extend that cluster with this FF so the placer
        // pins the FF to the SAME slice's XQ FF bel — the only way to
        // legalise the per-slice CRY_C/CRY_S → FFD pip and emit
        // AQ1_SEL=C_BIT/S_BIT. Without this, the placer scatters the FFs
        // across other CLBs and the router falls back to the longer
        // lut_out → fabric → cross-slice → FFD path (AQ1_SEL=XI).
        if (lut->cluster != ClusterId()) {
            if (!lut->params.count(ctx->id("CARRY_MODE")))
                continue;
            // Cluster root has its own cell entry; we constrain relative to
            // it. The carry root sits at the cluster's name (== root cell's
            // name) per pack_carry's setup.
            auto root_it = ctx->cells.find(lut->cluster);
            if (root_it == ctx->cells.end())
                continue;
            CellInfo *root = root_it->second.get();
            ff->cluster = lut->cluster;
            ff->constr_abs_z = false;
            ff->constr_z = lut->constr_z + FF_XQ_Z_BASE;  // same slice's XQ bel
            ff->constr_x = lut->constr_x;
            ff->constr_y = lut->constr_y;
            ff->params[id_FF_SLOT] = Property("XQ");
            root->constr_children.push_back(ff);
            n_clustered++;
            continue;
        }

        // LUT must drive exactly this FF.D — no other sinks, or the cluster
        // doesn't gain us the dedicated within-slice LUT→FF path (the LUT's
        // O would still need to fan out to the fabric for the other users,
        // which is fine, but the "single fanout" check is a conservative
        // proxy for whether colocation is actually beneficial vs forcing a
        // suboptimal placement).
        if (d_net->users.entries() != 1) continue;

        if (forced_lut_bel != BelId()) {
            // Skip cluster wiring: forcing both bels lands them in the same site
            // already and avoids the placer's "cluster with fixed root" path
            // (which crashes in placer_heap's chain-extent bookkeeping when the
            // root is a single-cell macro pinned to a specific bel).
            lut->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(forced_lut_bel));
            ff->attrs[ctx->id("BEL")]  = std::string(ctx->nameOfBel(forced_ff_bel));
        } else {
            lut->cluster = lut->name;
            lut->constr_abs_z = false;
            lut->constr_z = 0;

            ff->cluster = lut->name;
            ff->constr_abs_z = false;
            // Cluster only targets the XQ FF bel — the silicon's dedicated
            // LUT→FF lane (AQ1_SEL=X_RAW = 100000) routes the LUT result
            // directly into the XQ FF, with no equivalent path into OXQ.
            // OXQ FFs are reachable only by post-placement routing.
            ff->constr_z = FF_XQ_Z_BASE;  // 4
            ff->constr_x = 0;
            ff->constr_y = 0;
            // Pin the slot so the placer cannot land this FF in the OXQ
            // band (which lives at a different constr_z and would silently
            // break the cluster's dedicated-lane assumption).
            ff->params[id_FF_SLOT] = Property("XQ");

            lut->constr_children.push_back(ff);
        }
        n_clustered++;
    }

    log_info("slg47910: clustered %d LUT+FF pairs\n", n_clustered);
}


// Convert RAM32X1D → SLG_RAM32X1D.
// Lower RAM32M into a native two-half cluster of slice RAM cells matching the
// vendor pack/place shape: one 4-slice bitplane half per L1 site, with bitplane
// 1 in the horizontally adjacent L1 site.
void SLG47910Impl::pack_lutram()
{
    log_info("slg47910: packing distributed RAM...\n");
    int count = 0, lowered_ram32m = 0, lowered_ram32m_halves = 0, lowered_ram32m_cells = 0;
    std::vector<std::pair<IdString, std::unique_ptr<CellInfo>>> new_cells;
    std::vector<IdString> to_remove;

    static const IdString dpra_ports[]      = {id_DPRA0, id_DPRA1, id_DPRA2, id_DPRA3, id_DPRA4};
    static const IdString slice_ram_ports[] = {id_I0,    id_I1,    id_I2,    id_I3,    id_I4,    id_I5};

    auto finalise_sitewide_ram32x1d = [&](CellInfo *ram) {
        if (ram->type == id_RAM32X1D) {
            ram->params[ctx->id("RAM_TYPE")] = Property(ram->type.str(ctx));
            ram->type = id_SLG_RAM32X1D;
        } else {
            NPNR_ASSERT(ram->type == id_SLG_RAM32X1D);
        }

        bool has_dpra = false;
        for (int k = 0; k < 5; k++) {
            auto it = ram->ports.find(dpra_ports[k]);
            if (it != ram->ports.end() && it->second.net != nullptr)
                has_dpra = true;
        }
        ram->params[ctx->id("RAM_HAS_DPRA")] = Property(has_dpra ? 1 : 0);

        bool has_dpo = false;
        auto dpo_it = ram->ports.find(id_DPO);
        if (dpo_it != ram->ports.end() && dpo_it->second.net != nullptr)
            has_dpo = true;
        ram->params[ctx->id("RAM_HAS_DPO")] = Property(has_dpo ? 1 : 0);

        count++;
    };

    for (auto &[name, ci] : ctx->cells) {
        if (ci->type == id_RAM32M) {
            static const char memories[] = {'A', 'B', 'C', 'D'};
            std::array<std::array<uint32_t, 2>, 4> init_by_memory = {};
            std::string ram32m_group = ci->name.str(ctx);

            for (int mem_idx = 0; mem_idx < 4; mem_idx++) {
                char mem = memories[mem_idx];
                auto init_it = ci->params.find(ctx->idf("INIT_%c", mem));
                if (init_it == ci->params.end())
                    init_it = ci->params.find(ctx->idf("INIT%c", mem));
                if (init_it == ci->params.end())
                    continue;

                auto bits = init_it->second.extract(0, 64).as_bits();
                for (int addr = 0; addr < 32; ++addr)
                    for (int bit = 0; bit < 2; ++bit)
                        if (bits.at(addr * 2 + bit))
                            init_by_memory[mem_idx][bit] |= (1u << addr);
            }

            IdString root_name;
            CellInfo *root_cell = nullptr;
            for (int bit = 0; bit < 2; ++bit) {
                for (int mem_idx = 0; mem_idx < 4; ++mem_idx) {
                    char mem = memories[mem_idx];
                    IdString lowered_name = ctx->idf("%s$ram32m_b%d_%c", ci->name.c_str(ctx), bit, mem);
                    auto ram = std::make_unique<CellInfo>(ctx, lowered_name, id_SLG_RAM);
                    ram->attrs = ci->attrs;
                    ram->attrs.erase(ctx->id("BEL"));
                    ram->attrs.erase(ctx->id("LOC"));

                    ram->params[ctx->id("RAM_TYPE")]         = Property(std::string("RAM32M"));
                    ram->params[ctx->id("RAM32M_NATIVE")]    = Property(1);
                    ram->params[ctx->id("RAM32M_BITPLANE")]  = Property(bit);
                    ram->params[ctx->id("RAM32M_GROUP")]     = Property(ram32m_group);
                    ram->params[ctx->id("RAM32M_MEMORY")]    = Property(std::string(1, mem));
                    ram->params[id_INIT]                     = Property(int64_t(init_by_memory[mem_idx][bit]), 32);

                    ram->addInput(id_WCLK);
                    ram->addInput(id_WE);
                    for (int k = 0; k < 6; ++k) ram->addInput(slice_ram_ports[k]);
                    for (int k = 0; k < 5; ++k) ram->addInput(dpra_ports[k]);
                    ram->addOutput(id_SPO);

                    if (NetInfo *wclk = ci->getPort(id_WCLK))  ram->connectPort(id_WCLK, wclk);
                    if (NetInfo *we   = ci->getPort(id_WE))     ram->connectPort(id_WE,   we);
                    for (int k = 0; k < 5; ++k) {
                        if (NetInfo *wa   = ci->getPort(ctx->idf("ADDRD[%d]", k)))
                            ram->connectPort(slice_ram_ports[k], wa);
                        if (NetInfo *dpra = ci->getPort(ctx->idf("ADDR%c[%d]", mem, k)))
                            ram->connectPort(dpra_ports[k], dpra);
                    }
                    if (NetInfo *d = ci->getPort(ctx->idf("DI%c[%d]", mem, bit)))
                        ram->connectPort(id_I5, d);

                    IdString dout_port = ctx->idf("DO%c[%d]", mem, bit);
                    if (NetInfo *spo = ci->getPort(dout_port)) {
                        ci->disconnectPort(dout_port);
                        ram->connectPort(id_SPO, spo);
                    }

                    if (root_cell == nullptr) {
                        root_name = ram->name;
                        ram->cluster = root_name;
                        ram->constr_abs_z = false;
                        ram->constr_z = 0;
                        root_cell = ram.get();
                    } else {
                        ram->cluster = root_name;
                        ram->constr_abs_z = false;
                        ram->constr_z = mem_idx;
                        ram->constr_x = 0;
                        ram->constr_y = 2 * bit;
                        root_cell->constr_children.push_back(ram.get());
                    }

                    new_cells.emplace_back(ram->name, std::move(ram));
                    lowered_ram32m_cells++;
                    count++;
                }
                lowered_ram32m_halves++;
            }

            std::vector<IdString> old_ports;
            old_ports.reserve(ci->ports.size());
            for (auto &port : ci->ports) old_ports.push_back(port.first);
            for (IdString port : old_ports) ci->disconnectPort(port);

            to_remove.push_back(name);
            lowered_ram32m++;
            continue;
        }
        if (ci->type != id_RAM32X1D)
            continue;
        finalise_sitewide_ram32x1d(ci.get());
    }

    for (auto &[cname, cell] : new_cells) ctx->cells[cname] = std::move(cell);
    for (auto &cname : to_remove)          ctx->cells.erase(cname);

    if (lowered_ram32m) {
        log_info("slg47910: packed %d distributed RAM cells "
                 "(%d native RAM32X1D, %d native RAM32M slice cells across %d halves from %d RAM32M cells)\n",
                 count, count - lowered_ram32m_cells, lowered_ram32m_cells,
                 lowered_ram32m_halves, lowered_ram32m);
    } else {
        log_info("slg47910: packed %d distributed RAM cells\n", count);
    }
}

// Convert SRL16E → SLG_RAM. Remap ports for the memory CLB bel.
void SLG47910Impl::pack_srl()
{
    log_info("slg47910: packing SRL cells...\n");
    int count = 0;
    for (auto &[name, ci] : ctx->cells) {
        if (ci->type != id_SRL16E) continue;

        ci->params[ctx->id("RAM_TYPE")] = Property(ci->type.str(ctx));
        ci->type = id_SLG_RAM;

        // SRL16E port mapping:
        //   CLK → WCLK   (shared CLB clock)
        //   CE  → WE     (shift enable maps to write enable on RAMSRL)
        //   D   → I5     (data input)
        //   A0-A3 → I0-I3 (shift depth address, dedicated pins)
        //   Q   → SPO    (output)
        if (ci->ports.count(id_CLK))  ci->renamePort(id_CLK, id_WCLK);
        if (ci->ports.count(id_CE))   ci->renamePort(id_CE,  id_WE);
        if (ci->ports.count(id_D))    ci->renamePort(id_D,   id_I5);
        if (ci->ports.count(id_Q))    ci->renamePort(id_Q,   id_SPO);

        static const IdString a_ports[] = {id_A0, id_A1, id_A2, id_A3};
        static const IdString i_ports[] = {id_I0, id_I1, id_I2, id_I3};
        for (int k = 0; k < 4; k++)
            if (ci->ports.count(a_ports[k]))
                ci->renamePort(a_ports[k], i_ports[k]);

        count++;
    }
    log_info("slg47910: packed %d SRL cells\n", count);
}

// Convert MUXF7/MUXF8 → SLG_LUT (S → I2), with phase-5 promotion paths
// that bind qualifying cells to the silicon's mux_a7/mux_c7/mux_x8
// network so EXTRA_IN-source XQ FFs become reachable from open synth.
//
// Promotion forms cluster-constrained groupings:
//   MUXF7 (3-4 cells): I0_LUT @ z=0, I1_LUT @ z=1, MUXF7 @ z=25,
//                       optional FF child @ z=4 (same slice's XQ FF).
//   MUXF8 (7-8 cells): A_LUT @ z=0, B_LUT @ z=1, C_LUT @ z=2, D_LUT @ z=3,
//                       MUXF7_a7 @ z=25, MUXF7_c7 @ z=27, MUXF8 @ z=28,
//                       optional FF child @ z=5 (slice B's XQ FF —
//                       mux_x8 feeds slice B's EXTRA_IN).
// Cells that don't qualify (multi-fanout drivers, non-SLG_LUT inputs,
// pre-clustered drivers) fall back to MUXF7/MUXF8 → SLG_LUT lowering.
void SLG47910Impl::pack_muxes()
{
    log_info("slg47910: packing MUXF...\n");
    int n_lut = 0, n_muxf7 = 0, n_muxf8 = 0;

    auto get_port_net = [&](CellInfo *ci, IdString port) -> NetInfo * {
        auto it = ci->ports.find(port);
        if (it == ci->ports.end()) return nullptr;
        return it->second.net;
    };

    auto cluster_free_lut = [&](CellInfo *ci) -> bool {
        return ci && ci->type == id_SLG_LUT && ci->cluster == ClusterId();
    };

    auto fallback_to_lut = [&](CellInfo *ci) {
        ci->type = id_SLG_LUT;
        if (ci->ports.count(ctx->id("S")))
            ci->renamePort(ctx->id("S"), id_I2);
        n_lut++;
    };

    // Collect MUXF8 and MUXF7 cells separately. MUXF8 is processed first
    // so its driving MUXF7s are still raw cells (not yet promoted).
    std::vector<CellInfo *> muxf7s, muxf8s;
    for (auto &[name, ci] : ctx->cells) {
        if (ci->type == id_MUXF7) muxf7s.push_back(ci.get());
        else if (ci->type == id_MUXF8) muxf8s.push_back(ci.get());
    }

    // --- MUXF8 promotion ---
    for (CellInfo *ci : muxf8s) {
        NetInfo *i0_net = get_port_net(ci, id_I0);
        NetInfo *i1_net = get_port_net(ci, id_I1);
        NetInfo *o_net  = get_port_net(ci, id_O);
        CellInfo *muxf7_a = (i0_net && i0_net->driver.cell) ? i0_net->driver.cell : nullptr;
        CellInfo *muxf7_c = (i1_net && i1_net->driver.cell) ? i1_net->driver.cell : nullptr;
        if (!muxf7_a || !muxf7_c ||
            muxf7_a->type != id_MUXF7 || muxf7_c->type != id_MUXF7 ||
            muxf7_a->cluster != ClusterId() || muxf7_c->cluster != ClusterId() ||
            i0_net->users.entries() != 1 || i1_net->users.entries() != 1) {
            continue;  // fall back to SLG_LUT below
        }
        // mux_x8's O wire connects only to slice B's EXTRA_IN, then on
        // to slice B's XQ FF via the FFDMUX_EXTRA pip. The only legal
        // consumer is a single slice-local FF.D — anything else (LUT
        // inputs in other CLBs, multi-fanout to fabric) has no chipdb
        // route, so the placer would fail at routing time.
        CellInfo *ff_child = nullptr;
        if (o_net && o_net->users.entries() == 1) {
            PortRef &usr = *o_net->users.begin();
            if (usr.cell && usr.cell->type == id_SLG_FF && usr.port == id_D &&
                usr.cell->cluster == ClusterId())
                ff_child = usr.cell;
        }
        if (!ff_child) continue;
        NetInfo *aa_net = get_port_net(muxf7_a, id_I0);
        NetInfo *ab_net = get_port_net(muxf7_a, id_I1);
        NetInfo *ca_net = get_port_net(muxf7_c, id_I0);
        NetInfo *cb_net = get_port_net(muxf7_c, id_I1);
        CellInfo *lut_a = (aa_net && aa_net->driver.cell) ? aa_net->driver.cell : nullptr;
        CellInfo *lut_b = (ab_net && ab_net->driver.cell) ? ab_net->driver.cell : nullptr;
        CellInfo *lut_c = (ca_net && ca_net->driver.cell) ? ca_net->driver.cell : nullptr;
        CellInfo *lut_d = (cb_net && cb_net->driver.cell) ? cb_net->driver.cell : nullptr;
        if (!cluster_free_lut(lut_a) || !cluster_free_lut(lut_b) ||
            !cluster_free_lut(lut_c) || !cluster_free_lut(lut_d) ||
            aa_net->users.entries() != 1 || ab_net->users.entries() != 1 ||
            ca_net->users.entries() != 1 || cb_net->users.entries() != 1) {
            continue;
        }

        muxf7_a->type = id_SLG_MUX_F7;
        muxf7_c->type = id_SLG_MUX_F7;
        ci->type      = id_SLG_MUX_F8;
        // Cluster root = LUT A (slice A's LUT bel @ z=0).
        lut_a->cluster = lut_a->name;
        lut_a->constr_abs_z = false;
        lut_a->constr_z = 0;
        auto add_child = [&](CellInfo *child, int z) {
            child->cluster = lut_a->name;
            child->constr_abs_z = false;
            child->constr_z = z;
            lut_a->constr_children.push_back(child);
        };
        add_child(lut_b, 1);
        add_child(lut_c, 2);
        add_child(lut_d, 3);
        add_child(muxf7_a, MUXF7_Z_BASE);          // z=25 (slice A's mux_a7)
        add_child(muxf7_c, 2 + MUXF7_Z_BASE);      // z=27 (slice C's mux_c7)
        add_child(ci,      MUXF8_Z);               // z=28 (mux_x8)
        ff_child->params[id_FF_SLOT] = Property("XQ");
        add_child(ff_child, FF_XQ_Z_BASE + 1);     // z=5 (slice B's XQ FF)
        n_muxf8++;
    }

    // --- MUXF7 promotion (cells not already clustered by the MUXF8 pass) ---
    for (CellInfo *ci : muxf7s) {
        if (ci->cluster != ClusterId()) continue;  // already in a MUXF8 cluster
        NetInfo *i0_net = get_port_net(ci, id_I0);
        NetInfo *i1_net = get_port_net(ci, id_I1);
        NetInfo *o_net  = get_port_net(ci, id_O);
        CellInfo *i0_lut = (i0_net && i0_net->driver.cell) ? i0_net->driver.cell : nullptr;
        CellInfo *i1_lut = (i1_net && i1_net->driver.cell) ? i1_net->driver.cell : nullptr;
        if (!cluster_free_lut(i0_lut) || !cluster_free_lut(i1_lut) ||
            i0_net->users.entries() != 1 || i1_net->users.entries() != 1) {
            continue;
        }
        // mux_a7 / mux_c7's O wire connects only to the target slice's
        // EXTRA_IN, then on to the slice's XQ FF via FFDMUX_EXTRA. The
        // only legal consumer is a single slice-local FF.D — anything
        // else (LUT inputs in other CLBs, multi-fanout to fabric) has
        // no chipdb route.
        CellInfo *ff_child = nullptr;
        if (o_net && o_net->users.entries() == 1) {
            PortRef &usr = *o_net->users.begin();
            if (usr.cell && usr.cell->type == id_SLG_FF && usr.port == id_D &&
                usr.cell->cluster == ClusterId())
                ff_child = usr.cell;
        }
        if (!ff_child) continue;

        ci->type = id_SLG_MUX_F7;
        i0_lut->cluster = i0_lut->name;
        i0_lut->constr_abs_z = false;
        i0_lut->constr_z = 0;
        auto add_child = [&](CellInfo *child, int z) {
            child->cluster = i0_lut->name;
            child->constr_abs_z = false;
            child->constr_z = z;
            i0_lut->constr_children.push_back(child);
        };
        add_child(i1_lut, 1);
        add_child(ci, MUXF7_Z_BASE);
        ff_child->params[id_FF_SLOT] = Property("XQ");
        add_child(ff_child, FF_XQ_Z_BASE);  // same slice as LUT root
        n_muxf7++;
    }

    // --- Fallback: any MUXF7/MUXF8 cell left unpromoted → SLG_LUT ---
    for (auto &[name, cic] : ctx->cells) {
        CellInfo *ci = cic.get();
        if (ci->type.in(id_MUXF7, id_MUXF8))
            fallback_to_lut(ci);
    }

    log_info("slg47910: packed MUXF cells: %d as SLG_LUT, %d as SLG_MUX_F7, %d as SLG_MUX_F8\n",
             n_lut, n_muxf7, n_muxf8);
}

// Expand each CARRY4 into 4 SLG_LUT cells in carry mode, absorbing driving
// LUTs and building inter-CLB carry chains.
//
// SLG47910 carry architecture (datasheet Figure 7):
//   Each CLB has a hardwired 4-bit carry chain: CIN → A → B → C → D → COUT.
//   Per slice: LUT computes P[i] (propagate), G[i] = DI source.
//     CO[i] = P[i] ? C[i] : G[i]   (MUXCY — carry propagate/generate)
//     O[i]  = P[i] XOR C[i]         (XORCY — sum output)
//   MUX_xI with XOR=1 outputs XORCY (sum), XOR=0 outputs MUXCY (carry tap).
//   DI[i] routes through MUX_x6 (I5 input).
//
// Strategy:
//   1. Build chains by tracing CO[3] → CI connections.
//   2. For each slice, absorb the LUT driving S[i] into the carry LUT.
//   3. Inter-CLB carry uses dedicated COUT/CIN wires + pips.
//   4. All cells in a chain get placement constraints for vertical adjacency.
void SLG47910Impl::pack_carry()
{
    log_info("slg47910: packing CARRY4...\n");

    std::vector<CellInfo *> carry4s;
    for (auto &[name, ci] : ctx->cells)
        if (ci->type == id_CARRY4)
            carry4s.push_back(ci.get());
    if (carry4s.empty())
        return;

    auto get_bus_net = [&](CellInfo *ci, const char *base, int idx) -> NetInfo * {
        IdString port = ctx->idf("%s[%d]", base, idx);
        auto it = ci->ports.find(port);
        if (it == ci->ports.end() || it->second.net == nullptr) return nullptr;
        return it->second.net;
    };
    auto get_port_net = [&](CellInfo *ci, IdString port) -> NetInfo * {
        auto it = ci->ports.find(port);
        if (it == ci->ports.end()) return nullptr;
        return it->second.net;
    };

    // Map: net name → CARRY4 whose CO[3] drives it.
    dict<IdString, CellInfo *> co3_drivers;
    for (auto *ci : carry4s) {
        NetInfo *co3 = get_bus_net(ci, "CO", 3);
        if (co3) co3_drivers[co3->name] = ci;
    }

    // Build carry chains by tracing CO[3] → CI connections.
    pool<IdString> visited;
    std::vector<std::vector<CellInfo *>> chains;

    for (auto *ci : carry4s) {
        if (visited.count(ci->name)) continue;
        NetInfo *ci_net = get_port_net(ci, ctx->id("CI"));
        if (ci_net) {
            auto it = co3_drivers.find(ci_net->name);
            if (it != co3_drivers.end() && it->second != ci)
                continue; // not a chain root
        }

        std::vector<CellInfo *> chain;
        CellInfo *cur = ci;
        while (cur && !visited.count(cur->name)) {
            chain.push_back(cur);
            visited.insert(cur->name);
            NetInfo *co3 = get_bus_net(cur, "CO", 3);
            cur = nullptr;
            if (co3) {
                for (auto &usr : co3->users) {
                    if (usr.cell->type == id_CARRY4 && usr.port == ctx->id("CI")) {
                        cur = usr.cell;
                        break;
                    }
                }
            }
        }
        chains.push_back(std::move(chain));
    }

    // Check if a LUT driving S[i] can be absorbed into the carry cell.
    // Must be single-fanout LUT1-5 or INV.
    auto is_absorbable = [&](NetInfo *net) -> CellInfo * {
        if (!net || !net->driver.cell) return nullptr;
        CellInfo *drv = net->driver.cell;
        if (net->users.entries() != 1) return nullptr;
        if (drv->type.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4, id_LUT5, id_INV))
            return drv;
        return nullptr;
    };

    int total_carry = 0, absorbed_luts = 0;
    pool<IdString> cells_to_remove;

    for (auto &chain : chains) {
        IdString root_name;
        CellInfo *root_cell = nullptr;

        for (int ci_idx = 0; ci_idx < (int)chain.size(); ci_idx++) {
            CellInfo *carry = chain[ci_idx];

            NetInfo *carry_ci = get_port_net(carry, ctx->id("CI"));
            NetInfo *s_nets[4], *di_nets[4], *o_nets[4], *co_nets[4];
            for (int i = 0; i < 4; i++) {
                s_nets[i]  = get_bus_net(carry, "S",  i);
                di_nets[i] = get_bus_net(carry, "DI", i);
                o_nets[i]  = get_bus_net(carry, "O",  i);
                co_nets[i] = get_bus_net(carry, "CO", i);
            }

            // Check absorption BEFORE disconnecting (net user counts change).
            CellInfo *absorbable[4];
            for (int sl = 0; sl < 4; sl++)
                absorbable[sl] = is_absorbable(s_nets[sl]);

            for (auto &[pname, pinfo] : carry->ports)
                carry->disconnectPort(pname);

            CellInfo *slice_cells[4];
            for (int sl = 0; sl < 4; sl++) {
                IdString sl_name = ctx->idf("%s$c%c", carry->name.c_str(ctx), 'A' + sl);
                auto cell = std::make_unique<CellInfo>(ctx, sl_name, id_SLG_LUT);
                cell->params[ctx->id("CARRY_MODE")]  = Property(1);
                cell->params[ctx->id("CARRY_SLICE")] = Property(sl);

                CellInfo *s_lut = absorbable[sl];
                if (s_lut) {
                    auto init_it = s_lut->params.find(id_INIT);
                    if (s_lut->type == id_INV) {
                        cell->params[id_INIT] = Property(0x1ull, 64);
                    } else if (init_it != s_lut->params.end()) {
                        cell->params[id_INIT] = Property(raw_lut_init(init_it->second), 64);
                    } else {
                        cell->params[id_INIT] = Property(0ull, 64);
                    }

                    // Move input connections from absorbed LUT.
                    // LUT1-5 use A[0]-A[4] bus ports (Yosys), INV uses I.
                    auto move_lut_inputs = [&](CellInfo *src_lut, CellInfo *dst_cell) {
                        if (src_lut->type == id_INV) {
                            NetInfo *inv_in = get_port_net(src_lut, ctx->id("I"));
                            if (inv_in) {
                                dst_cell->addInput(id_I0);
                                src_lut->disconnectPort(ctx->id("I"));
                                dst_cell->connectPort(id_I0, inv_in);
                            }
                            return;
                        }
                        for (int k = 0; k < 5; k++) {
                            IdString aport = ctx->idf("A[%d]", k);
                            IdString iport = lut_pin_id(ctx, k);
                            NetInfo *net = get_port_net(src_lut, aport);
                            if (!net)
                                net = get_port_net(src_lut, iport);
                            if (!net) continue;
                            dst_cell->addInput(iport);
                            if (src_lut->ports.count(aport))
                                src_lut->disconnectPort(aport);
                            if (src_lut->ports.count(iport))
                                src_lut->disconnectPort(iport);
                            dst_cell->connectPort(iport, net);
                        }
                    };
                    move_lut_inputs(s_lut, cell.get());

                    s_lut->disconnectPort(ctx->id("O"));
                    cells_to_remove.insert(s_lut->name);
                    absorbed_luts++;
                } else {
                    // No absorption: route S[i] through I4 (feed-through LUT).
                    cell->params[id_INIT] = Property(0xFFFF0000FFFF0000ull, 64);
                    cell->addInput(id_I4);
                    if (s_nets[sl]) cell->connectPort(id_I4, s_nets[sl]);
                }

                // DI[i] → I5 (generate signal via MUX_x6).
                cell->addInput(id_I5);
                if (di_nets[sl]) cell->connectPort(id_I5, di_nets[sl]);

                // Phase 2.5: split CO[sl] / S[sl] FF users onto dedicated
                // CRY_C / CRY_S-driven nets so the router can capture them
                // through the per-slice CRY_C/CRY_S → FFD pip (AQ1_SEL=
                // C_BIT / S_BIT). Each cell port can only drive one net, so
                // we re-home the FF users on a brand-new cap net and leave
                // any remaining fabric users on the original net (still
                // driven via id_O → lut_out, or for slice D via id_COUT /
                // tap_net per the chain logic below). pack_lut_ff_clusters
                // then pins each FF to the same slice as its driving carry
                // LUT (via the cluster-extension path) so the dedicated pip
                // is reachable.
                //
                // Slice D coexists with the chain-continuation logic
                // because (a) the FF cap net is separate from co_nets[3]
                // (which keeps chain CI users); (b) the existing tap_net
                // split, when triggered, operates on remaining non-FF
                // logic users — FF users were already moved out.
                auto split_ff_users = [&](NetInfo *net, IdString cry_port, const char *cap_suffix) {
                    if (net == nullptr) return;
                    std::vector<PortRef> ff_users;
                    for (auto &usr : net->users) {
                        if (usr.cell->type.in(id_FDRE, id_FDSE, id_FDCE, id_FDPE)
                            && usr.port == id_D) {
                            ff_users.push_back(usr);
                        }
                    }
                    if (ff_users.empty()) return;
                    NetInfo *cap_net = ctx->createNet(
                        ctx->idf("%s$%s%d", carry->name.c_str(ctx), cap_suffix, sl));
                    for (auto &usr : ff_users) {
                        usr.cell->disconnectPort(usr.port);
                        usr.cell->connectPort(usr.port, cap_net);
                    }
                    cell->addOutput(cry_port);
                    cell->connectPort(cry_port, cap_net);
                };
                split_ff_users(co_nets[sl], id_CRY_C, "co_cap");
                split_ff_users(o_nets[sl],  id_CRY_S, "s_cap");

                // The slice has one fabric-visible output lane (lut_out via
                // id_O). In carry mode it can emit XORCY (sum) or MUXCY
                // (carry tap), not both at once — the Phase 2.5 split above
                // doesn't relax this, since fabric users still go through
                // id_O. Re-check user counts now that FFs have been moved.
                NetInfo *fabric_o_net = o_nets[sl];
                bool route_carry_on_o = false;
                if (sl < 3) {
                    bool co_has_users = (co_nets[sl] != nullptr && co_nets[sl]->users.entries() > 0);
                    bool s_has_users  = (o_nets[sl]  != nullptr && o_nets[sl]->users.entries() > 0);
                    if (!s_has_users)
                        fabric_o_net = nullptr;
                    if (co_has_users) {
                        if (s_has_users)
                            log_error("slg47910: CARRY4 %s uses both O[%d] and CO[%d] as fabric "
                                      "logic; slice %c has only one fabric-visible carry output "
                                      "lane (FF users are handled by the Phase 2.5 split)\n",
                                      carry->name.c_str(ctx), sl, sl, 'A' + sl);
                        fabric_o_net = co_nets[sl];
                        route_carry_on_o = true;
                    }
                }

                cell->addOutput(id_O);
                if (fabric_o_net) cell->connectPort(id_O, fabric_o_net);
                if (route_carry_on_o)
                    cell->params[ctx->id("CARRY_CO_OUTPUT")] = Property(1);

                // CO[3] from slice D: dedicated COUT port for chain continuation.
                if (sl == 3 && co_nets[3]) {
                    bool co3_is_chain = false, co3_has_logic_users = false;
                    for (auto &usr : co_nets[3]->users) {
                        if (usr.cell->type == id_CARRY4 && usr.port == ctx->id("CI"))
                            co3_is_chain = true;
                        else
                            co3_has_logic_users = true;
                    }

                    if (co3_is_chain) {
                        if (co3_has_logic_users && o_nets[3]) {
                            log_error("slg47910: CARRY4 %s CO[3] fans out to logic while also "
                                      "feeding the next carry CI, and O[3] is also in use; "
                                      "slice D cannot route both sum and carry simultaneously\n",
                                      carry->name.c_str(ctx));
                        }
                        if (co3_has_logic_users) {
                            // Split: COUT keeps chain users; O emits MUXCY to logic.
                            NetInfo *chain_net = co_nets[3];
                            IdString tap_name = ctx->idf("%s$co3_tap", carry->name.c_str(ctx));
                            NetInfo *tap_net = ctx->createNet(tap_name);

                            std::vector<PortRef> logic_users;
                            for (auto &usr : chain_net->users) {
                                if (usr.cell->type == id_CARRY4 && usr.port == ctx->id("CI")) continue;
                                logic_users.push_back(usr);
                            }
                            for (auto &usr : logic_users) {
                                usr.cell->disconnectPort(usr.port);
                                usr.cell->connectPort(usr.port, tap_net);
                            }

                            cell->addOutput(id_COUT);
                            cell->connectPort(id_COUT, chain_net);
                            cell->connectPort(id_O, tap_net);
                            cell->params[ctx->id("CARRY_CO_OUTPUT")] = Property(1);
                        } else {
                            cell->addOutput(id_COUT);
                            cell->connectPort(id_COUT, co_nets[3]);
                        }
                    } else if (!o_nets[3]) {
                        if (co3_has_logic_users)
                            cell->connectPort(id_O, co_nets[3]);
                        cell->params[ctx->id("CARRY_CO_OUTPUT")] = Property(1);
                    } else if (co3_has_logic_users) {
                        log_error("slg47910: CARRY4 %s uses both O[3] and CO[3] as logic; "
                                  "slice D has only one fabric-visible carry output lane\n",
                                  carry->name.c_str(ctx));
                    }
                }

                // Cluster constraints for vertical adjacency.
                if (ci_idx == 0 && sl == 0) {
                    root_name = sl_name;
                    cell->cluster = sl_name;
                    cell->constr_abs_z = false;
                    cell->constr_z = 0;
                    root_cell = cell.get();
                } else {
                    cell->cluster = root_name;
                    cell->constr_abs_z = false;
                    cell->constr_z = sl;
                    cell->constr_x = 2 * ci_idx;
                    cell->constr_y = 0;
                    root_cell->constr_children.push_back(cell.get());
                }

                slice_cells[sl] = cell.get();
                ctx->cells[sl_name] = std::move(cell);
            }

            // Slice A carry-in for inter-CLB chaining.
            if (ci_idx > 0 && carry_ci) {
                slice_cells[0]->addInput(id_CIN);
                slice_cells[0]->connectPort(id_CIN, carry_ci);
                slice_cells[0]->params[ctx->id("CARRY_HAS_CI")] = Property(1);
            } else if (ci_idx == 0 && carry_ci) {
                bool is_const = false;
                if (carry_ci->driver.cell) {
                    IdString drv_type = carry_ci->driver.cell->type;
                    if (drv_type == ctx->id("GND") || drv_type == ctx->id("VCC") ||
                        drv_type == ctx->id("$PACKER_GND") || drv_type == ctx->id("$PACKER_VCC"))
                        is_const = true;
                }
                if (!is_const &&
                    carry_ci->name != ctx->id("$PACKER_GND_NET") &&
                    carry_ci->name != ctx->id("$PACKER_VCC_NET")) {
                    slice_cells[0]->addInput(id_CIN);
                    slice_cells[0]->connectPort(id_CIN, carry_ci);
                    slice_cells[0]->params[ctx->id("CARRY_HAS_CI")] = Property(1);
                }
            }

            cells_to_remove.insert(carry->name);
            total_carry++;
        }
    }

    for (auto &name : cells_to_remove)
        ctx->cells.erase(name);

    log_info("slg47910: expanded %d CARRY4 into %d carry-mode LUTs "
             "(%d LUTs absorbed, %zu chains)\n",
             total_carry, total_carry * 4, absorbed_luts, chains.size());
}

// Replace VCC/GND constant drivers with all-1 / all-0 LUTs.
void SLG47910Impl::pack_constants()
{
    log_info("slg47910: packing constants...\n");
    const dict<IdString, Property> vcc_params = {{id_INIT, Property(0xFFFFFFFFFFFFFFFFull, 64)}};
    const dict<IdString, Property> gnd_params = {{id_INIT, Property(0x0ull, 64)}};
    h.replace_constants(
        CellTypePort(id_SLG_LUT, id_O),
        CellTypePort(id_SLG_LUT, id_O),
        vcc_params, gnd_params);
}

NEXTPNR_NAMESPACE_END
