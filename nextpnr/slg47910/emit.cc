/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *  Post-route JSON export: placement (for emit_sdc.py) and routing (for emit_route.py).
 */

#include "slg47910.h"

#include <fstream>
#include <map>
#include <vector>

NEXTPNR_NAMESPACE_BEGIN

// Emit placement data as JSON for the downstream Python tools.
// Output sections: luts, ffs, rams, clocks, iobs.
void SLG47910Impl::emit_placement()
{
    log_info("slg47910: writing placement to %s\n", sdc_out.c_str());
    std::ofstream f(sdc_out);
    if (!f) { log_error("cannot write %s\n", sdc_out.c_str()); return; }

    // Collect placed clock-input cells first so FF/RAM emitters can look up
    // which clock net maps to which domain/token/side.
    struct ExportedClock {
        std::string cell_name, port_name, net_name, token, side;
        int domain = 0;
    };
    std::vector<ExportedClock> exported_clocks;
    std::map<std::string, ExportedClock> clocks_by_net;

    for (auto &[cname, ci] : ctx->cells) {
        if (ci->type != id_SLG_CLKIN) continue;
        if (ci->bel == BelId()) continue;

        ExportedClock clock;
        clock.cell_name = cname.str(ctx);
        clock.port_name = clock.cell_name;
        if (clock.port_name.rfind("$clk_", 0) == 0)
            clock.port_name = clock.port_name.substr(5);
        clock.net_name = connected_net_name(ci.get(), id_PAD);

        auto token_it  = ci->params.find(ctx->id("CLK_TOKEN"));
        auto side_it   = ci->params.find(ctx->id("CLK_SIDE"));
        auto domain_it = ci->params.find(ctx->id("CLK_DOMAIN"));
        if (token_it  != ci->params.end()) clock.token  = token_it->second.as_string();
        if (side_it   != ci->params.end()) clock.side   = side_it->second.as_string();
        if (domain_it != ci->params.end()) clock.domain = int(domain_it->second.as_int64());

        exported_clocks.push_back(clock);
        if (!clock.net_name.empty())
            clocks_by_net[clock.net_name] = clock;
    }

    auto lookup_clock = [&](const std::string &net_name) -> ExportedClock {
        auto it = clocks_by_net.find(net_name);
        return (it != clocks_by_net.end()) ? it->second : ExportedClock();
    };

    // --- LUTs ---
    f << "{\n\"luts\": [\n";
    bool first = true;
    for (auto &[cname, ci] : ctx->cells) {
        if (ci->type != id_SLG_LUT) continue;
        BelId bel = ci->bel;
        if (bel == BelId()) continue;
        auto it = bel_to_site.find(bel);
        if (it == bel_to_site.end()) continue;
        auto &site = db.sites[it->second];
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (site.type == "L1") ? 8 : 0;
        int slice = loc.z - z_off - LUT_Z_BASE;

        int64_t init = 0;
        auto ip = ci->params.find(id_INIT);
        if (ip != ci->params.end()) init = ip->second.as_int64();

        // Find which site input pin feeds each LUT input.
        int pin_map[6] = {-1, -1, -1, -1, -1, -1};
        for (int k = 0; k < LUT_K; k++) {
            WireId luti = ctx->getBelPinWire(bel, lut_pin_id(ctx, k));
            if (luti == WireId()) continue;
            for (auto pip : ctx->getPipsUphill(luti)) {
                if (ctx->getBoundPipNet(pip) == nullptr) continue;
                WireId src = ctx->getPipSrcWire(pip);
                auto pit = wire_to_site_pin.find(src);
                if (pit != wire_to_site_pin.end())
                    pin_map[k] = pit->second;
                break;
            }
        }

        bool carry_mode   = false;
        bool carry_has_ci = false;
        auto cm  = ci->params.find(ctx->id("CARRY_MODE"));
        auto hci = ci->params.find(ctx->id("CARRY_HAS_CI"));
        if (cm  != ci->params.end() && cm->second.as_int64())  carry_mode   = true;
        if (hci != ci->params.end() && hci->second.as_int64()) carry_has_ci = true;

        if (!first) f << ",\n";
        first = false;
        f << "  {\"name\":\""     << json_escape(cname.str(ctx))
          << "\",\"site_type\":\"" << json_escape(site.type)
          << "\",\"sx\":"         << site.sx << ",\"sy\":" << site.sy
          << ",\"slice\":"        << slice << ",\"init\":" << (uint64_t)init
          << ",\"pins\":["        << pin_map[0] << "," << pin_map[1]
          << ","                  << pin_map[2] << "," << pin_map[3]
          << ","                  << pin_map[4] << "," << pin_map[5] << "]";
        if (carry_mode)   f << ",\"carry_mode\":true";
        if (carry_has_ci) f << ",\"carry_has_ci\":true";
        {
            auto cco = ci->params.find(ctx->id("CARRY_CO_OUTPUT"));
            if (cco != ci->params.end() && cco->second.as_int64())
                f << ",\"carry_co_output\":true";
        }
        f << "}";
    }

    // --- FFs ---
    // Per-slice slice-mux state, accumulated alongside the FF emit so we can
    // emit the "slice_muxes" section after this loop without re-traversing
    // ctx->cells. Keyed by (site_type, sx, sy, slice). Values track whether
    // any FF (XQ or OXQ) is bound on this slice — used to decide whether to
    // emit the AMX_SEL=00010 / OAQ_SEL=010 vendor-default for the slice.
    struct SliceMuxState {
        std::string site_type;
        int sx = 0, sy = 0, slice = 0;
        bool has_oxq_ff = false;   // OXQ FF bound → AMX_SEL=01 (OXQ source)
        bool has_xq_ff  = false;   // XQ  FF bound → emit AQ1_SEL field
        bool has_any_ff = false;   // any FF on this slice (XQ or OXQ)
        int oaq_sel = 2;           // default 010 (XQ source) — vendor default
        // Phase 2: per-slice AQ1_SEL value derived from the XQ FF's D-input
        // source. One-hot 6-bit field for slices A/B/C, 5-bit for slice D
        // (no EXTRA_IN). Vendor default = 100000 (X_RAW, bit 5).
        //   bit 0: EXTRA_IN (A/B/C only)   bit 1: XI (site input, inverted)
        //   bit 2: OX0 (Phase 4 — emitted when LUT source is LUT5 mode)
        //   bit 3: C_BIT                   bit 4: S_BIT
        //   bit 5: X_RAW (vendor default — LUT6 mode)
        int aq1_sel = 32;          // default 100000 (X_RAW)
        // Phase 3: AMX_SEL value derived from the slice's lut_out fanning
        // out to the xMUX site-output pin (OMUX pip is bound). Only fires
        // when no OXQ FF is bound on this slice (the two share the xMUX
        // wire and are mutually exclusive at the router level).
        bool has_phase3_amx = false;
        int amx_sel_phase3 = 0;
    };
    std::map<std::tuple<std::string, int, int, int>, SliceMuxState> slice_mux_state;

    f << "\n],\n\"ffs\": [\n";
    first = true;
    for (auto &[cname, ci] : ctx->cells) {
        if (ci->type != id_SLG_FF) continue;
        BelId bel = ci->bel;
        if (bel == BelId()) continue;
        auto it = bel_to_site.find(bel);
        if (it == bel_to_site.end()) continue;
        auto &site = db.sites[it->second];
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (site.type == "L1") ? 8 : 0;
        int dz = loc.z - z_off;

        // 8-FF model: classify the bel into (slot, slice). XQ band at
        // FF_XQ_Z_BASE..+4; OXQ band at FF_OXQ_Z_BASE..+4. Fall back to the
        // XQ math if the Z somehow lands outside both bands (shouldn't happen).
        std::string slot;
        int slice;
        if (dz >= FF_OXQ_Z_BASE && dz < FF_OXQ_Z_BASE + SLICES_PER_CLB) {
            slot  = "OXQ";
            slice = dz - FF_OXQ_Z_BASE;
        } else {
            slot  = "XQ";
            slice = dz - FF_XQ_Z_BASE;
        }

        std::string ff_type;
        auto type_it = ci->params.find(ctx->id("FF_TYPE"));
        if (type_it != ci->params.end()) ff_type = type_it->second.as_string();

        int64_t ff_init = 0;
        auto init_it = ci->params.find(id_INIT);
        if (init_it != ci->params.end()) ff_init = init_it->second.as_int64();

        std::string clk_net = connected_net_name(ci.get(), id_CLK);
        ExportedClock clock = lookup_clock(clk_net);
        int d_pin   = trace_site_input_pin(ctx->getBelPinWire(bel, id_D));
        int clk_pin = trace_site_input_pin(ctx->getBelPinWire(bel, id_CLK));
        int ce_pin  = trace_site_input_pin(ctx->getBelPinWire(bel, id_CE));
        int sr_pin  = trace_site_input_pin(ctx->getBelPinWire(bel, id_R));

        if (!first) f << ",\n";
        first = false;
        f << "  {\"name\":\""     << json_escape(cname.str(ctx))
          << "\",\"site_type\":\"" << json_escape(site.type)
          << "\",\"sx\":"         << site.sx << ",\"sy\":" << site.sy
          << ",\"slice\":"        << slice
          << ",\"slot\":\""       << slot << "\""
          << ",\"ff_type\":\""    << json_escape(ff_type) << "\""
          << ",\"init\":"         << ff_init
          << ",\"d_pin\":"        << d_pin
          << ",\"clk_pin\":"      << clk_pin
          << ",\"ce_pin\":"       << ce_pin
          << ",\"sr_pin\":"       << sr_pin
          << ",\"clk_domain\":"   << clock.domain
          << ",\"clk_token\":\""  << json_escape(clock.token) << "\""
          << ",\"clk_side\":\""   << json_escape(clock.side)  << "\""
          << ",\"clk_net\":\""    << json_escape(clk_net)     << "\""
          << ",\"ce_net\":\""     << json_escape(connected_net_name(ci.get(), id_CE)) << "\""
          << ",\"sr_net\":\""     << json_escape(connected_net_name(ci.get(), id_R))  << "\""
          << ",\"is_c_inverted\":" << (ci->params.count(id_IS_C_INVERTED) ? ci->params.at(id_IS_C_INVERTED).as_int64() : 0)
          << ",\"is_r_inverted\":" << (ci->params.count(id_IS_R_INVERTED) ? ci->params.at(id_IS_R_INVERTED).as_int64() : 0)
          << ",\"is_s_inverted\":" << (ci->params.count(id_IS_S_INVERTED) ? ci->params.at(id_IS_S_INVERTED).as_int64() : 0)
          << ",\"is_d_inverted\":" << (ci->params.count(id_IS_D_INVERTED) ? ci->params.at(id_IS_D_INVERTED).as_int64() : 0)
          << "}";

        // Accumulate slice-mux state.
        auto key = std::make_tuple(site.type, site.sx, site.sy, slice);
        SliceMuxState &state = slice_mux_state[key];
        state.site_type = site.type;
        state.sx = site.sx; state.sy = site.sy; state.slice = slice;
        state.has_any_ff = true;
        if (slot == "OXQ") {
            state.has_oxq_ff = true;
            // Derive OAQ_SEL from the OXQ FF's D-input pip. Trace the bound
            // uphill wire from the FFD_OXQ sink; if it traces back to the
            // slice's XQ FF Q wire, source = XQ (010); LUT output → OX0 (100);
            // any site-input → XI (001). Fall back to vendor default 010 if
            // the trace can't be resolved (placer ran without router data).
            WireId d_wire = ctx->getBelPinWire(bel, id_D);
            WireId src = trace_bound_uphill_wire(d_wire);
            if (src != WireId()) {
                IdString wname = ctx->wire_info(src).type;
                if (wname == ctx->id("FFQ"))      state.oaq_sel = 2; // XQ FF Q
                else if (wname == ctx->id("LUTO")) {
                    // Phase 4: silicon's MUX_OXQ.I2 is OX0 (LUT5_0), not the
                    // LUT6 mux result. The OAQMUX_OX0 pip carries the right
                    // value only when the LUT cell is in LUT5 mode (≤5
                    // inputs or I5-independent). For LUT6 cells the pip is
                    // silicon-invalid; fall back to the vendor default 010
                    // so a downstream emit_sdc consumer doesn't trust a
                    // bogus value. (Router-side fix is to remove the pip
                    // for LUT6 bels; deferred until the chipdb gains
                    // per-bel pip masks.)
                    state.oaq_sel = 2;
                    NetInfo *net = ctx->getBoundWireNet(src);
                    if (net && net->driver.cell && net->driver.cell->type == id_SLG_LUT) {
                        if (cell_param_string(net->driver.cell, ctx->id("LUT_MODE")) == "LUT5")
                            state.oaq_sel = 4; // OX0
                    }
                }
                else                               state.oaq_sel = 1; // XI (site input)
            } else {
                state.oaq_sel = 2;
            }
        } else {
            // XQ FF: derive AQ1_SEL from the FF's D-input pip (Phase 2). Trace
            // the bound uphill wire and classify the source by its IdString
            // type tag — LUTO → X_RAW / OX0, CRY_C → C_BIT, CRY_S → S_BIT,
            // EXTRA_IN → EXTRA_IN, anything else (site-input pin) → XI.
            //
            // Per-slice bit indexing (silicon, per docs/schematics/slice.v's
            // MUX_XQ instantiation):
            //   slice A/B/C (6-bit): {EXTRA_IN=0, XI=1, OX0=2, C_BIT=3,
            //                          S_BIT=4, X_RAW=5}
            //   slice D     (5-bit): {XI=0, OX0=1, C_BIT=2, S_BIT=3,
            //                          X_RAW=4}            — drops EXTRA_IN.
            // So slice D's value is the A/B/C value shifted down by 1.
            //
            // Phase 4 — LUT5/LUT6 wire split. The chipdb collapses OX0 (bit 2)
            // and X_RAW (bit 5) onto a single `lut_out` wire; the LUT bel's
            // bound cell carries the LUT5/LUT6 mode tag (set by pack_luts).
            // When the source wire is LUTO and the driving cell is LUT5 mode,
            // silicon picks the OX0 path (bit 2) — the LUT5_0 output bypasses
            // the LUT6 mux. LUT6-mode cells stay on bit 5 (the vendor default
            // for monolithic 6-input LUTs).
            state.has_xq_ff = true;
            WireId d_wire = ctx->getBelPinWire(bel, id_D);
            WireId src = trace_bound_uphill_wire(d_wire);
            int abc_bit = 5; // X_RAW default (vendor 100000)
            if (src != WireId()) {
                IdString wname = ctx->wire_info(src).type;
                // The vendor feeds the flip-flop from X_RAW (the LUT6 output)
                // even for a LUT of five or fewer inputs (osc_probe's LUT1:
                // AQ1_SEL_D = 10000). OX0 is the LUT5_0 half of a split LUT,
                // which this flow never configures.
                if (wname == ctx->id("LUTO"))            abc_bit = 5;  // X_RAW
                else if (wname == id_CRY_C)            abc_bit = 3;  // C_BIT
                else if (wname == id_CRY_S)            abc_bit = 4;  // S_BIT
                else if (wname == id_EXTRA_IN)         abc_bit = 0;  // EXTRA_IN
                else                                   abc_bit = 1;  // XI
            }
            // Slice D drops EXTRA_IN; remaining bits shift down by 1.
            // EXTRA_IN routed to a slice-D FF is invalid (silicon has no
            // EXTRA_IN on slice D); chipdb prevents it, but guard anyway.
            int field_bit = (slice == 3) ? (abc_bit - 1) : abc_bit;
            if (slice == 3 && abc_bit == 0)
                field_bit = 0; // shouldn't happen; bit 0 unused on slice D
            state.aq1_sel = 1 << field_bit;
        }
    }

    // --- Phase 3: AMX_SEL for combinational xMUX fabric outputs ---
    // For each placed SLG_LUT cell, check whether the OMUX pip from its
    // lut_out wire into the slice's xMUX site-output pin (27 + 3*sl) is
    // routed. If yes, the silicon's MUX_X is selecting the slice's own
    // combinational output (LUT result for normal mode, sum/carry for
    // carry mode). Phase 1 already handles the OXQ case (xMUX driven by
    // OXQ_Q → AMX_SEL bit 1); the OMUX pip and AMXMUX_OXQ pip terminate
    // at the same wire, so the router's "one driver per wire" rule means
    // at most one of (Phase 1 OXQ path, Phase 3 lut_out path) fires.
    //
    // Per-slice AMX_SEL bit indexing (slice.v MUX_X):
    //   A/B/C (5-bit): {EXTRA_IN=0, OXQ=1, OX0=2, C_BIT=3, S_BIT=4}
    //   D     (4-bit): {OXQ=0, OX0=1, C_BIT=2, S_BIT=3} — drops EXTRA_IN.
    for (auto &[cname, ci] : ctx->cells) {
        if (ci->type != id_SLG_LUT) continue;
        BelId bel = ci->bel;
        if (bel == BelId()) continue;
        auto it = bel_to_site.find(bel);
        if (it == bel_to_site.end()) continue;
        auto &site = db.sites[it->second];
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (site.type == "L1") ? 8 : 0;
        int slice = loc.z - z_off - LUT_Z_BASE;  // 0..3
        if (slice < 0 || slice >= SLICES_PER_CLB) continue;
        int expected_xmux_pin = 27 + 3 * slice;

        WireId o_wire = ctx->getBelPinWire(bel, id_O);
        if (o_wire == WireId()) continue;
        bool xmux_used = false;
        for (PipId pip : ctx->getPipsDownhill(o_wire)) {
            if (ctx->getBoundPipNet(pip) == nullptr) continue;
            WireId dst = ctx->getPipDstWire(pip);
            auto pit = wire_to_site_output_pin.find(dst);
            if (pit != wire_to_site_output_pin.end() && pit->second == expected_xmux_pin) {
                xmux_used = true;
                break;
            }
        }
        if (!xmux_used) continue;

        // Mode → A/B/C-indexed source bit.
        bool carry_mode = cell_param_true(ci.get(), ctx->id("CARRY_MODE"));
        bool carry_co_out = cell_param_true(ci.get(), ctx->id("CARRY_CO_OUTPUT"));
        int abc_bit;
        if (carry_mode) {
            abc_bit = carry_co_out ? 3 : 4;  // C_BIT vs S_BIT
        } else {
            // Phase 4: MUX_X.I2 is OX0 (LUT5_0), not the LUT6 mux result.
            // Skip Phase 3 emit for LUT6-mode cells — the OMUX→xMUX path
            // would carry a sub-result, not the cell's logical output, so
            // emitting bit 2 would document silicon-invalid routing.
            // (Router-side fix is to remove the OMUX pip for LUT6 bels;
            // deferred until the chipdb gains per-bel pip masks.)
            if (cell_param_string(ci.get(), ctx->id("LUT_MODE")) != "LUT5")
                continue;
            abc_bit = 2;                      // OX0 (LUT5_0 result via MUX_X.I2)
        }
        int field_bit = (slice == 3) ? (abc_bit - 1) : abc_bit;
        int amx_sel_value = 1 << field_bit;

        auto key = std::make_tuple(site.type, site.sx, site.sy, slice);
        SliceMuxState &state = slice_mux_state[key];
        state.site_type = site.type;
        state.sx = site.sx; state.sy = site.sy; state.slice = slice;
        // Phase 3's value only fires when the OXQ FF is NOT bound: the
        // OMUX pip and AMXMUX_OXQ pip share the xMUX wire, so the router
        // can pick only one; the Phase 1 has_oxq_ff path already covers
        // the OXQ case.
        if (!state.has_oxq_ff) {
            state.amx_sel_phase3 = amx_sel_value;
            state.has_phase3_amx = true;
        }
    }

    // --- RAMs ---
    f << "\n],\n\"rams\": [\n";
    first = true;
    static const IdString wa_ports[]   = {id_A0,    id_A1,    id_A2,    id_A3,    id_A4};
    static const IdString dpra_ports[] = {id_DPRA0, id_DPRA1, id_DPRA2, id_DPRA3, id_DPRA4};
    static const IdString ram_in_pins[]= {id_I0,    id_I1,    id_I2,    id_I3,    id_I4,    id_I5};

    for (auto &[cname, ci] : ctx->cells) {
        if (!ci->type.in(id_SLG_RAM, id_SLG_RAM32X1D)) continue;
        BelId bel = ci->bel;
        if (bel == BelId()) continue;
        auto it = bel_to_site.find(bel);
        if (it == bel_to_site.end()) continue;
        auto &site = db.sites[it->second];
        Loc loc = ctx->getBelLocation(bel);
        bool is_sitewide = (ci->type == id_SLG_RAM32X1D);
        int slice = is_sitewide ? 3 : (loc.z - 8 - RAM_Z_BASE);

        std::string ram_type;
        auto rt = ci->params.find(ctx->id("RAM_TYPE"));
        if (rt != ci->params.end()) ram_type = rt->second.as_string();

        int64_t init = 0;
        auto ip = ci->params.find(id_INIT);
        if (ip != ci->params.end()) init = ip->second.as_int64();

        bool native_ram32m = cell_param_true(ci.get(), ctx->id("RAM32M_NATIVE"));

        int pin_map[6] = {-1, -1, -1, -1, -1, -1};
        if (is_sitewide) {
            for (int k = 0; k < 5; k++)
                pin_map[k] = trace_site_input_pin_from_bel(bel, wa_ports[k]);
            pin_map[5] = trace_site_input_pin_from_bel(bel, id_D);
        } else {
            for (int k = 0; k < LUT_K; k++) {
                WireId w = ctx->getBelPinWire(bel, ram_in_pins[k]);
                if (w == WireId()) continue;
                for (auto pip : ctx->getPipsUphill(w)) {
                    if (ctx->getBoundPipNet(pip) == nullptr) continue;
                    WireId src = ctx->getPipSrcWire(pip);
                    auto pit = wire_to_site_pin.find(src);
                    if (pit != wire_to_site_pin.end())
                        pin_map[k] = pit->second;
                    break;
                }
            }
        }

        int dpra_map[5];
        for (int k = 0; k < 5; k++)
            dpra_map[k] = trace_site_input_pin_from_bel(bel, dpra_ports[k]);

        std::string wa_nets[5];
        for (int k = 0; k < 5; k++)
            wa_nets[k] = connected_net_name(ci.get(), is_sitewide ? wa_ports[k] : ram_in_pins[k]);

        std::string dpra_nets[5];
        for (int k = 0; k < 5; k++)
            dpra_nets[k] = connected_net_name(ci.get(), dpra_ports[k]);

        std::string d_net    = connected_net_name(ci.get(), is_sitewide ? id_D   : id_I5);
        std::string spo_net  = connected_net_name(ci.get(), id_SPO);
        std::string dpo_net  = connected_net_name(ci.get(), id_DPO);
        std::string wclk_net = connected_net_name(ci.get(), id_WCLK);
        ExportedClock clock  = lookup_clock(wclk_net);

        int wclk_pin = trace_site_input_pin(ctx->getBelPinWire(bel, id_WCLK));
        int we_pin   = trace_site_input_pin(ctx->getBelPinWire(bel, id_WE));

        bool has_dpra = false, has_dpo = false;
        auto has_dpra_it = ci->params.find(ctx->id("RAM_HAS_DPRA"));
        auto has_dpo_it  = ci->params.find(ctx->id("RAM_HAS_DPO"));
        if (has_dpra_it != ci->params.end()) has_dpra = (has_dpra_it->second.as_int64() != 0);
        if (has_dpo_it  != ci->params.end()) has_dpo  = (has_dpo_it->second.as_int64()  != 0);

        if (!first) f << ",\n";
        first = false;
        f << "  {\"name\":\""     << json_escape(cname.str(ctx))
          << "\",\"ram_type\":\""  << json_escape(ram_type)
          << "\",\"site_type\":\"" << json_escape(site.type)
          << "\",\"sx\":"         << site.sx << ",\"sy\":" << site.sy
          << ",\"slice\":"        << slice << ",\"init\":" << (uint64_t)init
          << ",\"layout\":\""     << (is_sitewide ? "rbb6m_single"
                                     : native_ram32m ? "ram32m_native_half" : "slice_local")
          << "\""
          << ",\"pins\":["        << pin_map[0] << "," << pin_map[1]
          << ","                  << pin_map[2] << "," << pin_map[3]
          << ","                  << pin_map[4] << "," << pin_map[5] << "]"
          << ",\"wa_pins\":["     << pin_map[0] << "," << pin_map[1]
          << ","                  << pin_map[2] << "," << pin_map[3]
          << ","                  << pin_map[4] << "]"
          << ",\"wa_nets\":["
          << "\"" << json_escape(wa_nets[0]) << "\","
          << "\"" << json_escape(wa_nets[1]) << "\","
          << "\"" << json_escape(wa_nets[2]) << "\","
          << "\"" << json_escape(wa_nets[3]) << "\","
          << "\"" << json_escape(wa_nets[4]) << "\"]"
          << ",\"d_pin\":"        << pin_map[5]
          << ",\"d_net\":\""      << json_escape(d_net) << "\""
          << ",\"dpra_pins\":["   << dpra_map[0] << "," << dpra_map[1]
          << ","                  << dpra_map[2] << "," << dpra_map[3]
          << ","                  << dpra_map[4] << "]"
          << ",\"dpra_nets\":["
          << "\"" << json_escape(dpra_nets[0]) << "\","
          << "\"" << json_escape(dpra_nets[1]) << "\","
          << "\"" << json_escape(dpra_nets[2]) << "\","
          << "\"" << json_escape(dpra_nets[3]) << "\","
          << "\"" << json_escape(dpra_nets[4]) << "\"]"
          << ",\"wclk_pin\":"     << wclk_pin
          << ",\"clk_domain\":"   << clock.domain
          << ",\"clk_token\":\""  << json_escape(clock.token) << "\""
          << ",\"clk_side\":\""   << json_escape(clock.side)  << "\""
          << ",\"we_pin\":"       << we_pin
          << ",\"has_dpra\":"     << (has_dpra ? "true" : "false")
          << ",\"has_dpo\":"      << (has_dpo  ? "true" : "false")
          << ",\"wclk_net\":\""   << json_escape(wclk_net) << "\""
          << ",\"we_net\":\""     << json_escape(connected_net_name(ci.get(), id_WE))   << "\""
          << ",\"spo_net\":\""    << json_escape(spo_net) << "\""
          << ",\"dpo_net\":\""    << json_escape(dpo_net) << "\"";
        if (native_ram32m) {
            f << ",\"ram32m_group\":\""
              << json_escape(cell_param_string(ci.get(), ctx->id("RAM32M_GROUP"))) << "\""
              << ",\"ram32m_bitplane\":"
              << cell_param_int(ci.get(), ctx->id("RAM32M_BITPLANE"));
        }
        f << "}";
    }

    // --- Clocks ---
    f << "\n],\n\"clocks\": [\n";
    first = true;
    for (const auto &clock : exported_clocks) {
        if (!first) f << ",\n";
        first = false;
        f << "  {\"name\":\""    << json_escape(clock.cell_name)
          << "\",\"port\":\""    << json_escape(clock.port_name)
          << "\",\"net\":\""     << json_escape(clock.net_name)
          << "\",\"token\":\""   << json_escape(clock.token)
          << "\",\"side\":\""    << json_escape(clock.side)
          << "\",\"domain\":"    << clock.domain
          << "}";
    }

    // --- IOBs ---
    f << "\n],\n\"iobs\": [\n";
    first = true;
    for (auto &[cname, ci] : ctx->cells) {
        if (ci->type != id_SLG_IOB) continue;
        BelId bel = ci->bel;
        if (bel == BelId()) continue;
        Loc loc = ctx->getBelLocation(bel);
        auto port_it = ci->ports.find(id_PAD);
        PortType port_type = (port_it != ci->ports.end()) ? port_it->second.type : PORT_INOUT;
        const char *dir = "inout";
        if      (port_type == PORT_IN)  dir = "output";
        else if (port_type == PORT_OUT) dir = "input";
        std::string pad_in_sm = trace_iob_pad_in_sm_from_bel(bel, id_PAD);
        // OE routing fields: present only for east-edge output IOBs that have
        // an OE bel pin (added in preRoute for gx=31 output pads).
        std::string oe_sm;
        int oe_pin = -1;
        if (loc.x == 31 && port_type == PORT_IN) {
            oe_sm  = trace_iob_pad_in_sm_from_bel(bel, id_OE);
            oe_pin = trace_site_input_pin_from_bel(bel, id_OE);
        }
        if (!first) f << ",\n";
        first = false;
        f << "  {\"name\":\""    << json_escape(cname.str(ctx))
          << "\",\"gx\":"        << loc.x << ",\"gy\":" << loc.y
          << ",\"dir\":\""       << dir << "\""
          << ",\"pad_in_pin\":"  << trace_site_input_pin_from_bel(bel, id_PAD)
          << ",\"pad_out_pin\":" << trace_site_output_pin_from_bel(bel, id_PAD)
          << ",\"pad_in_sm\":\""  << json_escape(pad_in_sm) << "\""
          << ",\"pad_in_d0_branch\":" << parse_edge_d0_branch(pad_in_sm)
          << ",\"oe_pin\":"      << oe_pin
          << ",\"oe_sm\":\""     << json_escape(oe_sm) << "\""
          << ",\"oe_d0_branch\":" << parse_edge_d0_branch(oe_sm)
          << ",\"net\":\""        << json_escape(connected_net_name(ci.get(), id_PAD)) << "\""
          << "}";
    }

    // --- Slice MUXes (8-FF rewrite, Phases 1+2) ---
    // One entry per slice with at least one bound FF. Emits:
    //   * `oaq_sel`  (Phase 1): derived from the OXQ FF's D-input source,
    //                always present (defaults to 010 = XQ feedback).
    //   * `amx_sel`  (Phase 1): present only when an OXQ FF is bound, forces
    //                AMX_SEL=00010 (OXQ → xMUX).
    //   * `aq1_sel`  (Phase 2): derived from the XQ FF's D-input source, one
    //                of {X_RAW=100000, XI=000010, C_BIT=001000, S_BIT=010000,
    //                EXTRA_IN=000001}. Present only when an XQ FF is bound.
    // emit_sdc.py turns these into set_case_analysis lines that pack.py's
    // AQ1/AMX/OAQ_SEL tier maps to per-RBB bitstream positions.
    f << "\n],\n\"slice_muxes\": [\n";
    first = true;
    for (auto &[key, state] : slice_mux_state) {
        if (!state.has_any_ff) continue;
        if (!first) f << ",\n";
        first = false;
        f << "  {\"sx\":"   << state.sx
          << ",\"sy\":"     << state.sy
          << ",\"site_type\":\"" << state.site_type << "\""
          << ",\"slice\":"  << state.slice
          << ",\"oaq_sel\":" << state.oaq_sel;
        if (state.has_oxq_ff) {
            // AMX_SEL bit indexing per silicon (slice.v MUX_X):
            //   A/B/C (5-bit): {EXTRA_IN=0, OXQ=1, OX0=2, C_BIT=3, S_BIT=4}
            //   D     (4-bit): {OXQ=0, OX0=1, C_BIT=2, S_BIT=3} — drops EXTRA_IN.
            // OXQ-as-xMUX source: slice D bit 0, others bit 1.
            int oxq_bit = (state.slice == 3) ? 0 : 1;
            f << ",\"amx_sel\":" << (1 << oxq_bit);
        } else if (state.has_phase3_amx) {
            f << ",\"amx_sel\":" << state.amx_sel_phase3;
        }
        if (state.has_xq_ff)
            f << ",\"aq1_sel\":" << state.aq1_sel;
        f << "}";
    }

    f << "\n]\n}\n";
    log_info("slg47910: placement data written\n");
}

// Emit routing decisions as JSON for downstream SM bit computation.
// Output schema:
//   {
//     "pips":       [[src_node_id, dst_node_id], ...],   // data routing
//     "clock_pips": [[src_node_id, dst_node_id], ...],   // clock fabric
//     "pad_nodes":  [[virtual_node_id, "wire name"], ...],
//     "nets":       [{"name": "...", "pips": [[s,d], ...]}, ...]
//   }
//
// "pips" carries every bound chipdb pip tagged "PIP" (the data-routing
// fabric); "clock_pips" carries every bound pip tagged "CLKPIP" (added by
// gen_chipdb.py's --enable-clock-fabric layer). Site-local pips (MUX,
// OMUX, FFDMUX, CARRY, GCLK) are still skipped here — they encode placement
// decisions, not routing. Node IDs below 40960 are SM routing nodes, 40960
// and up are site-pin nodes (see tools/emit_route.py).
//
// "nets" is the per-net view and is the part that carries NET ATTRIBUTION.
// Without it, "which net does this SM node carry?" can only be guessed by
// BFS from driver pins, which leaves every constant-driven net (e.g.
// $PACKER_VCC on an FF clock-enable) unattributed because it has no driver
// cell. Rekb's `ingest build` consumes it directly.
//
// "nets" also includes IOPAD pips (pad <-> IOB site pin), which "pips"
// drops. Those are the turns that made routing.json a disconnected graph:
// a constant entering the fabric through an IOB pad had no edge, so its
// downstream nodes looked like orphans. IOPAD endpoints are synthetic
// wires with no chipdb node, so they get VIRTUAL node ids allocated above
// the whole node space (>= node_to_wire.size()) and are named in
// "pad_nodes". Consumers that care about SM nodes must ignore any id >=
// len(rnode2sm) - tools/emit_route.py already does, since it filters on
// `dst < 40960 and src < 40960`.
void SLG47910Impl::emit_routing()
{
    log_info("slg47910: writing routing to %s\n", route_out.c_str());

    // Reverse map: WireId → routing graph node ID.
    dict<WireId, int> wire_to_node;
    for (size_t i = 0; i < node_to_wire.size(); i++)
        if (node_to_wire[i] != WireId())
            wire_to_node[node_to_wire[i]] = (int)i;

    std::ofstream f(route_out);
    if (!f) { log_error("cannot write %s\n", route_out.c_str()); return; }

    const IdString id_pip    = ctx->id("PIP");
    const IdString id_clkpip = ctx->id("CLKPIP");

    // Synthetic (node-less) wires - the IOB PAD and OE wires created in
    // init_iob_bels() - get virtual node IDs so IOPAD pips can be emitted.
    // Kept in insertion order so the output stays deterministic.
    const int virtual_base = (int)node_to_wire.size();
    std::vector<std::pair<int, std::string>> pad_nodes;

    auto node_for = [&](WireId w) -> int {
        auto it = wire_to_node.find(w);
        if (it != wire_to_node.end())
            return it->second;
        if (w == WireId())
            return -1;
        int id = virtual_base + (int)pad_nodes.size();
        wire_to_node[w] = id;
        pad_nodes.emplace_back(id, ctx->getWireName(w).str(ctx));
        return id;
    };

    struct NetExport {
        std::string name;
        std::vector<std::pair<int, int>> pips;
    };
    std::vector<NetExport> net_exports;
    std::vector<std::pair<int,int>> data_edges, clock_edges;

    for (auto &[nname, net] : ctx->nets) {
        NetExport ne;
        ne.name = nname.str(ctx);
        for (auto &w : net->wires) {
            PipId pip = w.second.pip;
            if (pip == PipId()) continue;
            WireId src_wire = ctx->getPipSrcWire(pip);
            WireId dst_wire = ctx->getPipDstWire(pip);
            int src = node_for(src_wire);
            int dst = node_for(dst_wire);
            if (src < 0 || dst < 0) continue;
            ne.pips.emplace_back(src, dst);

            IdString ptype = ctx->getPipType(pip);
            // The legacy "pips"/"clock_pips" arrays are the data router's
            // input and keep their exact old meaning: every pip of that type
            // whose ends are BOTH node-backed. That includes site-pin nodes
            // (ids 40960..node_to_wire.size()-1) - tools/emit_route.py needs
            // those: it turns `src >= 40960 -> dst < 40960` into
            // `site_to_routing`, and the U0-enable pass enables a U0 node only
            // when a site pin feeds it. Filtering those out here silently
            // disabled U0B0N0/U1B0N0@(29,28) in build/osc_f2828 (2 bits).
            // Only the virtual PAD ids (>= virtual_base) must be excluded.
            if (src >= virtual_base || dst >= virtual_base)
                continue;
            if (ptype == id_pip)
                data_edges.emplace_back(src, dst);
            else if (ptype == id_clkpip)
                clock_edges.emplace_back(src, dst);
        }
        if (!ne.pips.empty())
            net_exports.push_back(std::move(ne));
    }

    auto emit_array = [&](const char *key, const std::vector<std::pair<int,int>> &edges) {
        f << "  \"" << key << "\": [";
        bool first = true;
        for (auto &e : edges) {
            if (!first) f << ",";
            first = false;
            f << "\n    [" << e.first << "," << e.second << "]";
        }
        if (!edges.empty()) f << "\n  ";
        f << "]";
    };

    auto quote = [&](const std::string &s) {
        f << "\"";
        for (char c : s) {
            if (c == '"' || c == '\\') f << '\\' << c;
            else if (c == '\n') f << "\\n";
            else if ((unsigned char)c < 0x20) f << ' ';
            else f << c;
        }
        f << "\"";
    };

    f << "{\n";
    emit_array("pips", data_edges);
    f << ",\n";
    emit_array("clock_pips", clock_edges);
    f << ",\n";

    // Virtual nodes standing in for synthetic PAD/OE wires.
    f << "  \"pad_nodes\": [";
    for (size_t i = 0; i < pad_nodes.size(); i++) {
        f << (i ? "," : "") << "\n    [" << pad_nodes[i].first << ",";
        quote(pad_nodes[i].second);
        f << "]";
    }
    if (!pad_nodes.empty()) f << "\n  ";
    f << "]";

    // Per-net view: the attribution source, and the complete pip graph.
    f << ",\n  \"nets\": [";
    for (size_t i = 0; i < net_exports.size(); i++) {
        f << (i ? "," : "") << "\n    {\"name\":";
        quote(net_exports[i].name);
        f << ",\"pips\":[";
        for (size_t j = 0; j < net_exports[i].pips.size(); j++) {
            f << (j ? "," : "") << "[" << net_exports[i].pips[j].first << ","
              << net_exports[i].pips[j].second << "]";
        }
        f << "]}";
    }
    if (!net_exports.empty()) f << "\n  ";
    f << "]";

    f << "\n}\n";
    log_info("slg47910: wrote %d fabric routing edges, %d clock-fabric edges, "
             "%d nets, %d pad nodes\n",
             (int)data_edges.size(), (int)clock_edges.size(),
             (int)net_exports.size(), (int)pad_nodes.size());
}

NEXTPNR_NAMESPACE_END
