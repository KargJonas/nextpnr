/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *  Architecture initialisation: routing wires/pips, CLB/IOB/clock BELs,
 *  and pin-constraints loading.
 */

#include "slg47910.h"

#include <fstream>
#include <map>
#include <sstream>

NEXTPNR_NAMESPACE_BEGIN

void SLG47910Impl::add_bel_conflict(BelId a, BelId b)
{
    if (a == BelId() || b == BelId() || a == b)
        return;
    bel_conflicts[a].push_back(b);
    bel_conflicts[b].push_back(a);
}

// Decal layout within a 1.0 x 1.0 tile:
//   slice A (z=0): row 0.05..0.25
//   slice B (z=1): row 0.27..0.47
//   slice C (z=2): row 0.49..0.69
//   slice D (z=3): row 0.71..0.91
// Within each slice row:
//   LUT bel  = left   half   (0.06..0.36)
//   FF  bel  = middle 1/3rd  (0.40..0.58)
//   RAM bel  = right  third  (0.62..0.92, only on L1 memory CLBs)
// IOB / CLKIN occupy the whole tile.
//
// The slice-y offset is encoded in the decal name so we can register one
// graphic per (cell_type, slice) and reuse it at every (x, y) tile via
// setBelDecal(bel, x, y, name).
namespace {
constexpr float SLICE_Y0[4] = {0.05f, 0.27f, 0.49f, 0.71f};
constexpr float SLICE_H    = 0.20f;
constexpr float LUT_X1 = 0.06f, LUT_X2 = 0.36f;
constexpr float FF_X1  = 0.40f, FF_X2  = 0.58f;
constexpr float RAM_X1 = 0.62f, RAM_X2 = 0.92f;
constexpr char SLICE_LETTER[4] = {'A', 'B', 'C', 'D'};

inline IdString lut_decal_name(Context *ctx, int sl) { return ctx->idf("LUT_%c", SLICE_LETTER[sl]); }
inline IdString ff_decal_name (Context *ctx, int sl) { return ctx->idf("FF_%c",  SLICE_LETTER[sl]); }
inline IdString ram_decal_name(Context *ctx, int sl) { return ctx->idf("RAM_%c", SLICE_LETTER[sl]); }
} // namespace

void SLG47910Impl::init_decals()
{
    // Bel decals: one box per (cell_type, slice). STYLE_INACTIVE renders gray;
    // the SVG writer auto-promotes to STYLE_ACTIVE (red fill) for placed bels
    // via Arch::getBelDecal's `result.decal.active = getBoundBelCell(bel)`.
    for (int sl = 0; sl < 4; sl++) {
        float y1 = SLICE_Y0[sl];
        float y2 = y1 + SLICE_H;
        ctx->addDecalGraphic(IdStringList(lut_decal_name(ctx, sl)),
                             GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE,
                                            LUT_X1, y1, LUT_X2, y2, 1.0f));
        ctx->addDecalGraphic(IdStringList(ff_decal_name(ctx, sl)),
                             GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE,
                                            FF_X1, y1, FF_X2, y2, 1.0f));
        ctx->addDecalGraphic(IdStringList(ram_decal_name(ctx, sl)),
                             GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE,
                                            RAM_X1, y1, RAM_X2, y2, 1.0f));
    }
    // IOB: full tile minus a small inset so the perimeter ring is visible.
    ctx->addDecalGraphic(IdStringList(ctx->id("IOB")),
                         GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE,
                                        0.10f, 0.10f, 0.90f, 0.90f, 1.0f));
    // CLKIN bels share grid (0, 0) — render small, offset by z. (Only 3 specs.)
    ctx->addDecalGraphic(IdStringList(ctx->id("CLKIN")),
                         GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE,
                                        0.30f, 0.30f, 0.70f, 0.70f, 1.0f));
    // Wire decal: a thin line stub at each wire's home tile so routing is
    // visible at all. The SVG writer renders STYLE_INACTIVE gray and the
    // router-bound wires automatically promote to STYLE_ACTIVE (red).
    ctx->addDecalGraphic(IdStringList(ctx->id("WIRE")),
                         GraphicElement(GraphicElement::TYPE_LINE, GraphicElement::STYLE_INACTIVE,
                                        0.45f, 0.45f, 0.55f, 0.55f, 1.0f));
}

void SLG47910Impl::init_routing_wires()
{
    log_info("slg47910: creating routing wires...\n");
    size_t max_id = 0;
    for (auto &w : db.wires) max_id = std::max<size_t>(max_id, w.id);
    node_to_wire.assign(max_id + 1, WireId());
    IdString wire_decal = ctx->id("WIRE");
    for (auto &w : db.wires) {
        IdString base = ctx->id(w.name);
        WireId wire = ctx->addWire(h.xy_id(w.x, w.y, base), base, w.x, w.y);
        node_to_wire[w.id] = wire;
        // Tag for SVG/GUI rendering. The wire-decal is a tiny stub at the
        // wire's home tile; routed wires inherit STYLE_ACTIVE automatically
        // so routing usage shows up in red.
        ctx->setWireDecal(wire, w.x, w.y, IdStringList(wire_decal));
    }
}

void SLG47910Impl::init_routing_pips()
{
    log_info("slg47910: creating routing pips...\n");
    int skipped = 0;
    for (auto &p : db.pips) {
        WireId src = node_to_wire[p.src];
        WireId dst = node_to_wire[p.dst];
        if (src == WireId() || dst == WireId()) { skipped++; continue; }

        // The extracted vendor edge costs behave like femtoseconds: the median
        // non-zero pip cost in chipdb.json is about 2.2e6, which maps to a
        // plausible ~2.2 ns when divided by 1e6. The old /1000 scaling
        // inflated routed timing by three orders of magnitude.
        delay_t delay = (p.cost > 0) ? std::max((delay_t)0.001f, (delay_t)p.cost / 1000000.0f)
                                     : (delay_t)0.0f;
        IdStringList name = IdStringList::concat(ctx->getWireName(dst), ctx->getWireName(src));
        // Optional chipdb-level pip-type tag (Option A step C). Empty type
        // defaults to "PIP" (data routing); chipdb.cc fills it from the
        // optional JSON "type" field. "CLKPIP" marks clock-fabric pips so
        // emit.cc can split them into a separate routing.json section.
        IdString type_id = ctx->id(p.type.empty() ? "PIP" : p.type.c_str());
        ctx->addPip(name, type_id, src, dst, delay, Loc(0, 0, 0));
    }
    if (skipped)
        log_warning("slg47910: skipped %d pips with invalid wire refs\n", skipped);
}

IdString SLG47910Impl::lut_pin_id(Context *ctx, int input_idx)
{
    static const IdString pins[] = {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
    NPNR_ASSERT(input_idx >= 0 && input_idx < LUT_K);
    (void)ctx;
    return pins[input_idx];
}

void SLG47910Impl::init_clb_bels()
{
    log_info("slg47910: creating CLB sub-bels...\n");
    int n_lut = 0, n_ff = 0, n_ram = 0;
    std::map<std::pair<int,int>, std::vector<BelId>> l0_bels_by_xy, l1_bels_by_xy;

    for (int site_idx = 0; site_idx < (int)db.sites.size(); site_idx++) {
        auto &site = db.sites[site_idx];
        if (site.type != "L0" && site.type != "L1") continue;
        bool is_l1 = (site.type == "L1");
        int x = site.x, y = site.y;
        auto &site_bels = is_l1 ? l1_bels_by_xy[{x,y}] : l0_bels_by_xy[{x,y}];

        // Collect site input/output wires and record pin indices.
        std::vector<WireId> site_inputs, site_outputs;
        std::vector<WireId> site_input_by_pin(24);
        std::map<int, WireId> site_output_by_pin;
        // Chipdb-backed CLB clock input (added by gen_chipdb.py's
        // --enable-clock-fabric). Captured separately so the data-routing
        // fan-in pips skip it — the only legal driver is the CLKROW →
        // CLBCLK pip from the clock fabric.
        WireId chipdb_clk_wire;
        for (auto &pin : site.pins) {
            WireId w = node_to_wire[pin.node];
            if (pin.dir == 'I' && pin.name == "CLK") {
                chipdb_clk_wire = w;
                continue;
            }
            int pin_idx = parse_site_pin_index(pin.name);
            if (pin.dir == 'I') {
                if (pin_idx >= 0 && pin_idx < 24)
                    wire_to_site_pin[w] = pin_idx;
                else
                    wire_to_site_pin[w] = (int)site_inputs.size();
                site_inputs.push_back(w);
                if (pin_idx >= 0 && pin_idx < 24)
                    site_input_by_pin.at(pin_idx) = w;
            } else {
                if (pin_idx >= 0) {
                    wire_to_site_output_pin[w] = pin_idx;
                    site_output_by_pin[pin_idx] = w;
                } else {
                    wire_to_site_output_pin[w] = (int)site_outputs.size();
                }
                site_outputs.push_back(w);
            }
        }

        int z_off = is_l1 ? 8 : 0;
        const char *prefix = is_l1 ? "M" : "L";

        // Shared CLB control wires: one CLK, CE, and SR per CLB.
        // When the chipdb exposes a "CLK" site pin (--enable-clock-fabric),
        // we bind FF/RAM clock pins directly to that node-backed wire so the
        // router reaches them through the chipdb's
        // CLKPAD → spine → row → CLBCLK chain. No data-routing fan-in MUXes
        // for CLK in that case — the only legal driver is the chipdb's
        // CLKROW → CLBCLK pip from the clock fabric.
        const bool clock_fabric_enabled = (chipdb_clk_wire != WireId());
        WireId clb_clk_wire = clock_fabric_enabled
            ? chipdb_clk_wire
            : ctx->addWire(h.xyz_id(x, y, z_off, ctx->id("CLK")), ctx->id("CLK"), x, y);
        global_clock_sinks.push_back(clb_clk_wire);
        WireId clb_ce_wire = ctx->addWire(
            h.xyz_id(x, y, z_off, ctx->id("CE")), ctx->id("CE"), x, y);
        WireId clb_sr_wire = ctx->addWire(
            h.xyz_id(x, y, z_off, ctx->id("SR")), ctx->id("SR"), x, y);
        for (auto &si : site_inputs) {
            if (!clock_fabric_enabled)
                ctx->addPip(IdStringList::concat(ctx->getWireName(clb_clk_wire), ctx->getWireName(si)),
                            ctx->id("MUX"), si, clb_clk_wire, 10, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(clb_ce_wire),  ctx->getWireName(si)),
                        ctx->id("MUX"), si, clb_ce_wire,  50, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(clb_sr_wire),  ctx->getWireName(si)),
                        ctx->id("MUX"), si, clb_sr_wire,  50, Loc(x, y, 0));
        }

        // L1 WE wire — shared write-enable for distributed RAM.
        WireId clb_we_wire;
        if (is_l1) {
            clb_we_wire = ctx->addWire(
                h.xyz_id(x, y, z_off, ctx->id("WE")), ctx->id("WE"), x, y);
            for (auto &si : site_inputs) {
                ctx->addPip(IdStringList::concat(ctx->getWireName(clb_we_wire), ctx->getWireName(si)),
                            ctx->id("MUX"), si, clb_we_wire, 50, Loc(x, y, 0));
            }
        }

        std::array<BelId, SLICES_PER_CLB> lut_bels;
        std::array<BelId, SLICES_PER_CLB> ff_bels;
        std::array<BelId, SLICES_PER_CLB> oxq_ff_bels;
        std::array<BelId, SLICES_PER_CLB> srl_bels;
        std::array<WireId, SLICES_PER_CLB> lut_out_wires;
        // Phase 5: retain per-slice EXTRA_IN wires across the slice loop so
        // the post-loop MUXF7 bel setup can pip into them.
        std::array<WireId, SLICES_PER_CLB> extra_in_wires;

        for (int sl = 0; sl < SLICES_PER_CLB; sl++) {
            // --- LUT bel ---
            int lut_z = z_off + LUT_Z_BASE + sl;
            BelId lut_bel = ctx->addBel(
                h.xyz_id(x, y, lut_z, ctx->idf("%sLUT", prefix)),
                id_SLG_LUT, Loc(x, y, lut_z), false, false);
            ctx->setBelDecal(lut_bel, x, y, IdStringList(lut_decal_name(ctx, sl)));
            bel_to_site[lut_bel] = site_idx;
            lut_bels[sl] = lut_bel;
            site_bels.push_back(lut_bel);

            WireId lut_out = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("LUTO")), ctx->id("LUTO"), x, y);
            ctx->addBelOutput(lut_bel, id_O, lut_out);
            lut_out_wires[sl] = lut_out;

            // LUT inputs: I0-I3 dedicated (per-slice pins), I4-I5 muxed.
            // Physical layout:
            //   Slice A→pins 0-3, B→6-9, C→12-15, D→18-21.
            //   I4 (A5): MUX_x5 — any site input, 4-bit select.
            //   I5 (A6): MUX_x6 — any site input, 4-bit select.
            // On L1 sites, dedicated pins have only 1 routing edge (vs 4 on L0);
            // the routing graph already reflects this, so the pip structure is
            // the same and routing pressure is handled by fabric-level pips.
            for (int k = 0; k < LUT_K; k++) {
                WireId lut_in = ctx->addWire(
                    h.xyz_id(x, y, z_off * LUT_K + sl * LUT_K + k, ctx->id("LUTI")),
                    ctx->id("LUTI"), x, y);
                ctx->addBelInput(lut_bel, lut_pin_id(ctx, k), lut_in);

                if (k < 4) {
                    int pin_idx = sl * 6 + k;
                    if (pin_idx < int(site_input_by_pin.size()) && site_input_by_pin.at(pin_idx) != WireId()) {
                        WireId si = site_input_by_pin.at(pin_idx);
                        ctx->addPip(IdStringList::concat(ctx->getWireName(lut_in), ctx->getWireName(si)),
                                    ctx->id("MUX"), si, lut_in, 1, Loc(x, y, 0));
                    }
                } else {
                    for (auto &si : site_inputs)
                        ctx->addPip(IdStringList::concat(ctx->getWireName(lut_in), ctx->getWireName(si)),
                                    ctx->id("MUX"), si, lut_in, 1, Loc(x, y, 0));
                }
            }

            // A LUT reaches the fabric through its own slice's O pin. Its MUX
            // and Q pins, and every other slice's pins, sit behind output muxes
            // the bitstream does not program for a plain LUT (silicon: a LUT
            // routed out through another slice's AMUX/AQ pin read inverted).
            auto o_out = site_output_by_pin.find(26 + 3 * sl);
            if (o_out != site_output_by_pin.end())
                ctx->addPip(IdStringList::concat(ctx->getWireName(o_out->second), ctx->getWireName(lut_out)),
                            ctx->id("OMUX"), lut_out, o_out->second, 1, Loc(x, y, 0));
            n_lut++;

            // --- XQ FF bel ---
            int ff_z = z_off + FF_XQ_Z_BASE + sl;
            BelId ff_bel = ctx->addBel(
                h.xyz_id(x, y, ff_z, ctx->idf("%sFF", prefix)),
                id_SLG_FF, Loc(x, y, ff_z), false, false);
            ctx->setBelDecal(ff_bel, x, y, IdStringList(ff_decal_name(ctx, sl)));
            bel_to_site[ff_bel] = site_idx;
            ff_bels[sl] = ff_bel;
            site_bels.push_back(ff_bel);

            WireId ff_d = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("FFD")), ctx->id("FFD"), x, y);
            ctx->addBelInput(ff_bel, id_D, ff_d);
            // Dedicated LUT→FF path within the slice.  The collapsed lut_out
            // wire represents both `X_RAW` (LUT6 output, MUX_XQ bit 5, the
            // vendor default) and `OX0` (LUT5_0 output, MUX_XQ bit 2) in the
            // silicon — splitting them requires separate LUT5/LUT6 wires
            // (out of Phase 2 scope). emit.cc reports `aq1_sel = X_RAW`
            // whenever the bound uphill source is this wire.
            ctx->addPip(IdStringList::concat(ctx->getWireName(ff_d), ctx->getWireName(lut_out)),
                        ctx->id("FFDMUX"), lut_out, ff_d, 1, Loc(x, y, 0));
            // FF D can also come from general routing (MUX_XQ bit 1 = XI).
            // Bit 1 is the *inverted* bit in the bitstream (see §7.5); the
            // encoder handles inversion — emit.cc just records aq1_sel value.
            for (auto &si : site_inputs)
                ctx->addPip(IdStringList::concat(ctx->getWireName(ff_d), ctx->getWireName(si)),
                            ctx->id("MUX"), si, ff_d, 2, Loc(x, y, 0));

            // --- Phase 2: per-slice MUX_XQ extra inputs ---
            // C_BIT / S_BIT: carry-out and sum-out of the slice's adder. The
            // LUT bel (in carry mode) drives these as additional fabric-local
            // outputs; pack_carry hooks them up to FF nets when a carry/sum
            // directly fans out to a same-slice FF without a buffer LUT.
            WireId cry_c = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("CRY_C")), id_CRY_C, x, y);
            WireId cry_s = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("CRY_S")), id_CRY_S, x, y);
            ctx->addBelOutput(lut_bel, id_CRY_C, cry_c);
            ctx->addBelOutput(lut_bel, id_CRY_S, cry_s);
            // MUX_XQ bit 3 (C_BIT) and bit 4 (S_BIT).
            ctx->addPip(IdStringList::concat(ctx->getWireName(ff_d), ctx->getWireName(cry_c)),
                        ctx->id("FFDMUX_C"), cry_c, ff_d, 1, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(ff_d), ctx->getWireName(cry_s)),
                        ctx->id("FFDMUX_S"), cry_s, ff_d, 1, Loc(x, y, 0));

            // EXTRA_IN (MUX_XQ bit 0): exists only on slices A/B/C in silicon
            // (slice D drops it). The driver is the CLB-level
            // mux_a7/mux_c7/mux_x8 network (docs/schematics/clbl.v), which
            // is not yet modelled in the chipdb — the wire is created with
            // no driver as a placeholder so the FF_D pip exists but the
            // router can't legalise it. EXTRA_IN becomes reachable from the
            // open flow once the A7-network model lands (Phase 3+).
            if (sl < 3) {
                WireId extra_in = ctx->addWire(
                    h.xyz_id(x, y, z_off + sl, ctx->id("EXTRA_IN")), id_EXTRA_IN, x, y);
                ctx->addPip(IdStringList::concat(ctx->getWireName(ff_d), ctx->getWireName(extra_in)),
                            ctx->id("FFDMUX_EXTRA"), extra_in, ff_d, 5, Loc(x, y, 0));
                extra_in_wires[sl] = extra_in;
            }

            WireId ff_out = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("FFQ")), ctx->id("FFQ"), x, y);
            ctx->addBelOutput(ff_bel, id_Q, ff_out);
            int ff_q_pin = 28 + 3 * sl;
            auto q_out = site_output_by_pin.find(ff_q_pin);
            if (q_out != site_output_by_pin.end()) {
                WireId so = q_out->second;
                ctx->addPip(IdStringList::concat(ctx->getWireName(so), ctx->getWireName(ff_out)),
                            ctx->id("OMUX"), ff_out, so, 1, Loc(x, y, 0));
            }

            ctx->addBelInput(ff_bel, id_CLK, clb_clk_wire);
            ctx->addBelInput(ff_bel, id_CE,  clb_ce_wire);
            ctx->addBelInput(ff_bel, id_R,   clb_sr_wire);
            n_ff++;

            // --- OXQ FF bel (DFF_OAQ, fed by MUX_OXQ / OAQ_SEL) ---
            // The OXQ FF has no dedicated routing pin — its Q feeds back into
            // the same slice's MUX_X (input I1, AMX_SEL=01) and only exits the
            // slice via the xMUX site-output pin (27 + 3*sl), which is shared
            // with the combinational MUX_X output. We model this by adding a
            // pip from OXQ_Q into the xMUX site-output wire in parallel with
            // the existing LUT→xMUX pip; the router picks one of the two and
            // the xMUX resource is therefore mutually exclusive between OXQ
            // and the slice's combinational MUX_X output. The isBelLocationValid
            // path enforces the corresponding fanout invariant.
            int oxq_z = z_off + FF_OXQ_Z_BASE + sl;
            BelId oxq_bel = ctx->addBel(
                h.xyz_id(x, y, oxq_z, ctx->idf("%sOFF", prefix)),
                id_SLG_FF, Loc(x, y, oxq_z), false, false);
            // Reuse the per-slice FF decal — the OXQ FF visually overlaps the
            // XQ FF box; users rarely need to tell them apart on the placed-svg.
            ctx->setBelDecal(oxq_bel, x, y, IdStringList(ff_decal_name(ctx, sl)));
            bel_to_site[oxq_bel] = site_idx;
            oxq_ff_bels[sl] = oxq_bel;
            site_bels.push_back(oxq_bel);

            WireId oxq_d = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("FFD_OXQ")), ctx->id("FFD_OXQ"), x, y);
            ctx->addBelInput(oxq_bel, id_D, oxq_d);
            // MUX_OXQ source set: {XI, XQ, OX0}. The three pips below model
            // the three OAQ_SEL one-hot values:
            //   OAQ_SEL=010 → XQ source: this slice's XQ FF feedback (vendor default)
            //   OAQ_SEL=100 → OX0 source: this slice's LUT5_0 output (approximated as
            //                  the slice's combinational LUT output wire)
            //   OAQ_SEL=001 → XI source: any site-input pin
            // Cost-bias (Phase 2): rank by silicon idiom. The XQ-feedback
            // path (OAQ_SEL=010, vendor default) and the LUT5_0 / OX0 path
            // (=100) stay cheapest; the site-input XI path (=001) costs
            // slightly more so the router prefers the in-cluster feedback
            // when both are reachable.  This addresses the Phase 1 litmus
            // finding that OAQ_SEL=001 was being chosen over =010 because
            // all sources shared cost 1.
            ctx->addPip(IdStringList::concat(ctx->getWireName(oxq_d), ctx->getWireName(ff_out)),
                        ctx->id("OAQMUX_XQ"), ff_out, oxq_d, 1, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(oxq_d), ctx->getWireName(lut_out)),
                        ctx->id("OAQMUX_OX0"), lut_out, oxq_d, 1, Loc(x, y, 0));
            for (auto &si : site_inputs)
                ctx->addPip(IdStringList::concat(ctx->getWireName(oxq_d), ctx->getWireName(si)),
                            ctx->id("OAQMUX_XI"), si, oxq_d, 2, Loc(x, y, 0));

            WireId oxq_out = ctx->addWire(
                h.xyz_id(x, y, z_off + sl, ctx->id("FFQ_OXQ")), ctx->id("FFQ_OXQ"), x, y);
            ctx->addBelOutput(oxq_bel, id_Q, oxq_out);
            // OXQ Q exits via the slice's xMUX site-output pin (27 + 3*sl).
            // The pip is added in parallel with the existing LUT→xMUX pip;
            // the router can pick only one source for the xMUX wire, which
            // expresses the silicon's MUX_X resource conflict naturally.
            int xmux_pin = 27 + 3 * sl;
            auto xmux_out = site_output_by_pin.find(xmux_pin);
            if (xmux_out != site_output_by_pin.end()) {
                WireId so = xmux_out->second;
                ctx->addPip(IdStringList::concat(ctx->getWireName(so), ctx->getWireName(oxq_out)),
                            ctx->id("AMXMUX_OXQ"), oxq_out, so, 1, Loc(x, y, 0));
            }

            ctx->addBelInput(oxq_bel, id_CLK, clb_clk_wire);
            ctx->addBelInput(oxq_bel, id_CE,  clb_ce_wire);
            ctx->addBelInput(oxq_bel, id_R,   clb_sr_wire);
            n_ff++;

            // --- Per-slice RAMSRL bel (L1 memory CLBs only) ---
            // Preserves the per-slice SRL16E view. RAM32X1D gets its own
            // site-wide bel below. We model this with LUT input wires (I0-I5),
            // an explicit read-address port (DPRA0-DPRA4), and WE/CLK controls.
            if (is_l1) {
                int ram_z = z_off + RAM_Z_BASE + sl;
                BelId ram_bel = ctx->addBel(
                    h.xyz_id(x, y, ram_z, ctx->idf("%sRAM", prefix)),
                    id_SLG_RAM, Loc(x, y, ram_z), false, false);
                ctx->setBelDecal(ram_bel, x, y, IdStringList(ram_decal_name(ctx, sl)));
                bel_to_site[ram_bel] = site_idx;
                srl_bels[sl] = ram_bel;
                site_bels.push_back(ram_bel);

                // Reuse the LUTI wires — same physical resource as LUT inputs.
                for (int k = 0; k < LUT_K; k++)
                    ctx->addBelInput(ram_bel, lut_pin_id(ctx, k),
                                     ctx->getBelPinWire(lut_bel, lut_pin_id(ctx, k)));

                // Dual-port read-address inputs on independent site-input fan-in.
                static const IdString dpra_ports[] = {id_DPRA0, id_DPRA1, id_DPRA2, id_DPRA3, id_DPRA4};
                for (int k = 0; k < 5; k++) {
                    WireId dpra_in = ctx->addWire(
                        h.xyz_id(x, y, ram_z, ctx->idf("DPRA%d", k)),
                        ctx->idf("DPRA%d", k), x, y);
                    ctx->addBelInput(ram_bel, dpra_ports[k], dpra_in);
                    for (auto &si : site_inputs)
                        ctx->addPip(IdStringList::concat(ctx->getWireName(dpra_in), ctx->getWireName(si)),
                                    ctx->id("MUX"), si, dpra_in, 1, Loc(x, y, 0));
                }

                // RAM data output reuses the LUT output wire.
                ctx->addBelOutput(ram_bel, id_SPO, lut_out);

                WireId dpo_out = ctx->addWire(
                    h.xyz_id(x, y, ram_z, ctx->id("DPO")), ctx->id("DPO"), x, y);
                ctx->addBelOutput(ram_bel, id_DPO, dpo_out);
                for (auto &so : site_outputs)
                    ctx->addPip(IdStringList::concat(ctx->getWireName(so), ctx->getWireName(dpo_out)),
                                ctx->id("OMUX"), dpo_out, so, 1, Loc(x, y, 0));

                ctx->addBelInput(ram_bel, id_WCLK, clb_clk_wire);
                ctx->addBelInput(ram_bel, id_WE,   clb_we_wire);
                n_ram++;
            }
        }

        // --- Carry chain wires ---
        // Create CIN/COUT per CLB. CIN feeds the chain at slice A; COUT exits
        // from slice D. Only L0 sites get inter-CLB carry pips.
        {
            WireId cin_wire = ctx->addWire(
                h.xyz_id(x, y, z_off, ctx->id("CIN")), id_CIN, x, y);
            WireId cout_wire = ctx->addWire(
                h.xyz_id(x, y, z_off, ctx->id("COUT")), id_COUT, x, y);
            ctx->addBelInput(lut_bels[0],  id_CIN,  cin_wire);
            ctx->addBelOutput(lut_bels[3], id_COUT, cout_wire);
            if (!is_l1)
                carry_wires_by_grid[{x,y}] = {cin_wire, cout_wire, x, y, site.sx, site.sy};
        }

        // --- Phase 5: per-CLB MUXF7 bels (mux_a7 + mux_c7) ---
        // Silicon's mux_a7 selects between slice A's LUT output (X) and
        // slice B's LUT output, controlled by slice A's I input (AI), and
        // its result feeds slice A's EXTRA_IN port (clbl.v lines 105-108).
        // Mirror for mux_c7 between slices C and D, feeding slice C's
        // EXTRA_IN.  mux_x8 (MUXF8 between A7/C7 → slice B's EXTRA_IN) is
        // deferred to a follow-up session.
        //
        // Each MUXF7 bel exposes I0/I1/S input pins and an O output. I0/I1
        // wires are bel-local and pip'd from the adjacent slices' lut_out
        // wires (so pack_muxes can use cluster constraints to force the
        // driving LUTs onto the correct slices). S is fed by any site
        // input (the encoder picks up the AI/CI net via the same MUX_x6
        // mechanism XI uses). The output wire pips into the slice's
        // EXTRA_IN wire, giving the router a path EXTRA_IN ← O ← I0/I1
        // ← lut_out which the per-slice FFDMUX_EXTRA pip already feeds
        // into MUX_XQ.I0 (AQ1_SEL_x[0] = EXTRA_IN).
        // MUXF7 bel z = z_off + slice_lo + MUXF7_Z_BASE so the relative
        // offset from the low slice's LUT bel (z = z_off + slice_lo) is the
        // same MUXF7_Z_BASE for both mux_a7 (slices A+B) and mux_c7 (C+D).
        // One cluster layout — LUT_lo @ constr_z=0, LUT_hi @ constr_z=1,
        // MUXF7 @ constr_z=MUXF7_Z_BASE — fits both mux slots, so the
        // placer can pick whichever is free.
        auto add_muxf7 = [&](int slice_lo, int slice_hi,
                             int target_slice, const char *suffix) -> WireId {
            if (extra_in_wires[target_slice] == WireId())
                return WireId();  // slice D has no EXTRA_IN
            int mux_z = z_off + slice_lo + MUXF7_Z_BASE;
            BelId mux_bel = ctx->addBel(
                h.xyz_id(x, y, mux_z, ctx->idf("%sMUXF7%s", prefix, suffix)),
                id_SLG_MUX_F7, Loc(x, y, mux_z), false, false);
            // Reuse the slice's LUT decal so a placed MUXF7 shows up roughly
            // where its inputs live (no dedicated decal yet).
            ctx->setBelDecal(mux_bel, x, y, IdStringList(lut_decal_name(ctx, target_slice)));
            bel_to_site[mux_bel] = site_idx;
            site_bels.push_back(mux_bel);

            WireId mux_o = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXFO")), ctx->id("MUXFO"), x, y);
            ctx->addBelOutput(mux_bel, id_O, mux_o);

            // I0 input wire pip'd from slice_lo's lut_out; I1 from slice_hi's.
            WireId i0_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXFI0")), ctx->id("MUXFI"), x, y);
            WireId i1_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXFI1")), ctx->id("MUXFI"), x, y);
            ctx->addBelInput(mux_bel, id_I0, i0_wire);
            ctx->addBelInput(mux_bel, id_I1, i1_wire);
            ctx->addPip(IdStringList::concat(ctx->getWireName(i0_wire), ctx->getWireName(lut_out_wires[slice_lo])),
                        ctx->id("MUXFIN"), lut_out_wires[slice_lo], i0_wire, 1, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(i1_wire), ctx->getWireName(lut_out_wires[slice_hi])),
                        ctx->id("MUXFIN"), lut_out_wires[slice_hi], i1_wire, 1, Loc(x, y, 0));

            // S input — any site-input pin can carry the select. Higher cost
            // than the cost-1 LUT inputs so the router doesn't accidentally
            // route a LUT result through S when a cheaper LUT-input pip
            // exists.
            WireId s_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXFS")), ctx->id("MUXFS"), x, y);
            ctx->addBelInput(mux_bel, id_S, s_wire);
            for (auto &si : site_inputs)
                ctx->addPip(IdStringList::concat(ctx->getWireName(s_wire), ctx->getWireName(si)),
                            ctx->id("MUX"), si, s_wire, 2, Loc(x, y, 0));

            // O → target slice's EXTRA_IN wire.
            ctx->addPip(
                IdStringList::concat(ctx->getWireName(extra_in_wires[target_slice]), ctx->getWireName(mux_o)),
                ctx->id("EXTRA_IN_DRIVE"), mux_o, extra_in_wires[target_slice], 1, Loc(x, y, 0));
            return mux_o;  // Phase 5b: mux_x8 cascades on these outputs.
        };
        WireId mux_a7_o = add_muxf7(/*slice_lo=*/0, /*slice_hi=*/1, /*target=*/0, "A");  // z = z_off+25
        WireId mux_c7_o = add_muxf7(/*slice_lo=*/2, /*slice_hi=*/3, /*target=*/2, "C");  // z = z_off+27

        // --- Phase 5b: per-CLB MUXF8 bel (mux_x8) ---
        // Silicon's mux_x8 = MUX2(A7, C7, sel=BI) — cascades the mux_a7
        // and mux_c7 outputs into slice B's EXTRA_IN (clbl.v line 108).
        // The 2 input pips come from the MUXF7 outputs already created
        // above (mux_a7_o, mux_c7_o), so a MUXF8 cluster requires both
        // MUXF7s to also be placed in the same CLB.
        if (mux_a7_o != WireId() && mux_c7_o != WireId() && extra_in_wires[1] != WireId()) {
            int mux_z = z_off + MUXF8_Z;
            BelId mux_bel = ctx->addBel(
                h.xyz_id(x, y, mux_z, ctx->idf("%sMUXF8", prefix)),
                id_SLG_MUX_F8, Loc(x, y, mux_z), false, false);
            ctx->setBelDecal(mux_bel, x, y, IdStringList(lut_decal_name(ctx, /*slice=*/1)));
            bel_to_site[mux_bel] = site_idx;
            site_bels.push_back(mux_bel);

            WireId mux_o = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXF8O")), ctx->id("MUXFO"), x, y);
            ctx->addBelOutput(mux_bel, id_O, mux_o);

            WireId i0_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXF8I0")), ctx->id("MUXFI"), x, y);
            WireId i1_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXF8I1")), ctx->id("MUXFI"), x, y);
            ctx->addBelInput(mux_bel, id_I0, i0_wire);
            ctx->addBelInput(mux_bel, id_I1, i1_wire);
            ctx->addPip(IdStringList::concat(ctx->getWireName(i0_wire), ctx->getWireName(mux_a7_o)),
                        ctx->id("MUXFIN"), mux_a7_o, i0_wire, 1, Loc(x, y, 0));
            ctx->addPip(IdStringList::concat(ctx->getWireName(i1_wire), ctx->getWireName(mux_c7_o)),
                        ctx->id("MUXFIN"), mux_c7_o, i1_wire, 1, Loc(x, y, 0));

            WireId s_wire = ctx->addWire(
                h.xyz_id(x, y, mux_z, ctx->id("MUXF8S")), ctx->id("MUXFS"), x, y);
            ctx->addBelInput(mux_bel, id_S, s_wire);
            for (auto &si : site_inputs)
                ctx->addPip(IdStringList::concat(ctx->getWireName(s_wire), ctx->getWireName(si)),
                            ctx->id("MUX"), si, s_wire, 2, Loc(x, y, 0));

            // O → slice B's EXTRA_IN wire.
            ctx->addPip(
                IdStringList::concat(ctx->getWireName(extra_in_wires[1]), ctx->getWireName(mux_o)),
                ctx->id("EXTRA_IN_DRIVE"), mux_o, extra_in_wires[1], 1, Loc(x, y, 0));
        }

        if (is_l1) {
            // --- Site-wide RAM32X1D bel ---
            // Grounded on the vendor-observed RBB6M shape:
            //   write address: D-side input ports (pins 18-21)
            //   read address:  C-side input ports (pins 12-15)
            //   D:             C6-style muxed input
            int ram32x1d_z = z_off + RAM32X1D_Z;
            BelId ram32x1d_bel = ctx->addBel(
                h.xyz_id(x, y, ram32x1d_z, ctx->idf("%sRAM32X1D", prefix)),
                id_SLG_RAM32X1D, Loc(x, y, ram32x1d_z), false, false);
            // Site-wide RAM32X1D occupies all 4 slices; render at slice-A
            // position with the wide RAM box so a placed RAM32X1D is visibly
            // distinct from a single-slice RAMSRL.
            ctx->setBelDecal(ram32x1d_bel, x, y, IdStringList(ram_decal_name(ctx, 0)));
            bel_to_site[ram32x1d_bel] = site_idx;
            site_bels.push_back(ram32x1d_bel);

            auto add_fixed_site_input = [&](IdString port, int pin_idx) {
                WireId w = ctx->addWire(h.xyz_id(x, y, ram32x1d_z, port), port, x, y);
                ctx->addBelInput(ram32x1d_bel, port, w);
                if (pin_idx >= 0 && pin_idx < int(site_input_by_pin.size()) &&
                    site_input_by_pin.at(pin_idx) != WireId()) {
                    WireId si = site_input_by_pin.at(pin_idx);
                    ctx->addPip(IdStringList::concat(ctx->getWireName(w), ctx->getWireName(si)),
                                ctx->id("MUX"), si, w, 1, Loc(x, y, 0));
                }
            };
            auto add_muxed_site_input = [&](IdString port) {
                WireId w = ctx->addWire(h.xyz_id(x, y, ram32x1d_z, port), port, x, y);
                ctx->addBelInput(ram32x1d_bel, port, w);
                for (auto &si : site_inputs)
                    ctx->addPip(IdStringList::concat(ctx->getWireName(w), ctx->getWireName(si)),
                                ctx->id("MUX"), si, w, 1, Loc(x, y, 0));
            };

            // Write address (D1-D5 → A0-A4).
            add_fixed_site_input(id_A0, 18);
            add_fixed_site_input(id_A1, 19);
            add_fixed_site_input(id_A2, 20);
            add_fixed_site_input(id_A3, 21);
            add_muxed_site_input(id_A4);

            // Read address (C1-C5 → DPRA0-DPRA4) and data input (C6 → D).
            add_fixed_site_input(id_DPRA0, 12);
            add_fixed_site_input(id_DPRA1, 13);
            add_fixed_site_input(id_DPRA2, 14);
            add_fixed_site_input(id_DPRA3, 15);
            add_muxed_site_input(id_DPRA4);
            add_muxed_site_input(id_D);

            // Outputs: DPO on slice C output, SPO on slice D output.
            ctx->addBelOutput(ram32x1d_bel, id_DPO, lut_out_wires[2]);
            ctx->addBelOutput(ram32x1d_bel, id_SPO, lut_out_wires[3]);

            ctx->addBelInput(ram32x1d_bel, id_WCLK, clb_clk_wire);
            ctx->addBelInput(ram32x1d_bel, id_WE,   clb_we_wire);

            // Site-wide RAM32X1D blocks FFs and per-slice SRL/RAM views (C/D
            // slices used for outputs; A/B remain available for helper logic).
            // Both XQ and OXQ FF bels conflict — the RAM occupies the entire
            // site (no slice-level FF is reachable while the RAM is bound).
            for (int sl = 0; sl < SLICES_PER_CLB; sl++) {
                add_bel_conflict(ram32x1d_bel, ff_bels[sl]);
                add_bel_conflict(ram32x1d_bel, oxq_ff_bels[sl]);
                add_bel_conflict(ram32x1d_bel, srl_bels[sl]);
            }
            add_bel_conflict(ram32x1d_bel, lut_bels[2]);
            add_bel_conflict(ram32x1d_bel, lut_bels[3]);
            n_ram++;
        }
    }

    // L0 and L1 are overlapping views of the same physical tile — any use of
    // one view blocks the other at the same grid coordinate.
    for (auto &kv : l1_bels_by_xy) {
        auto it = l0_bels_by_xy.find(kv.first);
        if (it == l0_bels_by_xy.end()) continue;
        for (BelId l1_bel : kv.second)
            for (BelId l0_bel : it->second)
                add_bel_conflict(l1_bel, l0_bel);
    }

    // Inter-CLB carry chain pips: adjacent CLBs in the same column differ by
    // dx=2 in grid X; the carry chain goes bottom-to-top (increasing sx).
    int carry_pips = 0;
    for (auto &[key, lo] : carry_wires_by_grid) {
        auto hi_it = carry_wires_by_grid.find({lo.grid_x + 2, lo.grid_y});
        if (hi_it == carry_wires_by_grid.end()) continue;
        auto &hi = hi_it->second;
        ctx->addPip(IdStringList::concat(ctx->getWireName(hi.cin), ctx->getWireName(lo.cout)),
                    ctx->id("CARRY"), lo.cout, hi.cin, 0, Loc(lo.grid_x, lo.grid_y, 0));
        carry_pips++;
    }
    log_info("slg47910: created %d inter-CLB carry chain pips\n", carry_pips);
    log_info("slg47910: created %d LUT bels, %d FF bels, %d RAM bels\n", n_lut, n_ff, n_ram);
}

void SLG47910Impl::init_iob_bels()
{
    log_info("slg47910: creating IOB bels...\n");
    int n = 0;
    for (auto &site : db.sites) {
        if (site.type != "L6") continue;
        BelId bel = ctx->addBel(
            h.xyz_id(site.x, site.y, 0, ctx->id("IOB")),
            id_SLG_IOB, Loc(site.x, site.y, 0), false, false);
        ctx->setBelDecal(bel, site.x, site.y, IdStringList(ctx->id("IOB")));
        auto coord = std::make_pair(site.x, site.y);
        if (iob_bels_by_coord.count(coord))
            log_error("slg47910: duplicate IOB site at (%d,%d)\n", site.x, site.y);
        iob_bels_by_coord[coord] = bel;

        // PAD port: synthetic wire connected to all site pins.
        // Site input pins = signals from routing (fabric→IOB = output pad).
        // Site output pins = signals to routing (IOB→fabric = input pad).
        WireId pad_wire = ctx->addWire(
            h.xyz_id(site.x, site.y, 0, ctx->id("PAD")), id_PAD, site.x, site.y);
        ctx->addBelPin(bel, id_PAD, pad_wire, PORT_INOUT);

        // East-edge output IOBs (gx=31) need an OE wire so VCC can be routed
        // to the output-enable mux input independently from the data path.
        WireId oe_wire;
        if (site.x == 31) {
            oe_wire = ctx->addWire(
                h.xyz_id(site.x, site.y, 0, ctx->id("OE")), id_OE, site.x, site.y);
            ctx->addBelPin(bel, id_OE, oe_wire, PORT_IN);
        }

        for (auto &pin : site.pins) {
            WireId w = node_to_wire[pin.node];
            int pin_idx = parse_site_pin_index(pin.name);
            if (pin.dir == 'O') {
                if (pin_idx >= 0)
                    wire_to_site_output_pin[w] = pin_idx;
                ctx->addPip(IdStringList::concat(ctx->getWireName(w), ctx->getWireName(pad_wire)),
                            ctx->id("IOPAD"), pad_wire, w, 1, Loc(site.x, site.y, 0));
            } else {
                if (pin_idx >= 0)
                    wire_to_site_pin[w] = pin_idx;
                ctx->addPip(IdStringList::concat(ctx->getWireName(pad_wire), ctx->getWireName(w)),
                            ctx->id("IOPAD"), w, pad_wire, 1, Loc(site.x, site.y, 0));
                // East-edge: also add a pip from this I-pin to the OE wire so
                // the router can route VCC to OE through a separate site input.
                if (oe_wire != WireId())
                    ctx->addPip(IdStringList::concat(ctx->getWireName(oe_wire), ctx->getWireName(w)),
                                ctx->id("IOPAD"), w, oe_wire, 1, Loc(site.x, site.y, 0));
            }
        }
        n++;
    }
    log_info("slg47910: created %d IOB bels\n", n);
}

void SLG47910Impl::init_clock_bels()
{
    log_info("slg47910: creating dedicated clock-input bels...\n");
    struct ClockBelSpec { const char *token; int domain, index; const char *side; };
    static const ClockBelSpec specs[] = {
        {"W0", 0, 0, "W"},
        {"N0", 0, 0, "N"},
        {"N1", 1, 1, "N"},
    };

    // When the chipdb is built with --enable-clock-fabric, each CLKPAD_<tok>
    // wire is a real node-backed wire connected via CLKPAD→spine→row→CLBCLK
    // pips to every L0/L1 CLK input. Use that wire as the BEL's PAD output
    // so the router has to traverse the chipdb's clock fabric. Without the
    // chipdb wires (legacy flow), fall back to a local wire + a GCLK
    // shortcut pip to every global_clock_sinks entry.
    std::map<std::string, WireId> chipdb_pad_wires;
    for (auto &w : db.wires) {
        const std::string &n = w.name;
        if (n.rfind("CLKPAD_", 0) == 0)
            chipdb_pad_wires[n.substr(7)] = node_to_wire[w.id];
    }
    const bool clock_fabric_enabled = !chipdb_pad_wires.empty();

    int n_bels = 0, n_pips = 0;
    for (int i = 0; i < int(sizeof(specs) / sizeof(specs[0])); i++) {
        const auto &spec = specs[i];
        int z = CLKIN_Z_BASE + i;
        WireId pad_wire;
        auto chipdb_it = chipdb_pad_wires.find(spec.token);
        if (chipdb_it != chipdb_pad_wires.end()) {
            pad_wire = chipdb_it->second;
        } else {
            IdString wire_name = ctx->idf("CLKPAD_%s", spec.token);
            pad_wire = ctx->addWire(h.xyz_id(0, 0, z, wire_name), wire_name, 0, 0);
        }
        BelId bel = ctx->addBel(
            h.xyz_id(0, 0, z, ctx->idf("CLKIN_%s", spec.token)),
            id_SLG_CLKIN, Loc(0, 0, z), false, false);
        // All CLKIN bels live at (0,0); render at the chip-corner tile.
        ctx->setBelDecal(bel, 0, 0, IdStringList(ctx->id("CLKIN")));
        ctx->addBelOutput(bel, id_PAD, pad_wire);
        clock_bels_by_token[spec.token] = bel;
        n_bels++;

        if (clock_fabric_enabled)
            continue;
        for (WireId sink : global_clock_sinks) {
            ctx->addPip(IdStringList::concat(ctx->getWireName(sink), ctx->getWireName(pad_wire)),
                        ctx->id("GCLK"), pad_wire, sink, 0, Loc(0, 0, z));
            n_pips++;
        }
    }
    if (clock_fabric_enabled)
        log_info("slg47910: created %d dedicated clock-input bels "
                 "(clock fabric in chipdb; no GCLK shortcut pips)\n", n_bels);
    else
        log_info("slg47910: created %d dedicated clock-input bels and %d global clock pips\n",
                 n_bels, n_pips);
}

SLG47910Impl::ClockConstraint
SLG47910Impl::parse_clock_constraint(const std::string &token, int lineno) const
{
    ClockConstraint cc;
    cc.token = token;
    if (token == "W0") {
        cc.side = "W"; cc.domain = 0; cc.index = 0;
    } else if (token == "N0") {
        cc.side = "N"; cc.domain = 0; cc.index = 0;
    } else if (token == "N1") {
        cc.side = "N"; cc.domain = 1; cc.index = 1;
    } else {
        log_error("slg47910: %s:%d: unsupported clock token '%s' "
                  "(expected one of W0, N0, N1)\n",
                  pin_constraints_path.c_str(), lineno, token.c_str());
    }
    auto bel_it = clock_bels_by_token.find(token);
    if (bel_it == clock_bels_by_token.end())
        log_error("slg47910: internal error: missing clock BEL for token '%s'\n", token.c_str());
    cc.bel = bel_it->second;
    return cc;
}

void SLG47910Impl::load_pin_constraints()
{
    constrained_io_bels.clear();
    constrained_clock_bels.clear();
    if (pin_constraints_path.empty())
        return;

    std::ifstream f(pin_constraints_path);
    if (!f)
        log_error("slg47910: cannot open pin constraints '%s'\n", pin_constraints_path.c_str());

    const auto &gpio_tbl = gpio_name_to_coord();
    std::map<std::pair<int,int>, std::string> claimed_coords;
    std::map<std::string, std::string> claimed_clock_tokens;
    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.erase(comment);

        std::istringstream iss(line);
        std::string keyword, port_name, extra;
        if (!(iss >> keyword))
            continue;
        if (keyword != "set_io" && keyword != "set_clk")
            log_error("slg47910: %s:%d: expected 'set_io' or 'set_clk'\n",
                      pin_constraints_path.c_str(), lineno);
        if (!(iss >> port_name))
            log_error("slg47910: %s:%d: expected port name after '%s'\n",
                      pin_constraints_path.c_str(), lineno, keyword.c_str());
        if (constrained_io_bels.count(port_name) || constrained_clock_bels.count(port_name))
            log_error("slg47910: %s:%d: duplicate constraint for port '%s'\n",
                      pin_constraints_path.c_str(), lineno, port_name.c_str());

        if (keyword == "set_clk") {
            std::string token;
            if (!(iss >> token) || (iss >> extra))
                log_error("slg47910: %s:%d: expected 'set_clk <port> <W0|N0|N1>'\n",
                          pin_constraints_path.c_str(), lineno);
            for (auto &ch : token) ch = toupper(ch);
            auto claimed_it = claimed_clock_tokens.find(token);
            if (claimed_it != claimed_clock_tokens.end())
                log_error("slg47910: %s:%d: dedicated clock token '%s' already claimed by port '%s'\n",
                          pin_constraints_path.c_str(), lineno, token.c_str(), claimed_it->second.c_str());
            constrained_clock_bels[port_name] = parse_clock_constraint(token, lineno);
            claimed_clock_tokens[token] = port_name;
            continue;
        }

        // Accept "set_io <port> GPIO<n>" or "set_io <port> <gx> <gy>".
        int gx, gy;
        std::string loc_token;
        if (!(iss >> loc_token))
            log_error("slg47910: %s:%d: expected GPIO name or coordinate after port name\n",
                      pin_constraints_path.c_str(), lineno);

        std::string loc_upper = loc_token;
        for (auto &ch : loc_upper) ch = toupper(ch);

        auto gpio_it = gpio_tbl.find(loc_upper);
        if (gpio_it != gpio_tbl.end()) {
            gx = gpio_it->second.first;
            gy = gpio_it->second.second;
            if (iss >> extra)
                log_error("slg47910: %s:%d: unexpected token after GPIO name '%s'\n",
                          pin_constraints_path.c_str(), lineno, loc_token.c_str());
        } else {
            try { gx = std::stoi(loc_token); }
            catch (...) {
                log_error("slg47910: %s:%d: '%s' is not a valid GPIO name or coordinate\n",
                          pin_constraints_path.c_str(), lineno, loc_token.c_str());
            }
            if (!(iss >> gy) || (iss >> extra))
                log_error("slg47910: %s:%d: expected 'set_io <port> <gx> <gy>' or "
                          "'set_io <port> GPIO<n>'\n",
                          pin_constraints_path.c_str(), lineno);
        }

        // The two numbers are the vendor's IOB name RBB_<row>_<col>, so the IOB
        // sits at graph (x=col, y=row): GPIO10 = RBB_31_25 is the north-edge IOB
        // at (25,31) (the vendor routes GPIO10 there; chip_def lists GPIO8..15 as
        // fixed IOs (8..27, 31)).
        auto coord = std::make_pair(gy, gx);
        auto bel_it = iob_bels_by_coord.find(coord);
        if (bel_it == iob_bels_by_coord.end())
            log_error("slg47910: %s:%d: (%d,%d) is not an SLG47910 IOB site\n",
                      pin_constraints_path.c_str(), lineno, gx, gy);

        auto claimed_it = claimed_coords.find(coord);
        if (claimed_it != claimed_coords.end())
            log_error("slg47910: %s:%d: IOB (%d,%d) already claimed by port '%s'\n",
                      pin_constraints_path.c_str(), lineno, gx, gy, claimed_it->second.c_str());

        constrained_io_bels[port_name] = bel_it->second;
        claimed_coords[coord] = port_name;
    }

    log_info("slg47910: loaded %zu IOB constraints and %zu clock constraints from %s\n",
             constrained_io_bels.size(), constrained_clock_bels.size(),
             pin_constraints_path.c_str());
}

NEXTPNR_NAMESPACE_END
