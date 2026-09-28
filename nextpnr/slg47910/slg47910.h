/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *  Shared implementation state — included by all slg47910_*.cc sub-modules.
 */

#ifndef SLG47910_H
#define SLG47910_H

#include <map>
#include <string>
#include <vector>

#include "log.h"
#include "nextpnr.h"
#include "viaduct_api.h"
#include "viaduct_helpers.h"

// constid table — each sub-module that references id_XXX symbols must define
// VIADUCT_CONSTIDS before including viaduct_constids.h.  Only the translation
// unit that also defines GEN_INIT_CONSTIDS actually emits the symbol; the
// others just reference it via extern linkage.  See slg47910.cc.
#define VIADUCT_CONSTIDS "viaduct/slg47910/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

// ---------------------------------------------------------------------------
// Chipdb
// ---------------------------------------------------------------------------

struct ChipDB
{
    struct Pin { std::string name; int node; char dir; /* 'I' or 'O' */ };
    struct Site { int id; std::string type; int x, y, z; int sx, sy; std::vector<Pin> pins; };
    struct Wire { int id; std::string name; int x, y; };
    // ``type`` carries the optional chipdb pip-type tag (Option A step C).
    // Empty string = default "PIP" (data routing). Currently the only other
    // value emitted by ``tools/gen_chipdb.py`` is "CLKPIP" for clock-fabric
    // pips; arch_init.cc maps it to ``ctx->id(type)`` and emit.cc uses it to
    // route the pip into the right ``routing.json`` section.
    struct Pip  { int src, dst, cost; std::string type; };

    int grid_w = 32, grid_h = 32;
    std::vector<Site> sites;
    std::vector<Wire> wires;
    std::vector<Pip>  pips;

    void load(const std::string &path);
};

// ---------------------------------------------------------------------------
// Bel Z-index layout
// ---------------------------------------------------------------------------

static const int SLICES_PER_CLB = 4;
static const int LUT_K          = 6;

// 8-FF model (Phase 1 of the slice-output mux rewrite): each slice has
//   - an XQ FF (the "primary" DFF_AQ, driven by MUX_XQ / AQ1_SEL)
//   - an OXQ FF (the "secondary" DFF_OAQ, driven by MUX_OXQ / OAQ_SEL)
// Only the XQ FF has a dedicated routing pin (xQ at site pin 28+3*sl). The
// OXQ FF Q feeds back into the same slice's MUX_X (input I1, AMX_SEL=01)
// and only escapes the slice via the xMUX pin (27+3*sl), which is shared
// with the combinational MUX_X output. See docs/silicon/reference.md §3.1.
//
// L0 sites (z_off=0): z=0..3 LUTs, z=4..7 XQ FFs, z=21..24 OXQ FFs.
// L1 sites (z_off=8): z=8..11 LUTs, z=12..15 XQ FFs, z=16..19 SRL/RAM,
//                     z=20 RAM32X1D, z=29..32 OXQ FFs.
// Extended-band Z layout (DP2): keeps XQ Z range stable for the legacy
// path while OXQ lives at a separate band so cluster constr_z is unambiguous.
static const int LUT_Z_BASE      = 0;
static const int FF_XQ_Z_BASE    = 4;
static const int FF_OXQ_Z_BASE   = 21;
static const int FF_Z_BASE       = FF_XQ_Z_BASE;  // backward-compat alias
static const int RAM_Z_BASE      = 8;   // relative to z_off, L1 only
static const int RAM32X1D_Z      = 12;  // relative to z_off, L1 only
// Phase 5 — CLB-wide MUXF7 muxes per silicon clbl.v lines 105-108.
// mux_a7 at z = slice_lo(=0) + MUXF7_Z_BASE = 25 feeds slice A's
// EXTRA_IN; mux_c7 at z = slice_lo(=2) + MUXF7_Z_BASE = 27 feeds
// slice C's EXTRA_IN. Phase 5b — MUXF8 / mux_x8 at z = MUXF8_Z = 28
// cascades mux_a7's and mux_c7's outputs into slice B's EXTRA_IN.
static const int MUXF7_Z_BASE    = 25;
static const int MUXF8_Z         = 28;
static const int CLKIN_Z_BASE    = 100;

// ---------------------------------------------------------------------------
// SLG47910Impl — the full Viaduct implementation
// ---------------------------------------------------------------------------

struct SLG47910Impl : ViaductAPI
{
    SLG47910Impl(const dict<std::string, std::string> &args);
    ~SLG47910Impl() {}

    // ViaductAPI overrides
    void init(Context *ctx) override;
    void pack() override;
    void prePlace() override;
    void preRoute() override;
    void postRoute() override;

    BoundingBox getRouteBoundingBox(WireId src, WireId dst) const override;
    delay_t     estimateDelay(WireId src, WireId dst)       const override;

    bool checkBelAvail(BelId bel)                               const override;
    bool checkPipAvailForNet(PipId pip, const NetInfo *net)     const override;
    bool isBelLocationValid(BelId bel, bool explain_invalid)    const override;
    IdString getBelBucketForCellType(IdString cell_type)        const override;
    bool isValidBelForCellType(IdString cell_type, BelId bel)   const override;

    // -----------------------------------------------------------------------
    // Shared state (accessed by all sub-modules)
    // -----------------------------------------------------------------------

    ViaductHelpers h;
    ChipDB db;

    std::string chipdb_path, pin_constraints_path, sdc_out, route_out;

    // Debug knob: pin every single-fanout LUT+FF cluster to an RBB target.
    // Set via ``--vopt force_ff_loc=<rbb_x>,<rbb_y>``. Negative values disable.
    int force_ff_rbb_x = -1;
    int force_ff_rbb_y = -1;

    // node_id → WireId mapping built during init.
    std::vector<WireId> node_to_wire;

    // BelId → chipdb site index (for SDC emission).
    dict<BelId, int> bel_to_site;

    // Fabric IOB BEL lookup keyed by (gx, gy).
    std::map<std::pair<int,int>, BelId> iob_bels_by_coord;

    // Requested top-level port → locked IOB BEL.
    std::map<std::string, BelId> constrained_io_bels;

    struct ClockConstraint {
        std::string token;
        std::string side;
        int domain = 0;
        int index  = 0;
        BelId bel;
    };
    // Requested top-level clock port → dedicated clock input selection.
    std::map<std::string, ClockConstraint> constrained_clock_bels;

    // Dedicated clock token → synthetic clock BEL.
    std::map<std::string, BelId> clock_bels_by_token;

    // Cross-BEL exclusivity rules (L0/L1 overlap + RAM/SRL exclusion).
    dict<BelId, std::vector<BelId>> bel_conflicts;

    // WireId → site input pin index (for MUX select derivation).
    dict<WireId, int> wire_to_site_pin;

    // WireId → site output pin index (for IOB input-lane export).
    dict<WireId, int> wire_to_site_output_pin;

    // Shared CLB clock sink wires driven by dedicated clock-input BELs.
    std::vector<WireId> global_clock_sinks;

    // Carry chain infrastructure: COUT/CIN wires per CLB, keyed by grid (x,y).
    struct CarryWires { WireId cin, cout; int grid_x, grid_y; int sx, sy; };
    std::map<std::pair<int,int>, CarryWires> carry_wires_by_grid;

    // -----------------------------------------------------------------------
    // Helpers (called from sub-modules)
    // -----------------------------------------------------------------------

    bool ff_is_set_type(const CellInfo *ci) const;
    bool ff_has_sr(const CellInfo *ci) const;
    bool cell_param_true(const CellInfo *ci, IdString key) const;
    int64_t cell_param_int(const CellInfo *ci, IdString key, int64_t default_value = 0) const;
    std::string cell_param_string(const CellInfo *ci, IdString key) const;
    bool is_native_ram32m_cell(const CellInfo *ci) const;
    void add_timing_models();

    static int parse_site_pin_index(const std::string &name);
    static int parse_edge_d0_branch(const std::string &wire_name);
    static std::string json_escape(const std::string &s);

    // GPIO name → (gx, gy) fabric coordinate table.
    static const std::map<std::string, std::pair<int,int>> &gpio_name_to_coord();

    WireId trace_bound_uphill_wire(WireId sink_wire) const;
    WireId trace_bound_downhill_wire(WireId source_wire) const;

    int trace_site_input_pin(WireId sink_wire) const;
    int trace_site_input_pin_from_bel(BelId bel, IdString port) const;
    int trace_site_output_pin(WireId source_wire) const;
    int trace_site_output_pin_from_bel(BelId bel, IdString port) const;

    std::string trace_iob_pad_in_sm(WireId sink_wire) const;
    std::string trace_iob_pad_in_sm_from_bel(BelId bel, IdString port) const;

    std::string connected_net_name(const CellInfo *ci, IdString port) const;

    ClockConstraint parse_clock_constraint(const std::string &token, int lineno) const;

    // arch_init.cc
    void add_bel_conflict(BelId a, BelId b);
    void init_routing_wires();
    void init_routing_pips();
    static IdString lut_pin_id(Context *ctx, int input_idx);
    void init_clb_bels();
    void init_iob_bels();
    void init_clock_bels();
    void load_pin_constraints();
    // Decal registration for nextpnr's `--placed-svg` / `--routed-svg` / GUI.
    // Registers one box per bel type + a thin wire decal, so the SVG writer
    // has graphics to emit. Called from init() before any addBel/addWire.
    void init_decals();

    // pack.cc
    void pack_io();
    void pack_luts();
    void pack_ffs();
    void pack_lut_ff_clusters();
    void pack_lutram();
    void pack_srl();
    void pack_muxes();
    void pack_carry();
    void pack_constants();

    // emit.cc
    void emit_placement();
    void emit_routing();
};

NEXTPNR_NAMESPACE_END

#endif // SLG47910_H
