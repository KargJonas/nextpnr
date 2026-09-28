/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *
 *  Architecture:
 *    140 CLBs (L0), 40 memory CLBs (L1, subset of L0 locations), 184 IOBs (L6).
 *    Each CLB has 4 slices (A-D), each with a LUT6 + FF.
 *    24 routing input pins, 12 routing output pins per CLB.
 *    Internal crossbar muxes connect any routing input to any LUT input.
 *
 *  Bel model (per L0 CLB, z_off=0):
 *    z=0..3  : SLG_LUT (LUT6 with 6 inputs + 1 output)
 *    z=4..7  : SLG_FF  (flip-flop with D, Q, C, CE, R)
 *
 *  Bel model (per L1 memory CLB, z_off=8):
 *    z=8..11  : SLG_LUT       (same as L0 but with constrained routing)
 *    z=12..15 : SLG_FF        (same as L0)
 *    z=16..19 : SLG_RAM       (per-slice SRL16E view)
 *    z=20     : SLG_RAM32X1D  (site-wide RAM32X1D view)
 *
 *  Internal wires + pips model the crossbar.
 *
 *  IOB model:
 *    z=0      : SLG_IOB   (PAD port)
 *    z=100+   : SLG_CLKIN (dedicated global clock input port)
 *
 *  Implementation is split across:
 *    chipdb.cc    — ChipDB JSON loader
 *    arch_init.cc — routing wire/pip/bel setup, pin constraints
 *    pack.cc      — cell packing (IO, LUT, FF, RAM, SRL, CARRY4, MUXF)
 *    emit.cc      — post-route JSON export (placement + routing)
 *    slg47910.cc  — ViaductAPI shell + shared helper implementations
 */

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

#define GEN_INIT_CONSTIDS
#include "slg47910.h"

NEXTPNR_NAMESPACE_BEGIN

// ---------------------------------------------------------------------------
// SLG47910Impl — constructor and ViaductAPI overrides
// ---------------------------------------------------------------------------

SLG47910Impl::SLG47910Impl(const dict<std::string, std::string> &args)
{
    for (auto &a : args) {
        if (a.first == "chipdb")
            chipdb_path = a.second;
        else if (a.first == "pin_constraints")
            pin_constraints_path = a.second;
        else if (a.first == "sdc_out")
            sdc_out = a.second;
        else if (a.first == "route_out")
            route_out = a.second;
        else if (a.first == "force_ff_loc") {
            size_t comma = a.second.find(',');
            if (comma == std::string::npos)
                log_error("slg47910: force_ff_loc='%s' must be 'rbb_x,rbb_y'\n", a.second.c_str());
            force_ff_rbb_x = std::stoi(a.second.substr(0, comma));
            force_ff_rbb_y = std::stoi(a.second.substr(comma + 1));
        }
        else
            log_error("unrecognised slg47910 option '%s'\n", a.first.c_str());
    }
    if (chipdb_path.empty())
        log_error("slg47910: --vopt chipdb=<path> is required\n");
}

void SLG47910Impl::init(Context *ctx)
{
    init_uarch_constids(ctx);
    ViaductAPI::init(ctx);
    h.init(ctx);

    log_info("slg47910: loading chipdb %s\n", chipdb_path.c_str());
    db.load(chipdb_path);
    log_info("slg47910: chipdb has %zu sites, %zu wires, %zu pips\n",
             db.sites.size(), db.wires.size(), db.pips.size());

    // z range: up to 16 bels + up to 16*6 LUTI wires for L0+L1.
    h.resize_ids(db.grid_w, db.grid_h, 128);
    init_decals();
    init_routing_wires();
    init_routing_pips();
    init_clb_bels();
    init_iob_bels();
    init_clock_bels();
    load_pin_constraints();
}

void SLG47910Impl::pack()
{
    auto type_has_prefix = [&](const CellInfo *ci, const char *prefix) {
        const std::string type = ci->type.str(ctx);
        return type.rfind(prefix, 0) == 0;
    };

    for (auto &[name, ci] : ctx->cells) {
        if (type_has_prefix(ci.get(), "RAMB") || type_has_prefix(ci.get(), "FIFO")) {
            log_error("slg47910: BRAM primitive %s (%s) is not supported; "
                      "the current chipdb/uarch does not expose BRAM BELs. "
                      "Keep `-nobram` enabled or remap this storage into supported LUTRAM.\n",
                      ctx->nameOf(ci.get()), ci->type.c_str(ctx));
        }
        if (type_has_prefix(ci.get(), "PLLE2") || type_has_prefix(ci.get(), "MMCME2")) {
            log_error("slg47910: PLL primitive %s (%s) is not supported; "
                      "the current chipdb/uarch does not expose PLL BELs.\n",
                      ctx->nameOf(ci.get()), ci->type.c_str(ctx));
        }
    }

    pack_io();
    pack_carry();
    pack_luts();
    pack_ffs();
    pack_lut_ff_clusters();
    pack_lutram();
    pack_srl();
    pack_muxes();
    pack_constants();
}

void SLG47910Impl::prePlace()
{
    add_timing_models();
}

void SLG47910Impl::preRoute()
{
    // Use router2 by default. Router1's A* estimate is inadmissible for this
    // architecture: site-local wires sit at CLB grid coordinates while routing
    // nodes they connect to may be at offset positions (±1 in each axis),
    // causing the heuristic to prune valid paths. Router2 (PathFinder-style)
    // completes blinky in 0.04 s; router1's BFS fallback takes 1000+ seconds.
    ctx->settings[ctx->id("router")] = std::string("router2");

    // An output pad's enable is a constant, set in the IOB's own config (its
    // O1/SR/CE selects, as the vendor does); it is not routed through the fabric.
}

void SLG47910Impl::postRoute()
{
    if (!sdc_out.empty())   emit_placement();
    if (!route_out.empty()) emit_routing();
}

// The EFLX routing fabric uses a 32x32 grid where routing nodes have
// coordinates that don't align with CLB grid positions. Site pins for a CLB
// at (gx,gy) may have coordinates offset by ±1 or more, so always return the
// full chip bounding box to avoid pruning valid routes.
BoundingBox SLG47910Impl::getRouteBoundingBox(WireId src, WireId dst) const
{
    (void)src; (void)dst;
    BoundingBox bb;
    bb.x0 = 0; bb.y0 = 0;
    bb.x1 = db.grid_w - 1; bb.y1 = db.grid_h - 1;
    return bb;
}

// Keep the delay estimate deliberately loose: site pins are offset from CLB
// coordinates and legal routes often detour through neighbouring switchboxes,
// so plain Manhattan distance is inadmissibly optimistic.
delay_t SLG47910Impl::estimateDelay(WireId src, WireId dst) const
{
    const WireInfo &s = ctx->wire_info(src);
    const WireInfo &d = ctx->wire_info(dst);
    return abs(s.x - d.x) + abs(s.y - d.y) + 1;
}

// An IOB's PAD wire joins the site's input and output pins (init_iob_bels), so
// without this the router could pass any net through an unconfigured IOB
// (fabric -> IOB in-pin -> PAD -> IOB out-pin -> fabric), which is no path on
// silicon. A PAD is only ever the source of the net its input cell drives, or
// a sink of the net its output cell takes.
bool SLG47910Impl::checkPipAvailForNet(PipId pip, const NetInfo *net) const
{
    WireId src = ctx->getPipSrcWire(pip), dst = ctx->getPipDstWire(pip);
    if (ctx->getWireType(src) == id_PAD)
        return net != nullptr && ctx->getNetinfoSourceWire(net) == src;
    if (ctx->getWireType(dst) == id_PAD) {
        if (net == nullptr)
            return false;
        for (auto &usr : net->users)
            for (WireId w : ctx->getNetinfoSinkWires(net, usr))
                if (w == dst)
                    return true;
        return false;
    }
    return true;
}

bool SLG47910Impl::checkBelAvail(BelId bel) const
{
    auto it = bel_conflicts.find(bel);
    if (it == bel_conflicts.end()) return true;
    for (BelId other : it->second)
        if (other != BelId() && ctx->getBoundBelCell(other) != nullptr)
            return false;
    return true;
}

bool SLG47910Impl::isBelLocationValid(BelId bel, bool explain_invalid) const
{
    CellInfo *bound = ctx->getBoundBelCell(bel);
    if (bound == nullptr) return true;

    // BEL-exclusivity conflicts (L0/L1 overlap, RAM/SRL/FF exclusion).
    auto it = bel_conflicts.find(bel);
    if (it != bel_conflicts.end()) {
        for (BelId other : it->second) {
            CellInfo *other_cell = (other != BelId()) ? ctx->getBoundBelCell(other) : nullptr;
            if (other_cell == nullptr) continue;
            if (explain_invalid)
                log_warning("slg47910: BEL %s (%s) conflicts with %s (%s)\n",
                            ctx->nameOfBel(bel), ctx->nameOf(bound),
                            ctx->nameOfBel(other), ctx->nameOf(other_cell));
            return false;
        }
    }

    // Helper: classify an FF bel's Z into (slot, slice) — slot is "XQ" or
    // "OXQ"; returns slice -1 if the Z is outside both FF bands.
    auto classify_ff_z = [](int z, int z_off) -> std::pair<std::string, int> {
        int dz = z - z_off;
        if (dz >= FF_XQ_Z_BASE && dz < FF_XQ_Z_BASE + SLICES_PER_CLB)
            return {"XQ", dz - FF_XQ_Z_BASE};
        if (dz >= FF_OXQ_Z_BASE && dz < FF_OXQ_Z_BASE + SLICES_PER_CLB)
            return {"OXQ", dz - FF_OXQ_Z_BASE};
        return {"", -1};
    };

    // FF_SLOT enforcement: a cell whose params declare a specific slot
    // must land on the matching bel band. ANY accepts both.
    if (bound->type == id_SLG_FF) {
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (loc.z >= 8) ? 8 : 0;
        auto [bel_slot, bel_slice] = classify_ff_z(loc.z, z_off);
        if (bel_slice >= 0) {
            std::string want = cell_param_string(bound, id_FF_SLOT);
            if (want.empty()) want = "ANY";
            if (want != "ANY" && want != bel_slot) {
                if (explain_invalid)
                    log_warning("slg47910: FF %s requests slot %s but is bound to %s bel %s\n",
                                ctx->nameOf(bound), want.c_str(), bel_slot.c_str(),
                                ctx->nameOfBel(bel));
                return false;
            }
        }
    }

    // SR type compatibility: all FFs in a CLB must agree on SET-type (FDSE/FDPE)
    // vs RESET-type (FDRE/FDCE) because MUX_SR polarity (XOR bit) is shared per CLB.
    // Iterates over both XQ and OXQ FF bels (8 per CLB total).
    if (bound->type == id_SLG_FF && ff_has_sr(bound)) {
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (loc.z >= 8) ? 8 : 0;
        auto [my_slot, my_slice] = classify_ff_z(loc.z, z_off);
        if (my_slice >= 0) {
            bool my_set = ff_is_set_type(bound);
            for (int band_base : {FF_XQ_Z_BASE, FF_OXQ_Z_BASE}) {
                for (int sl = 0; sl < SLICES_PER_CLB; sl++) {
                    int other_z = z_off + band_base + sl;
                    if (other_z == loc.z) continue;
                    BelId other_ff = ctx->getBelByLocation(Loc(loc.x, loc.y, other_z));
                    if (other_ff == BelId()) continue;
                    CellInfo *other_cell = ctx->getBoundBelCell(other_ff);
                    if (other_cell == nullptr || other_cell->type != id_SLG_FF) continue;
                    if (!ff_has_sr(other_cell)) continue;
                    if (ff_is_set_type(other_cell) != my_set) {
                        if (explain_invalid)
                            log_warning("slg47910: FF %s (%s) SR type conflicts with %s (%s) in same CLB\n",
                                        ctx->nameOf(bound), my_set ? "SET" : "RESET",
                                        ctx->nameOf(other_cell),
                                        ff_is_set_type(other_cell) ? "SET" : "RESET");
                        return false;
                    }
                }
            }
        }
    }

    // CE compatibility: all FFs in a CLB share one CE wire, so they must all
    // be driven by the same net. Iterates over both XQ and OXQ FF bels.
    if (bound->type == id_SLG_FF) {
        auto my_ce_it = bound->ports.find(id_CE);
        NetInfo *my_ce = (my_ce_it != bound->ports.end()) ? my_ce_it->second.net : nullptr;
        Loc loc = ctx->getBelLocation(bel);
        int z_off = (loc.z >= 8) ? 8 : 0;
        auto [my_slot, my_slice] = classify_ff_z(loc.z, z_off);
        if (my_slice >= 0) {
            for (int band_base : {FF_XQ_Z_BASE, FF_OXQ_Z_BASE}) {
                for (int sl = 0; sl < SLICES_PER_CLB; sl++) {
                    int other_z = z_off + band_base + sl;
                    if (other_z == loc.z) continue;
                    BelId other_ff = ctx->getBelByLocation(Loc(loc.x, loc.y, other_z));
                    if (other_ff == BelId()) continue;
                    CellInfo *other_cell = ctx->getBoundBelCell(other_ff);
                    if (other_cell == nullptr || other_cell->type != id_SLG_FF) continue;
                    auto other_ce_it = other_cell->ports.find(id_CE);
                    NetInfo *other_ce = (other_ce_it != other_cell->ports.end()) ? other_ce_it->second.net : nullptr;
                    if (my_ce != other_ce) {
                        if (explain_invalid)
                            log_warning("slg47910: FF %s CE (%s) conflicts with FF %s CE (%s) in same CLB\n",
                                        ctx->nameOf(bound),
                                        my_ce ? ctx->nameOf(my_ce) : "(none)",
                                        ctx->nameOf(other_cell),
                                        other_ce ? ctx->nameOf(other_ce) : "(none)");
                        return false;
                    }
                }
            }
        }
    }

    // xMUX resource conflict (OXQ vs combinational MUX_X output).
    // The OXQ FF Q exits the slice via the xMUX site-output pin, which is
    // shared with the slice's combinational MUX_X output. The routing graph
    // expresses this as a wire-level conflict: the LUT→xMUX and OXQ_Q→xMUX
    // pips both terminate at the same wire, so the router enforces "only one
    // driver at a time" naturally. We do not duplicate that here — placement
    // legality leaves it to the router (Phase 1 design choice). A coarser
    // bel-level interlock would be conservative; the routing-level check is
    // exact and avoids false rejections during the placer's swap moves.

    Loc loc = ctx->getBelLocation(bel);
    int z_off = (loc.z >= 8) ? 8 : 0;

    auto site_native_ram32m_group = [&](int x, int y) -> std::string {
        for (int sl = 0; sl < 4; sl++) {
            BelId ram_bel = ctx->getBelByLocation(Loc(x, y, z_off + RAM_Z_BASE + sl));
            if (ram_bel == BelId()) continue;
            CellInfo *ram = ctx->getBoundBelCell(ram_bel);
            if (ram == nullptr || !is_native_ram32m_cell(ram)) continue;
            return cell_param_string(ram, ctx->id("RAM32M_GROUP"));
        }
        return std::string();
    };

    auto site_has_native_ram32m = [&](int x, int y) -> bool {
        return !site_native_ram32m_group(x, y).empty();
    };

    if (loc.z >= 8) {
        if (bound->type.in(id_SLG_LUT, id_SLG_FF) && site_has_native_ram32m(loc.x, loc.y)) {
            if (explain_invalid)
                log_warning("slg47910: native RAM32M occupies L1 site (%d,%d), so %s cannot share it\n",
                            loc.x, loc.y, ctx->nameOf(bound));
            return false;
        }
        if (bound->type == id_SLG_RAM && !is_native_ram32m_cell(bound) && site_has_native_ram32m(loc.x, loc.y)) {
            if (explain_invalid)
                log_warning("slg47910: native RAM32M occupies L1 site (%d,%d), so %s cannot share it\n",
                            loc.x, loc.y, ctx->nameOf(bound));
            return false;
        }
    }

    if (is_native_ram32m_cell(bound)) {
        std::string my_group = cell_param_string(bound, ctx->id("RAM32M_GROUP"));
        int64_t my_bitplane  = cell_param_int(bound, ctx->id("RAM32M_BITPLANE"));
        int partner_y = loc.y + ((my_bitplane == 0) ? 2 : -2);

        if (ctx->getBelByLocation(Loc(loc.x, partner_y, z_off + RAM_Z_BASE)) == BelId()) {
            if (explain_invalid)
                log_warning("slg47910: native RAM32M half %s at (%d,%d) has no paired L1 neighbour at (%d,%d)\n",
                            ctx->nameOf(bound), loc.x, loc.y, loc.x, partner_y);
            return false;
        }

        for (int sl = 0; sl < 4; sl++) {
            BelId ram_bel = ctx->getBelByLocation(Loc(loc.x, loc.y, z_off + RAM_Z_BASE + sl));
            if (ram_bel == BelId()) continue;
            CellInfo *other = ctx->getBoundBelCell(ram_bel);
            if (other == nullptr || other == bound) continue;
            if (!is_native_ram32m_cell(other) ||
                cell_param_string(other, ctx->id("RAM32M_GROUP")) != my_group ||
                cell_param_int(other, ctx->id("RAM32M_BITPLANE")) != my_bitplane) {
                if (explain_invalid)
                    log_warning("slg47910: native RAM32M site (%d,%d) cannot mix %s with %s\n",
                                loc.x, loc.y, ctx->nameOf(bound), ctx->nameOf(other));
                return false;
            }
        }
    }

    return true;
}

IdString SLG47910Impl::getBelBucketForCellType(IdString cell_type) const
{
    if (cell_type == id_SLG_IOB)      return id_SLG_IOB;
    if (cell_type == id_SLG_CLKIN)    return id_SLG_CLKIN;
    if (cell_type == id_SLG_FF)       return id_SLG_FF;
    if (cell_type == id_SLG_RAM32X1D) return id_SLG_RAM32X1D;
    if (cell_type == id_SLG_RAM)      return id_SLG_RAM;
    if (cell_type == id_SLG_MUX_F7)   return id_SLG_MUX_F7;
    if (cell_type == id_SLG_MUX_F8)   return id_SLG_MUX_F8;
    return id_SLG_LUT;
}

bool SLG47910Impl::isValidBelForCellType(IdString cell_type, BelId bel) const
{
    return (getBelBucketForCellType(cell_type) == ctx->getBelType(bel));
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

bool SLG47910Impl::ff_is_set_type(const CellInfo *ci) const
{
    auto it = ci->params.find(ctx->id("FF_TYPE"));
    if (it == ci->params.end()) return false;
    return (it->second.as_string() == "FDSE" || it->second.as_string() == "FDPE");
}

bool SLG47910Impl::ff_has_sr(const CellInfo *ci) const
{
    auto it = ci->ports.find(id_R);
    return (it != ci->ports.end() && it->second.net != nullptr);
}

bool SLG47910Impl::cell_param_true(const CellInfo *ci, IdString key) const
{
    auto it = ci->params.find(key);
    return (it != ci->params.end() && it->second.as_int64() != 0);
}

int64_t SLG47910Impl::cell_param_int(const CellInfo *ci, IdString key, int64_t default_value) const
{
    auto it = ci->params.find(key);
    return (it != ci->params.end()) ? it->second.as_int64() : default_value;
}

std::string SLG47910Impl::cell_param_string(const CellInfo *ci, IdString key) const
{
    auto it = ci->params.find(key);
    return (it != ci->params.end()) ? it->second.as_string() : std::string();
}

bool SLG47910Impl::is_native_ram32m_cell(const CellInfo *ci) const
{
    return ci->type == id_SLG_RAM && cell_param_true(ci, ctx->id("RAM32M_NATIVE"));
}

int SLG47910Impl::parse_site_pin_index(const std::string &name)
{
    if (name.size() < 2) return -1;
    char *end = nullptr;
    long value = std::strtol(name.c_str() + 1, &end, 10);
    if (end == name.c_str() + 1) return -1;
    return int(value);
}

int SLG47910Impl::parse_edge_d0_branch(const std::string &wire_name)
{
    size_t pos = wire_name.rfind("D0B");
    if (pos == std::string::npos) return -1;
    const char *start = wire_name.c_str() + pos + 3;
    char *end = nullptr;
    long value = std::strtol(start, &end, 10);
    if (end == start || end == nullptr || *end != 'N') return -1;
    return int(value);
}

std::string SLG47910Impl::json_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"':  out += "\\\""; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:   out += c;      break;
        }
    }
    return out;
}

// SLG47910 STQFN-24 package: GPIO name → fabric IOB (gx, gy).
// Derived from the vendor io_spec (SoC-level IOB bonding table).
// GPIO[0:7] are on the west edge (gx=0), GPIO[8:18] on the east (gx=31).
// GPIO[3:6] double as SPI pins during configuration.
const std::map<std::string, std::pair<int,int>> &SLG47910Impl::gpio_name_to_coord()
{
    static const std::map<std::string, std::pair<int,int>> tbl = {
        {"GPIO0",  { 0,  6}}, {"GPIO1",  { 0,  7}}, {"GPIO2",  { 0,  8}},
        {"GPIO3",  { 0,  9}}, {"GPIO4",  { 0, 10}}, {"GPIO5",  { 0, 22}},
        {"GPIO6",  { 0, 23}}, {"GPIO7",  { 0, 24}}, {"GPIO8",  {31, 27}},
        {"GPIO9",  {31, 26}}, {"GPIO10", {31, 25}}, {"GPIO11", {31, 24}},
        {"GPIO12", {31, 23}}, {"GPIO13", {31, 22}}, {"GPIO14", {31,  9}},
        {"GPIO15", {31,  8}}, {"GPIO16", {31,  6}}, {"GPIO17", {31,  5}},
        {"GPIO18", {31,  4}},
    };
    return tbl;
}

WireId SLG47910Impl::trace_bound_uphill_wire(WireId sink_wire) const
{
    if (sink_wire == WireId()) return WireId();
    for (auto pip : ctx->getPipsUphill(sink_wire)) {
        if (ctx->getBoundPipNet(pip) == nullptr) continue;
        return ctx->getPipSrcWire(pip);
    }
    return WireId();
}

WireId SLG47910Impl::trace_bound_downhill_wire(WireId source_wire) const
{
    if (source_wire == WireId()) return WireId();
    for (auto pip : ctx->getPipsDownhill(source_wire)) {
        if (ctx->getBoundPipNet(pip) == nullptr) continue;
        return ctx->getPipDstWire(pip);
    }
    return WireId();
}

int SLG47910Impl::trace_site_input_pin(WireId sink_wire) const
{
    WireId src = trace_bound_uphill_wire(sink_wire);
    auto it = wire_to_site_pin.find(src);
    return (it != wire_to_site_pin.end()) ? it->second : -1;
}

int SLG47910Impl::trace_site_input_pin_from_bel(BelId bel, IdString port) const
{
    return trace_site_input_pin(ctx->getBelPinWire(bel, port));
}

int SLG47910Impl::trace_site_output_pin(WireId source_wire) const
{
    WireId dst = trace_bound_downhill_wire(source_wire);
    auto it = wire_to_site_output_pin.find(dst);
    return (it != wire_to_site_output_pin.end()) ? it->second : -1;
}

int SLG47910Impl::trace_site_output_pin_from_bel(BelId bel, IdString port) const
{
    return trace_site_output_pin(ctx->getBelPinWire(bel, port));
}

std::string SLG47910Impl::trace_iob_pad_in_sm(WireId sink_wire) const
{
    WireId site_input = trace_bound_uphill_wire(sink_wire);
    WireId route_wire = trace_bound_uphill_wire(site_input);
    if (route_wire == WireId()) return std::string();
    return ctx->getWireName(route_wire).str(ctx);
}

std::string SLG47910Impl::trace_iob_pad_in_sm_from_bel(BelId bel, IdString port) const
{
    return trace_iob_pad_in_sm(ctx->getBelPinWire(bel, port));
}

std::string SLG47910Impl::connected_net_name(const CellInfo *ci, IdString port) const
{
    auto it = ci->ports.find(port);
    if (it == ci->ports.end() || it->second.net == nullptr) return "";
    return it->second.net->name.str(ctx);
}

void SLG47910Impl::add_timing_models()
{
    // Coarse vendor-grounded timings from the RE artifacts:
    // - FF clk->Q is consistently reported around 0.80 ns.
    // - LUT data paths land roughly in the 0.49-1.03 ns range.
    // - Carry paths show about 1.27 ns for CIN->COUT and about 0.75-0.79 ns
    //   for carry-visible output taps.
    //
    // This is intentionally a compact timing model rather than a full vendor
    // database, but it gives nextpnr real interior cell arcs for timing-driven
    // placement/routing and Fmax reporting.
    static constexpr delay_t kFfClkToQ        = 0.80f;
    static constexpr delay_t kFfSetup         = 0.35f;
    static constexpr delay_t kFfHold          = 0.15f;
    static constexpr delay_t kLutFastDelay    = 0.49f;
    static constexpr delay_t kLutMuxedDelay   = 1.03f;
    static constexpr delay_t kCarryToOut      = 0.79f;
    static constexpr delay_t kCarryToCout     = 1.27f;
    static constexpr delay_t kCarryLogicToOut = 1.29f;
    static constexpr delay_t kCarryLogicToCout = 1.30f;
    static constexpr delay_t kRamReadDelay    = 1.10f;
    static constexpr delay_t kRamSetup        = 0.35f;
    static constexpr delay_t kRamHold         = 0.15f;

    auto has_port = [&](const CellInfo *ci, IdString port) {
        return ci->ports.count(port) != 0;
    };
    auto add_comb_if_present = [&](const CellInfo *ci, IdString from, IdString to, delay_t delay) {
        if (has_port(ci, from) && has_port(ci, to))
            ctx->addCellTimingDelay(ci->name, from, to, delay);
    };
    auto add_setup_hold_if_present = [&](const CellInfo *ci, IdString port, IdString clock,
                                         delay_t setup, delay_t hold) {
        if (has_port(ci, port) && has_port(ci, clock))
            ctx->addCellTimingSetupHold(ci->name, port, clock, setup, hold);
    };

    int lut_count = 0, ff_count = 0, ram_count = 0;
    const IdString lut_inputs[] = {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
    const IdString dpra_inputs[] = {id_DPRA0, id_DPRA1, id_DPRA2, id_DPRA3, id_DPRA4};
    const IdString wa_inputs[] = {id_A0, id_A1, id_A2, id_A3, id_A4};

    for (auto &[name, cell] : ctx->cells) {
        CellInfo *ci = cell.get();

        if (ci->type == id_SLG_LUT) {
            bool carry_mode = cell_param_true(ci, ctx->id("CARRY_MODE"));
            if (carry_mode) {
                for (int k = 0; k < 5; ++k) {
                    add_comb_if_present(ci, lut_inputs[k], id_O, kCarryLogicToOut);
                    add_comb_if_present(ci, lut_inputs[k], id_COUT, kCarryLogicToCout);
                }
                add_comb_if_present(ci, id_I5, id_O, kCarryToOut);
                add_comb_if_present(ci, id_I5, id_COUT, kCarryToCout);
                add_comb_if_present(ci, id_CIN, id_O, kCarryToOut);
                add_comb_if_present(ci, id_CIN, id_COUT, kCarryToCout);
            } else {
                for (int k = 0; k < 4; ++k)
                    add_comb_if_present(ci, lut_inputs[k], id_O, kLutFastDelay);
                add_comb_if_present(ci, id_I4, id_O, kLutMuxedDelay);
                add_comb_if_present(ci, id_I5, id_O, kLutMuxedDelay);
            }
            lut_count++;
            continue;
        }

        if (ci->type == id_SLG_FF) {
            if (has_port(ci, id_CLK))
                ctx->addCellTimingClock(ci->name, id_CLK);
            add_setup_hold_if_present(ci, id_D, id_CLK, kFfSetup, kFfHold);
            add_setup_hold_if_present(ci, id_CE, id_CLK, kFfSetup, kFfHold);

            std::string ff_type = cell_param_string(ci, ctx->id("FF_TYPE"));
            if (ff_type == "FDRE" || ff_type == "FDSE")
                add_setup_hold_if_present(ci, id_R, id_CLK, kFfSetup, kFfHold);

            if (has_port(ci, id_Q) && has_port(ci, id_CLK))
                ctx->addCellTimingClockToOut(ci->name, id_Q, id_CLK, kFfClkToQ);
            ff_count++;
            continue;
        }

        if (ci->type == id_SLG_RAM32X1D) {
            if (has_port(ci, id_WCLK))
                ctx->addCellTimingClock(ci->name, id_WCLK);
            for (IdString port : wa_inputs) {
                add_setup_hold_if_present(ci, port, id_WCLK, kRamSetup, kRamHold);
                add_comb_if_present(ci, port, id_SPO, kRamReadDelay);
            }
            for (IdString port : dpra_inputs)
                add_comb_if_present(ci, port, id_DPO, kRamReadDelay);
            add_setup_hold_if_present(ci, id_D, id_WCLK, kRamSetup, kRamHold);
            add_setup_hold_if_present(ci, id_WE, id_WCLK, kRamSetup, kRamHold);
            ram_count++;
            continue;
        }

        if (ci->type == id_SLG_RAM) {
            if (has_port(ci, id_WCLK))
                ctx->addCellTimingClock(ci->name, id_WCLK);

            std::string ram_type = cell_param_string(ci, ctx->id("RAM_TYPE"));
            if (ram_type == "SRL16E") {
                for (int k = 0; k < 4; ++k)
                    add_comb_if_present(ci, lut_inputs[k], id_SPO, kRamReadDelay);
                add_setup_hold_if_present(ci, id_I5, id_WCLK, kRamSetup, kRamHold);
                add_setup_hold_if_present(ci, id_WE, id_WCLK, kRamSetup, kRamHold);
            } else {
                for (IdString port : dpra_inputs)
                    add_comb_if_present(ci, port, id_SPO, kRamReadDelay);
                for (IdString port : lut_inputs)
                    add_setup_hold_if_present(ci, port, id_WCLK, kRamSetup, kRamHold);
                add_setup_hold_if_present(ci, id_WE, id_WCLK, kRamSetup, kRamHold);
            }

            ram_count++;
            continue;
        }
    }

    log_info("slg47910: annotated timing for %d LUT/carry, %d FF, %d RAM/SRL cells\n",
             lut_count, ff_count, ram_count);
}

// ---------------------------------------------------------------------------
// ViaductArch registration
// ---------------------------------------------------------------------------

namespace {
struct SLG47910Arch : ViaductArch
{
    SLG47910Arch() : ViaductArch("slg47910") {}
    std::unique_ptr<ViaductAPI> create(const dict<std::string, std::string> &args) override
    {
        return std::make_unique<SLG47910Impl>(args);
    }
} slg47910Arch;
} // namespace

NEXTPNR_NAMESPACE_END
