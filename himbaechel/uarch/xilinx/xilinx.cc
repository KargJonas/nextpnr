/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2023  gatecat <gatecat@ds0.me>
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

#include <boost/algorithm/string.hpp>
#include <queue>
#include <regex>

#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "util.h"

#include "placer_heap.h"
#include "timing_opt.h"
#include "placer_static.h"
#include "router2.h"

#include "xilinx.h"
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "himbaechel_helpers.h"

#define GEN_INIT_CONSTIDS
#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

struct FFControlSet
{
    unsigned flags = 0;
    enum
    {
        IS_LATCH = 1,
        IS_CLKINV = 2,
        IS_SRINV = 4,
        FFSYNC = 8,
    };
    IdString clk, sr, ce;
    bool operator==(const FFControlSet &other) const
    {
        return flags == other.flags && clk == other.clk && ce == other.ce && sr == other.sr;
    };
    unsigned hash() const
    {
        unsigned hash = mkhash(clk.hash(), sr.hash());
        hash = mkhash(hash, ce.hash());
        hash = mkhash(hash, flags);
        return hash;
    }
};

XilinxImpl::~XilinxImpl() {};

po::options_description XilinxImpl::getUArchOptions()
{
    po::options_description specific("Xilinx specific options");
    specific.add_options()("fasm", po::value<std::string>(), "fasm bitstream output file");
    specific.add_options()("xdc", po::value<std::string>(), "name of constraints file");
    specific.add_options()("placement", po::value<std::string>(),
                           "placement dump (JSON: cell -> tile/site/bel/type) for external LVS");
    specific.add_options()("delay-matrix", po::value<std::string>(),
                           "interconnect delay model: on by default (measured per tile offset); 'off' uses the "
                           "tuned formula; a path caches the table there; 'build' rebuilds it fresh (the default)");
    specific.add_options()("hold-fix", po::value<std::string>()->implicit_value(""),
                           "after routing, fix hold-time (min-delay) violations: small deficits by a routing "
                           "detour, larger ones by a feedthrough LUT; optional value sets the max passes (default 8)");
    specific.add_options()("preplaced", po::value<std::string>(),
                           "before placing, pin the cells named in this file to the bels they had in a reference "
                           "build (scripts/routing_dump.py from that build's --write JSON), packer-made cells included");
    specific.add_options()("prerouted", po::value<std::string>(),
                           "before routing, give the nets named in this file the routes they had in a reference "
                           "build (scripts/routing_dump.py from that build's --write JSON), locked; the router "
                           "then routes only what is new -- a frozen routing, for adding to a routed design");
    specific.add_options()("holdbufs", po::value<std::string>(),
                           "before placing, re-create the reference build's hold-fix feedthrough buffers listed in "
                           "this file (scripts/routing_dump.py), so -o preplaced / -o prerouted replay them too");
    specific.add_options()("clock-tree", po::value<std::string>(),
                           "global clock insertion delay model: on (default; dies with measured clock delays only), "
                           "off, or force (an uncalibrated die takes the xc7vx485t's); see clock_tree.cc");
    specific.add_options()("bram-place", po::value<std::string>(),
                           "place block RAMs by read-mux group and the rest again around them: on (default; dies with "
                           "measured clock delays only), off, or force (any die); see bram_place.cc");
    specific.add_options()("timing-opt", po::value<std::string>(),
                           "after placing, the timing-driven placement optimisation on the critical paths: R[,N] moves a "
                           "cell up to R tiles, N rounds (default 30); off unless given");
    specific.add_options()("hold-detour-max", po::value<double>(),
                           "hold deficit (ns) up to which a routing detour is used instead of a feedthrough LUT "
                           "(default 0.5)");
    return specific;
}

void XilinxImpl::init_database(Arch *arch)
{
    const ArchArgs &args = arch->args;
    init_uarch_constids(arch);
    std::smatch match;
    // Accept either a full part name (xc7s50csga324-1) or a bare die name
    // (xc7s50), the latter selecting no package.
    //
    // The class gains s, for spartan7.  xc7vx is spelled out as its own
    // alternative because a Virtex-7 XT part name puts a letter where
    // \d+ expects a digit: xc7vx485t never matched xc7[azkv]\d+t? at all,
    // so no Virtex-7 part was reachable through this flow, whatever the
    // chipdb held.  The class loses v, which only ever matched plain
    // (non-XT) Virtex-7 names such as xc7v585t; openXC7/prjxray-db carries
    // no plain xc7v part -- virtex7/ holds xc7vx485t alone -- so nothing
    // that could previously be built stops building.
    //
    // The class also gains the Zynq-7000S dies, spelled out ahead of the
    // generic class because there the trailing s is part of the die name:
    // xc7z007s is a die, while xc7z030sbg485-1's s belongs to the package
    // (sbg485).  Without them xc7z007sclg400-1 parses as die "xc7z007" with
    // package "sclg400" -- neither the die's chipdb nor a real package.
    //
    // The speed grade is the set that prjxray-db part directories actually
    // use (1, 2, 3, 2L, 1IL, 1Q) and nothing else.  1IL and 1Q are
    // Spartan-7 grades; the previous pattern ([0-9]L?) rejected them.
    // The grade is not a timing corner: set_speed_grade below is DEFAULT.
    std::regex devicere = std::regex(
            "(xc7z007s|xc7z012s|xc7z014s|xc7[azks]\\d+t?|xc7vx\\d+t?)"
            "([a-z0-9]*)(?:-(1IL|1Q|2L|[123]))?");
    if (!std::regex_match(args.device, match, devicere)) {
        log_error("Invalid device %s\n", args.device.c_str());
    }
    std::string die = match[1].str();
    if (die == "xc7a35t")
        die = "xc7a50t";
    // xc7z007s is the xc7z010 die with fewer resources enabled, and has no
    // chipdb of its own (see prjxray-db/zynq7/mapping/devices.yaml).
    if (die == "xc7z007s")
        die = "xc7z010";
    // prjxray-db/zynq7/mapping/devices.yaml: xc7z035's fabric is xc7z045.
    const bool xc7z035_uses_xc7z045 = die == "xc7z035";
    if (xc7z035_uses_xc7z045)
        die = "xc7z045";
    // prjxray-db/spartan7/mapping/devices.yaml: xc7s75's fabric is xc7s100.
    const bool xc7s75_uses_xc7s100 = die == "xc7s75";
    if (xc7s75_uses_xc7s100)
        die = "xc7s100";
    chipdb_die = die;
    arch->load_chipdb(stringf("xilinx/chipdb-%s.bin", die.c_str()));
    std::string package = match[2].str();
    // A bare die name carries no package, and set_package("") is not the
    // same as not setting one; a design that needs PACKAGE_PIN constraints
    // has to pass a part-form name.
    if (!package.empty())
        arch->set_package(package);
    arch->set_speed_grade("DEFAULT");
}

// prjxray names a bel "<site type>_<bel name>"; where the primitive is named
// after its site type the chipdb bel type is the primitive name twice, e.g.
// RAMB18E1_RAMB18E1 (or, as in IDELAYE2_FINEDELAY_IDELAYE2_FINEDELAY, an
// already compound name twice).  Fold such a bel type onto the primitive name
// it repeats.
static IdString primitive_name_of_bel_type(Context *ctx, IdString bel_type)
{
    const std::string &name = bel_type.str(ctx);
    const size_t half = name.size() / 2;
    const bool has_a_middle_separator = half != 0 && (name.size() % 2) == 1 && name[half] == '_';
    const bool second_half_repeats_the_first =
            has_a_middle_separator && name.compare(0, half, name, half + 1, half) == 0;
    if (!second_half_repeats_the_first)
        return bel_type;
    return ctx->id(name.substr(0, half));
}

void XilinxImpl::init(Context *ctx)
{
    h.init(ctx);
    HimbaechelAPI::init(ctx);

    // Resolve the repeated-name bel types once (the placer looks buckets up per
    // cell); everything else buckets as its own type.
    for (auto bel : ctx->getBels()) {
        IdString bel_type = ctx->getBelType(bel);
        IdString primitive_name = primitive_name_of_bel_type(ctx, bel_type);
        const bool bel_type_repeats_its_primitive = primitive_name != bel_type;
        if (bel_type_repeats_its_primitive)
            primitive_bucket_for_bel_type.emplace(bel_type, primitive_name);
    }

    tile_status.resize(ctx->chip_info->tile_insts.size());
    for (int i = 0; i < ctx->chip_info->tile_insts.ssize(); i++) {
        auto extra_data = tile_extra_data(i);
        tile_status.at(i).site_variant.resize(extra_data->sites.ssize());
    }
}

IdString XilinxImpl::getBelBucketForCellType(IdString cell_type) const
{
    auto bucket = primitive_bucket_for_bel_type.find(cell_type);
    const bool type_is_a_repeated_primitive = bucket != primitive_bucket_for_bel_type.end();
    return type_is_a_repeated_primitive ? bucket->second : cell_type;
}

IdString XilinxImpl::getBelBucketForBel(BelId bel) const
{
    return getBelBucketForCellType(ctx->getBelType(bel));
}

SiteIndex XilinxImpl::get_bel_site(BelId bel) const
{
    auto &bel_data = chip_bel_info(ctx->chip_info, bel);
    auto site_key = BelSiteKey::unpack(bel_data.site);
    return SiteIndex(bel.tile, site_key.site);
}

IdString XilinxImpl::get_site_name(SiteIndex site) const
{
    const auto &site_data = tile_extra_data(site.tile)->sites[site.site];
    return ctx->idf("%s_X%dY%d", IdString(site_data.name_prefix).c_str(ctx), site_data.site_x, site_data.site_y);
}

BelId XilinxImpl::get_site_bel(SiteIndex site, IdString bel_name) const
{
    const auto &tile_data = chip_tile_info(ctx->chip_info, site.tile);
    for (int32_t i = 0; i < tile_data.bels.ssize(); i++) {
        const auto &bel_data = tile_data.bels[i];
        if (BelSiteKey::unpack(bel_data.site).site != site.site)
            continue;
        if (reinterpret_cast<const XlnxBelExtraDataPOD *>(bel_data.extra_data.get())->name_in_site != bel_name.index)
            continue;
        return BelId(site.tile, i);
    }
    return BelId();
}

IdString XilinxImpl::bel_name_in_site(BelId bel) const
{
    const auto &bel_data = chip_bel_info(ctx->chip_info, bel);
    return IdString(reinterpret_cast<const XlnxBelExtraDataPOD *>(bel_data.extra_data.get())->name_in_site);
}

IdStringList XilinxImpl::get_site_bel_name(BelId bel) const
{
    return IdStringList::concat(get_site_name(get_bel_site(bel)), bel_name_in_site(bel));
}

WireId XilinxImpl::lookup_wire(int tile, IdString wire_name) const
{
    const auto &tdata = chip_tile_info(ctx->chip_info, tile);
    for (int wire = 0; wire < tdata.wires.ssize(); wire++) {
        if (IdString(tdata.wires[wire].name) == wire_name)
            return ctx->normalise_wire(tile, wire);
    }
    return WireId();
}

void XilinxImpl::notifyBelChange(BelId bel, CellInfo *cell)
{
    auto &ts = tile_status.at(bel.tile);
    auto &bel_data = chip_bel_info(ctx->chip_info, bel);
    auto site_key = BelSiteKey::unpack(bel_data.site);
    // Update bound site variant for pip validity use later on
    if (cell && cell->type != id_PAD && site_key.site >= 0 && site_key.site < int(ts.site_variant.size())) {
        ts.site_variant.at(site_key.site) = site_key.site_variant;
    }

    if (!cell_tags_set) {
        // This will happen when loading a pre-placed design, at the time the frontend calls attributesToArchInfo cell
        // tags aren't set and this will fail. We resolve it in preRoute
        return;
    }

    if (is_logic_tile(bel))
        update_logic_bel(bel, cell);
    if (is_bram_tile(bel))
        update_bram_bel(bel, cell);
}

void XilinxImpl::update_logic_bel(BelId bel, CellInfo *cell)
{
    int z = ctx->getBelLocation(bel).z;
    NPNR_ASSERT(z < 128);
    auto &tts = tile_status.at(bel.tile);
    if (!tts.lts)
        tts.lts = std::make_unique<LogicTileStatus>();
    auto &ts = *(tts.lts);
    auto tags = get_tags(cell), last_tags = get_tags(ts.cells[z]);
    if ((z == ((3 << 4) | BEL_6LUT)) || (z == ((3 << 4) | BEL_5LUT))) {
        if ((tags && tags->lut.is_memory) || (last_tags && last_tags->lut.is_memory)) {
            // Special case - memory write port invalidates everything
            for (int i = 0; i < 8; i++)
                ts.eights[i].dirty = true;
            // if (xc7)
            ts.halfs[0].dirty = true; // WCLK and CLK0 shared
        }
    }
    if ((((z & 0xF) == BEL_6LUT) || ((z & 0xF) == BEL_5LUT)) &&
        ((tags && tags->lut.is_srl) || (last_tags && last_tags->lut.is_srl))) {
        // SRLs invalidate everything due to write clock
        for (int i = 0; i < 8; i++)
            ts.eights[i].dirty = true;
        // if (xc7)
        ts.halfs[0].dirty = true; // WCLK and CLK0 shared
    }

    ts.cells[z] = cell;

    // determine which sections to mark as dirty
    switch (z & 0xF) {
    case BEL_FF:
    case BEL_FF2:
        ts.halfs[(z >> 4) / 4].dirty = true;
        if ((((z >> 4) / 4) == 0) /*&& xc7*/)
            ts.eights[3].dirty = true;
    /* fall-through */
    case BEL_6LUT:
    case BEL_5LUT:
        ts.eights[z >> 4].dirty = true;
        break;
    case BEL_F7MUX:
        ts.eights[z >> 4].dirty = true;
        ts.eights[(z >> 4) + 1].dirty = true;
        break;
    case BEL_F8MUX:
        ts.eights[(z >> 4) + 1].dirty = true;
        ts.eights[(z >> 4) + 2].dirty = true;
        break;
    case BEL_CARRY4:
        for (int i = ((z >> 4) / 4) * 4; i < (((z >> 4) / 4) + 1) * 4; i++)
            ts.eights[i].dirty = true;
        break;
    }
}

void XilinxImpl::update_bram_bel(BelId bel, CellInfo *cell)
{
    IdString type = ctx->getBelType(bel);
    if (!type.in(id_RAMBFIFO18E2_RAMBFIFO18E2, id_RAMBFIFO36E2_RAMBFIFO36E2, id_RAMB18E2_RAMB18E2, id_FIFO36E2_FIFO36E2,
                 id_RAMBFIFO36E1_RAMBFIFO36E1, id_RAMB36E1_RAMB36E1, id_RAMB18E1_RAMB18E1))
        return;
    auto &tts = tile_status.at(bel.tile);
    if (!tts.bts)
        tts.bts = std::make_unique<BRAMTileStatus>();
    Loc loc = ctx->getBelLocation(bel);
    int z = loc.z;
    bool is_site_variant_bel = z >= 12;
    if (is_site_variant_bel) {
        // A bel imported by a non-primary site variant (given a fresh unique z
        // by the chipdb generator) is the same physical hardware as its
        // primary twin: fold it into the primary slot so the BRAM tile status
        // keeps its compact z-indexed layout.
        bool found = false;
        for (auto other : ctx->getBelsByTile(loc.x, loc.y)) {
            bool is_other_bel_of_same_type = other != bel && ctx->getBelType(other) == type;
            if (!is_other_bel_of_same_type)
                continue;
            int oz = ctx->getBelLocation(other).z;
            bool other_is_also_variant = oz >= 12;
            if (other_is_also_variant)
                continue;
            if (bel_name_in_site(other) != bel_name_in_site(bel))
                continue;
            z = oz;
            found = true;
            break;
        }
        if (!found)
            NPNR_ASSERT(found);
    }
    NPNR_ASSERT(z >= 0 && z < 12);
    tts.bts->cells[z] = cell;
}

bool XilinxImpl::is_pip_unavail(PipId pip) const
{
    const auto &pip_data = chip_pip_info(ctx->chip_info, pip);
    const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pip_data.extra_data.get());
    unsigned pip_type = pip_data.flags;

    // The regional-clock (BUFR) datapath is a BUFFER, not routing.
    //
    // CK_BUFRCLK* is a BUFR's OUTPUT and RCLK_BEFORE_DIV -> RCLK_OUT ->
    // RCLK2RCLK is the path through its divider.  Connectivity-wise they look
    // like ordinary arcs, so the router will thread a global clock through them
    // -- and prefers to, because from a pin in the IO column the adjacent
    // HCLK_IOI3 tile is nearer than the CMT column carrying the dedicated
    // pad->BUFG route.
    //
    // Using that path obliges the design to place and enable a BUFR on the
    // regional clock.  With nothing enforcing it, a clock-to-BUFG net comes out
    // as
    //     I2IOCLK_BOT1 -> IO_PLL_CLK3_DMUX -> RCLK3 -> RCLK_BEFORE_DIV1
    //                  -> RCLK2RCLK1 -> CK_BUFRCLK1 -> CLK_HROW -> BUFGCTRL
    // while the utilisation report says BUFR_BUFR: 0/20 -- a route whose buffer
    // was never configured.  On hardware (Sonata, xc7a50tcsg324-1) that is a
    // valid config with no clock: the bitstream loads and nothing runs.
    //
    // So when the design instantiates no BUFR, refuse the BUFR datapath.  The
    // router then takes the direct pad->BUFG route (HCLK_CMT_CCIO* ->
    // CLK_HROW_CK_IN_L* -> CK_BUFG_CASCO* -> BUFGCTRL), which is what Vivado
    // does unprompted, and the same design then runs on the board.
    //
    // Conservative on purpose: when a BUFR IS present the path stays available,
    // since deciding which regional clock a given BUFR serves needs placement
    // context this predicate does not have.
    if (pip_type == PIP_TILE_ROUTING) {
        if (!design_has_bufr_valid) {
            design_has_bufr = false;
            for (auto &cell : ctx->cells)
if (cell.second->type.in(id_BUFR, id_BUFR_BUFR)) {
                    design_has_bufr = true;
                    break;
                }
            design_has_bufr_valid = true;
        }
        if (!design_has_bufr) {
            IdString dst = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pip_data.dst_wire].name);
            const std::string &d = dst.str(ctx);
            bool drives_bufr_clock_path =
                    d.find("CK_BUFRCLK") != std::string::npos || d.find("RCLK_BEFORE_DIV") != std::string::npos ||
                    d.find("RCLK_OUT") != std::string::npos || d.find("RCLK2RCLK") != std::string::npos;
            if (drives_bufr_clock_path)
                return true;
        }
        IdString tt = IdString(chip_tile_info(ctx->chip_info, pip.tile).type_name);
        std::string tts = tt.str(ctx);
        bool tile_is_clk_bufg_r =
                (boost::starts_with(tts, "CLK_BUFG_TOP_R") || boost::starts_with(tts, "CLK_BUFG_BOT_R"));
        if (tile_is_clk_bufg_r) {
            bool has_bound_bufgctrl = false;
            const auto &tile_data = chip_tile_info(ctx->chip_info, pip.tile);
            for (int32_t i = 0; i < tile_data.bels.ssize(); ++i) {
                CellInfo *bound = ctx->getBoundBelCell(BelId(pip.tile, i));
                bool is_bound_bufgctrl = bound != nullptr && bound->type == id_BUFGCTRL;
                if (is_bound_bufgctrl) {
                    has_bound_bufgctrl = true;
                    break;
                }
            }
            if (!has_bound_bufgctrl)
                return true;
            // Never route THROUGH a BUFGCTRL (its I0/I1 -> O pseudo pip). A bound BUFGCTRL's nets end at
            // I0/I1 and start at O, so they never need it; an unbound slot in a tile that has a bound
            // BUFGCTRL passes the tile check above, but the FASM writer's per-slot phantom-BUFGCTRL guard
            // drops that slot's IN_USE/ZINV_CE0/ZINV_S0, so the route-through comes out as a buffer that
            // passes nothing: the net is a constant on hardware (seen as stuck register bits behind a
            // data net that took the global clock spine).
            IdString dst = IdString(tile_data.wires[pip_data.dst_wire].name);
            IdString src = IdString(tile_data.wires[pip_data.src_wire].name);
            const std::string &ds = dst.str(ctx), &ss = src.str(ctx);
            bool through_bufgctrl = boost::starts_with(ds, "CLK_BUFG_BUFGCTRL") && boost::ends_with(ds, "_O") &&
                                    boost::starts_with(ss, "CLK_BUFG_BUFGCTRL") &&
                                    (boost::ends_with(ss, "_I0") || boost::ends_with(ss, "_I1"));
            if (through_bufgctrl)
                return true;
        }
    }

    // A pip prjxray has no bits for cannot be programmed, so routing through it
    // produces a bitstream that SILENTLY lacks the connection --
    // XRAY_ALLOW_MISSING_FEATURES drops the fasm line and bitgen carries on.
    // The chipdb generator flags these by checking every tile-routing pip
    // against segbits_<tile>.db and ppips_<tile>.db; see PIP_CFG_NO_BITS.
    //
    // The IO-column clock inputs are such a case: nothing in the artix7
    // database defines HCLK_IOI_I2IOCLK_*.  The router used it to carry a pad's
    // clock to a BUFG -- I2IOCLK -> IO_PLL_CLK3_DMUX -> RCLK3 -> CK_BUFRCLK1 --
    // where Vivado takes the dedicated clock-capable input path and never
    // touches these wires.  The result was a board that configures and does
    // nothing at all, because the ROOT clock never reached its buffer.
    //
    // The pseudo-pip table is the exception: those pips carry hand-written fasm
    // in fasm.cc and are emittable despite having no database entry, so the
    // router must still be allowed to use them.
    bool routing_pip_without_bits =
            pip_type == PIP_TILE_ROUTING && (uint32_t(extra_data.pip_config) & PIP_CFG_NO_BITS);
    if (routing_pip_without_bits) {
        if (!pseudo_pip_keys_valid) {
            xlnx_build_pseudo_pip_config(ctx, pseudo_pip_config);
            pseudo_pip_keys_valid = true;
        }
        IdString tt = IdString(chip_tile_info(ctx->chip_info, pip.tile).type_name);
        IdString src = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pip_data.src_wire].name);
        IdString dst = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pip_data.dst_wire].name);
        // The rule is: reject a pip iff FasmBackend::write_pip would emit a
        // feature that prjxray cannot resolve.  Anywhere write_pip
        // deliberately emits NOTHING, the pip costs no bits and is fine to
        // use, so the two exemption paths there must be mirrored here or we
        // reject pips that were never a problem.  (Banning the DSP class broke
        // VCC -> DSP48.OPMODE*INV_OUT routing outright.)
        std::string tts = tt.str(ctx);
        bool writer_emits_nothing = false;
        bool is_dsp_tile = tts == "DSP_L" || tts == "DSP_R";
        bool is_sing_ioi3_tile = tts == "RIOI3_SING" || tts == "LIOI3_SING" || tts == "RIOI_SING";
        if (is_dsp_tile) {
            // fasm.cc: "FIXME: PPIPs missing for DSPs" -- whole tile skipped
            writer_emits_nothing = true;
        } else if (is_sing_ioi3_tile) {
            // fasm.cc: "FIXME: PPIPs missing for SING IOI3s"
            std::string sn = src.str(ctx), dn = dst.str(ctx);
            bool from_imux_or_ctrl = sn.find("IMUX") != std::string::npos || sn.find("CTRL0") != std::string::npos;
            bool to_clock_wire = dn.find("CLK") != std::string::npos;
            writer_emits_nothing = from_imux_or_ctrl && !to_clock_wire;
        }
        bool has_pseudo_pip_fasm = pseudo_pip_config.count(PseudoPipKey{tt, dst, src});
        bool pip_is_unprogrammable = !writer_emits_nothing && !has_pseudo_pip_fasm;
        if (pip_is_unprogrammable)
            return true;
    }
    if (pip_type == PIP_SITE_ENTRY) {
        WireId dst = ctx->getPipDstWire(pip);
        if (ctx->getWireType(dst) == id_INTENT_SITE_GND) {
            const auto &lts = tile_status[dst.tile].lts;
            if (lts && (lts->cells[BEL_5LUT] != nullptr || lts->cells[BEL_6LUT] != nullptr))
                return true; // Ground driver only available if lowest 5LUT and 6LUT not used
        }
    } else if (pip_type == PIP_CONST_DRIVER) {
        WireId dst = ctx->getPipDstWire(pip);
        const auto &lts = tile_status[dst.tile].lts;
        if (lts && (lts->cells[BEL_5LUT] != nullptr || lts->cells[BEL_6LUT] != nullptr))
            return true; // Ground driver only available if lowest 5LUT and 6LUT not used
    } else if (pip_type == PIP_SITE_INTERNAL) {
        if (extra_data.bel_name == ID_TRIBUF)
            return true;
        auto site = BelSiteKey::unpack(extra_data.site_key);
        // Check site variant of PIP matches configured site variant of tile
        if (site.site >= 0 && site.site < int(tile_status[pip.tile].site_variant.size())) {
            if (site.site_variant > 0 && site.site_variant != tile_status[pip.tile].site_variant.at(site.site))
                return true;
        }
    } else if (pip_type == PIP_LUT_PERMUTATION) {
        const auto &lts = tile_status[pip.tile].lts;
        if (!lts)
            return false;
        int eight = (extra_data.pip_config >> 8) & 0xF;

        if (((extra_data.pip_config >> 4) & 0xF) == (extra_data.pip_config & 0xF))
            return false; // from==to, always valid

        auto lut6 = get_tags(lts->cells[(eight << 4) | BEL_6LUT]);
        if (lut6 && (lut6->lut.is_memory || lut6->lut.is_srl))
            return true;
        auto lut5 = get_tags(lts->cells[(eight << 4) | BEL_5LUT]);
        if (lut5 && (lut5->lut.is_memory || lut5->lut.is_srl))
            return true;
    } else if (pip_type == PIP_LUT_ROUTETHRU) {
        int eight = (extra_data.pip_config >> 8) & 0xF;
        int dest = (extra_data.pip_config & 0x1);
        if (eight == 0)
            return true; // FIXME: conflict with ground
        if (dest & 0x1)
            return true; // FIXME: routethru to MUX
        const auto &lts = tile_status[pip.tile].lts;
        if (!lts)
            return false;
        const CellInfo *lut6 = lts->cells[(eight << 4) | BEL_6LUT];
        if (lut6)
            return true;
        const CellInfo *lut5 = lts->cells[(eight << 4) | BEL_5LUT];
        if (lut5)
            return true;
    }

    return false;
}

// set_property LOC <site> [get_cells <name>], for cells that are not pads.
//
// The XDC reader already stores LOC on any cell, but until now only pack_io
// acted on it, so a constraint on an MMCM, a BUFG or a transceiver parsed
// cleanly and did nothing.  That is worse than rejecting it: the design places
// somewhere else and nothing says so.
//
// It matters for the clocking around a gigabit transceiver, where which CMT
// column an MMCM sits in decides whether the GT's clocks can reach it at all.
// The sites Vivado chooses are the ones known to work; this is how a design
// says "put it there".
//
// With only_types, just the cells of those types: the packer applies the
// clocking cells' LOCs before it places their neighbours from them.
void XilinxImpl::apply_loc_constraints(const pool<IdString> &only_types)
{
    dict<std::pair<IdString, IdString>, BelId> by_site_and_type;
    dict<IdString, int> site_seen;
    for (BelId bel : ctx->getBels()) {
        // Not every bel sits in a site the tile enumerates -- routing bels and
        // the pseudo-bels carry an index that is not one of them -- and asking
        // for the name of one of those walks off the end of the array.
        SiteIndex si = get_bel_site(bel);
        const auto &sites = tile_extra_data(si.tile)->sites;
        bool bel_in_enumerated_site = si.site >= 0 && si.site < int32_t(sites.ssize());
        if (!bel_in_enumerated_site)
            continue;
        IdString site = get_site_name(si);
        site_seen[site]++;
        by_site_and_type.emplace(std::make_pair(site, ctx->getBelType(bel)), bel);
    }

    // Which cell wants which bel, worked out before anything moves.  Packing
    // has already bound some of these -- clock buffers especially -- so the
    // wanted site can be occupied by another constrained cell that has not been
    // moved yet, and binding them one at a time collides on an ordering that
    // means nothing.  Resolve first, then unbind everything that is in the
    // wrong place, then bind.
    std::vector<std::pair<CellInfo *, BelId>> wanted;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        auto loc = ci->attrs.find(id_LOC);
        if (loc == ci->attrs.end())
            continue;
        const bool type_is_left_for_later = !only_types.empty() && !only_types.count(ci->type);
        if (type_is_left_for_later)
            continue;
        // A pad's LOC is a PACKAGE_PIN, not a site, and pack_io resolves it
        // against the package rather than the tile grid.  That one belongs to
        // pack_io; this pass is for everything else.
        if (ci->attrs.count(id_PACKAGE_PIN))
            continue;
        const std::string &loc_str = loc->second.as_string();
        // A LOC can name either a SITE (SLICE_X0Y0, MMCME2_ADV_X1Y5) or a
        // PACKAGE PIN (C10, AH8) -- XDCs in the wild use it for both, and
        // pack_io is what resolves the pin case against the package.  Only the
        // site form belongs to this pass, and a site name always carries its
        // coordinates.
        auto xpos = loc_str.rfind("_X");
        bool loc_names_a_site = xpos != std::string::npos && loc_str.find('Y', xpos) != std::string::npos;
        if (!loc_names_a_site)
            continue;
        IdString site = ctx->id(loc_str);
        if (!site_seen.count(site))
            log_error("cell '%s' is constrained to site '%s', which this device does not have\n",
                      ctx->nameOf(ci), site.c_str(ctx));
        auto found = by_site_and_type.find(std::make_pair(site, ci->type));
        if (found == by_site_and_type.end())
            log_error("cell '%s' of type '%s' is constrained to site '%s', which has no bel of that type\n",
                      ctx->nameOf(ci), ci->type.c_str(ctx), site.c_str(ctx));
        wanted.emplace_back(ci, found->second);
    }

    for (auto &w : wanted) {
        const bool bound_to_wrong_bel = w.first->bel != BelId() && w.first->bel != w.second;
        if (bound_to_wrong_bel) {
            // pack_gt binds a transceiver to the site its pads are hardwired
            // to. A LOC that names a different site used to win silently, and
            // the bitstream then configured that other channel while the pads
            // stayed on the first. The pads cannot be rerouted, so this is an
            // error rather than a move.
            const bool is_gt_channel = w.first->type.in(id_GTPE2_CHANNEL, id_GTXE2_CHANNEL);
            const bool is_gt_common = w.first->type.in(id_GTPE2_COMMON, id_GTXE2_COMMON);
            const bool gt_placed_from_its_pads = is_gt_channel || is_gt_common;
            if (gt_placed_from_its_pads)
                log_error("cell '%s' of type '%s' is constrained to site '%s', but its pads place it at '%s'\n",
                          ctx->nameOf(w.first), w.first->type.c_str(ctx),
                          get_site_name(get_bel_site(w.second)).c_str(ctx),
                          get_site_name(get_bel_site(w.first->bel)).c_str(ctx));
            ctx->unbindBel(w.first->bel);
        }
    }
    // A wanted bel can still be occupied by a cell nobody constrained, because
    // packing put it there before any of this ran.  A LOC is a requirement,
    // not a preference, so the squatter yields and the placer finds it
    // somewhere else; only another CONSTRAINED cell is a genuine conflict.
    for (auto &w : wanted) {
        if (ctx->checkBelAvail(w.second))
            continue;
        CellInfo *sitting = ctx->getBoundBelCell(w.second);
        bool occupied_by_another_cell = sitting != nullptr && sitting != w.first;
        if (!occupied_by_another_cell)
            continue;
        if (sitting->attrs.count(id_LOC))
            continue;   // both constrained here: reported as a conflict below
        ctx->unbindBel(w.second);
    }
    int placed = 0;
    for (auto &w : wanted) {
        if (w.first->bel == w.second)
            continue;   // packing already put it exactly there
        if (!ctx->checkBelAvail(w.second))
            log_error("cell '%s' is constrained to site '%s', already taken by '%s'\n", ctx->nameOf(w.first),
                      get_site_name(get_bel_site(w.second)).c_str(ctx),
                      ctx->nameOf(ctx->getBoundBelCell(w.second)));
        ctx->bindBel(w.second, w.first, STRENGTH_LOCKED);
        placed++;
    }
    if (placed)
        log_info("Placed %d cell(s) from LOC constraints.\n", placed);
}

// -o preplaced=<file>: one line per cell, "<cell name>\t<bel name>", as
// scripts/routing_dump.py writes from a reference build's --write JSON.
// Applied after packing, so the packer's own cells (split LUTs, constant
// LUTs, feed-throughs) are pinned along with the netlist's; the placer's
// constraint pass binds them as a set.  A cell the file names that this
// design lacks is noted and skipped.
void XilinxImpl::apply_preplaced(bool verbose)
{
    const ArchArgs &args = ctx->args;
    if (!args.options.count("preplaced"))
        return;
    std::string path = args.options.at("preplaced").as<std::string>();
    std::ifstream in(path);
    if (!in)
        log_error("cannot read preplaced file '%s'\n", path.c_str());
    int pinned = 0, missing = 0;
    std::string line;
    while (std::getline(in, line)) {
        auto tab = line.find('\t');
        const bool line_has_no_tab = (tab == std::string::npos);
        if (line_has_no_tab)
            continue;
        auto it = ctx->cells.find(ctx->id(line.substr(0, tab)));
        const bool this_design_has_no_such_cell = (it == ctx->cells.end());
        if (this_design_has_no_such_cell) {
            const bool note_this_missing_cell = verbose && missing < 40;
            if (note_this_missing_cell)
                log_info("Pre-placed: no cell '%s' in this design\n", line.substr(0, tab).c_str());
            missing++;
            continue;
        }
        it->second->attrs[ctx->id("BEL")] = line.substr(tab + 1);
        pinned++;
        // The pinned cells' tiles are theirs alone: a new cell placed beside
        // them would need site routing (bypass pins, output muxes) the
        // reference routes may already lock, and the router cannot rip those up.
        BelId bel = ctx->getBelByNameStr(line.substr(tab + 1));
        const bool bel_name_resolved = (bel != BelId());
        if (bel_name_resolved)
            frozen_tiles.insert(bel.tile);
    }
    if (verbose)
        log_info("Pre-placed: %d cell(s) pinned to their reference bel, %d not in this design; %d tile(s) closed to other cells.\n",
                 pinned, missing, int(frozen_tiles.size()));
}

// -o holdbufs=<file>: one line per hold-fix feedthrough buffer of the
// reference build, "<cell>\t<input net>\t<output net>\t<sink cell>\t<sink port>"
// (scripts/routing_dump.py writes it).  Each is re-created here, before
// placement, exactly as fixup_hold() makes them, so the reference's bels
// and routes name only cells and nets this design has; the hold-fix then
// finds nothing to do on the replayed part.
void XilinxImpl::apply_holdbufs()
{
    const ArchArgs &args = ctx->args;
    if (!args.options.count("holdbufs"))
        return;
    std::string path = args.options.at("holdbufs").as<std::string>();
    std::ifstream in(path);
    if (!in)
        log_error("cannot read holdbufs file '%s'\n", path.c_str());
    int made = 0, skipped = 0;
    std::string line;
    while (std::getline(in, line)) {
        std::vector<std::string> f;
        boost::split(f, line, boost::is_any_of("\t"));
        const bool record_is_malformed = (f.size() != 5);
        if (record_is_malformed)
            continue;
        IdString buf_name = ctx->id(f[0]), in_net = ctx->id(f[1]), out_net = ctx->id(f[2]), sink_name = ctx->id(f[3]),
                 sink_port = ctx->id(f[4]);
        const bool buffer_is_not_as_in_the_reference = !ctx->nets.count(in_net) || !ctx->cells.count(sink_name) ||
                                                       ctx->cells.count(buf_name) != 0 || ctx->nets.count(out_net) != 0;
        if (buffer_is_not_as_in_the_reference) {
            log_info("holdbufs: %s skipped (net %s, sink %s: not as in the reference)\n", f[0].c_str(), f[1].c_str(),
                     f[3].c_str());
            skipped++;
            continue;
        }
        NetInfo *net = ctx->nets.at(in_net).get();
        CellInfo *sink = ctx->cells.at(sink_name).get();
        const bool sink_port_is_not_on_that_net = (sink->getPort(sink_port) != net);
        if (sink_port_is_not_on_that_net) {
            log_info("holdbufs: %s skipped (%s.%s is not on %s)\n", f[0].c_str(), f[3].c_str(), f[4].c_str(),
                     f[1].c_str());
            skipped++;
            continue;
        }
        NetInfo *buf_out = ctx->createNet(out_net);
        CellInfo *buf = ctx->createCell(buf_name, id_SLICE_LUTX);
        buf->addInput(id_A1);
        buf->addOutput(id_O6);
        buf->params[id_INIT] = Property(2, 2);
        buf->attrs[id_X_ORIG_TYPE] = std::string("LUT1");
        buf->attrs[ctx->id("X_ORIG_PORT_A1")] = std::string("I0");
        buf->attrs[ctx->id("X_ORIG_PORT_O6")] = std::string("O");
        buf->connectPort(id_A1, net);
        buf->connectPort(id_O6, buf_out);
        sink->disconnectPort(sink_port);
        sink->connectPort(sink_port, buf_out);
        made++;
    }
    log_info("Hold buffers: %d re-created from the reference, %d skipped.\n", made, skipped);
    const bool some_buffer_was_re_created = made != 0;
    if (some_buffer_was_re_created) {
        ctx->assignArchInfo();
        assign_cell_tags();
    }
}

// With -o prerouted, the logic and block-RAM tiles a reference route passes
// through are closed to new cells as well: a route-through LUT, or a block
// RAM's address cascade used as a way in to the block RAM below, is a site
// resource a new cell there would need.
void XilinxImpl::close_routed_tiles()
{
    const ArchArgs &args = ctx->args;
    if (!args.options.count("prerouted"))
        return;
    std::ifstream in(args.options.at("prerouted").as<std::string>());
    std::string line;
    int closed = 0;
    while (std::getline(in, line)) {
        auto tab = line.find('\t');
        const bool line_has_no_tab = (tab == std::string::npos);
        if (line_has_no_tab)
            continue;
        size_t pos = tab + 1;
        while (pos < line.size()) {
            size_t semi = line.find(';', pos);
            const bool no_more_separators = (semi == std::string::npos);
            if (no_more_separators)
                semi = line.size();
            // a wire is "<tile>/<name>", a pip "<tile>/<dst>/<src>", a strength a number: only wires wanted
            std::string tok = line.substr(pos, semi - pos);
            pos = semi + 1;
            const bool token_is_not_a_wire = tok.empty() || std::count(tok.begin(), tok.end(), '/') != 1;
            if (token_is_not_a_wire)
                continue;
            WireId w = ctx->getWireByName(IdStringList::parse(ctx, tok));
            const bool wire_name_did_not_resolve = (w == WireId() || w.tile < 0);
            if (wire_name_did_not_resolve)
                continue;
            auto close = [&](int tile) {
                const bool tile_is_out_of_range = tile < 0 || tile >= ctx->chip_info->width * ctx->chip_info->height;
                if (tile_is_out_of_range)
                    return;
                BelId probe;
                probe.tile = tile;
                probe.index = 0;
                const bool tile_holds_cells_and_is_not_closed_yet =
                        (is_logic_tile(probe) || is_bram_tile(probe)) && frozen_tiles.count(tile) == 0;
                if (tile_holds_cells_and_is_not_closed_yet) {
                    frozen_tiles.insert(tile);
                    closed++;
                }
            };
            close(w.tile);
            // An interconnect tile's pin-feeding wires (bypass, fan-out, input
            // muxes, control) belong to the site tiles beside it: a locked one
            // -- a bounce through BYP_ALT7 -- can be the only way into a pin
            // (DX) a new cell there would need.
            std::string wn = tok.substr(tok.find('/') + 1);
            const bool wire_feeds_a_site_pin = wn.rfind("BYP", 0) == 0 || wn.rfind("FAN", 0) == 0 ||
                                               wn.rfind("IMUX", 0) == 0 || wn.rfind("CTRL", 0) == 0 ||
                                               wn.rfind("GFAN", 0) == 0;
            if (wire_feeds_a_site_pin) {
                int x, y;
                tile_xy(ctx->chip_info, w.tile, x, y);
                const bool tile_has_a_neighbour_to_the_left = (x > 0);
                if (tile_has_a_neighbour_to_the_left)
                    close(tile_by_xy(ctx->chip_info, x - 1, y));
                const bool tile_has_a_neighbour_to_the_right = (x + 1 < ctx->chip_info->width);
                if (tile_has_a_neighbour_to_the_right)
                    close(tile_by_xy(ctx->chip_info, x + 1, y));
            }
        }
    }
    const bool some_tiles_were_closed = closed != 0;
    if (some_tiles_were_closed)
        log_info("Pre-routed: %d more tile(s) closed to new cells, a reference route passes through them.\n", closed);
}

void XilinxImpl::prePlace()
{
    apply_holdbufs();
    apply_preplaced();
    close_routed_tiles();
    // Before placement, so the measured table reaches the placer (through
    // predictDelay and criticality) as well as the router's A* guidance.  On by
    // default: the measured matrix converges the router (rocket goes from 672
    // grinding iterations to ~23) and calibrates the placer (a 60%-pessimistic
    // pre-route estimate becomes ~3%), so it is the right behaviour to get
    // without a flag.  "-o delay-matrix=off" falls back to the tuned formula;
    // "-o delay-matrix=<file>" caches the table there; "-o delay-matrix=build"
    // (or absent) builds it fresh.  If the build cannot find enough to measure
    // on a given device it leaves dm_valid false and the formula stands, so an
    // untested device degrades rather than breaks.
    const ArchArgs &dm_args = ctx->args;
    std::string dm = dm_args.options.count("delay-matrix") ? dm_args.options["delay-matrix"].as<std::string>() : "build";
    if (dm != "off") {
        if (dm != "build")
            ctx->settings[ctx->id("xilinx/delayMatrixFile")] = dm;
        build_delay_matrix();
    }
    apply_loc_constraints();
    init_clock_tree();
    assign_cell_tags();
    index_control_sets();
    cell_tags_set = true;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel != BelId())
            notifyBelChange(ci->bel, ci);
    }
}

void XilinxImpl::postPlace()
{
    // "-o timing-opt=R[,N]": the timing-driven placement optimisation of common/place/timing_opt.cc (upstream; the iCE40 flow
    // runs it behind --opt-timing) on the LUTs and flops of the critical paths, moving a cell up to R tiles, N rounds (default
    // 30), with the measured delay matrix giving its estimates.  Off unless asked for, until it is measured on xc7
    // (docs/physopt-pass-proposal.md in the tools repository).
    replace_around_brams();
    const bool timing_opt_asked = ctx->args.options.count("timing-opt");
    if (timing_opt_asked) {
        std::string v = ctx->args.options.at("timing-opt").as<std::string>();
        TimingOptCfg cfg(ctx);
        cfg.cellTypes.insert(id_SLICE_LUTX);
        cfg.cellTypes.insert(id_SLICE_FFX);
        cfg.radius = std::max(1, std::atoi(v.c_str()));
        auto comma = v.find(',');
        const bool has_iterations = comma != std::string::npos;
        if (has_iterations)
            cfg.iterations = std::max(1, std::atoi(v.c_str() + comma + 1));
        log_info("timing-opt: radius %d tiles, %d rounds\n", cfg.radius, cfg.iterations);
        timing_opt(ctx, cfg);
    }
    fixup_placement();
    ctx->assignArchInfo();
}

void XilinxImpl::configureRouter2(Router2Cfg &cfg)
{
    // Routing configuration proven on xc7 by nextpnr-xilinx
    cfg.bb_margin_x = 4;
    cfg.bb_margin_y = 4;
    cfg.backwards_max_iter = 200;
    cfg.perf_profile = true;
}

void XilinxImpl::configurePlacerHeap(PlacerHeapCfg &cfg)
{
    cfg.hpwl_scale_x = 2;
    cfg.hpwl_scale_y = 1;
    // command.cc stores the generic HeAP default when --placer-heap-beta is
    // absent, as the text std::to_string(0.9), so the setting is always
    // present.  This arch wants 0.5 in that case.  Any other value came from
    // the user and is kept.  An explicit 0.9 is the same text and gets 0.5
    // as well.
    const std::string beta_text = ctx->setting<std::string>("placerHeap/beta");
    const bool beta_is_generic_default = beta_text == std::to_string(0.9);
    if (beta_is_generic_default)
        cfg.beta = 0.5;
    cfg.placeAllAtOnce = true;
    cfg.get_cell_legalisation_weight = [this](Context *, CellInfo *ci) {
        if (ci->type != id_SLICE_LUTX)
            return 1;
        auto tags = get_tags(ci);
        // Place memory first, because they require entire SLICEMs
        return tags->lut.is_memory ? 100 : 1;
    };

    cfg.ff_bel_bucket = id_SLICE_FFX;
    cfg.ff_control_set_groups.resize(2);
    for (int z = 0; z < 8; z++) {
        cfg.ff_control_set_groups.at(z / 4).push_back((z << 4) | BEL_FF);
        cfg.ff_control_set_groups.at(z / 4).push_back((z << 4) | BEL_FF2);
    }
    cfg.ctrl_set_max_radius = std::vector<int>{18, 15, 12, 9, 6, 3};

    cfg.get_cell_control_set = [this](Context *, const CellInfo *ci) {
        if (ci->type != id_SLICE_FFX)
            return -1;
        auto tags = get_tags(ci);
        return tags->ff.control_set;
    };
}

void XilinxImpl::configurePlacerStatic(PlacerStaticCfg &cfg)
{
    cfg.hpwl_scale_x = 2;
    cfg.hpwl_scale_y = 1;

    cfg.glbBufTypes.insert(id_PSEUDO_GND);
    cfg.glbBufTypes.insert(id_PSEUDO_VCC);
    cfg.glbBufTypes.insert(id_BUFGCTRL);
    cfg.glbBufTypes.insert(id_BUFG_BUFG);

    cfg.timing_c = 500;
    cfg.timing_mx = 25;
    cfg.timing_my = 50;

    cfg.predict_delay = [](Context *ctx, const PlacerStaticCfg &cfg, Loc src_loc, IdString src_pin, Loc dst_loc,
                           IdString dst_pin) -> delay_t {
        if (dst_pin == id_CIN && src_pin == id_CO3)
            return 0;
        // TODO: improve sophistication here based on old nextpnr-xilinx code
        int dist_x = std::abs(dst_loc.x - src_loc.x), dist_y = std::abs(dst_loc.y - src_loc.y);
        return 500 + 12 * (2 * std::max(dist_y - 6, 0) + 4 * std::min(dist_y, 6) + std::max(dist_x - 12, 0) +
                           2 * std::min(dist_x, 12));
    };

    {
        cfg.cell_groups.emplace_back();
        auto &comb = cfg.cell_groups.back();
        comb.name = ctx->id("COMB");
        comb.bel_area[id_SLICE_LUTX] = StaticRect(1.0f, 0.0625f);
        comb.bel_area[id_CARRY4] = StaticRect(0.0f, 0.0f);
        comb.bel_area[id_SELMUX2_1] = StaticRect(0.0f, 0.0f);

        comb.cell_area[id_SLICE_LUTX] = StaticRect(1.0f, 0.125f);
        comb.cell_area[id_CARRY4] = StaticRect(1.0f, 0.5f);
        comb.cell_area[id_SELMUX2_1] = StaticRect(1.0f, 0.125f);

        comb.zero_area_cells.insert(id_CARRY4);
        comb.zero_area_cells.insert(id_SELMUX2_1);

        comb.spacer_rect = StaticRect(1.0f, 0.125f);
    }

    {
        cfg.cell_groups.emplace_back();
        auto &comb = cfg.cell_groups.back();
        comb.name = ctx->id("FF");
        // Assume one FF occupies 1.5 bels due to control set packing and slice input restrictions
        comb.cell_area[id_SLICE_FFX] = StaticRect(1.0f, 0.0875f);
        comb.bel_area[id_SLICE_FFX] = StaticRect(1.0f, 0.0625f);
        comb.spacer_rect = StaticRect(1.0f, 0.125f);
    }

    {
        cfg.cell_groups.emplace_back();
        auto &comb = cfg.cell_groups.back();
        comb.name = ctx->id("RAM");
        comb.cell_area[id_RAMB18E1_RAMB18E1] = StaticRect(1.0f, 3.0f);
        comb.bel_area[id_RAMB18E1_RAMB18E1] = StaticRect(1.0f, 3.0f);
        comb.cell_area[id_RAMB36E1_RAMB36E1] = StaticRect(1.0f, 6.0f);
        comb.bel_area[id_RAMB36E1_RAMB36E1] = StaticRect(0.0f, 0.0f);
        comb.spacer_rect = StaticRect(1.0f, 3.0f);
    }
    {
        cfg.cell_groups.emplace_back();
        auto &comb = cfg.cell_groups.back();
        comb.name = ctx->id("DSP");
        comb.cell_area[id_DSP48E1_DSP48E1] = StaticRect(1.0f, 3.0f);
        comb.bel_area[id_DSP48E1_DSP48E1] = StaticRect(1.0f, 3.0f);
        comb.spacer_rect = StaticRect(1.0f, 3.0f);
    }
    cfg.get_cell_area_override = [this](Context *ctx, const CellInfo *ci) -> std::optional<StaticRect> {
        if (ci->type != id_SLICE_LUTX)
            return {};
        auto tags = get_tags(ci);
        if (tags->lut.is_memory ||
            (ci->cluster != ClusterId() && ctx->getClusterRootCell(ci->cluster)->type == id_CARRY4)) {
            // macro LUTs use either a half or whole LUT, always
            return {(tags->lut.input_count == 6) ? StaticRect(1.0f, 0.125f) : StaticRect(1.0f, 0.0625f)};
        } else {
            switch (tags->lut.input_count) { // sliding scale, smaller LUTs pack better
            case 6:
                return {StaticRect(1.0f, 0.125f)};
            case 5:
                return {StaticRect(1.0f, 0.125f)};
            case 4:
                return {StaticRect(1.0f, 0.1f)};
            case 3:
                return {StaticRect(1.0f, 0.08f)};
            case 2:
                return {StaticRect(1.0f, 0.06f)};
            case 1:
                return {StaticRect(1.0f, 0.04f)};
            default:
                NPNR_ASSERT_FALSE("unhandled LUT input count");
            }
        }
    };
}

void XilinxImpl::preRoute()
{
    if (!cell_tags_set) {
        // We loaded a pre-placed design. Need to set tags and update bel-cell map
        assign_cell_tags();
        index_control_sets();
        cell_tags_set = true;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            notifyBelChange(ci->bel, ci);
        }
        if (ctx->nets.count(ctx->id("$PACKER_GND_NET"))) {
            ctx->nets.at(ctx->id("$PACKER_GND_NET"))->constant_value = id_GND;
        }
        if (ctx->nets.count(ctx->id("$PACKER_VCC_NET"))) {
            ctx->nets.at(ctx->id("$PACKER_VCC_NET"))->constant_value = id_VCC;
        }
    }
    find_source_sink_locs();
    route_clocks();
    apply_prerouted();
}

// -o prerouted=<file>: one line per net, "<net name>\t<wire>;<pip>;<strength>;..."
// as nextpnr's own ROUTING attribute spells it (a source wire has an empty
// pip).  A net whose name and driver survive packing gets its reference
// route bound with STRENGTH_LOCKED; a net that has gone, or whose route
// collides with something already bound (the clocks are routed first), is
// left to the router with a note.
void XilinxImpl::apply_prerouted()
{
    const ArchArgs &args = ctx->args;
    if (!args.options.count("prerouted"))
        return;
    std::string path = args.options.at("prerouted").as<std::string>();
    std::ifstream in(path);
    if (!in)
        log_error("cannot read prerouted file '%s'\n", path.c_str());
    int bound_nets = 0, missing = 0, collided = 0, pruned = 0;
    std::string line;
    while (std::getline(in, line)) {
        auto tab = line.find('\t');
        const bool line_has_no_tab = (tab == std::string::npos);
        if (line_has_no_tab)
            continue;
        IdString net_id = ctx->id(line.substr(0, tab));
        auto it = ctx->nets.find(net_id);
        const bool this_design_has_no_such_net = (it == ctx->nets.end());
        if (this_design_has_no_such_net) {
            const bool note_this_missing_net = (missing < 10);
            if (note_this_missing_net)
                log_info("Pre-routed: net '%s' is not in this design\n", net_id.c_str(ctx));
            missing++;
            continue;
        }
        NetInfo *ni = it->second.get();
        std::vector<std::pair<WireId, PipId>> previous;   // the route this replaces, if any
        const bool route_clocks_got_here_first = !ni->wires.empty();
        if (route_clocks_got_here_first) {
            // route_clocks got here first.  The reference's tree replaces
            // its work even when it reached every sink: a clock with a sink
            // added since (a probe) would otherwise get a fresh tree, and
            // the arrival at every reference leaf with it.  The router adds
            // the new leaves to the locked tree.
            log_info("Pre-routed: net '%s' was routed already (%zu wires); the reference route replaces it\n",
                     net_id.c_str(ctx), ni->wires.size());
            // Remember it in the order the router built it -- a wire's pip
            // can only be re-bound once its source wire is back -- so that a
            // reference route which turns out to collide leaves the net as
            // it found it rather than unrouted.
            for (auto &w : ni->wires)
                previous.push_back(std::make_pair(w.first, w.second.pip));
            std::stable_sort(previous.begin(), previous.end(),
                             [&](const std::pair<WireId, PipId> &a, const std::pair<WireId, PipId> &b) {
                                 const bool a_is_a_source_wire = (a.second == PipId());
                                 const bool b_is_a_source_wire = (b.second == PipId());
                                 return a_is_a_source_wire && !b_is_a_source_wire;
                             });
            for (auto &w : previous)
                ctx->unbindWire(w.first);
        }
        std::vector<std::string> strs;
        boost::split(strs, line.substr(tab + 1), boost::is_any_of(";"));
        std::vector<PipId> pips_bound;
        std::vector<WireId> wires_bound;
        bool reference_route_collided = false;
        for (size_t i = 0; i + 2 < strs.size(); i += 3) {
            const std::string &wire = strs[i], &pip = strs[i + 1];
            const bool entry_is_a_source_wire = pip.empty();
            if (entry_is_a_source_wire) {
                WireId w = ctx->getWireByNameStr(wire);
                const bool wire_name_did_not_resolve = (w == WireId());
                if (wire_name_did_not_resolve) {
                    reference_route_collided = true;
                    break;
                }
                NetInfo *bound_net = ctx->getBoundWireNet(w);
                const bool wire_is_someone_elses = (bound_net != nullptr && bound_net != ni);
                if (wire_is_someone_elses) {
                    reference_route_collided = true;
                    break;
                }
                const bool wire_is_still_free = (bound_net == nullptr);
                if (wire_is_still_free) {
                    ctx->bindWire(w, ni, STRENGTH_LOCKED);
                    wires_bound.push_back(w);
                }
            } else {
                PipId p = ctx->getPipByNameStr(pip);
                const bool pip_name_did_not_resolve = (p == PipId());
                if (pip_name_did_not_resolve) {
                    reference_route_collided = true;
                    break;
                }
                WireId dst = ctx->getPipDstWire(p);
                const bool pip_destination_is_taken = (ctx->getBoundWireNet(dst) != nullptr);
                if (pip_destination_is_taken) {
                    reference_route_collided = true;
                    break;
                }
                ctx->bindPip(p, ni, STRENGTH_LOCKED);
                pips_bound.push_back(p);
            }
        }
        if (reference_route_collided) {
            for (auto p : pips_bound)
                ctx->unbindPip(p);
            for (auto w : wires_bound)
                ctx->unbindWire(w);
            // Put back whatever route_clocks had made, so a net this pass
            // cannot improve is no worse for having been tried.
            for (auto &w : previous) {
                const bool wire_has_been_taken_since = (ctx->getBoundWireNet(w.first) != nullptr);
                if (wire_has_been_taken_since)
                    continue;
                const bool wire_came_from_no_pip = (w.second == PipId());
                if (wire_came_from_no_pip)
                    ctx->bindWire(w.first, ni, STRENGTH_LOCKED);
                else {
                    const bool pip_is_free_again = ctx->checkPipAvail(w.second);
                    if (pip_is_free_again)
                        ctx->bindPip(w.second, ni, STRENGTH_LOCKED);
                }
            }
            collided++;
            continue;
        }
        // Keep only what leads from the source to a sink this design has:
        // a reference route can end at a hold-fix buffer that does not exist
        // yet, and a dangling branch fails the router's tree check.
        pool<WireId> keep;
        for (auto &usr : ni->users) {
            for (int i = 0; i < ctx->getNetinfoSinkWireCount(ni, usr); i++) {
                WireId cur = ctx->getNetinfoSinkWire(ni, usr, i);
                auto wire_is_still_to_walk = [&]() {
                    const bool wire_is_off_this_route = cur == WireId() || ni->wires.count(cur) == 0;
                    if (wire_is_off_this_route)
                        return false;
                    const bool wire_is_new_to_the_keep_set = keep.count(cur) == 0;
                    return wire_is_new_to_the_keep_set;
                };
                while (wire_is_still_to_walk()) {
                    keep.insert(cur);
                    PipId p = ni->wires.at(cur).pip;
                    const bool wire_is_the_source = (p == PipId());
                    if (wire_is_the_source)
                        break;
                    cur = ctx->getPipSrcWire(p);
                }
            }
        }
        std::vector<WireId> drop;
        for (auto &w : ni->wires) {
            const bool wire_is_not_on_a_path_to_a_sink = (keep.count(w.first) == 0);
            if (wire_is_not_on_a_path_to_a_sink)
                drop.push_back(w.first);
        }
        for (WireId w : drop) {
            PipId p = ni->wires.at(w).pip;
            const bool wire_was_reached_through_a_pip = (p != PipId());
            if (wire_was_reached_through_a_pip)
                ctx->unbindPip(p);
            else
                ctx->unbindWire(w);
        }
        const bool route_lost_a_dangling_branch = !drop.empty();
        if (route_lost_a_dangling_branch)
            pruned += drop.size();
        bound_nets++;
    }
    log_info("Pre-routed: %d net(s) given their reference route (%d dangling wire(s) pruned), %d not in this design, "
             "%d collided and left to the router.\n",
             bound_nets, pruned, missing, collided);
}

// The top-level port a clock net comes from, through clock buffers and IOB
// input buffers (the way a clock constraint propagates from a pad), or
// IdString() if there is none or a buffer has more than one clock input.
// A BUFR is followed from O back to I only when BUFR_DIVIDE is BYPASS or
// the parameter is absent: a divider is a different clock and keeps its
// net's name. BUFR_DIVIDE is not a constid, so the name is matched as a
// string, the way the BUFGMUX types are, and no id is created.
IdString XilinxImpl::clock_source_port(const NetInfo *net) const
{
    static const dict<IdString, std::vector<IdString>> clock_inputs = {
            {id_BUFGCTRL, {id_I0, id_I1}}, {id_BUFHCE_BUFHCE, {id_I}},       {id_BUFIO_BUFIO, {id_I}},
            {id_BUFMRCE, {id_I}},          {id_IOB33M_INBUF_EN, {id_PAD}},   {id_IOB33S_INBUF_EN, {id_PAD}},
            {id_IOB33_INBUF_EN, {id_PAD}}, {id_IOB18_INBUF_DCIEN, {id_PAD}}, {id_IOB18M_INBUF_DCIEN, {id_PAD}}};
    for (int hop = 0; hop < 8; hop++) {
        for (auto &user : net->users) {
            const bool user_is_pad = user.cell->type == id_PAD;
            if (user_is_pad)
                return user.cell->name;
        }
        const CellInfo *buf = net->driver.cell;
        const bool is_undivided_bufr = buf != nullptr && buf->type == id_BUFR_BUFR && net->driver.port == id_O;
        if (is_undivided_bufr) {
            std::string divide = "BYPASS";
            for (const auto &param : buf->params) {
                const bool is_divide_param = param.first.str(ctx) == "BUFR_DIVIDE";
                if (is_divide_param) {
                    const bool value_is_string = param.second.is_string;
                    if (value_is_string)
                        divide = param.second.str;
                    break;
                }
            }
            const bool passes_the_clock = divide == "BYPASS";
            if (!passes_the_clock)
                return IdString();
            const NetInfo *clock_in = buf->getPort(id_I);
            const bool carries_a_signal = clock_in != nullptr && clock_in->constant_value == IdString();
            if (!carries_a_signal)
                return IdString();
            net = clock_in;
            continue;
        }
        auto inputs = buf == nullptr ? clock_inputs.end() : clock_inputs.find(buf->type);
        const bool is_a_buffer = inputs != clock_inputs.end();
        if (!is_a_buffer)
            return IdString();
        const NetInfo *clock_in = nullptr;
        int count = 0;
        for (IdString port : inputs->second) {
            const NetInfo *in = buf->getPort(port);
            const bool carries_a_signal = in != nullptr && in->constant_value == IdString();
            if (carries_a_signal) {
                clock_in = in;
                count++;
            }
        }
        const bool one_clock_input = count == 1;
        if (!one_clock_input)
            return IdString();
        net = clock_in;
    }
    return IdString();
}

// --report keys each clock by its net. Since YosysHQ/yosys#6103 synth_xilinx
// maps with abc9, and the net behind a clock buffer comes out of it with an
// internal name like $abc$20890$aiger$o102. Key such a clock by the port it
// comes from instead, in the timing results only: placement, routing and the
// netlist stay as they are, and the port's name is already an id.
void XilinxImpl::name_clocks_after_ports()
{
    TimingResult &result = ctx->timing_result;
    pool<IdString> clocks;
    auto add_pair = [&](const ClockPair &pair) {
        clocks.insert(pair.start.clock);
        clocks.insert(pair.end.clock);
    };
    for (auto &clock : result.clock_fmax)
        clocks.insert(clock.first);
    for (auto &path : result.clock_paths)
        add_pair(path.second.clock_pair);
    for (auto &path : result.xclock_paths)
        add_pair(path.clock_pair);
    for (auto clock : result.empty_paths)
        clocks.insert(clock);

    dict<IdString, IdString> port_of;
    dict<IdString, int> clocks_at_port;
    for (auto clock : clocks) {
        auto net = ctx->nets.find(clock);
        const bool is_internal_net = clock.c_str(ctx)[0] == '$' && net != ctx->nets.end();
        if (!is_internal_net)
            continue;
        const IdString port = clock_source_port(net->second.get());
        const bool comes_from_a_port = port != IdString();
        if (!comes_from_a_port)
            continue;
        port_of[clock] = port;
        clocks_at_port[port]++;
    }
    dict<IdString, IdString> rename;
    for (auto &entry : port_of) {
        const bool name_is_free = clocks_at_port.at(entry.second) == 1 && !clocks.count(entry.second);
        if (name_is_free)
            rename[entry.first] = entry.second;
    }
    const bool nothing_to_rename = rename.empty();
    if (nothing_to_rename)
        return;

    auto renamed = [&](IdString clock) {
        auto found = rename.find(clock);
        return found == rename.end() ? clock : found->second;
    };
    auto rename_pair = [&](ClockPair &pair) {
        pair.start.clock = renamed(pair.start.clock);
        pair.end.clock = renamed(pair.end.clock);
    };
    // A dict iterates from its last entry: rebuild these two in their order,
    // which is the order of the report's critical paths.
    std::vector<std::pair<IdString, ClockFmax>> fmax(result.clock_fmax.begin(), result.clock_fmax.end());
    result.clock_fmax.clear();
    for (auto it = fmax.rbegin(); it != fmax.rend(); ++it)
        result.clock_fmax[renamed(it->first)] = it->second;
    std::vector<std::pair<IdString, CriticalPath>> paths(result.clock_paths.begin(), result.clock_paths.end());
    result.clock_paths.clear();
    for (auto it = paths.rbegin(); it != paths.rend(); ++it) {
        rename_pair(it->second.clock_pair);
        result.clock_paths[renamed(it->first)] = std::move(it->second);
    }
    for (auto &path : result.xclock_paths)
        rename_pair(path.clock_pair);
    for (auto &path : result.min_delay_violations)
        rename_pair(path.clock_pair);
    for (auto &net : result.detailed_net_timings)
        for (auto &sink : net.second)
            rename_pair(sink.clock_pair);
    std::vector<IdString> empty_paths(result.empty_paths.begin(), result.empty_paths.end());
    result.empty_paths.clear();
    for (auto it = empty_paths.rbegin(); it != empty_paths.rend(); ++it)
        result.empty_paths.insert(renamed(*it));
    for (auto &entry : rename)
        log_info("Clock '%s' is reported as '%s', the port it comes from.\n", entry.first.c_str(ctx),
                 entry.second.c_str(ctx));
}

void XilinxImpl::postRoute()
{
    // Insert feedthrough buffers on hold-violating arcs and reroute, before
    // routing is finalised and FASM is written.  No-op unless -o hold-fix.
    fixup_hold();
    fixup_routing();
    // After the last timing analysis (hold-fix runs one).
    name_clocks_after_ports();
    ctx->assignArchInfo();
    const ArchArgs &args = ctx->args;
    if (args.options.count("fasm")) {
        write_fasm(args.options["fasm"].as<std::string>());
    }
    if (args.options.count("placement")) {
        write_placement(args.options["placement"].as<std::string>());
    }
}

void XilinxImpl::postRouteArchInfo()
{
    // The framework's archInfoToAttributes() serialised NEXTPNR_BEL and
    // ROUTING in the generic himbaechel form -- a bel as "<tile>/<site>.<bel>"
    // and a site wire as "<tile>/<site>.<pin>".  The demo-projects regression
    // checkers were written against nextpnr-xilinx's form instead: a site bel
    // is "<site>/<bel>" and a site wire is "SITEWIRE/<site>/<pin>".
    // check_const_pins.py (const-holdout) splits NEXTPNR_BEL on '/' to get the
    // slice name and the lane letter (the first character of the bel name),
    // then matches "SITEWIRE/<slice>/<pin>" against each net's ROUTING; under
    // the generic form the lane letter comes out as the 'S' of "SLICE" and the
    // site wires never match, so every RAM32M address pin reads as unrouted.
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel == BelId())
            continue;
        const SiteIndex site = get_bel_site(ci->bel);
        // PSEUDO_GND / PSEUDO_VCC and friends sit on bels with no site, so the
        // unpacked site index is 0xFFFF and must not index the sites array.
        const bool bel_has_site = site.site >= 0 && site.site < tile_extra_data(site.tile)->sites.ssize();
        if (bel_has_site)
            ci->attrs[ctx->id("NEXTPNR_BEL")] = get_site_bel_name(ci->bel).str(ctx);
    }
    auto wire_name = [&](WireId wire) -> std::string {
        const IdString type = ctx->getWireType(wire);
        const bool is_site_wire = type == id_INTENT_SITE_WIRE || type == id_INTENT_SITE_GND;
        if (!is_site_wire)
            return ctx->getWireName(wire).str(ctx);
        const std::string full = IdString(chip_wire_info(ctx->chip_info, wire).name).str(ctx);
        const size_t dot = full.find('.');
        const std::string pin = (dot == std::string::npos) ? full : full.substr(dot + 1);
        // The chipdb generator stores the owning site's index in a site wire's
        // flags field (lookup_site_wire sets nw.flags = site.primary.index).
        const int32_t site_idx = chip_wire_info(ctx->chip_info, wire).flags;
        const bool site_index_valid = site_idx >= 0 && site_idx < tile_extra_data(wire.tile)->sites.ssize();
        if (!site_index_valid)
            return ctx->getWireName(wire).str(ctx);
        const SiteIndex site(wire.tile, site_idx);
        return stringf("SITEWIRE/%s/%s", get_site_name(site).c_str(ctx), pin.c_str());
    };
    for (auto &net : ctx->nets) {
        NetInfo *ni = net.second.get();
        std::string routing;
        bool first = true;
        for (auto &item : ni->wires) {
            if (!first)
                routing += ";";
            routing += wire_name(item.first);
            routing += ";";
            if (item.second.pip != PipId())
                routing += ctx->getPipName(item.second.pip).str(ctx);
            routing += ";" + std::to_string(item.second.strength);
            first = false;
        }
        ni->attrs[ctx->id("ROUTING")] = routing;
    }
}

IdString XilinxImpl::bel_tile_type(BelId bel) const
{
    return IdString(chip_tile_info(ctx->chip_info, bel.tile).type_name);
}

bool XilinxImpl::is_logic_tile(BelId bel) const
{
    return bel_tile_type(bel).in(id_CLEL_L, id_CLEL_R, id_CLEM, id_CLEM_R, id_CLBLL_L, id_CLBLL_R, id_CLBLM_L,
                                 id_CLBLM_R);
}
bool XilinxImpl::is_bram_tile(BelId bel) const { return bel_tile_type(bel).in(id_BRAM, id_BRAM_L, id_BRAM_R); }

const XlnxTileInstExtraDataPOD *XilinxImpl::tile_extra_data(int tile) const
{
    return reinterpret_cast<const XlnxTileInstExtraDataPOD *>(ctx->chip_info->tile_insts[tile].extra_data.get());
}

std::string XilinxImpl::tile_name(int tile) const
{
    const auto &data = *tile_extra_data(tile);
    return stringf("%s_X%dY%d", IdString(data.name_prefix).c_str(ctx), data.tile_x, data.tile_y);
}

Loc XilinxImpl::rel_site_loc(SiteIndex site) const
{
    const auto &site_data = tile_extra_data(site.tile)->sites[site.site];
    return Loc(site_data.rel_x, site_data.rel_y, 0);
}

SiteIndex XilinxImpl::rel_site(SiteIndex site, int dx, int dy) const
{
    const auto &base_site_data = tile_extra_data(site.tile)->sites[site.site];
    for (size_t i = 0; i < tile_extra_data(site.tile)->sites.size(); i++) {
        const auto &site_data = tile_extra_data(site.tile)->sites[i];
        if (site_data.name_prefix == base_site_data.name_prefix && site_data.rel_x == (base_site_data.rel_x + dx) &&
            site_data.rel_y == (base_site_data.rel_y + dy))
            return SiteIndex(site.tile, i);
    }
    return SiteIndex();
}

int XilinxImpl::hclk_for_iob(BelId pad) const
{
    std::string tile_type = bel_tile_type(pad).str(ctx);
    int ioi = pad.tile;
    if (boost::starts_with(tile_type, "LIOB"))
        ioi += 1;
    else if (boost::starts_with(tile_type, "RIOB"))
        ioi -= 1;
    else
        NPNR_ASSERT_FALSE("unknown IOB side");
    return hclk_for_ioi(ioi);
}

int XilinxImpl::hclk_for_ioi(int tile) const
{
    WireId ioclk0;
    auto &td = chip_tile_info(ctx->chip_info, tile);
    for (int i = 0; i < td.wires.ssize(); i++) {
        std::string name = IdString(td.wires[i].name).str(ctx);
        if (name == "IOI_IOCLK0" || name == "IOI_SING_IOCLK0") {
            ioclk0 = ctx->normalise_wire(tile, i);
            break;
        }
    }
    NPNR_ASSERT(ioclk0 != WireId());
    for (auto uh : ctx->getPipsUphill(ioclk0))
        return uh.tile;
    NPNR_ASSERT_FALSE("failed to find HCLK pips");
}

namespace {
// The chipdb's RAMB timing variants carry the SDF's names for the write
// modes. An unset WRITE_MODE is WRITE_FIRST, the primitive's default, as in
// the FASM writer.
const char *bram_write_mode_code(const CellInfo *ci, IdString param)
{
    std::string mode = str_or_default(ci->params, param, "WRITE_FIRST");
    const bool no_change = mode == "NO_CHANGE";
    if (no_change)
        return "NC";
    const bool read_first = mode == "READ_FIRST";
    if (read_first)
        return "RF";
    return "WF";
}
} // namespace

bool XilinxImpl::cell_param_is_one(const CellInfo *cell, const char *name) const
{
    for (const auto &param : cell->params) {
        const bool same_name = param.first.str(ctx) == name;
        if (same_name)
            return int_or_default(cell->params, param.first, 0) == 1;
    }
    return false;
}

bool XilinxImpl::bram_clock_on_falling_edge(const CellInfo *cell, IdString clock_port) const
{
    const std::string clock_name = clock_port.str(ctx);
    const bool port_a = clock_name.compare(0, 9, "CLKARDCLK") == 0;
    if (port_a)
        return cell_param_is_one(cell, "IS_CLKARDCLK_INVERTED");
    const bool port_b = clock_name.compare(0, 9, "CLKBWRCLK") == 0;
    if (port_b)
        return cell_param_is_one(cell, "IS_CLKBWRCLK_INVERTED");
    return false;
}

bool XilinxImpl::clock_on_falling_edge(const CellInfo *cell, IdString clock_port) const
{
    const bool is_ff = cell->type == id_SLICE_FFX;
    if (is_ff) {
        const bool clock_is_ck = clock_port == id_CK;
        if (clock_is_ck)
            return int_or_default(cell->params, id_IS_C_INVERTED, 0) == 1;
        return false;
    }
    const bool is_lut = cell->type == id_SLICE_LUTX;
    if (is_lut) {
        const bool clock_is_clk = clock_port == id_CLK;
        if (clock_is_clk)
            return int_or_default(cell->params, id_IS_WCLK_INVERTED, 0) == 1;
        return false;
    }
    const bool is_bram = cell->type == id_RAMB18E1_RAMB18E1 || cell->type == id_RAMB36E1_RAMB36E1;
    if (is_bram)
        return bram_clock_on_falling_edge(cell, clock_port);
    return false;
}

TimingClockingInfo XilinxImpl::getPortClockingInfo(const CellInfo *cell, IdString port, int index) const
{
    TimingClockingInfo info = ctx->get_port_clocking_info_default(cell, port, index);
    const bool falling = clock_on_falling_edge(cell, info.clock_port);
    if (falling) {
        // Only the rising arcs stored in the chipdb are flipped. An arc that
        // the chipdb already marks falling is left alone.
        const bool chipdb_says_rising = info.edge == RISING_EDGE;
        if (chipdb_says_rising)
            info.edge = FALLING_EDGE;
    }
    return info;
}

void XilinxImpl::assign_cell_tags()
{
    cell_tags.resize(ctx->cells.size());
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        auto &ct = cell_tags.at(ci->flat_index);
        if (ci->type == id_SLICE_LUTX) {
            ct.lut.input_count = 0;
            for (IdString a : {id_A1, id_A2, id_A3, id_A4, id_A5, id_A6}) {
                NetInfo *pn = ci->getPort(a);
                if (pn != nullptr)
                    ct.lut.input_sigs[ct.lut.input_count++] = pn;
            }
            ct.lut.output_count = 0;
            for (IdString o : {id_O6, id_O5}) {
                NetInfo *pn = ci->getPort(o);
                if (pn != nullptr)
                    ct.lut.output_sigs[ct.lut.output_count++] = pn;
            }
            for (int i = ct.lut.output_count; i < 2; i++)
                ct.lut.output_sigs[i] = nullptr;
            ct.lut.di1_net = ci->getPort(id_DI1);
            ct.lut.di2_net = ci->getPort(id_DI2);
            ct.lut.wclk = ci->getPort(id_CLK);
            ct.lut.wclk_inv = bool_or_default(ci->params, id_IS_WCLK_INVERTED, false);
            ct.lut.we = ci->getPort(id_WE);
            ct.lut.memory_group = 0; // fixme
            ct.lut.is_srl = ci->attrs.count(id_X_LUT_AS_SRL);
            ct.lut.is_memory = ci->attrs.count(id_X_LUT_AS_DRAM);
            // A LUT-RAM or SRL output also changes on the clock: those cells take the
            // timing the chipdb keeps under their attribute, with clock-to-out arcs.
            const bool lut_is_clocked = ct.lut.is_srl || ct.lut.is_memory;
            if (lut_is_clocked) {
                const int mem_timing = ctx->get_cell_timing_idx(ct.lut.is_srl ? id_X_LUT_AS_SRL : id_X_LUT_AS_DRAM);
                const bool chipdb_has_mem_timing = mem_timing != -1;
                if (chipdb_has_mem_timing)
                    ci->timing_index = mem_timing;
            }
            ct.lut.only_drives_carry = false;
            if (ci->cluster != ClusterId() && ct.lut.output_count > 0 && ct.lut.output_sigs[0] != nullptr &&
                ct.lut.output_sigs[0]->users.entries() == 1 &&
                (*ct.lut.output_sigs[0]->users.begin()).cell->type == id_CARRY4)
                ct.lut.only_drives_carry = true;

            const IdString addr_msb_sigs[] = {id_WA7, id_WA8, id_WA9};
            for (int i = 0; i < 3; i++)
                ct.lut.address_msb[i] = ci->getPort(addr_msb_sigs[i]);

        } else if (ci->type == id_SLICE_FFX) {
            ct.ff.d = ci->getPort(id_D);
            ct.ff.clk = ci->getPort(id_CK);
            ct.ff.ce = ci->getPort(id_CE);
            ct.ff.sr = ci->getPort(id_SR);
            // IS_C_INVERTED is what an FF actually carries: yosys writes it, and
            // pack_ffs sets it for the FD*_1 falling-edge variants.  It is also
            // what the FASM writer reads for CLKINV, so the legality check has
            // to agree with it or a rising- and a falling-edge FF can share a
            // half-slice whose CLKINV bit cannot serve both.
            ct.ff.is_clkinv = int_or_default(ci->params, id_IS_C_INVERTED, 0) == 1;
            ct.ff.is_srinv = bool_or_default(ci->params, id_IS_R_INVERTED, false) ||
                             bool_or_default(ci->params, id_IS_S_INVERTED, false) ||
                             bool_or_default(ci->params, id_IS_CLR_INVERTED, false) ||
                             bool_or_default(ci->params, id_IS_PRE_INVERTED, false);
            ct.ff.is_latch = ci->attrs.count(id_X_FF_AS_LATCH);
            ct.ff.ffsync = ci->attrs.count(id_X_FFSYNC);
        } else if (ci->type.in(id_F7MUX, id_F8MUX, id_F9MUX, id_SELMUX2_1)) {
            ct.mux.sel = ci->getPort(id_S0);
            ct.mux.out = ci->getPort(id_OUT);
        } else if (ci->type == id_CARRY4) {
            for (int i = 0; i < 4; i++) {
                ct.carry.out_sigs[i] = ci->getPort(ctx->idf("O%d", i));
                ct.carry.cout_sigs[i] = ci->getPort(ctx->idf("CO%d", i));
                ct.carry.x_sigs[i] = nullptr;
            }
            ct.carry.x_sigs[0] = ci->getPort(id_CYINIT);
        } else if (ci->type == id_RAMB18E1_RAMB18E1 || ci->type == id_RAMB36E1_RAMB36E1) {
            bool read_sdp = ((ci->type == id_RAMB18E1_RAMB18E1 &&
                              int_or_default(ci->params, ctx->id("READ_WIDTH_B"), 0) == 36) ||
                             (ci->type == id_RAMB36E1_RAMB36E1 &&
                              int_or_default(ci->params, ctx->id("READ_WIDTH_B"), 0) == 72));
            bool write_sdp = ((ci->type == id_RAMB18E1_RAMB18E1 &&
                               int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) == 36) ||
                              (ci->type == id_RAMB36E1_RAMB36E1 &&
                               int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0) == 72));
            // The data-in setup and hold of each port follow its WRITE_MODE.
            const char *write_mode_a = bram_write_mode_code(ci, ctx->id("WRITE_MODE_A"));
            const char *write_mode_b = bram_write_mode_code(ci, ctx->id("WRITE_MODE_B"));
            IdString variant = ctx->idf("%s_%s_%s_%s_%s", ci->type.c_str(ctx), write_sdp ? "WSDP" : "WTDP",
                                        read_sdp ? "RSDP" : "RTDP", write_mode_a, write_mode_b);
            ci->timing_index = ctx->get_cell_timing_idx(variant);
        }
    }
}

void XilinxImpl::index_control_sets()
{
    idict<FFControlSet> control_sets;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_SLICE_FFX) {
            auto &ct = cell_tags.at(ci->flat_index);
            FFControlSet ctrl_set;
            ctrl_set.clk = ct.ff.clk ? ct.ff.clk->name : IdString();
            ctrl_set.ce = ct.ff.ce ? ct.ff.ce->name : IdString();
            ctrl_set.sr = ct.ff.sr ? ct.ff.sr->name : IdString();
            ctrl_set.flags = (ct.ff.is_clkinv ? FFControlSet::IS_CLKINV : 0) |
                             (ct.ff.is_srinv ? FFControlSet::IS_SRINV : 0) |
                             (ct.ff.is_latch ? FFControlSet::IS_LATCH : 0) | (ct.ff.ffsync ? FFControlSet::FFSYNC : 0);
            ct.ff.control_set = control_sets(ctrl_set);
        }
    }
    log_info("Indexed %d control sets.\n", int(control_sets.size()));
}

bool XilinxImpl::is_general_routing(WireId wire) const
{
    IdString intent = ctx->getWireType(wire);
    return !intent.in(id_INTENT_DEFAULT, id_NODE_DEDICATED, id_NODE_OPTDELAY, id_NODE_OUTPUT, id_NODE_INT_INTERFACE,
                      id_PINFEED, id_INPUT, id_PADOUTPUT, id_PADINPUT, id_IOBINPUT, id_IOBOUTPUT, id_GENERIC,
                      id_IOBIN2OUT, id_INTENT_SITE_WIRE, id_INTENT_SITE_GND);
}

void XilinxImpl::find_source_sink_locs()
{
    for (auto &net : ctx->nets) {
        NetInfo *ni = net.second.get();
        for (auto &usr : ni->users) {
            BelId bel = usr.cell->bel;
            if (bel == BelId() || is_logic_tile(bel))
                continue; // don't need to do this for logic bels, which are always next to their INT
            WireId sink = ctx->getNetinfoSinkWire(ni, usr, 0);
            if (sink == WireId() || sink_locs.count(sink))
                continue;
            std::queue<WireId> visit;
            dict<WireId, WireId> backtrace;
            int iter = 0;
            // as this is a best-effort optimisation to slightly improve routing,
            // don't spend too long with a nice low iteration limit
            const int iter_max = 500;
            visit.push(sink);
            while (!visit.empty() && iter < iter_max) {
                ++iter;
                WireId cursor = visit.front();
                visit.pop();
                if (is_general_routing(cursor)) {
                    Loc loc(0, 0, 0);
                    tile_xy(ctx->chip_info, cursor.tile, loc.x, loc.y);
                    sink_locs[sink] = loc;

                    while (backtrace.count(cursor)) {
                        cursor = backtrace.at(cursor);
                        if (!sink_locs.count(cursor)) {
                            sink_locs[cursor] = loc;
                        }
                    }

                    break;
                }
                for (auto pip : ctx->getPipsUphill(cursor)) {
                    WireId src = ctx->getPipSrcWire(pip);
                    if (!backtrace.count(src)) {
                        backtrace[src] = cursor;
                        visit.push(src);
                    }
                }
            }
        }
        auto &drv = ni->driver;
        if (drv.cell != nullptr) {
            BelId bel = drv.cell->bel;
            if (bel == BelId() || is_logic_tile(bel))
                continue; // don't need to do this for logic bels, which are always next to their INT
            WireId source = ctx->getNetinfoSourceWire(ni);
            if (source == WireId() || source_locs.count(source))
                continue;
            std::queue<WireId> visit;
            dict<WireId, WireId> backtrace;
            int iter = 0;
            // A best-effort optimisation, so bounded -- but generously: a
            // BSCAN's pins leave the CFG_CENTER tile further than 500 wires
            // of search, and a source with no exit location gets a bounding
            // box its route cannot fit, which costs the router a failed
            // bounded search per arc (seconds each, on every routing pass).
            const int iter_max = 20000;
            visit.push(source);
            while (!visit.empty() && iter < iter_max) {
                ++iter;
                WireId cursor = visit.front();
                visit.pop();
                if (is_general_routing(cursor)) {
                    Loc loc(0, 0, 0);
                    tile_xy(ctx->chip_info, cursor.tile, loc.x, loc.y);
                    source_locs[source] = loc;

                    while (backtrace.count(cursor)) {
                        cursor = backtrace.at(cursor);
                        if (!source_locs.count(cursor)) {
                            source_locs[cursor] = loc;
                        }
                    }

                    break;
                }
                for (auto pip : ctx->getPipsDownhill(cursor)) {
                    WireId dst = ctx->getPipDstWire(pip);
                    if (!backtrace.count(dst)) {
                        backtrace[dst] = cursor;
                        visit.push(dst);
                    }
                }
            }
        }
    }
}

delay_t XilinxImpl::estimateDelay(WireId src, WireId dst) const
{
    int sx, sy, dx, dy;
    tile_xy(ctx->chip_info, src.tile, sx, sy);
    tile_xy(ctx->chip_info, dst.tile, dx, dy);
    auto src_type = ctx->getWireType(src);
    auto fnd_src = source_locs.find(src);
    if (fnd_src != source_locs.end()) {
        sx = fnd_src->second.x;
        sy = fnd_src->second.y;
    } else {
        if (src_type.in(id_DOUBLE, id_BENTQUAD, id_HQUAD, id_VQUAD)) {
            for (auto pip : ctx->getPipsDownhill(src)) {
                tile_xy(ctx->chip_info, pip.tile, sx, sy);
                break;
            }
        }
    }
    auto fnd_snk = sink_locs.find(dst);
    if (fnd_snk != sink_locs.end()) {
        dx = fnd_snk->second.x;
        dy = fnd_snk->second.y;
    } else {
        auto dst_type = ctx->getWireType(dst);
        if (dst_type.in(id_DOUBLE, id_BENTQUAD, id_HQUAD, id_VQUAD)) {
            for (auto pip : ctx->getPipsUphill(dst)) {
                tile_xy(ctx->chip_info, pip.tile, dx, dy);
                break;
            }
        }
    }

    // A measured offset, when we have one, in place of the tuned formula: it
    // knows that a diagonal is one hop and that one tile and two tiles cost
    // the same hop, neither of which a separable linear formula can express.
    delay_t base;
    delay_t measured = delay_matrix_lookup(dx - sx, dy - sy);
    bool have_measured_delay = measured >= 0;
    if (have_measured_delay) {
        base = measured; // in or out of window -- extrapolated when out
    } else {
        // No measured matrix at all: the tuned formula from nextpnr-xilinx.
        int dist_x = std::abs(dx - sx), dist_y = std::abs(dy - sy);
        base = 30 * std::min(dist_x, 18) + 10 * std::max(dist_x - 18, 0) + 60 * std::min(dist_y, 6) +
               20 * std::max(dist_y - 6, 0) + 300;
        base = (base * 3) / 2; // xc7
    }
    if (fnd_snk != sink_locs.end())
        base += 1000;
    bool same_tile = dx == sx && dy == sy;
    if (src_type == id_NODE_PINFEED && same_tile)
        base -= 200;
    else if (src_type.in(id_NODE_LOCAL, id_NODE_PINBOUNCE) && same_tile)
        base -= 100;
    if (src_type == id_NODE_CLE_OUTPUT)
        base -= 80;
    return base;
}

delay_t XilinxImpl::predictDelay(BelId src_bel, IdString src_pin, BelId dst_bel, IdString dst_pin) const
{
    bool both_bels_known = src_bel != BelId() && dst_bel != BelId();
    if (!both_bels_known)
        return 0;
    delay_t clock_tree = predict_clock_tree_delay(src_bel, dst_bel);
    const bool clock_tree_applies = clock_tree >= 0;
    if (clock_tree_applies)
        return clock_tree;
    int sx, sy, dx, dy;
    tile_xy(ctx->chip_info, src_bel.tile, sx, sy);
    tile_xy(ctx->chip_info, dst_bel.tile, dx, dy);
    // A carry chain costs nothing between adjacent slices: CIN takes the
    // dedicated path from the slice below, not general routing.  From
    // upstream; orthogonal to the tuned model below, which never sees these
    // pins because a chain is placed by its own rules.
    if (dst_pin == id_CIN && src_pin == id_CO3)
        return 0;
    // Tuned predict-delay ported from nextpnr-xilinx arch.cc
    if (src_bel.tile == dst_bel.tile) {
        Loc dl = ctx->getBelLocation(src_bel), sl = ctx->getBelLocation(dst_bel);
        bool same_slice = (dl.z >> 4) == (sl.z >> 4);
        bool source_is_ff2 = (dl.z & 0xF) == BEL_FF2;
        if (same_slice)
            return 0;
        else if (source_is_ff2)
            return 700; // penalize FF2 as it makes routing harder
        else
            return 150;
    }
    delay_t measured = delay_matrix_lookup(dx - sx, dy - sy);
    bool have_measured_delay = measured >= 0;
    if (have_measured_delay)
        return measured; // in or out of window -- extrapolated when out
    int dist_x = std::abs(dx - sx), dist_y = std::abs(dy - sy);
    delay_t base = 30 * std::min(dist_x, 18) + 10 * std::max(dist_x - 18, 0) + 60 * std::min(dist_y, 6) +
                   20 * std::max(dist_y - 6, 0) + 300;
    return (base * 3) / 2; // xc7 (no measured matrix)
}

BoundingBox XilinxImpl::getRouteBoundingBox(WireId src, WireId dst) const
{
    int x0, y0, x1, y1;
    auto expand = [&](int x, int y) {
        x0 = std::min(x0, x);
        x1 = std::max(x1, x);
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
    };

    tile_xy(ctx->chip_info, src.tile, x0, y0);
    x1 = x0;
    y1 = y0;

    int dx, dy;
    tile_xy(ctx->chip_info, dst.tile, dx, dy);
    expand(dx, dy);

    auto fnd_src = source_locs.find(src);
    if (fnd_src != source_locs.end()) {
        expand(fnd_src->second.x, fnd_src->second.y);
    }
    auto fnd_snk = sink_locs.find(dst);
    if (fnd_snk != sink_locs.end()) {
        expand(fnd_snk->second.x, fnd_snk->second.y);
    }
    return {x0 - 2, y0 - 2, x1 + 2, y1 + 2};
}

namespace {
struct XilinxArch : HimbaechelArch
{
    XilinxArch() : HimbaechelArch("xilinx") {};
    bool match_device(const std::string &device) override { return device.size() > 3 && device.substr(0, 3) == "xc7"; }
    std::unique_ptr<HimbaechelAPI> create(const std::string &device) override { return std::make_unique<XilinxImpl>(); }
} xilinxArch;
} // namespace

NEXTPNR_NAMESPACE_END
