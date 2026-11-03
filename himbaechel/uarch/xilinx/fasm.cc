/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-2023  gatecat <gatecat@ds0.me>
 *  Copyright (C) 2023  Hans Baier <hansfbaier@gmail.com>
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
#include <boost/range/adaptor/reversed.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <system_error>

#include "extra_data.h"
#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "pins.h"
#include "util.h"

#include "xilinx.h"
#include "version.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace Xc7MMCM {
extern const uint16_t filter_lookup_low[];
extern const uint16_t filter_lookup_low_ss[];
extern const uint16_t filter_lookup_high[];
extern const uint16_t filter_lookup_optimized[];
extern const int64_t lk_table[];
}; // namespace Xc7MMCM

// The hand-maintained pseudo-pip table.  Out here rather than inside the fasm
// writer because XilinxImpl::is_pip_unavail consults it too: a pip in this table
// IS expressible even though the chipdb marked it PIP_CFG_NO_BITS, so the router
// must be allowed to use exactly these and no others.
void xlnx_build_pseudo_pip_config(Context *ctx, dict<PseudoPipKey, std::vector<std::string>> &pp_config)
{
    /*
     * Create the mapping from pseudo pip tile type, dest wire, and source wire, to
     * the config bits set when that pseudo pip is used
     */
    bool is_virtex7 = boost::starts_with(ctx->args.device, "xc7v");
    // Virtex-7 names the I/O column tiles LIOI/RIOI; the smaller parts name
    // them LIOI3/RIOI3.  Getting this wrong is silent: the pseudo-pip lookup
    // misses, the router still uses the pip, and the OLOGIC configuration that
    // has to accompany it is simply never emitted.  That is what left the SD
    // card's tristate path unconfigured on this board while DDR3, which sits on
    // the right-hand column and had its own virtex7-aware block below, worked.
    const std::string ioi = is_virtex7 ? "IOI" : "IOI3";
    for (std::string s : {"L", "R"})
        for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
            for (std::string i :
                 (s2 == "_SING") ? std::vector<std::string>{"", "0", "1"} : std::vector<std::string>{"0", "1"}) {
                pp_config[{ctx->id(s + ioi + s2), ctx->id(s + "IOI_OLOGIC" + i + "_OQ"),
                           ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                                  "OLOGIC_Y" + i + ".OQUSED",
                                                                  "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
                // Virtex-7 encodes the ILOGIC input path differently: the direct
                // route needs no bits at all, and neither route carries ZINV_D or
                // IDELAY_TYPE_FIXED -- segbits_lioi.db does not even define
                // IDELAY_Y*.IDELAY_TYPE_FIXED, so emitting it aborts fasm2frames.
                // The RIOI block below has always known this; the left column
                // now uses the same rule instead of its own.
                auto ioi_direct  = PseudoPipKey{ctx->id(s + ioi + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                                                ctx->id(s + "IOI_ILOGIC" + i + "_D")};
                auto ioi_delayed = PseudoPipKey{ctx->id(s + ioi + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                                                ctx->id(s + "IOI_ILOGIC" + i + "_DDLY")};
                if (is_virtex7) {
                    pp_config[ioi_direct]  = {};
                    pp_config[ioi_delayed] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0"};
                } else {
                    pp_config[ioi_direct]  = {"IDELAY_Y" + i + ".IDELAY_TYPE_FIXED",
                                              "ILOGIC_Y" + i + ".ZINV_D"};
                    pp_config[ioi_delayed] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0",
                                              "ILOGIC_Y" + i + ".ZINV_D"};
                }
                pp_config[{ctx->id(s + ioi + s2), ctx->id(s + "IOI_OLOGIC" + i + "_TQ"),
                           ctx->id("IOI_OLOGIC" + i + "_T1")}] = {"OLOGIC_Y" + i + ".ZINV_T1"};
                // The entry above only fires when the route crosses OLOGIC's
                // T1->TQ transit as a tile pip, which is how DDR3 reaches the
                // right column.  With router2 and a plain tristate buffer the
                // transit is internal to the bel and no pip is emitted, so the
                // inversion bit was never written and the pad's output enable
                // came out backwards -- the FPGA drove the SD bus exactly when
                // the card was trying to answer.  The pad-side pip below is
                // present precisely when OLOGIC drives the pad's tristate, and
                // reproduces the ZINV_T1 set that bit2fasm extracts from the
                // working Vivado bitstream, including only IOB_Y1 of the
                // TBYTESRC tile, where sdcard_clk needs no tristate at all.
                pp_config[{ctx->id(s + ioi + s2), ctx->id(s + "IOI_T" + i),
                           ctx->id(s + "IOI_OLOGIC" + i + "_TQ")}] = {"OLOGIC_Y" + i + ".ZINV_T1"};
                if (i == "0") {
                    pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_O_IN1, id_IOB_O_OUT0}] = {};
                    pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_O_OUT0, id_IOB_O0}] = {};
                    pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_T_IN1, id_IOB_T_OUT0}] = {};
                    pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_T_OUT0, id_IOB_T0}] = {};
                    pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_DIFFI_IN0, id_IOB_PADOUT1}] = {};
                    // ...and the same set for the 1.8V banks, on both columns.
                    // RIOB18 is also registered in the RIOI block below; the
                    // values are identical, so registering it here as well is
                    // harmless and keeps the two columns symmetric.
                    pp_config[{ctx->id(s + "IOB18" + s2), id_IOB_O_IN1, id_IOB_O_OUT0}] = {};
                    pp_config[{ctx->id(s + "IOB18" + s2), id_IOB_O_OUT0, id_IOB_O0}] = {};
                    pp_config[{ctx->id(s + "IOB18" + s2), id_IOB_T_IN1, id_IOB_T_OUT0}] = {};
                    pp_config[{ctx->id(s + "IOB18" + s2), id_IOB_T_OUT0, id_IOB_T0}] = {};
                    pp_config[{ctx->id(s + "IOB18" + s2), id_IOB_DIFFI_IN0, id_IOB_PADOUT1}] = {};
                }
            }

    for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
        for (std::string i : (s2 == "_SING") ? std::vector<std::string>{"0"} : std::vector<std::string>{"0", "1"}) {
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OQ"),
                       ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                              "OLOGIC_Y" + i + ".OQUSED",
                                                              "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                       ctx->id("RIOI_OLOGIC" + i + "_OQ")}] = {};
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_O" + i), ctx->id("RIOI_ODELAY" + i + "_DATAOUT")}] = {};
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                       ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                              "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
            auto rioi_direct = PseudoPipKey{ctx->id("RIOI" + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                                             ctx->id("RIOI_ILOGIC" + i + "_D")};
            auto rioi_delayed = PseudoPipKey{ctx->id("RIOI" + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                                              ctx->id("RIOI_ILOGIC" + i + "_DDLY")};
            if (is_virtex7) {
                pp_config[rioi_direct] = {};
                pp_config[rioi_delayed] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0"};
            } else {
                pp_config[rioi_direct] = {"ILOGIC_Y" + i + ".ZINV_D"};
                pp_config[rioi_delayed] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0", "ILOGIC_Y" + i + ".ZINV_D"};
            }
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_TQ"),
                       ctx->id("IOI_OLOGIC" + i + "_T1")}] = {"OLOGIC_Y" + i + ".ZINV_T1"};
            pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                       ctx->id("RIOI_ODELAY" + i + "_ODATAIN")}] = {"OLOGIC_Y" + i + ".ZINV_ODATAIN"};
            if (i == "0") {
                pp_config[{ctx->id("RIOB18" + s2), id_IOB_O_IN1, id_IOB_O_OUT0}] = {};
                pp_config[{ctx->id("RIOB18" + s2), id_IOB_O_OUT0, id_IOB_O0}] = {};
                pp_config[{ctx->id("RIOB18" + s2), id_IOB_T_IN1, id_IOB_T_OUT0}] = {};
                pp_config[{ctx->id("RIOB18" + s2), id_IOB_T_OUT0, id_IOB_T0}] = {};
                pp_config[{ctx->id("RIOB18" + s2), id_IOB_DIFFI_IN0, id_IOB_PADOUT1}] = {};
            }
        }

    for (std::string s1 : {"TOP", "BOT"}) {
        for (std::string s2 : {"L", "R"}) {
            for (int i = 0; i < 12; i++) {
                std::string ii = std::to_string(i);
                std::string hck = s2 + ii;
                std::string buf = std::string((s2 == "R") ? "X1Y" : "X0Y") + ii;
                pp_config[{ctx->id("CLK_HROW_" + s1 + "_R"), ctx->id("CLK_HROW_CK_HCLK_OUT_" + hck),
                           ctx->id("CLK_HROW_CK_MUX_OUT_" + hck)}] = {"BUFHCE.BUFHCE_" + buf + ".IN_USE",
                                                                      "BUFHCE.BUFHCE_" + buf + ".ZINV_CE"};
            }
        }

        for (int i = 0; i < 16; i++) {
            std::string ii = std::to_string(i);
            pp_config[{ctx->id("CLK_BUFG_" + s1 + "_R"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"),
                       ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I0")}] = {
                    "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE1_INVERTED",
                    "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE0", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S0"};
            pp_config[{ctx->id("CLK_BUFG_" + s1 + "_R"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"),
                       ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I1")}] = {
                    "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE0_INVERTED",
                    "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE1", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S1"};
        }
    }

    int rclk_y_to_i[4] = {2, 3, 0, 1};
    for (int y = 0; y < 4; y++) {
        std::string yy = std::to_string(y);
        std::string ii = std::to_string(rclk_y_to_i[y]);
        pp_config[{id_HCLK_IOI3, ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii)}] = {
                "BUFR_Y" + yy + ".IN_USE", "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"};
        pp_config[{id_HCLK_IOI, ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii)}] = {
                "BUFR_Y" + yy + ".IN_USE", "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"};
    }

    // FIXME: shouldn't these be in the X-RAY ppips database?
    for (char c : {'L', 'R'}) {
        for (int i = 0; i < 24; i++) {
            pp_config[{ctx->idf("INT_INTERFACE_%c", c), ctx->idf("INT_INTERFACE_LOGIC_OUTS_%c%d", c, i),
                       ctx->idf("INT_INTERFACE_LOGIC_OUTS_%c_B%d", c, i)}];
        }
    }
}

namespace {
// The FASM name of a logic-tile half.
std::string slice_site_name(int half, bool is_m)
{
    if (is_m)
        return half ? "SLICEL_X1" : "SLICEM_X0";
    else
        return half ? "SLICEL_X1" : "SLICEL_X0";
}

struct FasmBackend
{
    Context *ctx;
    XilinxImpl *uarch;
    std::ostream &out;
    std::vector<std::string> fasm_ctx;
    dict<int, std::vector<PipId>> pips_by_tile;

    // (tile_index, slot_y) pairs where a BUFGCTRL cell is actually bound.
    // Used by write_pip to suppress phantom BUFGCTRL.BUFGCTRL_X0Y*.IN_USE /
    // IS_*_INVERTED / ZINV_* bits and CLK_BUFG_*_R IMUX pip bits that the
    // router produces merely by crossing an idle BUFG tile.  Without this,
    // a single-BUFG design programs both the active CLK_BUFG_TOP_R site AND
    // a phantom one in the adjacent CLK_BUFG_BOT_R tile, contending for the
    // clock distribution backbone and leaving the FF clock dead on hardware
    // (port of nextpnr-xilinx, task #47).
    std::set<std::pair<int, int>> bufgctrl_bound_slots;

    bool device_is_virtex7() const { return boost::starts_with(ctx->args.device, "xc7v"); }

    // Is any BUFGCTRL cell bound anywhere in this tile?  This is the question
    // the tile-level phantom guard below actually means to ask.
    bool has_bound_bufgctrl(int tile) const
    {
        return std::any_of(bufgctrl_bound_slots.begin(), bufgctrl_bound_slots.end(),
                           [tile](const auto &entry) { return entry.first == tile; });
    }

    // KNOWN DEFECT, deliberately left live outside Virtex-7.
    //
    // The tile-level guard was originally written as a scan of slot over
    // [0,16) against bufgctrl_bound_slots.  That set stores
    // site_data.site_y, which is the DEVICE-GLOBAL BUFGCTRL row, not a
    // tile-local index: in openXC7/prjxray-db, CLK_BUFG_BOT_R holds
    // BUFGCTRL_X0Y0..X0Y15 and CLK_BUFG_TOP_R holds BUFGCTRL_X0Y16..X0Y31.
    // So for a BUFG placed in a CLK_BUFG_TOP_R tile the scan never matches,
    // write_pip returns early for every PIP of that tile, and the bitstream
    // programs a BUFGCTRL that has IN_USE set but neither an input mux nor
    // an output onto its GCLK spine -- while the whole downstream
    // distribution (CLK_HROW mux, CLK_BUFG_REBUF GCLK*_ENABLE, BUFHCE, leaf
    // clock) is still emitted.  Nothing about that is Virtex-7 specific; it
    // is wrong on every 7-series family.
    //
    // has_bound_bufgctrl() is applied on Virtex-7 only, because repairing it
    // everywhere changes the FASM of any design whose BUFG lands in a TOP
    // tile -- three of the five projects in .github/workflows/demos.yml --
    // and the committed goldens would have to move with it.  Re-goldening
    // those here would bury a clock-distribution fix inside an HP-bank
    // branch, where the golden diff is a side effect nobody reviews.  It
    // belongs in its own PR against the base, with the frame delta as the
    // subject.  Until then every other family keeps the old behaviour, so
    // this branch stays a no-op for them and the goldens stand as written.
    //
    // (The per-slot variant in write_pip has the same unit mismatch: it
    // parses the tile-local index out of a "BUFGCTRL.BUFGCTRL_X0Y<n>"
    // feature name -- pp_config builds those with a 0..15 loop counter, and
    // the cell-config path uses rel_site_loc() -- then looks it up against
    // the same device-global site_y.  Left alone here for the same reason.)
    bool bufgctrl_tile_guard(int tile) const
    {
        if (device_is_virtex7())
            return has_bound_bufgctrl(tile);
        for (int slot = 0; slot < 16; ++slot)
            if (bufgctrl_bound_slots.count({tile, slot}))
                return true;
        return false;
    }
    void populate_bufgctrl_bound_slots()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            bool is_placed_bufgctrl = ci->type == id_BUFGCTRL && ci->bel != BelId();
            if (is_placed_bufgctrl) {
                SiteIndex site = uarch->get_bel_site(ci->bel);
                auto rel = uarch->rel_site_loc(site);
                bufgctrl_bound_slots.insert({site.tile, rel.y});
            }
        }
    }

    dict<std::pair<int, int>, unsigned> lut_route_throughs;

    dict<IdString, pool<IdString>> invertible_pins;

    FasmBackend(Context *ctx, XilinxImpl *uarch, std::ostream &out) : ctx(ctx), uarch(uarch), out(out) {};

    void push(const std::string &x) { fasm_ctx.push_back(x); }

    void pop() { fasm_ctx.pop_back(); }

    void pop(int N)
    {
        for (int i = 0; i < N; i++)
            fasm_ctx.pop_back();
    }
    bool last_was_blank = true;
    void blank()
    {
        if (!last_was_blank)
            out << std::endl;
        last_was_blank = true;
    }

    void write_prefix()
    {
        for (auto &x : fasm_ctx)
            out << x << ".";
        last_was_blank = false;
    }

    void write_bit(const std::string &name, bool value = true)
    {
        if (value) {
            write_prefix();
            out << name << std::endl;
        }
    }

    void write_vector(const std::string &name, const std::vector<bool> &value, bool invert = false)
    {
        write_prefix();
        out << name << " = " << int(value.size()) << "'b";
        for (auto bit : boost::adaptors::reverse(value))
            out << ((bit ^ invert) ? '1' : '0');
        out << std::endl;
    }

    void write_int_vector(const std::string &name, uint64_t value, int width, bool invert = false)
    {
        std::vector<bool> bits(width, false);
        for (int i = 0; i < width; i++)
            bits[i] = (value & (1ULL << i)) != 0;
        write_vector(name, bits, invert);
    }

    // PseudoPipKey lives in xilinx.h now: XilinxImpl::is_pip_unavail needs it
    // too, so that the router rejects exactly the pips this writer cannot emit.

    dict<PseudoPipKey, std::vector<std::string>> pp_config;
    void get_pseudo_pip_data() { xlnx_build_pseudo_pip_config(ctx, pp_config); }

    void write_pip(PipId pip, NetInfo *net)
    {

        pips_by_tile[pip.tile].push_back(pip);

        auto dst_intent = ctx->getWireType(ctx->getPipDstWire(pip));
        if (dst_intent == id_PSEUDO_GND || dst_intent == id_PSEUDO_VCC)
            return;

        auto &pd = chip_pip_info(ctx->chip_info, pip);
        const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
        unsigned pip_type = pd.flags;

        if (pip_type != PIP_TILE_ROUTING && pip_type != PIP_SITE_INTERNAL)
            return;

        IdString src = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pd.src_wire].name);
        IdString dst = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pd.dst_wire].name);

        // handle certain site internal pips:
        // this is necessary, because in tristate outputs, the
        // ZINV_T1 bit needs to be set, because in the OLOGIC tiles the
        // tristate control signals are inverted if this bit is not set
        // this only applies to router1, because router2 does not generate
        // site internal pips here.
        if (pip_type == PIP_SITE_INTERNAL) {
            if (src.str(ctx) == "T1" && dst.str(ctx) == "T1INV_OUT") {
                auto srcwire_uphill_iter = ctx->getPipsUphill(ctx->getPipSrcWire(pip));
                auto uphill = srcwire_uphill_iter.begin();
                if (uphill != srcwire_uphill_iter.end()) {
                    // source wire should be like: LIOI3_X0Y73/IOI_OLOGIC1_T1
                    auto loc = ctx->getWireName(ctx->getPipSrcWire(*uphill)).str(ctx);
                    boost::replace_all(loc, "/", ".");
                    boost::erase_all(loc, "_T1");
                    boost::replace_all(loc, "IOI_OLOGIC", "OLOGIC_Y");
                    // the replacements transformed it into : LIOI3_X0Y73.OLOGIC_Y1
                    // a T1 tied low needs the site inverter active, as in the pseudo-pip rule below
                    auto dot = loc.find('.');
                    std::string tile_name = uarch->tile_name(pip.tile);
                    std::string site_y = dot == std::string::npos ? "" : loc.substr(dot + 1);
                    // write_iol_config names the top half of a SING tile Y1.
                    const bool top_sing_tile =
                            boost::contains(tile_name, "_SING_") && pip.tile < uarch->hclk_for_ioi(pip.tile);
                    if (top_sing_tile)
                        boost::replace_all(site_y, "Y0", "Y1");
                    const bool site_has_t1_tied_low = ologic_t1_low.count(tile_name + "/" + site_y) != 0;
                    if (!site_has_t1_tied_low)
                        out << loc << "." << "ZINV_T1" << std::endl;
                }
            }
            return;
        }

        // handle tile routing pips
        IdString tile_type = IdString(chip_tile_info(ctx->chip_info, pip.tile).type_name);
        PseudoPipKey ppk{tile_type, dst, src};

        if (pp_config.count(ppk)) {
            auto &pp = pp_config.at(ppk);
            std::string tile_name = uarch->tile_name(pip.tile);
            // Phantom-BUFGCTRL guard (pseudo-pip variant): if the router
            // crosses a CLK_BUFG_*_R tile that has NO bound BUFGCTRL, every
            // pseudo-pip feature for that tile is a phantom (the chipdb's
            // clock-distribution graph allows the path, but Vivado doesn't
            // actually use it).  Drop the whole emission -- not just the
            // BUFGCTRL.* config bits but also the CLK_BUFG_BUFGCTRL*_I0/I1
            // IMUX features that program a phantom clock-input mux at the
            // empty BUFG site, contending with the real one in the active
            // tile.  (Port of nextpnr-xilinx, task #47.)
            bool tile_is_clk_bufg_r =
                    (boost::starts_with(tile_name, "CLK_BUFG_TOP_R") || boost::starts_with(tile_name, "CLK_BUFG_BOT_R"));
            bool bufg_tile_has_no_bound_bufgctrl = tile_is_clk_bufg_r && !bufgctrl_tile_guard(pip.tile);
            if (bufg_tile_has_no_bound_bufgctrl)
                return;
            for (auto c : pp) {
                // The OMUX bypass and a bound output register are mutually
                // exclusive: OMUX.D1 routes the pad from the combinational D1
                // input, so asserting it on a site holding an ODDR drives the
                // pad from the unregistered signal and throws away the fixed
                // clock-to-out that putting a register at the pad buys.  The
                // cell writer has already emitted OQUSED and the DDR rate for
                // that site; only the mux selection has to be withheld.
                // write_io runs before write_routing, so the set is populated.
                if (boost::contains(c, ".OMUX.")) {
                    auto dot = c.find('.');
                    bool site_has_output_register =
                            dot != std::string::npos && ologic_registered.count(tile_name + "/" + c.substr(0, dot));
                    if (site_has_output_register)
                        continue;
                }
                if (boost::starts_with(tile_name, "RIOI3_SING") || boost::starts_with(tile_name, "LIOI3_SING") ||
                    boost::starts_with(tile_name, "RIOI_SING")) {
                    // Need to flip for top HCLK
                    bool is_top_sing = pip.tile < uarch->hclk_for_ioi(pip.tile);
                    if (is_top_sing) {
                        auto y0pos = c.find("Y0");
                        if (y0pos != std::string::npos)
                            c.replace(y0pos, 2, "Y1");
                    }
                }
                if (boost::contains(c, ".DATA_RATE_TQ.BUF")) {
                    auto dot = c.find('.');
                    bool site_has_tristate_register =
                            dot != std::string::npos && ologic_t_registered.count(tile_name + "/" + c.substr(0, dot));
                    if (site_has_tristate_register)
                        continue;
                }
                const bool is_zinv_t1 = boost::ends_with(c, ".ZINV_T1");
                if (is_zinv_t1) {
                    auto dot = c.find('.');
                    const bool site_has_t1_tied_low =
                            dot != std::string::npos && ologic_t1_low.count(tile_name + "/" + c.substr(0, dot));
                    if (site_has_t1_tied_low)
                        continue;
                }
                // Phantom-BUFGCTRL guard (per-slot variant): suppress
                // BUFGCTRL.BUFGCTRL_X0Y<n>.* features on (tile, slot) pairs
                // that have no actually-bound BUFGCTRL cell.
                if (boost::starts_with(c, "BUFGCTRL.BUFGCTRL_X0Y")) {
                    const std::string prefix = "BUFGCTRL.BUFGCTRL_X0Y";
                    size_t start = prefix.size();
                    size_t end = c.find('.', start);
                    if (end == std::string::npos)
                        end = c.size();
                    int slot = -1;
                    try {
                        slot = std::stoi(c.substr(start, end - start));
                    } catch (...) {
                        slot = -1;
                    }
                    bool slot_has_no_bound_bufgctrl = slot >= 0 && !bufgctrl_bound_slots.count({pip.tile, slot});
                    if (slot_has_no_bound_bufgctrl)
                        continue;
                }
                out << tile_name << "." << c << std::endl;
            }
            if (!pp.empty())
                last_was_blank = false;
        } else {
            if (extra_data.pip_config == 1)
                log_warning("Unprocessed route-thru %s.%s.%s\n!", tile_type.c_str(ctx), src.c_str(ctx), dst.c_str(ctx));

            std::string tile_name = uarch->tile_name(pip.tile);
            std::string dst_name = dst.str(ctx);
            std::string src_name = src.str(ctx);

            // Phantom-BUFGCTRL guard (regular-pip variant): the pp_config
            // branch already filters pseudo-pip emissions at CLK_BUFG_*_R
            // tiles that hold no bound BUFGCTRL; this handles the regular
            // PIPs the router crossed through the same tiles
            // (CLK_BUFG_BUFGCTRL*_I0/I1 IMUX hops, CLK_BUFG_CK_GCLK* output
            // PIPs).  Both classes program the unused-slot BUFGCTRL site;
            // the cell-config + routing pair together is what kills the
            // clock distribution on hardware.  (Port of nextpnr-xilinx.)
            bool tile_is_clk_bufg_r =
                    boost::starts_with(tile_name, "CLK_BUFG_TOP_R") || boost::starts_with(tile_name, "CLK_BUFG_BOT_R");
            bool bufg_tile_has_no_bound_bufgctrl = tile_is_clk_bufg_r && !bufgctrl_tile_guard(pip.tile);
            if (bufg_tile_has_no_bound_bufgctrl)
                return;

            if (boost::starts_with(tile_name, "DSP_L") || boost::starts_with(tile_name, "DSP_R")) {
                // FIXME: PPIPs missing for DSPs
                return;
            }
            std::string orig_dst_name = dst_name;
            if (boost::starts_with(tile_name, "RIOI3_SING") || boost::starts_with(tile_name, "LIOI3_SING") ||
                boost::starts_with(tile_name, "RIOI_SING")) {
                // FIXME: PPIPs missing for SING IOI3s
                if ((boost::contains(src_name, "IMUX") || boost::contains(src_name, "CTRL0")) &&
                    !boost::contains(dst_name, "CLK"))
                    return;
                auto spos = src_name.find("_SING_");
                if (spos != std::string::npos)
                    src_name.erase(spos, 5);
                // Need to flip for top HCLK
                // TODO

                bool is_top_sing = pip.tile < uarch->hclk_for_ioi(pip.tile);
                if (is_top_sing) {
                    auto us0pos = dst_name.find("_0");
                    if (us0pos != std::string::npos)
                        dst_name.replace(us0pos, 2, "_1");
                    auto ol0pos = dst_name.find("OLOGIC0");
                    if (ol0pos != std::string::npos) {
                        dst_name.replace(ol0pos, 7, "OLOGIC1");
                        us0pos = src_name.find("_0");
                        if (us0pos != std::string::npos)
                            src_name.replace(us0pos, 2, "_1");
                    }
                }
            }
            if (boost::contains(tile_name, "IOI")) {
                if (boost::contains(dst_name, "OCLKB") && boost::contains(src_name, "IOI_OCLKM_"))
                    return; // missing, not sure if really a ppip?
            }

            out << tile_name << ".";
            out << dst_name << ".";
            out << src_name << std::endl;

            if (boost::contains(tile_name, "IOI") && boost::starts_with(dst_name, "IOI_OCLK_")) {
                dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');
                orig_dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');

                WireId w = uarch->lookup_wire(pip.tile, ctx->id(orig_dst_name));

                NPNR_ASSERT(w != WireId());
                if (ctx->getBoundWireNet(w) == nullptr) {
                    out << tile_name << ".";
                    out << dst_name << ".";
                    out << src_name << std::endl;
                }
            }

            last_was_blank = false;
        }
    };

    // Get the set of input signals for a LUT-type cell
    std::vector<IdString> get_inputs(CellInfo *cell)
    {
        IdString type = ctx->id(str_or_default(cell->attrs, id_X_ORIG_TYPE, ""));
        const bool is_srl16_inputs = type == id_SRL16E || type.str(ctx) == "SRLC16E";
        if (is_srl16_inputs)
            return {id_A0, id_A1, id_A2, id_A3};
        if (type == id_LUT1)
            return {id_I0};
        else if (type == id_LUT2)
            return {id_I0, id_I1};
        else if (type == id_LUT3)
            return {id_I0, id_I1, id_I2};
        else if (type == id_LUT4)
            return {id_I0, id_I1, id_I2, id_I3};
        else if (type == id_LUT5)
            return {id_I0, id_I1, id_I2, id_I3, id_I4};
        else if (type == id_LUT6)
            return {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
        else if (type == id_RAMD64E)
            return {id_RADR0, id_RADR1, id_RADR2, id_RADR3, id_RADR4, id_RADR5};
        else if (type == id_SRLC32E)
            return {ctx->id("A[0]"), ctx->id("A[1]"), ctx->id("A[2]"), ctx->id("A[3]"), ctx->id("A[4]")};
        else if (type == id_RAMD32)
            return {id_RADR0, id_RADR1, id_RADR2, id_RADR3, id_RADR4};
        else
            NPNR_ASSERT_FALSE("unsupported LUT-type cell");
    }

    // Process LUT initialisation
    std::vector<bool> get_lut_init(CellInfo *lut6, CellInfo *lut5)
    {
        std::vector<bool> bits(64, false);

        std::vector<IdString> phys_inputs;
        for (int i = 1; i <= 6; i++)
            phys_inputs.push_back(ctx->id("A" + std::to_string(i)));

        for (int i = 0; i < 2; i++) {
            CellInfo *lut = (i == 1) ? lut5 : lut6;
            if (lut == nullptr)
                continue;
            auto lut_inputs = get_inputs(lut);
            dict<int, std::vector<std::string>> phys_to_log;
            dict<std::string, int> log_to_bit;
            for (int j = 0; j < int(lut_inputs.size()); j++)
                log_to_bit[lut_inputs[j].str(ctx)] = j;
            for (int j = 0; j < 6; j++) {
                // Get the LUT physical to logical mapping
                phys_to_log[j];
                if (!lut->attrs.count(ctx->idf("X_ORIG_PORT_%s", phys_inputs[j].c_str(ctx))))
                    continue;
                std::string orig = lut->attrs.at(ctx->idf("X_ORIG_PORT_%s", phys_inputs[j].c_str(ctx))).as_string();
                boost::split(phys_to_log[j], orig, boost::is_any_of(" "));
            }
            int lbound = 0, ubound = 64;
            // Fracturable LUTs
            if (lut5 && lut6) {
                lbound = (i == 1) ? 0 : 32;
                ubound = (i == 1) ? 32 : 64;
            }
            // SRL16E / SRLC32E: pack_srls() keeps the shift register's own
            // 16-/32-bit INIT on the SLICE_LUTX cell it creates, but that cell
            // has no logical LUT inputs to map through X_ORIG_PORT_A1..A6, so
            // the phys_to_log walk below would leave every bit equal to
            // init.str.at(0).  Each SRL INIT bit k is stored in the LUT INIT at
            // both bit 2k and 2k+1 (nextpnr-xilinx#181).  For SRL16E and SRLC16E,
            // pack_srls() ties A6 (LUT address bit 5) to VCC when
            // the cell sits in the 6LUT position (i==0), so only addresses
            // [32:64) are ever read -- the fracturable narrowing above selects
            // that half only when lut5 AND lut6 are BOTH non-null, an unrelated
            // condition, so override it by position.  SRLC32E ties only A1 and
            // uses the full 64-bit space regardless of position.
            std::string orig_type = str_or_default(lut->attrs, id_X_ORIG_TYPE, "");
            const bool lut_is_srl = (orig_type == "SRL16E") || (orig_type == "SRLC16E") || (orig_type == "SRLC32E");
            if (lut_is_srl) {
                const bool is_srl16e = orig_type == "SRL16E" || orig_type == "SRLC16E";
                if (is_srl16e) {
                    lbound = (i == 1) ? 0 : 32;
                    ubound = (i == 1) ? 32 : 64;
                }
                int width = (ubound - lbound) / 2;
                Property srl_init = get_or_default(lut->params, id_INIT, Property()).extract(0, width);
                for (int k = 0; k < width; k++) {
                    bool bit = (srl_init.str.at(k) == Property::S1);
                    bits[lbound + 2 * k] = bit;
                    bits[lbound + 2 * k + 1] = bit;
                }
                continue;
            }
            Property init = get_or_default(lut->params, id_INIT, Property()).extract(0, 64);
            for (int j = lbound; j < ubound; j++) {
                int log_index = 0;
                for (int k = 0; k < 6; k++) {
                    if ((j & (1 << k)) == 0)
                        continue;
                    for (auto &p2l : phys_to_log[k]) {
                        // find(), never operator[]: operator[] inserts a
                        // missing key with value 0, so an unparseable name was
                        // silently encoded as if the pin drove I0 -- the LUT
                        // then received a truth table that is not its function,
                        // in the bitstream only.  (Port of nextpnr-xilinx
                        // 7cfd1e90.)
                        auto lb = log_to_bit.find(p2l);
                        bool name_not_a_logical_input = lb == log_to_bit.end();
                        if (name_not_a_logical_input)
                            log_error("LUT '%s': X_ORIG_PORT_%s names logical input '%s', "
                                      "which this cell does not have\n",
                                      ctx->nameOf(lut), phys_inputs[k].c_str(ctx), p2l.c_str());
                        log_index |= (1 << lb->second);
                    }
                }
                bits[j] = (init.str.at(log_index) == Property::S1);
            }
        }
        return bits;
    };

    // Return the name for a half-logic-tile
    std::string get_half_name(int half, bool is_m) { return slice_site_name(half, is_m); }

    std::string get_bel_name(BelId bel) { return uarch->bel_name_in_site(bel).str(ctx); }

    void write_routing_bel(WireId dst_wire)
    {
        for (auto pip : ctx->getPipsUphill(dst_wire)) {
            if (ctx->getBoundPipNet(pip) != nullptr) {
                auto &pd = chip_pip_info(ctx->chip_info, pip);
                const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
                std::string belname = IdString(extra_data.bel_name).str(ctx);
                std::string pinname = IdString(extra_data.pip_config).str(ctx);
                bool skip_pinname = false;
                // Ignore modes with no associated bit (X-ray omission??)
                if (belname == "WEMUX" && pinname == "WE")
                    continue;
                // The all-zero state of DOUTMUX is the MC31 leg. A bitstream
                // that drives DMUX from an A6LUT MC31 carries no DOUTMUX
                // feature, and the database row for that leg also sets the
                // bit shared with A5FFMUX.IN_B.
                const bool doutmux_mc31_is_the_default = belname == "DOUTMUX" && pinname == "MC31";
                if (doutmux_mc31_is_the_default)
                    continue;

                if (belname.substr(1) == "DI1MUX") {
                    // prjxray names the non-default leg of these muxes after
                    // BOTH signals sharing it -- the LUTRAM write-data
                    // broadcast and the SRL MC31 cascade run through one
                    // config bit: ADI1MUX -> BDI1_BMC31, BDI1MUX -> DI_CMC31,
                    // CDI1MUX -> DI_DMC31 (there is no DDI1MUX).  The
                    // default own-letter leg (AI/BI/CI) keeps its plain name.
                    // Emitting the bare site-wire name (e.g. BMC31) makes
                    // fasm2frames reject the feature.  (Port of
                    // nextpnr-xilinx fasm.cc.)
                    bool uses_cascade_leg = pinname != std::string(1, belname[0]) + "I";
                    if (uses_cascade_leg) {
                        switch (belname[0]) {
                        case 'A':
                            pinname = "BDI1_BMC31";
                            break;
                        case 'B':
                            pinname = "DI_CMC31";
                            break;
                        case 'C':
                            pinname = "DI_DMC31";
                            break;
                        default:
                            break;
                        }
                    }
                    belname = "DI1MUX";
                }

                if (belname.substr(1) == "CY0") {
                    if (pinname.substr(1) == "5")
                        skip_pinname = true;
                    else
                        continue;
                }

                write_prefix();
                out << belname;
                if (!skip_pinname)
                    out << "." << pinname;
                out << std::endl;
            }
        }
    }

    // Process flipflops in a half-tile
    void write_ffs_config(int tile, int half)
    {
        bool found_ff = false;
        bool negedge_ff = false;
        bool is_latch = false;
        bool is_sync = false;
        bool is_clkinv = false;
        bool is_srused = false;
        bool is_ceused = false;

#define SET_CHECK(dst, src)                                                                                            \
    do {                                                                                                               \
        if (found_ff)                                                                                                  \
            NPNR_ASSERT(dst == (src));                                                                                 \
        else                                                                                                           \
            dst = (src);                                                                                               \
    } while (0)

        std::string tname = uarch->tile_name(tile);

        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;

        push(tname);
        push(get_half_name(half, boost::contains(tname, "CLBLM")));

        for (int i = 0; i < 4; i++) {
            CellInfo *ff1 = lts->cells[(half << 6) | (i << 4) | BEL_FF];
            CellInfo *ff2 = lts->cells[(half << 6) | (i << 4) | BEL_FF2];
            for (int j = 0; j < 2; j++) {
                CellInfo *ff = (j == 1) ? ff2 : ff1;
                if (ff == nullptr)
                    continue;
                push(get_bel_name(ff->bel));
                bool zrst = false, zinit = false;
                IdString srsig;
                std::string type = str_or_default(ff->attrs, id_X_ORIG_TYPE, "");
                // Vivado write_verilog omits parameters at their primitive
                // default, so a bare FDSE/FDPE arrives with NO INIT param -- and
                // their primitive default is INIT=1 (FDRE/FDCE default 0).  A
                // hardcoded 0 default set ZINI (init=0) on every set/preset FF,
                // breaking reset synchronizers and INIT=1 startup FSMs at
                // configuration.  A present-but-undefined INIT ('x') has to be
                // treated as absent as well: the netlists the regression cases
                // are built from carry it, and int_or_default() resolves an 'x'
                // bit to 0, which is the same bug by another route.
                const bool type_defaults_init_high =
                        (type == "FDSE" || type == "FDSE_1" || type == "FDPE" || type == "FDPE_1");
                const int default_init = type_defaults_init_high ? 1 : 0;
                auto init_it = ff->params.find(id_INIT);
                const bool init_is_defined = (init_it != ff->params.end()) && init_it->second.is_fully_def();
                const int init_val = init_is_defined ? int_or_default(ff->params, id_INIT, default_init) : default_init;
                zinit = (init_val != 1);
                if (type == "FDRE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDRE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDCE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDCE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else {
                    log_error("unsupported FF type: '%s'\n", type.c_str());
                }

                write_bit("ZINI", zinit);
                write_bit("ZRST", zrst);

                pop();
                if (negedge_ff)
                    SET_CHECK(is_clkinv, true);
                else
                    SET_CHECK(is_clkinv, int_or_default(ff->params, id_IS_C_INVERTED) == 1);

                NetInfo *sr = ff->getPort(id_SR), *ce = ff->getPort(id_CE);

                SET_CHECK(is_srused, sr != nullptr && sr->name != ctx->id("$PACKER_GND_NET"));
                SET_CHECK(is_ceused, ce != nullptr && ce->name != ctx->id("$PACKER_VCC_NET"));

                // Input mux
                write_routing_bel(ctx->getBelPinWire(ff->bel, id_D));

                found_ff = true;
            }
        }
        // A LUT-RAM or a shift register shares the half-slice clock inverter
        // with the flipflops: on 7-series the CLKINV/NOCLKINV bit inverts the
        // clock of every clocked element of the slice, so a distributed RAM
        // whose write clock is inverted (the RAM*X1S_1 Unisim variants, or
        // IS_WCLK_INVERTED on an imported netlist) or a shift register with
        // IS_CLK_INVERTED (what yosys writes for a negedge SRL16E/SRLC32E; the
        // packer renames it to IS_WCLK_INVERTED) is expressed by that same
        // bit.  The packer carries the parameter this far; ignoring it here
        // produced a FASM byte-identical to the non-inverted design, i.e. a
        // memory written on the wrong clock edge with a clean exit status.
        bool found_mem = false, mem_clkinv = false;
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 2; k++) {
                CellInfo *lut = lts->cells[(half << 6) | (i << 4) | (k ? BEL_5LUT : BEL_6LUT)];
                const bool lut_is_memory_or_srl =
                        lut != nullptr && (lut->attrs.count(id_X_LUT_AS_DRAM) || lut->attrs.count(id_X_LUT_AS_SRL));
                if (!lut_is_memory_or_srl)
                    continue;
                const bool lut_clkinv = bool_or_default(lut->params, id_IS_WCLK_INVERTED, false);
                const bool mem_disagrees = found_mem && (lut_clkinv != mem_clkinv);
                if (mem_disagrees) {
                    const std::string bel_name = ctx->getBelName(lut->bel).str(ctx);
                    log_error("FASM: LUT-RAM/SRL '%s' (type %s) at bel %s disagrees with its half-slice on "
                              "'IS_WCLK_INVERTED' (tile %s) -- control-set contention in the placement\n",
                              lut->name.c_str(ctx), lut->type.c_str(ctx), bel_name.c_str(),
                              tname.c_str());
                }
                mem_clkinv = lut_clkinv;
                found_mem = true;
            }
        }
        if (found_mem) {
            const bool mem_and_ff_disagree = found_ff && (mem_clkinv != is_clkinv);
            if (mem_and_ff_disagree)
                log_error("FASM: LUT-RAM in tile %s needs clock inversion %d, but the flipflops in the same "
                          "half-slice need %d -- control-set contention in the placement\n",
                          tname.c_str(), int(mem_clkinv), int(is_clkinv));
            is_clkinv = mem_clkinv;
        }
        write_bit("LATCH", is_latch);
        write_bit("FFSYNC", is_sync);
        write_bit("CLKINV", is_clkinv);
        write_bit("NOCLKINV", !is_clkinv);
        write_bit("SRUSEDMUX", is_srused);
        write_bit("CEUSEDMUX", is_ceused);
        pop(2);
    }

    // Get a named wire in the same site as a bel
    WireId get_site_wire(BelId site_bel, std::string name)
    {
        IdStringList bel_name = ctx->getBelName(site_bel);
        NPNR_ASSERT(bel_name.size() == 2);
        IdString tile_name = bel_name[0];
        const std::string &bel_name_str = bel_name[1].str(ctx);
        size_t sep_pos = bel_name_str.find('.');
        NPNR_ASSERT(sep_pos != std::string::npos);
        std::string site_name = bel_name_str.substr(0, sep_pos);
        IdString wire_name = ctx->idf("%s.%s", site_name.c_str(), name.c_str());
        WireId wire = ctx->getWireByName(IdStringList::concat(tile_name, wire_name));
        NPNR_ASSERT(wire != WireId());
        return wire;
    }

    // Process LUTs and associated functionality in a half
    void write_luts_config(int tile, int half)
    {
        bool wa7_used = false, wa8_used = false;

        std::string tname = uarch->tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");
        bool is_slicem = is_mtile && (half == 0);

        const auto &lts = uarch->tile_status.at(tile).lts;

        push(tname);
        push(get_half_name(half, is_mtile));

        // Write route through pips
        for (int i = 0; i < 4; i++) {
            auto found_rt = lut_route_throughs.find(std::make_pair(tile, half * 4 + i));
            if (found_rt != lut_route_throughs.end()) {
                std::string lutname = stringf("%cLUT", "ABCD"[i]);
                push(lutname);
                std::vector<bool> rt_init(64, false);
                for (unsigned b = 0; b < 64; b++) {
                    if (b & (1U << found_rt->second))
                        rt_init[b] = true;
                }
                write_vector("INIT[63:0]", rt_init);
                pop();
            }
        }
        if (lts) {
            // Write logic
            BelId bel_in_half =
                    ctx->getBelByLocation(Loc(tile % ctx->chip_info->width, tile / ctx->chip_info->width, half << 6));

            for (int i = 0; i < 4; i++) {
                CellInfo *lut6 = lts->cells[(half << 6) | (i << 4) | BEL_6LUT];
                CellInfo *lut5 = lts->cells[(half << 6) | (i << 4) | BEL_5LUT];
                // Write LUT initialisation
                if (lut6 != nullptr || lut5 != nullptr) {
                    std::string lutname = stringf("%cLUT", "ABCD"[i]);
                    // A route-through on the same position would be a second
                    // INIT for one LUT; the assembler ORs them and both the
                    // cell and the routed net come out wrong.
                    if (lut_route_throughs.count(std::make_pair(tile, half * 4 + i)))
                        log_error("%s.%s.%s holds cell '%s' and a route-through: two INITs for one LUT.\n",
                                  tname.c_str(), get_half_name(half, is_mtile).c_str(), lutname.c_str(),
                                  ctx->nameOf(lut6 != nullptr ? lut6 : lut5));
                    push(lutname);
                    write_vector("INIT[63:0]", get_lut_init(lut6, lut5));

                    // Write LUT mode config
                    bool is_small = false, is_ram = false, is_srl = false;
                    for (int j = 0; j < 2; j++) {
                        CellInfo *lut = (j == 1) ? lut5 : lut6;
                        if (lut == nullptr)
                            continue;
                        std::string type = str_or_default(lut->attrs, id_X_ORIG_TYPE);
                        const bool is_wide_ram = type == "RAMD64E" || type == "RAMS64E";
                        if (is_wide_ram) {
                            is_ram = true;
                        }
                        const bool is_small_ram = type == "RAMD32" || type == "RAMS32";
                        if (is_small_ram) {
                            is_ram = true;
                            is_small = true;
                        }
                        const bool is_srl16 = type == "SRL16E" || type == "SRLC16E";
                        if (is_srl16) {
                            is_srl = true;
                            is_small = true;
                        }
                        const bool is_srl32 = type == "SRLC32E";
                        if (is_srl32) {
                            is_srl = true;
                        }
                        wa7_used |= (lut->getPort(id_WA7) != nullptr);
                        wa8_used |= (lut->getPort(id_WA8) != nullptr);
                    }
                    if (is_slicem && i != 3) {
                        write_routing_bel(get_site_wire(bel_in_half, stringf("%cDI1MUX_OUT", "ABCD"[i])));
                    }
                    write_bit("SMALL", is_small);
                    write_bit("RAM", is_ram);
                    write_bit("SRL", is_srl);
                    pop();
                }
                write_routing_bel(get_site_wire(bel_in_half, stringf("%cMUX", "ABCD"[i])));
            }
            write_bit("WA7USED", wa7_used);
            write_bit("WA8USED", wa8_used);
            if (is_slicem)
                write_routing_bel(get_site_wire(bel_in_half, "WEMUX_OUT"));
        }

        pop(2);
    }

    void write_carry_config(int tile, int half)
    {
        std::string tname = uarch->tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");

        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;

        CellInfo *carry = lts->cells[half << 6 | BEL_CARRY4];
        if (carry == nullptr)
            return;

        push(tname);
        push(get_half_name(half, is_mtile));

        write_routing_bel(get_site_wire(carry->bel, "PRECYINIT_OUT"));
        if (carry->getPort(id_CIN) != nullptr)
            write_bit("PRECYINIT.CIN");
        push("CARRY4");
        for (char c : {'A', 'B', 'C', 'D'})
            write_routing_bel(get_site_wire(carry->bel, stringf("%cCY0_OUT", c)));
        pop(3);
    }

    void write_logic()
    {
        std::set<int> used_logic_tiles;
        for (auto &cell : ctx->cells) {
            if (uarch->is_logic_tile(cell.second->bel))
                used_logic_tiles.insert(cell.second->bel.tile);
        }
        for (auto &net : ctx->nets) {
            for (const auto &wire_pair : net.second->wires) {
                PipId pip = wire_pair.second.pip;
                if (pip == PipId())
                    continue;
                const auto &pip_data = chip_pip_info(ctx->chip_info, pip);
                const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pip_data.extra_data.get());
                unsigned pip_type = pip_data.flags;
                if (pip_type != PIP_LUT_ROUTETHRU)
                    continue;
                unsigned lut_idx = (extra_data.pip_config >> 8);
                unsigned lut_input = (extra_data.pip_config >> 1) & 0x7;
                lut_route_throughs[std::make_pair(pip.tile, lut_idx)] = lut_input;
                used_logic_tiles.insert(pip.tile);
            }
        }
        for (int tile : used_logic_tiles) {
            write_luts_config(tile, 0);
            write_luts_config(tile, 1);
            write_ffs_config(tile, 0);
            write_ffs_config(tile, 1);
            write_carry_config(tile, 0);
            write_carry_config(tile, 1);
            blank();
        }
    }

    void write_routing()
    {
        get_pseudo_pip_data();
        for (auto &net : ctx->nets) {
            NetInfo *ni = net.second.get();
            out << stringf("# routing for net %s", ni->name.c_str(ctx)) << std::endl;
            for (auto &w : ni->wires) {
                if (w.second.pip != PipId())
                    write_pip(w.second.pip, ni);
            }
            blank();
        }
    }

    struct BankIoConfig
    {
        bool stepdown = false;
        bool vref = false;
        bool tmds_33 = false;
        bool lvds_25 = false;
        bool only_diff = false;
    };

    dict<int, BankIoConfig> ioconfig_by_hclk;

    bool warned_dci = false;

    void write_io_config(CellInfo *pad)
    {
        NetInfo *pad_net = pad->getPort(id_PAD);
        NPNR_ASSERT(pad_net != nullptr);
        std::string iostandard = str_or_default(pad->attrs, id_IOSTANDARD, "LVCMOS33");
        std::string pulltype = str_or_default(pad->attrs, id_PULLTYPE, "NONE");
        std::string slew = str_or_default(pad->attrs, id_SLEW, "SLOW");

        Loc ioLoc = uarch->rel_site_loc(uarch->get_bel_site(pad->bel));

        // Several features below describe the *other* half of this tile, which
        // only exists to be described when nothing is bound to it.  A tile
        // whose two halves are both driven -- the VC707's LED pairs, say --
        // must not have either half pulled down.
        auto partner_pad_unused = [&]() {
            Loc bl = ctx->getBelLocation(pad->bel);
            for (auto other : ctx->getBelsByTile(bl.x, bl.y)) {
                if (other == pad->bel)
                    continue;
                CellInfo *oc = ctx->getBoundBelCell(other);
                bool partner_holds_pad = oc != nullptr && oc->type == id_PAD;
                if (partner_holds_pad)
                    return false;
            }
            return true;
        };
        // Whether the tile's *other* half is also a single-ended input.  The
        // database gives IOB_Y0 and IOB_Y1 the same bit for their low-voltage
        // LVCMOS .IN feature, with opposite polarity, so describing both halves
        // makes the second clear what the first set and fasm2frames rejects the
        // pair.  Vivado emits no .IN for such a tile at all.
        auto partner_pad_is_input = [&]() {
            Loc bl = ctx->getBelLocation(pad->bel);
            for (auto other : ctx->getBelsByTile(bl.x, bl.y)) {
                if (other == pad->bel)
                    continue;
                CellInfo *oc = ctx->getBoundBelCell(other);
                bool partner_holds_pad = oc != nullptr && oc->type == id_PAD;
                if (!partner_holds_pad)
                    continue;
                NetInfo *on = oc->getPort(id_PAD);
                if (on == nullptr)
                    continue;
                // Note: a driver does not disqualify it.  A bidirectional pad
                // -- an SD card's cmd and data lines, say -- both drives and
                // receives, and it is the receiving that collides here.
                for (auto &u : on->users)
                    if (boost::contains(u.cell->type.str(ctx), "INBUF"))
                        return true;
            }
            return false;
        };
        // Whether a cell of the OLOGIC beside this pad -- an ODDR on its data or
        // on its tristate, or an OSERDESE2 -- drives the pad's output buffer.
        auto pad_driven_by_ologic_cell = [&]() {
            CellInfo *obuf = pad_net->driver.cell;
            for (auto &port : obuf->ports) {
                NetInfo *net = port.second.net;
                const bool is_driven_input =
                        port.second.type == PORT_IN && net != nullptr && net->driver.cell != nullptr;
                if (!is_driven_input)
                    continue;
                const bool driver_is_ologic_cell =
                        net->driver.cell->type.in(id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF, id_OLOGICE2_TFF,
                                                  id_OLOGICE3_TFF, id_OSERDESE2_OSERDESE2);
                if (driver_is_ologic_cell)
                    return true;
            }
            return false;
        };
        bool is_output = false, is_input = false;
        if (pad_net->driver.cell != nullptr)
            is_output = true;
        for (auto &usr : pad_net->users)
            if (boost::contains(usr.cell->type.str(ctx), "INBUF"))
                is_input = true;
        std::string tile = uarch->tile_name(pad->bel.tile);
        push(tile);

        if (boost::ends_with(iostandard, "_T_DCI")) {
            if (!warned_dci)
                log_warning("DCI is not supported, will be removed.\n");
            warned_dci = true;
            iostandard.erase(iostandard.size() - 6, iostandard.size());
        }

        bool is_riob18 = boost::starts_with(tile, "RIOB18_");
        bool is_liob18 = boost::starts_with(tile, "LIOB18_");
        bool is_hp_bank = is_riob18 || is_liob18;
        bool is_sing = boost::contains(tile, "_SING_");
        bool is_top_sing = pad->bel.tile < uarch->hclk_for_iob(pad->bel);
        bool is_stepdown = false;
        bool is_lvcmos = boost::starts_with(iostandard, "LVCMOS");
        bool is_low_volt_lvcmos = iostandard == "LVCMOS12" || iostandard == "LVCMOS15" || iostandard == "LVCMOS18";

        // Within a non-SING I/O tile the two pads sit in numeric site order --
        // the lower-numbered site is the N half of the differential pair, the
        // upper is the P half -- but prjxray numbers the halves the other way
        // round: IOB_Y0 is the P/master half.  So the FASM index is the
        // complement of the relative site index, on every family.  Virtex-7
        // was excepted here once; a golden Vivado build of examples/
        // vc707-telegraph disproves the exception.  On that part AV40
        // (IO_L13P, the upper site) is IOB_Y0 in Vivado's bitstream and AU36
        // (IO_L8N, the lower site) is IOB_Y1, and the unused halves carry
        // PULLTYPE.PULLDOWN, which says unambiguously which half is driven.
        // With the exception in place every VC707 signal was configured onto
        // the other half of its pair: harmless for the LEDs, which occupy both
        // halves of their tiles, and for a reset that read a pulled-down pin,
        // but it left the UART driving AT36 instead of AU36 and so silent.
        // The guards below (yLoc == 0 / the hard-coded IOB_Y1 partner halves)
        // were all written against this convention and only fire under it.
        auto yLoc = is_sing ? (is_top_sing ? 1 : 0) : (1 - ioLoc.y);
        push("IOB_Y" + std::to_string(yLoc));
        bool is_master_half = yLoc == 0;

        bool has_diff_prefix = boost::starts_with(iostandard, "DIFF_");
        bool is_tmds33 = iostandard == "TMDS_33";
        bool is_lvds25 = iostandard == "LVDS_25";
        bool is_lvds = boost::starts_with(iostandard, "LVDS");
        bool only_diff = is_tmds33 || is_lvds25;
        bool is_diff = is_tmds33 || is_lvds || has_diff_prefix;
        if (has_diff_prefix)
            iostandard.erase(0, 5);
        bool is_sstl = iostandard == "SSTL12" || iostandard == "SSTL135" || iostandard == "SSTL15";

        bool is_right_hp_diff_input_master = is_riob18 && is_input && is_diff && is_master_half;
        if (is_right_hp_diff_input_master)
            write_bit("IBUFDS_BANK_GLUE");

        int hclk = uarch->hclk_for_iob(pad->bel);

        if (only_diff)
            ioconfig_by_hclk[hclk].only_diff = true;
        if (is_tmds33)
            ioconfig_by_hclk[hclk].tmds_33 = true;
        if (is_lvds25)
            ioconfig_by_hclk[hclk].lvds_25 = true;

        if (is_output) {
            // DRIVE
            int default_drive = (is_hp_bank && iostandard == "LVCMOS12") ? 8 : 12;
            int drive = int_or_default(pad->attrs, id_DRIVE, default_drive);

            bool is_3v3_standard = iostandard == "LVCMOS33" || iostandard == "LVTTL";
            bool hp_bank_given_3v3_standard = is_3v3_standard && is_hp_bank;
            if (hp_bank_given_3v3_standard)
                log_error("high performance banks (RIOB18) do not support IO standard %s\n", iostandard.c_str());

            bool is_lvcmos15_or_18 = iostandard == "LVCMOS18" || iostandard == "LVCMOS15";
            // On a right-hand HP bank each drive strength of LVCMOS12, LVCMOS15
            // and LVCMOS18 is a code of its own over thirteen bits per half,
            // and its row is <standard>.DRIVE.I<mA>.  The grouped rows below
            // cover only part of those bits.  The LVCMOS15_LVCMOS18 one is
            // LVCMOS18 at 12 mA exactly, so every other LVCMOS18 strength and
            // every LVCMOS15 one came out as that one; the LVCMOS12 one is
            // none of its four.  Measured on Vivado bitstreams of every
            // strength, with the same codes on z030, z100 and k160.  Virtex-7
            // keeps the old rows: the 485T is outside the Vivado licence these
            // were measured with.
            const bool drive_has_own_row = is_riob18 && !is_sing && is_low_volt_lvcmos && !device_is_virtex7();
            if (iostandard == "SSTL135")
                write_bit("SSTL135.DRIVE.I_FIXED");
            else if (drive_has_own_row)
                write_bit(iostandard + ".DRIVE.I" + std::to_string(drive));
            else if (is_hp_bank) {
                if (is_lvcmos15_or_18) {
                    write_bit("LVCMOS15_LVCMOS18.DRIVE.I12_I16_I2_I4_I6_I8");
                    // Drive strength is encoded by which of several
                    // overlapping feature bits are set, not by one bit per
                    // level: the bit above is shared by {2,4,6,8,12,16} mA and
                    // narrows to 12 or 8 only when this one joins it.  The
                    // branch that writes it lived in the non-HP chain below,
                    // so an HP-bank pad silently got some other strength.
                    bool lvcmos18_drive_12_or_8 = iostandard == "LVCMOS18" && (drive == 12 || drive == 8);
                    if (lvcmos18_drive_12_or_8 && !is_sing)
                        write_bit("LVCMOS18.DRIVE.I12_I8");
                }
                else if (iostandard == "LVCMOS12")
                    write_bit("LVCMOS12.DRIVE.I2_I4_I6_I8");
                else if (iostandard == "LVDS") {
                    // LVDS is a master-half feature group on a high-performance
                    // bank: the database defines LVDS.DRIVE.I_FIXED and
                    // LVDS.OUT on IOB_Y0 only -- on LIOB18 and RIOB18 alike,
                    // and on kintex7's RIOB18 too -- and a reference bitstream
                    // for an OBUFDS pair sets exactly those two on the master
                    // half and nothing LVDS on the slave.  Without the guard
                    // the slave emits a key no family defines and fasm2frames
                    // stops with FasmLookupError.  Same shape as the IOB33
                    // LVDS_25 / TMDS_33 cases below.
                    if (yLoc == 0) {
                        write_bit("LVDS.DRIVE.I_FIXED");
                        write_bit("LVDS.OUT");
                    }
                }
                else if (is_sstl) {
                    write_bit(iostandard + ".DRIVE.I_FIXED");
                }
            } else { // IOB33
                if (iostandard == "TMDS_33" && yLoc == 0) {
                    write_bit("TMDS_33.DRIVE.I_FIXED");
                    write_bit("TMDS_33.OUT");
                } else if (iostandard == "LVDS_25" && yLoc == 0) {
                    write_bit("LVDS_25.DRIVE.I_FIXED");
                    write_bit("LVDS_25.OUT");
                } else if ((iostandard == "LVCMOS15" && drive == 16) || iostandard == "SSTL15")
                    write_bit("LVCMOS15_SSTL15.DRIVE.I16_I_FIXED");
                else if (iostandard == "LVCMOS18" && (drive == 12 || drive == 8))
                    write_bit("LVCMOS18.DRIVE.I12_I8");
                // prjxray-db names both patterns as covering DRIVE=12
                // (I12_I8 and I12_I16), which cannot both be right.  Vivado
                // settles it per standard (a Vivado build of every drive of
                // each, and the four references in
                // prjxray-db/artix7/harness/{arty-a7/{swbut,uart,pmod},basys3/swbut}):
                // LVCMOS33 at its default drive 12 is I12_I16, and its drive
                // 16 has a pattern of its own, LVCMOS33.DRIVE.I16; LVTTL
                // drive 12 is I12_I8 and drive 16 is I12_I16.
                else if (iostandard == "LVCMOS33" && drive == 16)
                    write_bit("LVCMOS33.DRIVE.I16");
                else if ((iostandard == "LVCMOS33" && drive == 12) || (iostandard == "LVTTL" && drive == 16))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I16");
                else if ((iostandard == "LVCMOS33" && drive == 8) ||
                         (iostandard == "LVTTL" && (drive == 8 || drive == 12)))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I8");
                else if ((iostandard == "LVCMOS33" && drive == 4) || (iostandard == "LVTTL" && drive == 4))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I4");
                else if (drive == 8 && (iostandard == "LVCMOS12" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS12_LVCMOS25.DRIVE.I8");
                else if (drive == 4 &&
                         (iostandard == "LVCMOS15" || iostandard == "LVCMOS18" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS15_LVCMOS18_LVCMOS25.DRIVE.I4");
                else if (is_lvcmos || iostandard == "LVTTL")
                    write_bit(iostandard + ".DRIVE.I" + std::to_string(drive));
            }

            // SSTL output used
            if (is_riob18 && is_sstl)
                write_bit(iostandard + ".IN_USE");

            // SLEW
            bool hp_bank_slow_slew = is_hp_bank && slew == "SLOW";
            if (hp_bank_slow_slew) {
                if (!is_sing) {
                    if (iostandard == "SSTL135")
                        write_bit("SSTL135.SLEW.SLOW");
                    else if (iostandard == "SSTL15")
                        write_bit("SSTL15.SLEW.SLOW");
                    else {
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                        // A left-hand HP bank sets the wide alias too -- the
                        // input path above already does, and a golden Vivado
                        // build of examples/vc707-telegraph sets both on every
                        // driven LIOB18 half.  segbits_riob18.db defines no
                        // such key, hence the LIOB18 guard.
                        if (is_liob18)
                            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW."
                                      "SLOW");
                    }
                }
            } else if (slew == "SLOW") {
                if (iostandard != "LVDS_25" && iostandard != "TMDS_33")
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
            } else if (is_hp_bank)
                write_bit(iostandard + ".SLEW.FAST");
            else if (iostandard == "SSTL135" || iostandard == "SSTL15")
                write_bit("SSTL135_SSTL15.SLEW.FAST");
            else
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL.SLEW.FAST");

            // The mirror of IBUF_HP_BANK_GLUE on the input path: a driven half
            // of a left-hand HP bank carries a bank-level glue bit.  The
            // legacy flow's working VC707 bitstreams set it; ours did not, and
            // examples/vc707-telegraph's golden Vivado build sets it on every
            // driven LIOB18 output half.  SING tiles are included: their own
            // segbits_liob18_sing.db defines IOB_Y{0,1}.OBUF_HP_BANK_GLUE, so
            // the earlier !is_sing skip dropped a real, expressible bit.
            //
            // Its three bits are not the IOB's, though: they are OQUSED, OMUX.D1
            // and OSERDES.DATA_RATE_TQ.BUF of the OLOGIC beside it, which is how
            // an OLOGIC is set when it only passes the fabric signal through.
            // When an OLOGIC cell drives the pad, that cell configures the site,
            // and the glue would add OMUX.D1, driving the pad from D1 around the
            // register, and DATA_RATE_TQ.BUF, which contradicts the
            // DATA_RATE_TQ.DDR of an ODDR on T.  Vivado writes neither there.
            bool is_left_hp_single_ended = is_liob18 && !is_diff;
            const bool ologic_passes_through = is_left_hp_single_ended && !pad_driven_by_ologic_cell();
            if (ologic_passes_through)
                write_bit("OBUF_HP_BANK_GLUE");
        }

        // The unused half of a tile with a driven output is not left blank:
        // it takes the bank's slew and stepdown and is pulled down, Vivado's
        // default for an unused pin.  This is the output-side mirror of the
        // partner-half block in the input path below.
        bool is_left_hp_single_ended_output = is_output && is_liob18 && !is_diff && !is_sing;
        bool describe_unused_partner_of_output = is_left_hp_single_ended_output && partner_pad_unused();
        if (describe_unused_partner_of_output) {
            std::string saved = fasm_ctx.back();
            fasm_ctx.back() = "IOB_Y" + std::to_string(1 - yLoc);
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
            write_bit("PULLTYPE.PULLDOWN");
            fasm_ctx.back() = saved;
        }

        if (is_input) {
            // A single-ended input on a left-hand HP bank.  Either half can
            // hold one, but the two do not share a feature vocabulary:
            // segbits_liob18.db defines no IOB_Y1.LVCMOS12_LVCMOS15.IN and no
            // IOB_Y1.IBUF_HP_BANK_GLUE at all, and IOB_Y1's SSTL IN_ONLY key
            // carries LVCMOS18 in its name where IOB_Y0's does not.  So the
            // half decides the names, and writing IOB_Y0's on IOB_Y1 is a
            // FasmLookupError.
            //
            // This was guarded `yLoc == 0` because the reference bitstream it
            // was written from happened to have its input on the master half.
            // An input on the slave half then got no slew and no IN_ONLY at
            // all: on this board that was a UART that could transmit but not
            // receive, since AU36 (TX) is the master half of its tile and
            // AU33 (RX) is the slave half of its own.  A reset pin on a
            // master half hid it, because a reset that reads a pulled-down
            // pin looks exactly like a reset that works.
            //
            // The SING tile type has none of these keys -- only `.IN` --
            // hence the guard against it.
            bool is_left_hp_single_ended_input_only = is_liob18 && !is_output && !is_diff && !is_sing;
            if (is_left_hp_single_ended_input_only) {
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                if (yLoc == 0) {
                    // The glue and .IN are skipped when the other half also
                    // receives: they share a bit with IOB_Y1's .IN, and for a
                    // tile of two pure inputs Vivado writes neither -- only
                    // the IN_ONLY keys.  Taken from bit2fasm of the same SoC
                    // (LIOB18_X81Y6), not inferred.
                    if (!partner_pad_is_input()) {
                        write_bit("IBUF_HP_BANK_GLUE");
                        write_bit("LVCMOS12_LVCMOS15.IN");
                    }
                    write_bit("LVCMOS12_LVCMOS15_SSTL12_SSTL135_SSTL15.IN_ONLY");
                } else {
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
                }

                // The partner half, when nothing is bound to it, takes the
                // bank's slew and stepdown and is pulled down.  Which half
                // that is follows from this pad's, rather than being always
                // IOB_Y1 as it was when only IOB_Y0 could reach here.
                if (partner_pad_unused()) {
                    std::string saved = fasm_ctx.back();
                    fasm_ctx.back() = "IOB_Y" + std::to_string(1 - yLoc);
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
                    write_bit("PULLTYPE.PULLDOWN");
                    fasm_ctx.back() = saved;
                }
            }
            // Right-hand HP bank, single-ended input-only pad.  The
            // left-hand block above is the mirror of this one, but the two
            // tile types do not share a feature vocabulary: RIOB18 has no
            // IBUF_HP_BANK_GLUE, no STEPDOWN and no wide
            // LVCMOS..._LVTTL_SSTL135_SSTL15.SLEW.SLOW alias.
            //
            // The enable and IN_ONLY are written below, for either half, as
            // LVCMOS12_LVCMOS15_LVCMOS18.IN and
            // LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY: the
            // names a Vivado bitstream for such an input decodes to on zynq7.
            // This block wrote LVCMOS12_LVCMOS15.IN and
            // LVCMOS12_LVCMOS15_SSTL12_SSTL135_SSTL15.IN_ONLY on top.  The
            // zynq7 database has neither for RIOB18, so an input on IOB_Y0 of
            // a z030/z045/z100 HP bank did not assemble; and kintex7's short
            // .IN also clears 39_01, IOB_Y1's enable, so neither did a tile
            // whose IOB_Y1 receives too.
            const bool is_right_hp_single_ended_input_master = is_riob18 && !is_output && !is_diff && is_master_half;
            if (is_right_hp_single_ended_input_master) {
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");

                // The partner half, when nothing is bound to it, takes the
                // bank's slew and is pulled down.  A partner in use describes
                // itself, and its own PULLTYPE contradicts the pulldown.
                const bool describe_unused_partner = partner_pad_unused();
                if (describe_unused_partner) {
                    std::string saved = fasm_ctx.back();
                    fasm_ctx.back() = "IOB_Y1";
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                    write_bit("PULLTYPE.PULLDOWN");
                    fasm_ctx.back() = saved;
                }
            }
            if (!is_diff) {
                if (iostandard == "LVCMOS33" || iostandard == "LVTTL" || iostandard == "LVCMOS25") {
                    if (!is_riob18)
                        write_bit("LVCMOS25_LVCMOS33_LVTTL.IN");
                    else
                        log_error("high performance banks (RIOB18) do not support IO standard %s\n",
                                  iostandard.c_str());
                }

                if (is_sstl) {
                    ioconfig_by_hclk[hclk].vref = true;
                    if (!is_riob18)
                        write_bit("SSTL135_SSTL15.IN");

                    if (is_riob18) {
                        write_bit("SSTL12_SSTL135_SSTL15.IN");
                    }

                    if (!is_riob18 && pad->attrs.count(id_IN_TERM))
                        write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
                }

                // The two halves share this bit, with opposite polarity, so
                // describing it from both raises FasmInconsistentBits.  Taken
                // from what Vivado emits for a tile whose halves both receive
                // (bit2fasm of the same SoC): the bit is set once, from
                // IOB_Y1, and IOB_Y0 carries IBUF_HP_BANK_GLUE instead.  One
                // set bit serves both halves; describing it from neither
                // leaves it clear and the inputs deaf.
                //
                // That sharing is a high-performance bank property.  An HR
                // tile (LIOB33/RIOB33) gives each half its own .IN bits and
                // has no IBUF_HP_BANK_GLUE key at all, so there every
                // receiving half describes its own .IN -- the HyperRAM data
                // pairs on the Sonata are LVCMOS18 inouts on an HR bank.
                bool halves_share_in_bit = is_hp_bank;
                if (is_low_volt_lvcmos && !halves_share_in_bit) {
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                } else if (is_low_volt_lvcmos) {
                    if (!partner_pad_is_input())
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                    else if (is_output) {
                        // Both halves drive as well as receive -- SD cmd and
                        // data.  IOB_Y1 sets its .IN and IOB_Y0 the bank glue
                        // (LIOB18_X81Y10).  IOB_Y0 also needs its own .IN: the
                        // VC707's BPI flash data pads (a Vivado bitstream,
                        // LIOB18_X81Y83) set it, and without it those pads read
                        // as constants -- the flash then looked silent.
                        // The glue is a left-bank key only: RIOB18 has no
                        // IOB_Y0.IBUF_HP_BANK_GLUE and fasm2frames stops at
                        // it, while Vivado sets just the two .IN there
                        // (xc7z030/z100/k160t, the two halves as IOBUF and
                        // input, site bits identical without the glue).
                        if (yLoc == 1)
                            write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                        else {
                            if (is_liob18)
                                write_bit("IBUF_HP_BANK_GLUE");
                            write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                        }
                    }
                    else {
                        // Two pure inputs.  Vivado sets each half's own .IN (bit2fasm of LIOB18_X81Y128,
                        // the VC707 CPU_RESET tile, shows IOB_Y0.IN and IOB_Y1.IN both set).  This used
                        // to write neither, because the database's IOB_Y0.IN carried IOB_Y1's bit as a
                        // negated !39_01 and the pair could not be described; the database no longer does
                        // (prjxray-db 43af59b), and with neither written both inputs read as constant high.
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                    }
                }
            } else /* is_diff */ {
                if (is_riob18) {
                    // The high-performance differential receiver is configured
                    // on the P half; both halves use the high-performance slew.
                    if (yLoc == 0) {
                        write_bit("LVDS_SSTL12_SSTL135_SSTL15.IN_DIFF");
                        write_bit("SSTL12_SSTL135_SSTL15.IN_DIFF");
                        std::string saved = fasm_ctx.back();
                        fasm_ctx.back() = "DIFF";
                        write_bit("ZIBUF_LOW_PWR");
                        fasm_ctx.back() = saved;
                    }
                    // ...but only on an input-ONLY pad.  HP-bank slew is one
                    // shared bit, so on a bidirectional diff pad (DDR3 DQS,
                    // DIFF_SSTL15) this collides with the SSTL15.SLEW.FAST the
                    // is_output branch wrote for the same site and fasm2frames
                    // rejects the file (FasmInconsistentBits).  The driven
                    // direction owns the slew choice.
                    if (!is_output)
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                } else if (is_liob18) {
                    // The left-hand HP receiver, from a reference bitstream (an
                    // IBUFDS LVDS on the VC707 USER_CLOCK pins, LIOB18_X81Y79):
                    // the P half alone is set, with the same IN_DIFF bits as the
                    // right-hand side.  The database's LIOB18 LVDS.IN_DIFF key
                    // clears one of them (!38_126), so the SSTL key is the one
                    // that matches; nothing on the N half.
                    const bool is_p_half = yLoc == 0;
                    if (is_p_half) {
                        write_bit("SSTL12_SSTL135_SSTL15.IN_DIFF");
                        if (!is_output)
                            write_bit("IBUF_HP_BANK_GLUE");
                        std::string saved = fasm_ctx.back();
                        fasm_ctx.back() = "DIFF";
                        write_bit("ZIBUF_LOW_PWR");
                        fasm_ctx.back() = saved;
                    }
                    // As on the right-hand side, and as Vivado's bitstream
                    // decodes (both halves of LIOB18_X81Y79 for an input-only
                    // IBUFDS).  The feature is all cleared bits in the
                    // database, so the frames are the same either way.
                    if (!is_output)
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
                } else if (is_tmds33) {
                    // TMDS_33 has a receiver key of its own, defined on the P
                    // half only; the N half carries no IN_DIFF (a reference
                    // bitstream for an IBUFDS TMDS_33 sets exactly this one).
                    if (is_master_half)
                        write_bit("TMDS_33.IN_DIFF");
                } else {
                    write_bit("LVDS_25_SSTL135_SSTL15.IN_DIFF");
                }

                if (pad->attrs.count(id_IN_TERM))
                    write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
            }

            // IN_ONLY
            if (!is_output) {
                if (is_riob18) {
                    bool is_diff_master_half = is_diff && is_master_half;
                    if (is_diff_master_half) {
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
                        write_bit("LVCMOS12_LVCMOS15_SSTL12_SSTL135_SSTL15.IN_ONLY");
                    } else
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
                } else
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_"
                              "ONLY");
            }
        }

        // STEPDOWN is a left-hand HP feature: segbits_riob18.db defines no
        // STEPDOWN key at all, on virtex7 or kintex7, and a reference
        // bitstream for a right-hand HP input sets none.
        // is_liob18 matches LIOB18_SING_* too, and the SING tile type defines
        // neither STEPDOWN nor the LVCMOS18 drive bit -- fasm2frames stops
        // with FasmLookupError on either.  The original guard excluded SING
        // implicitly by only firing for inputs; widening it to outputs has to
        // exclude SING explicitly.
        bool bank_supports_stepdown = !is_hp_bank || (is_liob18 && !is_diff && !is_sing);
        bool standard_uses_stepdown = is_low_volt_lvcmos || is_sstl;
        if (bank_supports_stepdown && standard_uses_stepdown) {
            if (iostandard == "SSTL12") {
                log_error("SSTL12 is only available on high performance banks.");
            }
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
            ioconfig_by_hclk[hclk].stepdown = true;
            is_stepdown = true;
        }

        if (is_riob18 && (is_input || is_output) && (boost::contains(iostandard, "SSTL") || iostandard == "LVDS")) {
            if (((yLoc == 0) && (iostandard == "LVDS")) || boost::contains(iostandard, "SSTL")) {
                // TODO: I get bit conflicts with this, it seems to work anyway. Test more.
                // write_bit("LVDS.IN_USE");
            }
        }

        if (is_input && is_output && !is_diff && yLoc == 1) {
            if (is_riob18 && boost::starts_with(iostandard, "SSTL"))
                write_bit("SSTL12_SSTL135_SSTL15.IN");
        }

        // IN_TERM.NONE and IN_ONLY for TMDS_33 output, e.g. HDMI signals
        if (is_output && is_diff) {
            // The slave half of a high-performance LVDS driver carries the
            // input-only bit; the driver itself is enabled by IOB_Y0.LVDS.OUT.
            bool is_hp_lvds_slave_half = is_hp_bank && iostandard == "LVDS" && yLoc == 1;
            if (is_hp_lvds_slave_half)
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
            // The same holds for LVDS_25 on a high-range bank: the slave half
            // is input-only, as it already is for TMDS_33 below.
            const bool is_hr_lvds25_slave_half = !is_hp_bank && is_lvds25 && yLoc == 1;
            if (is_hr_lvds25_slave_half)
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
            if (is_tmds33 && yLoc == 1) {
                if (pad->attrs.count(id_IN_TERM))
                    write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
                else
                    write_bit("IN_TERM.NONE");
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
            }
        }

        write_bit("PULLTYPE." + pulltype);
        pop(); // IOB_YN

        BelId inv;

        auto pad_bel_site = uarch->get_bel_site(pad->bel);

        if (is_riob18)
            inv = uarch->get_site_bel(pad_bel_site, ctx->id("IOB18S.O_ININV"));
        else
            inv = uarch->get_site_bel(pad_bel_site, ctx->id("IOB33S.O_ININV"));

        // OUT_DIFF belongs to the pseudo-differential outputs (DIFF_SSTL*),
        // whose S half is really driven through the inverter.  A true LVDS
        // driver on a high-performance bank is enabled by IOB_Y0.LVDS.OUT
        // instead, and a reference bitstream for one sets no OUT_DIFF.  The
        // same holds for LVDS_25 and TMDS_33 on a high-range bank
        // (IOB_Y0.LVDS_25.OUT / IOB_Y0.TMDS_33.OUT).
        bool output_inverter_used = inv != BelId() && ctx->getBoundBelCell(inv) != nullptr;
        bool is_true_hp_lvds_driver = is_hp_bank && iostandard == "LVDS";
        bool is_true_hr_diff_driver = !is_hp_bank && (is_lvds25 || is_tmds33);
        if (output_inverter_used && !is_true_hp_lvds_driver && !is_true_hr_diff_driver)
            write_bit("OUT_DIFF");

        if (is_stepdown && !is_sing)
            write_bit("IOB_Y" + std::to_string(yLoc) + ".LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");

        pop(); // tile
    }

    // OLOGIC sites holding an output register, as "TILE/OLOGIC_Yn".  The
    // pseudo-pip that carries a signal through the OLOGIC sets OMUX.D1, which
    // selects the combinational D1->pad bypass.  That is right when the site is
    // a wire, and wrong when it holds an ODDR: the pad would be driven from the
    // unregistered input instead of the register's output, quietly undoing the
    // reason for putting a register at the pad at all.  Vivado sets OQUSED and
    // the DDR rate and leaves OMUX alone.
    std::set<std::string> ologic_registered;
    // OLOGIC sites whose OSERDESE2 has T1 tied to GND: the pad-tristate pseudo-pip must not set
    // ZINV_T1 there (the GND arrives as VCC_WIRE and needs the site inverter active).
    std::set<std::string> ologic_t1_low;

    // OLOGIC sites whose tristate flip-flop (TFF) holds an ODDR that drives the
    // pad's T input, as "TILE/OLOGIC_Yn".  The pseudo-pip that crosses the site
    // for the fabric data (D1 -> OQ) also sets DATA_RATE_TQ.BUF, the unregistered
    // tristate; Vivado sets DATA_RATE_TQ.DDR for such a site instead, and the two
    // are alternatives of one field.
    std::set<std::string> ologic_t_registered;

    void write_iol_config(CellInfo *ci)
    {
        std::string tile = uarch->tile_name(ci->bel.tile);
        push(tile);
        bool is_sing = boost::contains(tile, "_SING_");
        bool is_top_sing = ci->bel.tile < uarch->hclk_for_ioi(ci->bel.tile);

        auto site_key = uarch->get_bel_site(ci->bel);
        std::string site = uarch->get_site_name(site_key).str(ctx);
        std::string sitetype = site.substr(0, site.find('_'));
        Loc siteloc = uarch->rel_site_loc(site_key);
        std::string site_y = stringf("%s_Y%d", sitetype.c_str(),
                                     is_sing ? (is_top_sing ? 1 : 0) : (1 - siteloc.y));
        push(site_y);
        if (ci->type.in(id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF))
            ologic_registered.insert(tile + "/" + site_y);
        if (ci->type.in(id_OLOGICE2_TFF, id_OLOGICE3_TFF))
            ologic_t_registered.insert(tile + "/" + site_y);
        const bool is_oserdese2 = ci->type == id_OSERDESE2_OSERDESE2;
        if (is_oserdese2) {
            // pack_constants turns a GND tie on an invertible pin into the VCC net plus
            // IS_T1_INVERTED, so "tied low" is either form.
            NetInfo *t1 = ci->getPort(ctx->id("T1"));
            const bool inverted = bool_or_default(ci->params, ctx->id("IS_T1_INVERTED"), false);
            const bool tied_to_gnd = t1 != nullptr && t1->name == ctx->id("$PACKER_GND_NET");
            const bool tied_to_inverted_vcc =
                    t1 != nullptr && t1->name == ctx->id("$PACKER_VCC_NET") && inverted;
            const bool t1_tied_low = tied_to_gnd || tied_to_inverted_vcc;
            if (t1_tied_low)
                ologic_t1_low.insert(tile + "/" + site_y);
        }

        if (ci->type == id_ILOGICE3_IFF) {
            write_bit("IDDR.IN_USE");
            write_bit("IDDR_OR_ISERDES.IN_USE");
            write_bit("ISERDES.MODE.MASTER");
            write_bit("ISERDES.NUM_CE.N1");

            // When an IDELAYE2 drives D the IFF must capture the delayed
            // path: IFFDELMUXE3.P0, the same state the ISERDESE2 branch
            // below writes for IOBDELAY=IFD.  Vivado golden, measured on
            // xc7a100t across all three edge modes (issue #22): with an
            // IDELAYE2 in front of the IDDR Vivado sets IFFDELMUXE3.P0 and
            // leaves IDELMUXE3 at P1; with the IDDR fed straight from the
            // IBUF it leaves both at P1 -- it never switches the
            // combinatorial-path mux for an IDDR, so no IDELMUXE3 write
            // belongs on this path at all.  (Port of nextpnr-xilinx
            // 8b33d912's polarity, resolved by that golden.)
            NetInfo *d = ci->getPort(id_D);
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            if (boost::contains(drv->type.str(ctx), "IDELAYE2"))
                write_bit("IFFDELMUXE3.P0");

            // Clock edge.  DDR_CLK_EDGE is a three-valued parameter encoded in
            // two bits, and the third value is the state where BOTH bits are
            // clear:
            //
            //     OPPOSITE_EDGE         !bit_a    bit_b
            //     SAME_EDGE               bit_a  !bit_b
            //     SAME_EDGE_PIPELINED   !bit_a  !bit_b
            //
            // So SAME_EDGE_PIPELINED is expressed by writing NEITHER of the
            // other two -- there is no third feature to write (adding one
            // only produces a FasmLookupError; no such key exists in any
            // database).  (Port of nextpnr-xilinx 9a6a7e3b.)
            std::string edge = str_or_default(ci->params, id_DDR_CLK_EDGE, "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE")
                write_bit("IFF.DDR_CLK_EDGE.SAME_EDGE");
            else if (edge == "OPPOSITE_EDGE")
                write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            else if (edge == "SAME_EDGE_PIPELINED") { /* both bits clear: write nothing */
            } else
                log_error("unsupported clock edge parameter for cell '%s' at %s: %s. Supported are: SAME_EDGE, "
                          "OPPOSITE_EDGE and SAME_EDGE_PIPELINED",
                          ci->name.c_str(ctx), site.c_str(), edge.c_str());

            std::string srtype = str_or_default(ci->params, id_SRTYPE, "SYNC");
            if (srtype == "SYNC")
                write_bit("IFF.SRTYPE.SYNC");
            else
                write_bit("IFF.SRTYPE.ASYNC");

            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            write_bit("ZINV_D", !bool_or_default(ci->params, id_IS_D_INVERTED, false));

            // The IFF is physically a four-flop block shared with ISERDESE2;
            // an IDDR only exposes Q1/Q2, so Q3/Q4 were left unwritten -- and
            // on silicon that is observable (outputs read the wrong value
            // despite programmed INIT).  IDDR has no INIT_Q3/Q4 parameters;
            // Vivado derives them from the mode (measured with a Vivado build
            // of an IDDR for every mode and every INIT_Q1/INIT_Q2): Q3 and Q4
            // copy INIT_Q1 and INIT_Q2 in SAME_EDGE and SAME_EDGE_PIPELINED,
            // and are 1 in OPPOSITE_EDGE.  (Port of nextpnr-xilinx d455ae52.)
            const bool q3_q4_copy_q1_q2 = edge != "OPPOSITE_EDGE";
            int init_q[5] = {0, 0, 0, 0, 0};
            for (int i = 1; i <= 2; i++)
                init_q[i] = int_or_default(ci->params, ctx->id("INIT_Q" + std::to_string(i)), 0);
            init_q[3] = q3_q4_copy_q1_q2 ? init_q[1] : 1;
            init_q[4] = q3_q4_copy_q1_q2 ? init_q[2] : 1;
            for (int i = 1; i <= 4; i++) {
                const bool init_is_zero = init_q[i] == 0;
                if (init_is_zero)
                    write_bit("IFF.ZINIT_Q" + std::to_string(i));
            }

            auto sr_name = str_or_default(ci->attrs, id_X_ORIG_PORT_SR, "R");
            if (sr_name == "R") {
                for (int i = 1; i <= 4; i++)
                    write_bit("IFF.ZSRVAL_Q" + std::to_string(i));
            }
        } else if (ci->type.in(id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF)) {
            std::string edge = str_or_default(ci->params, id_DDR_CLK_EDGE, "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE")
                write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");

            write_bit("ODDR_TDDR.IN_USE");
            write_bit("OQUSED");
            write_bit("OSERDES.DATA_RATE_OQ.DDR");
            const BelId tff_bel = uarch->get_site_bel(site_key, ctx->id("TFF"));
            const bool tristate_is_an_oddr = tff_bel != BelId() && ctx->getBoundBelCell(tff_bel) != nullptr;
            if (!tristate_is_an_oddr)
                write_bit("OSERDES.DATA_RATE_TQ.BUF");

            std::string srtype = str_or_default(ci->params, id_SRTYPE, "SYNC");
            if (srtype == "SYNC")
                write_bit("OSERDES.SRTYPE.SYNC");

            for (std::string d : {"D1", "D2"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));

            auto init = int_or_default(ci->params, id_INIT, 0);
            if (init == 0)
                write_bit("ZINIT_OQ");

            write_bit("ODDR.SRUSED", ci->getPort(id_SR) != nullptr);
            auto sr_name = str_or_default(ci->attrs, id_X_ORIG_PORT_SR, "R");
            if (sr_name == "R")
                write_bit("ZSRVAL_OQ");

            auto clk_inv = bool_or_default(ci->params, id_IS_CLK_INVERTED);
            if (!clk_inv)
                write_bit("ZINV_CLK");
        } else if (ci->type.in(id_OLOGICE2_TFF, id_OLOGICE3_TFF)) {
            // An ODDR on the pad's T input.  Measured against Vivado 2026.1 (T1 = D1, T2 = D2):
            // the tristate register runs in DDR mode, SRTYPE has its own bit, INIT is
            // ZINIT_TQ (the library default is 0), T2 is not inverted.  The site's clock and
            // edge bits come from the output ODDR when there is one; alone, Vivado sets
            // SAME_EDGE whichever edge the ODDR asks for.
            const bool is_d1_inverted = bool_or_default(ci->params, ctx->id("IS_D1_INVERTED"), false);
            const bool is_d2_inverted = bool_or_default(ci->params, ctx->id("IS_D2_INVERTED"), false);
            if (is_d1_inverted || is_d2_inverted)
                log_error("%s '%s' drives a tristate input with an inverted D1 or D2, which is not supported\n",
                          ci->type.c_str(ctx), ctx->nameOf(ci));

            write_bit("OSERDES.DATA_RATE_TQ.DDR");
            std::string srtype = str_or_default(ci->params, id_SRTYPE, "SYNC");
            if (srtype == "SYNC")
                write_bit("OSERDES.TSRTYPE.SYNC");
            write_bit("TDDR.SRUSED", ci->getPort(id_SR) != nullptr);
            auto init = int_or_default(ci->params, id_INIT, 0);
            if (init == 0)
                write_bit("ZINIT_TQ");
            write_bit("ZINV_T2");

            const BelId outff_bel = uarch->get_site_bel(site_key, ctx->id("OUTFF"));
            const bool output_is_an_oddr = outff_bel != BelId() && ctx->getBoundBelCell(outff_bel) != nullptr;
            if (!output_is_an_oddr) {
                write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");
                auto clk_inv = bool_or_default(ci->params, id_IS_CLK_INVERTED);
                if (!clk_inv)
                    write_bit("ZINV_CLK");
            }
        } else if (ci->type == id_OSERDESE2_OSERDESE2) {
            write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");
            write_bit("ODDR.SRUSED");
            write_bit("ODDR_TDDR.IN_USE");

            auto serdes_mode = str_or_default(ci->params, id_SERDES_MODE, "MASTER");
            bool is_cascaded = (serdes_mode == "SLAVE");

            // For cascaded OSERDESE2, OQUSED must be set even though OQ is not connected
            write_bit("OQUSED", is_cascaded || ci->getPort(id_OQ));
            write_bit("ZINV_CLK", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            // A Tn tied to GND reaches the site through the interconnect VCC_WIRE (the only constant
            // an IMUX can select), so the site inverter must stay active: ZINV_Tn clear. Setting it
            // turned "always drive" (T1 = 0) into a permanently tri-stated pin.
            for (std::string t : {"T1", "T2", "T3", "T4"}) {
                NetInfo *tn = ci->getPort(ctx->id(t));
                bool tied_low = tn != nullptr && tn->name == ctx->id("$PACKER_GND_NET");
                write_bit("ZINV_" + t, (tn != nullptr || t == "T1") && !tied_low &&
                                               !bool_or_default(ci->params, ctx->id("IS_" + t + "_INVERTED"), false));
            }
            for (std::string d : {"D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));
            write_bit("ZINIT_OQ", !bool_or_default(ci->params, id_INIT_OQ, false));
            write_bit("ZINIT_TQ", !bool_or_default(ci->params, id_INIT_TQ, false));
            write_bit("ZSRVAL_OQ", !bool_or_default(ci->params, id_SRVAL_OQ, false));
            write_bit("ZSRVAL_TQ", !bool_or_default(ci->params, id_SRVAL_TQ, false));

            push("OSERDES");
            write_bit("IN_USE");
            std::string type = str_or_default(ci->params, id_DATA_RATE_OQ, "BUF");
            write_bit(std::string("DATA_RATE_OQ.") + ((ci->getPort(id_OQ) != nullptr) ? type : "DDR"));
            write_bit(std::string("DATA_RATE_TQ.") +
                      ((ci->getPort(id_TQ) != nullptr) ? str_or_default(ci->params, id_DATA_RATE_TQ, "BUF") : "BUF"));
            int width = int_or_default(ci->params, id_DATA_WIDTH, 8);
#if 0
            write_bit("DATA_WIDTH.W" + std::to_string(width));
            if (type == "DDR" && (width == 6 || width == 8)) {
                write_bit("DATA_WIDTH.DDR.W6_8");
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            } else if (type == "SDR" && (width == 2 || width == 4 || width == 5 || width == 6)) {
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            }
#else
            if (type == "DDR")
                write_bit("DATA_WIDTH.DDR.W" + std::to_string(width));
            else if (type == "SDR")
                write_bit("DATA_WIDTH.SDR.W" + std::to_string(width));
            else
                write_bit("DATA_WIDTH.W" + std::to_string(width));
#endif
            write_bit("SRTYPE.SYNC");
            write_bit("TSRTYPE.SYNC");
            // TRISTATE_WIDTH is a two-valued parameter whose W1 state is
            // encoded as the absence of bits, so only W4 appears in segbits.
            // Never writing it left every OSERDESE2 programmed as
            // TRISTATE_WIDTH=1, silently overriding the cell's own parameter
            // (defaults to 4 in the library).
            // (Port of nextpnr-xilinx c05f0d05.)
            bool tristate_width_is_4 = int_or_default(ci->params, ctx->id("TRISTATE_WIDTH"), 4) == 4;
            if (tristate_width_is_4)
                write_bit("TRISTATE_WIDTH.W4");
            if (is_cascaded)
                write_bit("SERDES_MODE.SLAVE");
            pop();
            // An explicitly requested CLKDIV inversion on an OSERDESE2 was
            // discarded: the bit (OLOGIC_Y*.IS_CLKDIV_INVERTED) was written
            // nowhere.  It sits on the OLOGIC, not inside the OSERDES prefix.
            // (Port of nextpnr-xilinx b9ed05a2.)
            write_bit("IS_CLKDIV_INVERTED",
                      bool_or_default(ci->params, ctx->id("IS_CLKDIV_INVERTED"), false));
        } else if (ci->type == id_ISERDESE2_ISERDESE2) {
            std::string data_rate = str_or_default(ci->params, id_DATA_RATE);
            write_bit("IDDR_OR_ISERDES.IN_USE");
            if (data_rate == "DDR")
                write_bit("IDDR.IN_USE");
            write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            write_bit("IFF.SRTYPE.SYNC");
            for (int i = 1; i <= 4; i++) {
                write_bit("IFF.ZINIT_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->idf("INIT_Q%d", i), false));
                write_bit("IFF.ZSRVAL_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->idf("SRVAL_Q%d", i), false));
            }
            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            // INV_OCLK and ZINV_OCLK are two DISTINCT physical bits; the
            // fuzzer sets them as exact complements, so exactly one of the
            // two must always be set.  Writing only the Z half left
            // IS_OCLK_INVERTED=TRUE with neither bit set, an unprogrammed
            // state that corresponds to no value of the parameter.
            // (Port of nextpnr-xilinx c05f0d05.)
            bool oclk_inv = bool_or_default(ci->params, id_IS_OCLK_INVERTED, false);
            write_bit("IFF.INV_OCLK", oclk_inv);
            write_bit("IFF.ZINV_OCLK", !oclk_inv);

            std::string iobdelay = str_or_default(ci->params, id_IOBDELAY, "NONE");
            write_bit("IFFDELMUXE3.P0", (iobdelay == "IFD"));
            write_bit("ZINV_D", !bool_or_default(ci->params, id_IS_D_INVERTED, false) && (iobdelay != "IFD"));

            push("ISERDES");
            write_bit("IN_USE");
            // A capture that comes from the partner OSERDESE2's feedback (the
            // read-training reference of a DDR3 PHY) needs the OFB input mux
            // selected; the packer places the pair either way, but without this
            // bit the datapath still reads the pad side and the reference never
            // carries the training pattern.  (The fork wrote it from the same
            // parameter: nextpnr-xilinx fasm.cc.)
            write_bit("OFB_USED", str_or_default(ci->params, ctx->id("OFB_USED"), "FALSE") == "TRUE");
            int width = int_or_default(ci->params, id_DATA_WIDTH, 8);
            std::string mode = str_or_default(ci->params, id_INTERFACE_TYPE, "NETWORKING");
            std::string rate = str_or_default(ci->params, id_DATA_RATE, "DDR");
            write_bit(mode + "." + rate + ".W" + std::to_string(width));
            write_bit("MODE." + str_or_default(ci->params, id_SERDES_MODE, "MASTER"));
            write_bit("NUM_CE.N" + std::to_string(int_or_default(ci->params, id_NUM_CE, 1)));
            pop();
        } else if (ci->type == id_IDELAYE2_IDELAYE2) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, id_CINVCTRL_SEL, "FALSE") == "TRUE");
            write_bit("PIPE_SEL", str_or_default(ci->params, id_PIPE_SEL, "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE", str_or_default(ci->params, id_HIGH_PERFORMANCE_MODE, "FALSE") == "TRUE");
            write_bit("DELAY_SRC_" + str_or_default(ci->params, id_DELAY_SRC, "IDATAIN"));
            write_bit("IDELAY_TYPE_" + str_or_default(ci->params, id_IDELAY_TYPE, "FIXED"));
            write_int_vector("IDELAY_VALUE[4:0]", int_or_default(ci->params, id_IDELAY_VALUE, 0), 5, false);
            write_int_vector("ZIDELAY_VALUE[4:0]", int_or_default(ci->params, id_IDELAY_VALUE, 0), 5, true);
            write_bit("IS_DATAIN_INVERTED", bool_or_default(ci->params, id_IS_DATAIN_INVERTED, false));
            write_bit("IS_IDATAIN_INVERTED", bool_or_default(ci->params, id_IS_IDATAIN_INVERTED, false));
        } else if (ci->type == id_ODELAYE2_ODELAYE2) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, id_CINVCTRL_SEL, "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE", str_or_default(ci->params, id_HIGH_PERFORMANCE_MODE, "FALSE") == "TRUE");
            auto type = str_or_default(ci->params, id_ODELAY_TYPE, "FIXED");
            if (type != "FIXED")
                write_bit("ODELAY_TYPE_" + type);
            write_int_vector("ODELAY_VALUE[4:0]", int_or_default(ci->params, id_ODELAY_VALUE, 0), 5, false);
            write_int_vector("ZODELAY_VALUE[4:0]", int_or_default(ci->params, id_ODELAY_VALUE, 0), 5, true);
            write_bit("ZINV_ODATAIN", !bool_or_default(ci->params, id_IS_ODATAIN_INVERTED, false));
        } else {
            NPNR_ASSERT_FALSE("unsupported IOLOGIC");
        }
        pop(2);
    }

    void write_io()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_PAD) {
                // GT pads are configured by the GT writers, not the IO path
                // (their tiles have no HCLK/IOI structure)
                if (ci->bel != BelId()) {
                    // GT pads live in the GT/OPAD tiles and are configured by
                    // the GT writers, not the IO path
                    std::string belname = ctx->nameOfBel(ci->bel);
                    bool is_gt_pad = boost::contains(belname, "OPAD") || boost::contains(belname, "IPAD") ||
                                     boost::contains(belname, "GTPE2_") || boost::contains(belname, "GTXE2_");
                    if (is_gt_pad) {
                        blank();
                        continue;
                    }
                }
                write_io_config(ci);
                blank();
            } else if (ci->type.in(id_ILOGICE3_IFF, id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF, id_OLOGICE2_TFF,
                                   id_OLOGICE3_TFF, id_OSERDESE2_OSERDESE2,
                                   id_ISERDESE2_ISERDESE2, id_IDELAYE2_IDELAYE2, id_ODELAYE2_ODELAYE2)) {
                write_iol_config(ci);
                blank();
            }
        }
        for (auto &hclk : ioconfig_by_hclk) {
            push(uarch->tile_name(hclk.first));
            write_bit("STEPDOWN", hclk.second.stepdown);
            write_bit("VREF.V_675_MV", hclk.second.vref);
            write_bit("ONLY_DIFF_IN_USE", hclk.second.only_diff);
            write_bit("TMDS_33_IN_USE", hclk.second.tmds_33);
            write_bit("LVDS_25_IN_USE", hclk.second.lvds_25);
            pop();
        }
    }

    // An input that goes straight to the fabric has no ILOGIC *cell*: the
    // router simply passes through the ILOGIC's D -> O bypass, so
    // write_iol_config never runs for it and the site is left unconfigured.
    // That is not harmless.  ZINV_D is the D-input inverter control, held in
    // the inverted sense prjxray spells "ZINV": set means NOT inverted, and
    // clear -- the state an unconfigured site is left in -- means the fabric
    // sees the complement of the pin.
    //
    // On this board that was a UART which could transmit perfectly and
    // receive nothing but framing errors, because an inverted serial line
    // still has edges and still clocks: it just never spells a character.
    // Vivado sets the bit on every ILOGIC half it uses.
    //
    // The routing says which halves those are.  A pip into <something>_ILOGIC
    // <n>_D is an input arriving at ILOGIC n, and the site index matches the
    // wire's -- confirmed against a golden bitstream on both the ILOGIC and
    // OLOGIC sides.
    void write_ilogic_bypass_inversion()
    {
        for (auto &entry : pips_by_tile) {
            int tile = entry.first;
            std::set<int> ilogics;
            for (const auto &wire : used_wires_starting_with(tile, "", false)) {
                auto pos = wire.find("_ILOGIC");
                bool is_ilogic_d_wire = pos != std::string::npos && boost::ends_with(wire, "_D");
                if (!is_ilogic_d_wire)
                    continue;
                // ..._ILOGIC<n>_D -- take the digits between the two.
                std::string index = wire.substr(pos + 7, wire.size() - (pos + 7) - 2);
                bool index_is_numeric = !index.empty() && index.find_first_not_of("0123456789") == std::string::npos;
                if (!index_is_numeric)
                    continue;
                ilogics.insert(std::stoi(index));
            }
            if (ilogics.empty())
                continue;
            // Only three tile types define the bit: segbits_lioi.db,
            // segbits_lioi_tbytesrc.db and segbits_lioi_tbyteterm.db.  The
            // SING variants and every right-hand RIOI* tile have no ZINV_D at
            // all, so an input there cannot have its inversion set from FASM
            // -- a real gap in the database, and emitting the key anyway is
            // only a FasmLookupError.
            std::string tile_name = uarch->tile_name(tile);
            bool tile_defines_zinv_d = boost::starts_with(tile_name, "LIOI_") && !boost::contains(tile_name, "_SING");
            if (!tile_defines_zinv_d)
                continue;
            push(tile_name);
            for (int index : ilogics) {
                push(stringf("ILOGIC_Y%d", index));
                write_bit("ZINV_D");
                pop();
            }
            pop();
            blank();
        }
    }

    std::vector<std::string> used_wires_starting_with(int tile, const std::string &prefix, bool is_source)
    {
        std::vector<std::string> wires;
        if (!pips_by_tile.count(tile))
            return wires;
        for (auto pip : pips_by_tile[tile]) {
            auto &pd = chip_pip_info(ctx->chip_info, pip);
            int wire_index = is_source ? pd.src_wire : pd.dst_wire;
            std::string wire = IdString(chip_wire_info(ctx->chip_info, WireId(pip.tile, wire_index)).name).str(ctx);
            if (boost::starts_with(wire, prefix))
                wires.push_back(wire);
        }
        return wires;
    }

    // A regional-buffer source name carries either BUFHCLK or BUFRCLK. Both
    // the emit test and the suffix that goes into hclk_by_row need the same
    // offset, so find it once and hand it back: testing with `contains` and
    // then calling `find` again separately is how the two drifted apart, and
    // it is why widening only the test would emit a bit whose row key is
    // still cut at the wrong place.
    //
    // Returns npos when the name is neither.
    static size_t regional_buffer_offset(const std::string &s)
    {
        size_t off = s.find("BUFHCLK");
        const bool is_bufhclk = (off != std::string::npos);
        if (is_bufhclk)
            return off;
        return s.find("BUFRCLK");
    }

    void write_clocking()
    {
        std::string name, type;

        std::set<std::string> all_gclk;
        dict<int, std::set<std::string>> hclk_by_row;

        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_BUFGCTRL) {
                push(uarch->tile_name(ci->bel.tile));
                auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
                push(stringf("BUFGCTRL.BUFGCTRL_X%dY%d", xy.x, xy.y));
                write_bit("IN_USE");
                write_bit("INIT_OUT", bool_or_default(ci->params, id_INIT_OUT));
                // Set for the true polarity of the pin, like the ZINV_* bits:
                // prjxray named them after the parameter its fuzzer varied
                // with IGNORE left at ground. Vivado sets the bit for a signal
                // that is not inverted and for IGNORE tied high.
                write_bit("IS_IGNORE0_INVERTED", !bool_or_default(ci->params, id_IS_IGNORE0_INVERTED));
                write_bit("IS_IGNORE1_INVERTED", !bool_or_default(ci->params, id_IS_IGNORE1_INVERTED));
                write_bit("ZINV_CE0", !bool_or_default(ci->params, id_IS_CE0_INVERTED));
                write_bit("ZINV_CE1", !bool_or_default(ci->params, id_IS_CE1_INVERTED));
                write_bit("ZINV_S0", !bool_or_default(ci->params, id_IS_S0_INVERTED));
                write_bit("ZINV_S1", !bool_or_default(ci->params, id_IS_S1_INVERTED));
                pop(2);
            } else if (ci->type == id_BUFHCE_BUFHCE) {
                push(uarch->tile_name(ci->bel.tile));
                auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
                push(stringf("BUFHCE.BUFHCE_X%dY%d", xy.x, xy.y));
                write_bit("IN_USE");
                write_bit("CE_TYPE.ASYNC", str_or_default(ci->params, id_CE_TYPE, "SYNC") == "ASYNC");
                write_bit("INIT_OUT", bool_or_default(ci->params, id_INIT_OUT));
                // CE is tied active by prepare_clocking; the physical pin is
                // active-low, so invert it (port of nextpnr-xilinx fasm.cc)
                write_bit("ZINV_CE", !bool_or_default(ci->params, id_IS_CE_INVERTED));
                pop(2);
            } else if (ci->type == id_PLLE2_ADV_PLLE2_ADV) {
                write_pll(ci);
            } else if (ci->type == id_MMCME2_ADV_MMCME2_ADV) {
                write_mmcm(ci);
            }
            blank();
        }

        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            std::string name = uarch->tile_name(tile);
            std::string type = ctx->get_tile_type(tile).str(ctx);
            push(name);
            if (type == "HCLK_L" || type == "HCLK_R" || type == "HCLK_L_BOT_UTURN" || type == "HCLK_R_BOT_UTURN") {
                auto used_sources = used_wires_starting_with(tile, "HCLK_CK_", true);
                push("ENABLE_BUFFER");
                for (auto s : used_sources) {
                    // BUFRCLK as well as BUFHCLK.  The four
                    // HCLK_L.ENABLE_BUFFER.HCLK_CK_BUFRCLK0..3 rows exist in
                    // prjxray-db (artix7: prjxray-db#13; kintex7, spartan7,
                    // zynq7: prjxray-db#22), so emitting the feature resolves
                    // there; virtex7 is unmeasured and will raise the
                    // FasmLookupError that used to be a silent omission.
                    size_t off = regional_buffer_offset(s);
                    const bool is_regional_buffer_source = (off != std::string::npos);
                    if (is_regional_buffer_source) {
                        write_bit(s);
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(off));
                    }
                }
                pop();
            } else if (boost::starts_with(type, "CLK_HROW")) {
                auto used_gclk = used_wires_starting_with(tile, "CLK_HROW_R_CK_GCLK", true);
                auto used_ck_in = used_wires_starting_with(tile, "CLK_HROW_CK_IN", true);
                for (auto s : used_gclk) {
                    write_bit(s + "_ACTIVE");
                    all_gclk.insert(s.substr(s.find("GCLK")));
                }
                for (auto s : used_ck_in) {
                    if (boost::contains(s, "HROW_CK_INT"))
                        continue;
                    write_bit(s + "_ACTIVE");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                auto used_ccio = used_wires_starting_with(tile, "HCLK_CMT_CCIO", true);
                for (auto s : used_ccio) {
                    write_bit(s + "_ACTIVE");
                    write_bit(s + "_USED");
                }
                auto used_hclk = used_wires_starting_with(tile, "HCLK_CMT_CK_", true);
                for (auto s : used_hclk) {
                    size_t off = regional_buffer_offset(s);
                    const bool is_regional_buffer_source = (off != std::string::npos);
                    if (is_regional_buffer_source) {
                        write_bit(s + "_USED");
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(off));
                    }
                }
            }
            pop();
            blank();
        }

        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            std::string name = uarch->tile_name(tile);
            std::string type = ctx->get_tile_type(tile).str(ctx);
            push(name);
            if (type == "CLK_BUFG_REBUF") {
                for (auto &gclk : all_gclk) {
                    write_bit(gclk + "_ENABLE_ABOVE");
                    write_bit(gclk + "_ENABLE_BELOW");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                for (auto &hclk : hclk_by_row[tile / ctx->chip_info->width]) {
                    write_bit("HCLK_CMT_CK_" + hclk + "_USED");
                }
            }
            pop();
            blank();
        }
    }

    void write_bram_width(CellInfo *ci, const std::string &name, bool is_36, bool is_y1)
    {
        // SDP mode spans both data sides of the memory, so the
        // "opposite-side" width markers must also be configured wide,
        // exactly like Vivado's golden bitstreams:
        //   - RAMB36E1 READ_WIDTH_A=72: the 72-bit read is the A port (lower
        //     36 bits) PLUS the B port (upper 36 bits) -> READ_WIDTH_B_18 on
        //     BOTH RAMB18 halves.  yosys leaves READ_WIDTH_B at 0; the
        //     width-1 default kills the B-side read path on silicon.
        //   - RAMB18E1 READ_WIDTH_A=36: READ_WIDTH_B_18.
        //   - RAMB18E1 WRITE_WIDTH_B=36: WRITE_WIDTH_A_18.
        //   - RAMB36E1 WRITE_WIDTH_B=72: WRITE_WIDTH_A_18 on BOTH halves (yosys
        //     leaves WRITE_WIDTH_A at 0; the 72-bit write is the B port plus
        //     the 36 low bits on the A port).
        // (Port of nextpnr-xilinx f1c77134.)
        const int read_width_a = int_or_default(ci->params, ctx->id("READ_WIDTH_A"), 0);
        const int write_width_b = int_or_default(ci->params, ctx->id("WRITE_WIDTH_B"), 0);
        const int this_width = int_or_default(ci->params, ctx->id(name), 0);
        const bool this_width_param_is_unset = (this_width == 0);
        const bool this_param_is_read_width_b = (name == "READ_WIDTH_B");
        const bool this_param_is_write_width_a = (name == "WRITE_WIDTH_A");
        const bool b_side_reads_half_the_word = (is_36 ? (read_width_a == 72) : (read_width_a == 36));
        const bool is_18_sdp_write = (!is_36 && (write_width_b == 36));
        const bool is_36_sdp_write = (is_36 && (write_width_b == 72));
        const bool a_side_writes_half_the_word = (is_18_sdp_write || is_36_sdp_write);

        const bool widen_unset_read_width_b =
                this_width_param_is_unset && this_param_is_read_width_b && b_side_reads_half_the_word;
        const bool widen_unset_write_width_a =
                this_width_param_is_unset && this_param_is_write_width_a && a_side_writes_half_the_word;

        if (widen_unset_read_width_b) {
            write_bit("READ_WIDTH_B_18");
            return;
        }
        if (widen_unset_write_width_a) {
            write_bit("WRITE_WIDTH_A_18");
            return;
        }

        int width = this_width;
        const bool width_unset = width == 0;
        if (width_unset) {
            // A port the design never uses keeps its width field clear, the
            // _1 encoding.  That is what Vivado writes for an instantiated
            // RAMB with a port width of 0, decoded from its bitstreams: on
            // this tree's x1 ROMs, its 9-bit RAMs and its 36-bit buffers, the
            // unused write port of each is _1, and on RAMs whose ports
            // differ (written 36 wide on A and read 9 on B, and the reverse)
            // both unused directions are _1.  Taking another port's width
            // instead put the wrong port's width on the unused one wherever
            // the widths differ.  (Vivado's own synthesis gives an unused port
            // the memory's width as a parameter, which is where an earlier
            // comparison against it saw all four markers set.)
            return;
        }
        int actual_width = width;
        if (is_36) {
            if (width == 1)
                actual_width = 1;
            else
                actual_width = width / 2;
        }
        if (((is_36 && width == 72) || (is_y1 && actual_width == 36)) && name == "READ_WIDTH_A") {
            write_bit(name + "_18");
        }
        if (actual_width == 36) {
            write_bit("SDP_" + name.substr(0, name.length() - 2) + "_36");
            // The 36-wide mode lives in the SDP bit plus the marker of the
            // WIDE side only.  Writing the other side's 18 marker collides
            // with that side's own width field (e.g. a RAMB18E1 with
            // READ_WIDTH_A=36 and an unused B port: READ_WIDTH_B_18 and the
            // B-side default READ_WIDTH_B_1 encode the SAME prjxray bit).
            // (Port of nextpnr-xilinx 1b7d51b9.)
            bool is_write_width = name == "WRITE_WIDTH_A" || name == "WRITE_WIDTH_B";
            if (is_write_width)
                write_bit(name.substr(0, name.size() - 1) + ((name == "WRITE_WIDTH_B") ? "B_18" : "A_18"));
            else if (name == "READ_WIDTH_B")
                write_bit(name.substr(0, name.size() - 1) + "B_18");
        } else {
            // A 36-bit (SDP) port already emits the _18 bits for BOTH the A
            // and B halves of its direction; the paired port of the same
            // direction is unused in SDP mode and defaults to width 1, which
            // would emit a conflicting _1 bit for a half the SDP branch set
            // to _18.  Skip it.  (Port of nextpnr-xilinx 11f9b694.)
            bool dir_is_sdp36 = false;
            {
                std::string dir = name.substr(0, name.size() - 2);
                for (const char *ab : {"A", "B"}) {
                    int w = int_or_default(ci->params, ctx->id(dir + "_" + ab), 0);
                    int aw = (is_36 && w != 1 && w != 0) ? w / 2 : w;
                    if (aw == 36)
                        dir_is_sdp36 = true;
                }
            }
            bool is_unused_sdp_partner_port = dir_is_sdp36 && actual_width == 1;
            if (is_unused_sdp_partner_port)
                return;
            write_bit(name + "_" + std::to_string(actual_width));
        }
    }

    void write_bram_init(int half, CellInfo *ci, bool is_36)
    {
        for (std::string mode : {"", "P"}) {
            for (int i = 0; i < (mode == "P" ? 8 : 64); i++) {
                bool has_init = false;
                std::vector<bool> init_data(256, false);
                if (is_36) {
                    for (int j = 0; j < 2; j++) {
                        IdString param = ctx->idf("INIT%s_%02X", mode.c_str(), i * 2 + j);
                        if (ci->params.count(param)) {
                            auto &init0 = ci->params.at(param);
                            has_init = true;
                            for (int k = half; k < 256; k += 2) {
                                if (k >= int(init0.str.size()))
                                    break;
                                init_data[j * 128 + (k / 2)] = init0.str[k] == Property::S1;
                            }
                        }
                    }
                } else {
                    IdString param = ctx->idf("INIT%s_%02X", mode.c_str(), i);
                    if (ci->params.count(param)) {
                        auto &init = ci->params.at(param);
                        has_init = true;
                        for (int k = 0; k < 256; k++) {
                            if (k >= int(init.str.size()))
                                break;
                            init_data[k] = init.str[k] == Property::S1;
                        }
                    }
                }
                if (has_init)
                    write_vector(stringf("INIT%s_%02X[255:0]", mode.c_str(), i), init_data);
            }
        }
    }

    // The word bit that Z{INIT,SRVAL}_{A,B}[k] of one RAMB18 half holds, as
    // Vivado's golden bitstreams place it (the index runs from the top of the
    // 18-bit half word down).  A 9-bit word ends in its parity bit, which
    // Vivado stores in slot 1, the parity slot of the 18-bit word.
    int bram_output_word_bit(bool is_36, int half, int port_width, int k)
    {
        const int top_down_bit = 17 - k;
        const bool is_18_nine_wide = (!is_36 && port_width == 9);
        if (is_18_nine_wide) {
            // The parity bit and bit 16 trade places.
            const bool k_is_parity_slot = (k == 1);
            if (k_is_parity_slot)
                return 8;
            const bool k_is_displaced_slot = (k == 9);
            if (k_is_displaced_slot)
                return 16;
            return top_down_bit;
        }
        // In a RAMB36 the 9-bit halves of an 18-bit port, and the low half of
        // a 9-bit port, end in a parity bit; it takes slot 1 and the bits
        // above it move down one slot.
        const bool is_36_eighteen_wide = (is_36 && port_width == 18);
        const bool is_36_nine_wide_low_half = (is_36 && port_width == 9 && half == 0);
        const int parity_bit = is_36_eighteen_wide ? 8 : (is_36_nine_wide_low_half ? 4 : -1);
        const bool word_has_parity_to_move = (parity_bit >= 0);
        if (word_has_parity_to_move) {
            const bool k_is_parity_slot = (k == 1);
            if (k_is_parity_slot)
                return parity_bit;
            const bool k_is_shifted_slot = (k >= 2 && k <= 17 - parity_bit);
            if (k_is_shifted_slot)
                return 18 - k;
        }
        return top_down_bit;
    }

    // INIT_A/B and SRVAL_A/B give the value of the output register of port A
    // or B at power-up and on reset.  The fabric stores them inverted, like
    // the other Z* bits.  The 36 bits of a RAMB36 word alternate between its
    // two RAMB18 halves: even bits in Y0, odd bits in Y1.  The word that
    // counts is the one of the read port.
    void write_bram_output_values(CellInfo *ci, int half, bool is_36)
    {
        for (const char *value : {"INIT", "SRVAL"}) {
            for (const char *port : {"A", "B"}) {
                const int port_width = int_or_default(ci->params, ctx->idf("READ_WIDTH_%s", port), 0);
                std::vector<bool> stored(18, true);
                const IdString param = ctx->idf("%s_%s", value, port);
                const bool param_is_given = ci->params.count(param) && !ci->params.at(param).is_string;
                if (param_is_given) {
                    const std::string &bits = ci->params.at(param).str;
                    for (int k = 0; k < 18; k++) {
                        const int word_bit = bram_output_word_bit(is_36, half, port_width, k);
                        const int param_bit = is_36 ? (2 * word_bit + half) : word_bit;
                        const bool bit_is_set = (param_bit < int(bits.size()) && bits[param_bit] == Property::S1);
                        stored[k] = !bit_is_set;
                    }
                }
                write_vector(stringf("Z%s_%s[17:0]", value, port), stored);
            }
        }
    }

    void write_bram_half(int tile, int half, CellInfo *ci)
    {
        push(uarch->tile_name(tile));
        push("RAMB18_Y" + std::to_string(half));
        if (ci != nullptr) {
            bool is_36 = ci->type == id_RAMB36E1_RAMB36E1;
            write_bit("IN_USE");
            write_bram_width(ci, "READ_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "READ_WIDTH_B", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_B", is_36, half == 1);
            write_bit("DOA_REG", bool_or_default(ci->params, id_DOA_REG, false));
            write_bit("DOB_REG", bool_or_default(ci->params, id_DOB_REG, false));
            for (auto &invpin : invertible_pins[ctx->id(ci->attrs[id_X_ORIG_TYPE].as_string())])
                write_bit("ZINV_" + invpin.str(ctx),
                          !bool_or_default(ci->params, ctx->id("IS_" + invpin.str(ctx) + "_INVERTED"), false));
            // REGCLKARDRCLK and REGCLKB are SITE pins, not RAMB18E1/RAMB36E1
            // cell ports, so they are absent from invertible_pins.  prjxray's
            // fuzzer solved this only for a port narrower than 36 bits: with
            // DO*_REG == 1 the output-register clock follows the corresponding
            // data clock, and with DO*_REG == 0 it is always inverted (tag 0,
            // the bit clear, which is what emitting nothing already gives).
            // (Port of nextpnr-xilinx e71acda2.)
            // A wide SDP read is outside that solve.  Vivado clocks both output
            // registers from REGCLKARDRCLK and leaves REGCLKB unconnected, so
            // ZINV_REGCLKB stays clear on both halves even when DOB_REG is 1.
            // Applying the narrow-port rule there sets the bit.
            bool port_a_output_registered = bool_or_default(ci->params, ctx->id("DOA_REG"), false);
            bool port_b_output_registered = bool_or_default(ci->params, ctx->id("DOB_REG"), false);
            if (port_a_output_registered)
                write_bit("ZINV_REGCLKARDRCLK",
                          !bool_or_default(ci->params, ctx->id("IS_CLKARDCLK_INVERTED"), false));
            const std::string ram_mode = str_or_default(ci->params, ctx->id("RAM_MODE"), "TDP");
            const bool sdp_leaves_regclkb_clear = ram_mode == "SDP";
            const bool emit_regclkb_invert = port_b_output_registered && !sdp_leaves_regclkb_clear;
            if (emit_regclkb_invert)
                write_bit("ZINV_REGCLKB",
                          !bool_or_default(ci->params, ctx->id("IS_CLKBWRCLK_INVERTED"), false));
            for (auto wrmode : {"WRITE_MODE_A", "WRITE_MODE_B"}) {
                std::string mode = str_or_default(ci->params, ctx->id(wrmode), "WRITE_FIRST");
                if (mode != "WRITE_FIRST")
                    write_bit(std::string(wrmode) + "_" + mode);
            }
            write_bram_output_values(ci, half, is_36);

            write_bram_init(half, ci, is_36);
        }
        pop();
        if (half == 0) {
            auto used_rdaddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRARDADDR", false);
            auto used_wraddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRBWRADDR", false);
            write_bit("CASCOUT_ARD_ACTIVE", !used_rdaddrcasc.empty());
            write_bit("CASCOUT_BWR_ACTIVE", !used_wraddrcasc.empty());
            // A RAMB36E1 port 1 or 9 bits wide -- the widths that do not
            // halve evenly between the RAMB18s -- also needs a tile-level
            // bit: width 1 is each half at 1 plus BRAM36_*_WIDTH_1, width 9
            // each half at 4 plus the same bit (prjxray's 027-bram36-config).
            // Without it the halves act as two independent memories and a
            // read returns the wrong bits.
            // A port the design never uses (width 0) leaves its bit clear,
            // as Vivado does: on an x1 ROM it sets the bit for both reads and
            // the write it has, not for the unused write port, and on a RAM
            // written 9 wide and read 36 wide only the 9-wide port's bit.
            bool is_ramb36 = ci != nullptr && ci->type == id_RAMB36E1_RAMB36E1;
            if (is_ramb36) {
                static const char *kWidths[] = {"READ_WIDTH_A", "READ_WIDTH_B", "WRITE_WIDTH_A",
                                                "WRITE_WIDTH_B"};
                for (const char *width : kWidths) {
                    int bits = int_or_default(ci->params, ctx->id(width), 0);
                    bool odd_width = bits == 1 || bits == 9;
                    write_bit(std::string("RAMB36.BRAM36_") + width + "_1", odd_width);
                }
                // The LOWER block of a cascaded pair is the one that drives
                // CASCADEOUTA/B; an UPPER block and a block on its own share
                // the cleared bit (prjxray's RAM_EXTENSION_*_NONE_OR_UPPER).
                for (char port : {'A', 'B'}) {
                    std::string extension = str_or_default(ci->params, ctx->idf("RAM_EXTENSION_%c", port), "NONE");
                    const bool lower_of_a_pair = extension == "LOWER";
                    write_bit(stringf("RAMB36.RAM_EXTENSION_%c_LOWER", port), lower_of_a_pair);
                }
            }
            // Ported from nextpnr-xilinx f0975539. Vivado sets both offsets
            // to all-ones on a used tile whose lower half is occupied.
            if (ci != nullptr) {
                write_vector("ZALMOST_EMPTY_OFFSET[12:0]", std::vector<bool>(13, true));
                write_vector("ZALMOST_FULL_OFFSET[12:0]", std::vector<bool>(13, true));
            }
        }
        pop();
    }

    void write_bram()
    {
        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            IdString type = ctx->get_tile_type(tile);
            if (type.in(id_BRAM_L, id_BRAM_R)) {
                CellInfo *l = nullptr, *u = nullptr;
                const auto &bts = uarch->tile_status[tile].bts;
                if (bts) {
                    if (bts->cells[BEL_RAM36] != nullptr) {
                        l = bts->cells[BEL_RAM36];
                        u = bts->cells[BEL_RAM36];
                    } else {
                        l = bts->cells[BEL_RAM18_L];
                        u = bts->cells[BEL_RAM18_U];
                    }
                }
                write_bram_half(tile, 0, l);
                write_bram_half(tile, 1, u);
                blank();
            }
        }
    }

    double float_or_default(CellInfo *ci, const std::string &name, double def)
    {
        IdString p = ctx->id(name);
        if (!ci->params.count(p))
            return def;
        auto &prop = ci->params.at(p);
        if (prop.is_string)
            return std::stod(prop.as_string());
        else
            return prop.as_int64();
    }

    // A PLL/MMCM output phase in degrees.  Vivado takes -360..360 (both ends included); outside
    // that range it drops the value with a critical warning (Netlist 29-72) and programs a phase
    // of 0, so the clock is not the one the design asks for.  Stop instead.
    double clkout_phase(CellInfo *ci, const std::string &name)
    {
        const double phase = float_or_default(ci, name + "_PHASE", 0);
        const bool phase_in_range = phase >= -360 && phase <= 360;
        if (!phase_in_range)
            log_error("%s: %s_PHASE must be in the range -360..360 (got %.10g)\n", ctx->nameOf(ci), name.c_str(),
                      phase);
        return phase;
    }

    // Counter settings of one divider, as Vivado 2026.1 programs them (measured against it for
    // DIVIDE 2..128 x DUTY_CYCLE 0.05..0.95 x PHASE -360..360 in the PLL and the MMCM).  HIGH and LOW
    // follow the duty cycle at half-cycle resolution (EDGE is the half cycle), clamped so each phase
    // keeps at least one cycle; the six bit fields drop the 64 of a 128 divide exactly as Vivado does.
    // The phase is in eighths of a VCO period: the MMCM rounds to the nearest eighth, the PLL
    // truncates and saturates at 511; a negative phase is taken modulo 360.
    struct ClkoutCounter
    {
        int high = 1, low = 1, phasemux = 0, delaytime = 0;
        bool no_count = false, edge = false;
    };

    ClkoutCounter calc_clkout_counter(double divide, double duty, double phase, bool is_mmcm)
    {
        ClkoutCounter counter;
        const bool is_bypassed = divide <= 1;
        if (is_bypassed) {
            counter.no_count = true;
            return counter;
        }
        const int integer_divide = int(floor(divide));
        const int duty_fixed = int(round(duty * 100000));
        const int half_cycles_high = (2 * integer_divide * duty_fixed + 50000) / 100000;
        const int half_cycles = std::max(2, std::min(half_cycles_high, 2 * integer_divide - 1));
        counter.high = half_cycles / 2;
        counter.low = integer_divide - counter.high;
        counter.edge = half_cycles & 1;
        const double phase_normalized = phase < 0 ? phase + 360 : phase;
        const double eighths = phase_normalized / 360 * divide * 8;
        const int phase_eighths = is_mmcm ? int(floor(eighths + 0.5)) : std::min(int(floor(eighths)), 511);
        counter.phasemux = phase_eighths % 8;
        counter.delaytime = phase_eighths / 8;
        return counter;
    }

    void write_pll_clkout(const std::string &name, CellInfo *ci)
    {
        int frac = 0;
        double divide = float_or_default(ci, name + ((name == "CLKFBOUT") ? "_MULT" : "_DIVIDE"), 1);
        // Xilinx's default for every *_PHASE attribute is 0.0 degrees, not 1.
        // With the old default of 1, a PLL whose RTL omits CLKFBOUT_PHASE got
        // phase_eights = floor((1/360) * MULT * 8), which for MULT=48 is 1 --
        // so we emitted CLKFBOUT_CLKOUT1_PHASE_MUX = 001 and shifted the
        // FEEDBACK clock by an eighth of a VCO period.  Vivado emits 0.
        double phase = clkout_phase(ci, name);
        double duty = float_or_default(ci, name + "_DUTY_CYCLE", 0.5);
        const ClkoutCounter counter = calc_clkout_counter(divide, duty, phase, false);
        const int high = counter.high, low = counter.low, phasemux = counter.phasemux, delaytime = counter.delaytime;
        const bool edge = counter.edge, no_count = counter.no_count;
        if (divide > 1 && (name == "CLKOUT1" || name == "CLKFBOUT"))
            frac = floor(divide * 8) - floor(divide) * 8;
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = ci->getPort(ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);
            write_bit(name + "_CLKOUT2_EDGE[0]", edge);
            write_bit(name + "_CLKOUT2_NO_COUNT[0]", no_count);
            write_int_vector(name + "_CLKOUT2_DELAY_TIME[5:0]", delaytime, 6);
            if (frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", edge);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    void write_pll(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("PLLE2_ADV");
        write_bit("IN_USE");
        // ZINV_RST / ZINV_PWRDWN: the bit SET means the pin IS inverted.
        //
        // Do NOT "fix" this to !IS_*_INVERTED to match the other ZINV_ writes
        // in this file.  I did, on the reasoning that Vivado emits ZINV_RST for
        // a design whose RTL says .RST(~nrst) while we emitted nothing -- but
        // Vivado emits it because it ABSORBS that inverter into the PLL
        // (IS_RST_INVERTED=1), whereas we keep the inverter as fabric logic and
        // so must leave the bit clear.  Setting it added a second inversion:
        // HW-observed on the Sonata that the PLL then ran ONLY while the reset
        // button was held, i.e. exactly backwards.
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, id_IS_PWRDWN_INVERTED, false));
        write_bit("ZINV_RST", bool_or_default(ci->params, id_IS_RST_INVERTED, false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, id_IS_CLKINSEL_INVERTED, false));
        write_pll_clkout("DIVCLK", ci);
        write_pll_clkout("CLKFBOUT", ci);
        write_pll_clkout("CLKOUT0", ci);
        write_pll_clkout("CLKOUT1", ci);
        write_pll_clkout("CLKOUT2", ci);
        write_pll_clkout("CLKOUT3", ci);
        write_pll_clkout("CLKOUT4", ci);
        write_pll_clkout("CLKOUT5", ci);

        std::string comp = str_or_default(ci->params, id_COMPENSATION, "INTERNAL");
        push("COMPENSATION");
        if (comp == "INTERNAL") {
            // write_bit("INTERNAL");
            write_bit("Z_ZHOLD_OR_CLKIN_BUF");
        } else {
            NPNR_ASSERT_FALSE("unsupported compensation type");
        }
        pop();

        // PLLE2 lock & loop-filter configuration, from CLKFBOUT_MULT.
        //
        // These are PLL-specific tables harvested from Vivado golden
        // bitstreams (one minimal PLLE2_ADV design per CLKFBOUT_MULT 2..64,
        // disassembled with prjxray bit2fasm):
        //  - the values depend on CLKFBOUT_MULT only;
        //  - LKTABLE matches the XAPP888 MMCM lock table only up to
        //    MULT=10; from MULT=11 the PLL table diverges (delay fields
        //    saturate at 31, LockCnt decreases per MULT).  The MMCM lock
        //    table must not be reused here;
        //  - TABLE (loop filter: CP/RES/LFHF) varies across the whole
        //    range; the old hardcoded value is Vivado's for MULT=4 only.
        //    A wrong loop filter yields a clock clean enough for a
        //    free-running counter but too jittery for synchronous logic;
        //  - BANDWIDTH=HIGH programs the same filter as OPTIMIZED; LOW has
        //    its own filter table; LKTABLE does not depend on BANDWIDTH.
        // (Port of nextpnr-xilinx e33b5f1a/74357a79.)
        static const int64_t plle2_lock_table[63] = {
                0x31BE8FA401LL, 0x423E8FA401LL, 0x5AFE8FA401LL, 0x73BE8FA401LL,
                0x8C7E8FA401LL, 0x9CFE8FA401LL, 0xB5BE8FA401LL, 0xCE7E8FA401LL,
                0xE73E8FA401LL, 0xFFF84FA401LL, 0xFFF39FA401LL, 0xFFEEEFA401LL,
                0xFFEBCFA401LL, 0xFFE8AFA401LL, 0xFFE71FA401LL, 0xFFE3FFA401LL,
                0xFFE26FA401LL, 0xFFE0DFA401LL, 0xFFDF4FA401LL, 0xFFDDBFA401LL,
                0xFFDC2FA401LL, 0xFFDA9FA401LL, 0xFFD90FA401LL, 0xFFD90FA401LL,
                0xFFD77FA401LL, 0xFFD5EFA401LL, 0xFFD5EFA401LL, 0xFFD45FA401LL,
                0xFFD45FA401LL, 0xFFD2CFA401LL, 0xFFD2CFA401LL, 0xFFD2CFA401LL,
                0xFFD13FA401LL, 0xFFD13FA401LL, 0xFFD13FA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL,
                0xFFCFAFA401LL, 0xFFCFAFA401LL, 0xFFCFAFA401LL};
        static const uint16_t plle2_filter_optimized[63] = {
                0x0DC, 0x17C, 0x1FC, 0x1EC, 0x35C, 0x3AC, 0x3B4, 0x3F4,
                0x3DC, 0x3EC, 0x3F4, 0x3CC, 0x394, 0x3D4, 0x3D4, 0x3D4,
                0x3D4, 0x1D8, 0x1D8, 0x1D8, 0x1D8, 0x170, 0x170, 0x170,
                0x304, 0x304, 0x304, 0x304, 0x304, 0x304, 0x304, 0x304,
                0x108, 0x108, 0x108, 0x0A0, 0x0A0, 0x0A0, 0x0D0, 0x0A0,
                0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0,
                0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x130, 0x130, 0x130, 0x130,
                0x130, 0x130, 0x130, 0x090, 0x090, 0x090, 0x090};
        static const uint16_t plle2_filter_low[63] = {
                0x0BC, 0x09C, 0x0B4, 0x094, 0x094, 0x0A4, 0x0B8, 0x0B8,
                0x084, 0x084, 0x098, 0x098, 0x098, 0x098, 0x0A8, 0x0A8,
                0x0A8, 0x0A8, 0x0B0, 0x0B0, 0x0B0, 0x0B0, 0x0B0, 0x0B0,
                0x0B0, 0x0B0, 0x0B0, 0x0B0, 0x0B0, 0x088, 0x088, 0x088,
                0x088, 0x088, 0x088, 0x088, 0x088, 0x088, 0x088, 0x0F0,
                0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x0F0, 0x090, 0x090,
                0x090, 0x090, 0x090, 0x090, 0x090, 0x090, 0x090, 0x090,
                0x090, 0x090, 0x090, 0x090, 0x090, 0x090, 0x090};
        int pll_mult = (int)float_or_default(ci, "CLKFBOUT_MULT", 1);
        if (pll_mult < 2)
            pll_mult = 2;
        if (pll_mult > 64)
            pll_mult = 64;
        std::string pll_bw = str_or_default(ci->params, ctx->id("BANDWIDTH"), "OPTIMIZED");
        write_int_vector("FILTREG1_RESERVED[11:0]", 0x8, 12);
        write_int_vector("LKTABLE[39:0]", plle2_lock_table[pll_mult - 2], 40);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", (pll_bw == "LOW") ? plle2_filter_low[pll_mult - 2]
                                                         : plle2_filter_optimized[pll_mult - 2],
                         10);
        pop(2);
    }

    // The counter of an MMCME2 output from its CLKOUTn_DIVIDE(_F) / CLKFBOUT_MULT_F, DUTY_CYCLE and PHASE.
    ClkoutCounter calc_mmcm_clkout_counter(const std::string &name, CellInfo *ci, double &divide)
    {
        divide = float_or_default(
                ci, name + ((name == "CLKFBOUT") ? "_MULT_F" : (name == "CLKOUT0" ? "_DIVIDE_F" : "_DIVIDE")), 1);
        // Xilinx's default for every *_PHASE attribute is 0.0 degrees, not 1;
        // the PLL writer above was corrected, this is the matching MMCM fix
        // (nextpnr-xilinx#191).  An unset CLKOUT*_PHASE otherwise shifts the
        // clock by an eighth of a VCO period.
        const double phase = clkout_phase(ci, name);
        const double duty = float_or_default(ci, name + "_DUTY_CYCLE", 0.5);
        return calc_clkout_counter(divide, duty, phase, true);
    }

    void write_mmcm_clkout(const std::string &name, CellInfo *ci)
    {
        int frac = 0;
        double divide;
        const ClkoutCounter counter = calc_mmcm_clkout_counter(name, ci, divide);
        int high = counter.high, low = counter.low;
        const int phasemux = counter.phasemux, delaytime = counter.delaytime;
        const bool edge = counter.edge;
        const bool no_count = counter.no_count;
        if (divide > 1 && (name == "CLKOUT0" || name == "CLKFBOUT"))
            frac = floor(divide * 8) - floor(divide) * 8;
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = ci->getPort(ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            auto is_clkout_5_or_6 = name == "CLKOUT5" || name == "CLKOUT6";
            auto is_clkout0 = name == "CLKOUT0";
            auto is_clkfbout = name == "CLKFBOUT";

            // Fractional divide, as Vivado programs it (measured over DIVIDE_F = 2.125..127.875
            // for CLKOUT0 and CLKFBOUT): the integer counter splits on the parity of the integer
            // part, and the fall of the fractional output goes to the CLKOUT5 (CLKOUT0) or
            // CLKOUT6 (CLKFBOUT) fractional registers.
            // EDGE stays what the duty cycle gives for the integer part (the odd/even split at 50 %).
            const bool is_fractional_counter = (is_clkout0 || is_clkfbout) && frac != 0;
            bool frac_wf_r = false;
            if (is_fractional_counter) {
                const int integer_part = int(floor(divide));
                const bool integer_part_odd = integer_part & 1;
                const bool one_eighth = frac == 1;
                const bool integer_part_above_three = integer_part >= 4;
                if (integer_part_odd) {
                    high = (integer_part - 1) / 2;
                    low = one_eighth ? high - 1 : high;
                } else {
                    high = low = integer_part / 2 - 1;
                }
                frac_wf_r = !integer_part_odd;
                const bool frac_wf_f = integer_part_odd ? one_eighth : !(one_eighth && integer_part_above_three);
                // the fall moves with the phase: PHASE_MUX_F = (base + phase in eighths) mod 8
                const int frac_phase_mux_f = (4 * integer_part_odd + (frac >> 1) + phasemux) & 7;
                // CLKOUT0 controls CLKOUT5_CLKOUT2, CLKFBOUT controls CLKOUT6_CLKOUT2
                std::string frac_conf_name = is_clkout0 ? "CLKOUT5_CLKOUT2_" : "CLKOUT6_CLKOUT2_";
                write_bit(frac_conf_name + "FRACTIONAL_FRAC_WF_F[0]", frac_wf_f);
                write_int_vector(frac_conf_name + "FRACTIONAL_PHASE_MUX_F[2:0]", frac_phase_mux_f, 3);
            }

            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);

            auto phase_mux_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_PHASE_MUX_F[0]" : "_CLKOUT2_PHASE_MUX[0]");
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);

            auto edge_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_EDGE[0]" : "_CLKOUT2_EDGE[0]");
            write_bit(edge_feature, edge);

            auto no_count_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_NO_COUNT[0]" : "_CLKOUT2_NO_COUNT[0]");
            write_bit(no_count_feature, no_count);

            auto delay_time_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_DELAY_TIME[5:0]" : "_CLKOUT2_DELAY_TIME[5:0]");
            write_int_vector(delay_time_feature, delaytime, 6);

            if (!is_clkout_5_or_6 && frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", 1);
                write_bit(name + "_CLKOUT2_FRAC_WF_R[0]", frac_wf_r);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    // From openXC7
    void write_mmcm(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("MMCME2_ADV");
        write_bit("IN_USE");
        // FIXME: should be INV not ZINV (XRay error?)
        // Same convention as write_pll above: bit SET means the pin IS
        // inverted.  See the note there -- negating these was tried and is
        // wrong; it inverts RST and holds the block in reset.
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, id_IS_PWRDWN_INVERTED, false));
        write_bit("ZINV_RST", bool_or_default(ci->params, id_IS_RST_INVERTED, false));
        write_bit("ZINV_PSEN", bool_or_default(ci->params, id_IS_PSEN_INVERTED, false));
        write_bit("ZINV_PSINCDEC", bool_or_default(ci->params, id_IS_PSINCDEC_INVERTED, false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, id_IS_CLKINSEL_INVERTED, false));
        write_mmcm_clkout("DIVCLK", ci);
        write_mmcm_clkout("CLKFBOUT", ci);
        write_mmcm_clkout("CLKOUT0", ci);
        write_mmcm_clkout("CLKOUT1", ci);
        write_mmcm_clkout("CLKOUT2", ci);
        write_mmcm_clkout("CLKOUT3", ci);
        write_mmcm_clkout("CLKOUT4", ci);
        write_mmcm_clkout("CLKOUT5", ci);
        write_mmcm_clkout("CLKOUT6", ci);

        std::string comp = str_or_default(ci->params, id_COMPENSATION, "INTERNAL");
        push("COMP");
        if (comp == "INTERNAL" || comp == "ZHOLD") {
            // does not seem to make a difference in vivado
            // both modes set this bit
            write_bit("Z_ZHOLD");
        } else {
            log_error("unsupported COMPENSATION type '%s' for MMCM (supported compensation types: INTERNAL, ZHOLD)\n",
                      comp.c_str());
        }
        pop();

        double clkfbout_mult_f = float_or_default(ci, "CLKFBOUT_MULT_F", 5.000);
        // lk_table[] holds 63 rows; the 64-row filter_lookup*[] tables and it
        // are both read at [mult-1], so 63 is the honest upper bound.  The old
        // pair of tests rejected 0 and >63 but let every NEGATIVE value
        // through, and (int) of an out-of-range double is undefined behaviour
        // that differs by host -- x86-64 yields INT_MIN, arm64 saturates to
        // INT_MAX -- so the same netlist crashed on one machine and passed the
        // guards on another (nextpnr-xilinx#78).  So range-check the double
        // itself, before any cast: NaN and +-inf fail the finiteness test, and
        // [1, 64) is exactly the set that truncates to 1..63.  Name the
        // offending value as given.
        bool clkfbout_mult_in_range =
                std::isfinite(clkfbout_mult_f) && clkfbout_mult_f >= 1.0 && clkfbout_mult_f < 64.0;
        if (!clkfbout_mult_in_range)
            log_error("MMCME2_ADV: CLKFBOUT_MULT_F must be in the range 1..63 (got %.10g)\n", clkfbout_mult_f);
        int clkfbout_mult = (int)clkfbout_mult_f;
        write_int_vector("LKTABLE[39:0]", Xc7MMCM::lk_table[clkfbout_mult - 1], 40);

        std::string bandwidth = str_or_default(ci->params, id_BANDWIDTH, "OPTIMIZED");
        const uint16_t *filter_lookup;
        if (bandwidth == "LOW")
            filter_lookup = Xc7MMCM::filter_lookup_low;
        else if (bandwidth == "LOW_SS")
            filter_lookup = Xc7MMCM::filter_lookup_low_ss;
        else if (bandwidth == "HIGH")
            filter_lookup = Xc7MMCM::filter_lookup_high;
        else
            filter_lookup = Xc7MMCM::filter_lookup_optimized;
        // Same registers as write_pll: the loop filter value goes in TABLE and
        // FILTREG1_RESERVED holds 0x8 (what Vivado programs for every MULT and
        // BANDWIDTH).  POWER_REG is 0x9900 when a fractional counter is in use or an
        // output is shifted by something other than half a VCO period (PHASE_MUX not
        // a multiple of 4), 0x0100 otherwise.
        write_int_vector("FILTREG1_RESERVED[11:0]", 0x8, 12);
        bool needs_fractional_power = false;
        for (const char *name : {"CLKFBOUT", "CLKOUT0", "CLKOUT1", "CLKOUT2", "CLKOUT3", "CLKOUT4", "CLKOUT5", "CLKOUT6"}) {
            const bool is_used = std::string(name) == "CLKFBOUT" || ci->getPort(ctx->id(name)) != nullptr;
            double divide;
            const ClkoutCounter counter = calc_mmcm_clkout_counter(name, ci, divide);
            const bool is_fractional_divide = divide != floor(divide);
            const bool is_phase_off_half_cycle = counter.phasemux % 4 != 0;
            if (is_used && (is_fractional_divide || is_phase_off_half_cycle))
                needs_fractional_power = true;
        }
        write_int_vector("POWER_REG_POWER_REG_POWER_REG[15:0]", needs_fractional_power ? 0x9900 : 0x0100, 16);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", filter_lookup[clkfbout_mult - 1], 10);
        pop(2);
    }
    void write_dsp_cell(CellInfo *ci)
    {
        auto tile_name = uarch->tile_name(ci->bel.tile);
        auto tile_side = tile_name.at(4);
        push(tile_name);
        push("DSP48");
        auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
        auto dsp = stringf("DSP_%d", xy.y);
        push(dsp);

        // Whether the site inverts bit i of a bussed pin.  yosys gives the bus
        // one IS_<bus>_INVERTED vector; pack_constants() adds the per-bit
        // IS_<bus>[i]_INVERTED when it turns a constant 0 into VCC through the
        // inverter.  That is an inversion on top of the netlist's, so the two
        // compose by XOR.
        auto pin_inverted = [&](const std::string &bus, int i) {
            const bool bus_inverts = (int_or_default(ci->params, ctx->id("IS_" + bus + "_INVERTED"), 0) >> i) & 0x1;
            const bool packer_inverts =
                    bool_or_default(ci->params, ctx->idf("IS_%s[%d]_INVERTED", bus.c_str(), i), false);
            return bus_inverts != packer_inverts;
        };

        // The pins pack_dsps() tied to a constant inside the tile, with the
        // constant net each one was on.
        std::vector<std::pair<std::string, std::string>> tile_tied_pins;
        pool<std::string> tile_tied_pin_names;
        for (std::string const_net_name : {"GND", "VCC"}) {
            const auto attr_value = str_or_default(ci->attrs, ctx->id("DSP_" + const_net_name + "_PINS"), "");
            std::vector<std::string> pins;
            boost::split(pins, attr_value, boost::is_any_of(" "));
            for (auto &pin : pins) {
                if (pin.empty())
                    continue;
                tile_tied_pins.emplace_back(pin, const_net_name);
                tile_tied_pin_names.insert(pin);
            }
        }

        // A pin tied inside the tile gets its logical value from the tieoff
        // (see below), so its inverter stays off.  That is what Vivado writes:
        // it refuses a tieoff behind an active inverter (DRC PDCN-6).
        // Written as one vector, every bit stated, as Vivado writes it:
        //   ZIS_INMODE_INVERTED[4:0] = 5'b11111
        // Writing only the bits that happen to be set leaves the rest to
        // whatever the frame already holds, and these are inverted-sense
        // bits -- an unwritten ZIS_*_INVERTED means the line IS inverted,
        // which silently changes what the DSP computes.  Stating the whole
        // vector also makes a FASM comparison against Vivado meaningful.
        auto write_bus_zinv = [&](std::string name, int width) {
            std::vector<bool> zinv(width);
            for (int i = 0; i < width; i++) {
                const bool tied_in_tile = tile_tied_pin_names.count(name + std::to_string(i));
                zinv[i] = tied_in_tile || !pin_inverted(name, i);
            }
            write_vector(stringf("ZIS_%s_INVERTED[%d:0]", name.c_str(), width - 1), zinv);
        };

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto areg = int_or_default(ci->params, ctx->id("AREG"), 1);
        if (areg == 0 || areg == 2)
            write_bit("AREG_" + std::to_string(areg));

        auto ainput = str_or_default(ci->params, ctx->id("A_INPUT"), "DIRECT");
        if (ainput == "CASCADE")
            write_bit("A_INPUT[0]");

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto breg = int_or_default(ci->params, ctx->id("BREG"), 1);
        if (breg == 0 || breg == 2)
            write_bit("BREG_" + std::to_string(breg));

        auto binput = str_or_default(ci->params, ctx->id("B_INPUT"), "DIRECT");
        if (binput == "CASCADE")
            write_bit("B_INPUT[0]");

        // Tolerate both int and string types for interoperability purposes
        auto use_dport = boolstr_or_default(ci->params, ctx->id("USE_DPORT"), false);
        if (use_dport == true)
            write_bit("USE_DPORT[0]");

        auto use_simd = str_or_default(ci->params, ctx->id("USE_SIMD"), "ONE48");
        // FOUR12 needs both features: USE_SIMD_FOUR12 cuts at 12 and 36,
        // USE_SIMD_FOUR12_TWO24 cuts at 24 (shared with TWO24)
        if (use_simd == "TWO24" || use_simd == "FOUR12")
            write_bit("USE_SIMD_FOUR12_TWO24");
        if (use_simd == "FOUR12")
            write_bit("USE_SIMD_FOUR12");

        // PATTERN
        const size_t pattern_size = 48;
        std::vector<bool> pattern_vector(pattern_size, false);
        bool pattern_found = boolvec_populate(ci->params, ctx->id("PATTERN"), pattern_vector);
        if (pattern_found) {
            write_vector("PATTERN[47:0]", pattern_vector);
        }

        // As prjxray's 100-dsp-mskpat fuzzer tags them: AUTORESET_PATDET_RESET
        // for RESET_MATCH and RESET_NOT_MATCH, _RESET_NOT_MATCH for the second only.
        auto autoreset_patdet = str_or_default(ci->params, ctx->id("AUTORESET_PATDET"), "NO_RESET");
        const bool autoreset_on_pattern = autoreset_patdet == "RESET_MATCH" || autoreset_patdet == "RESET_NOT_MATCH";
        if (autoreset_on_pattern)
            write_bit("AUTORESET_PATDET_RESET");
        const bool autoreset_on_not_match = autoreset_patdet == "RESET_NOT_MATCH";
        if (autoreset_on_not_match)
            write_bit("AUTORESET_PATDET_RESET_NOT_MATCH");

        // MASK
        // Yosys gives us 48 bit, but prjxray only recognizes 46 bits
        // The most significant two bits seem to be zero, so let us just truncate them
        const size_t mask_size = 48;
        std::vector<bool> mask_vector(mask_size, true);
        // The primitive's default MASK is all ones, and Vivado writes it out
        // even when the netlist does not mention it -- yosys never does.
        // Leaving it unwritten is a zero mask, which is a different pattern
        // detector.
        boolvec_populate(ci->params, ctx->id("MASK"), mask_vector);
        mask_vector.resize(46);
        write_vector("MASK[45:0]", mask_vector);

        auto sel_mask = str_or_default(ci->params, ctx->id("SEL_MASK"), "MASK");
        if (sel_mask == "C")
            write_bit("SEL_MASK_C");
        if (sel_mask == "ROUNDING_MODE1")
            write_bit("SEL_MASK_ROUNDING_MODE1");
        if (sel_mask == "ROUNDING_MODE2")
            write_bit("SEL_MASK_ROUNDING_MODE2");

        const std::string sel_pattern = str_or_default(ci->params, ctx->id("SEL_PATTERN"), "PATTERN");
        const bool sel_pattern_is_c = sel_pattern == "C";
        if (sel_pattern_is_c)
            write_bit("SEL_PATTERN_C");

        // ZADREG means ADREG is off. Vivado also sets it when USE_DPORT is
        // false, whatever ADREG says: the pre-adder is unused, so that
        // pipeline register stays off. The fuzzer only tags the bit while
        // USE_DPORT is true (fuzzers/100-dsp-mskpat/generate.py).
        const bool adreg_on = bool_or_default(ci->params, ctx->id("ADREG"), true);
        const bool zadreg_off = !use_dport || !adreg_on;
        write_bit("ZADREG[0]", zadreg_off);
        // The pipeline registers a DSP48E1 does not name are on: UG479 gives every
        // *REG attribute the default 1, and Vivado builds that.  yosys writes them
        // out when it infers the DSP, so only an instantiated one leaves them unset.
        write_bit("ZALUMODEREG[0]", !bool_or_default(ci->params, ctx->id("ALUMODEREG"), true));
        // prjxray's AREG_2_ACASCREG_1 is the conjunction AREG == 2 && ACASCREG == 1
        // (fuzzers/100-dsp-mskpat/generate.py), not ACASCREG: its Z bit is set
        // unless both hold.  Vivado agrees -- it clears the bit only for AREG=2
        // with ACASCREG=1.  UG479's default for ACASCREG/BCASCREG is 1.
        auto acascreg = int_or_default(ci->params, ctx->id("ACASCREG"), 1);
        auto bcascreg = int_or_default(ci->params, ctx->id("BCASCREG"), 1);
        write_bit("ZAREG_2_ACASCREG_1", !(areg == 2 && acascreg == 1));
        write_bit("ZBREG_2_BCASCREG_1", !(breg == 2 && bcascreg == 1));
        write_bit("ZCARRYINREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINREG"), true));
        write_bit("ZCARRYINSELREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINSELREG"), true));
        write_bit("ZCREG[0]", !bool_or_default(ci->params, ctx->id("CREG"), true));
        // Same rule as ZADREG, for the D input register.
        const bool dreg_on = bool_or_default(ci->params, ctx->id("DREG"), true);
        const bool zdreg_off = !use_dport || !dreg_on;
        write_bit("ZDREG[0]", zdreg_off);
        write_bit("ZINMODEREG[0]", !bool_or_default(ci->params, ctx->id("INMODEREG"), true));
        write_bus_zinv("ALUMODE", 4);
        write_bus_zinv("INMODE", 5);
        write_bus_zinv("OPMODE", 7);
        write_bit("ZMREG[0]", !bool_or_default(ci->params, ctx->id("MREG"), true));
        write_bit("ZOPMODEREG[0]", !bool_or_default(ci->params, ctx->id("OPMODEREG"), true));
        write_bit("ZPREG[0]", !bool_or_default(ci->params, ctx->id("PREG"), true));
        write_bit("USE_DPORT[0]", boolstr_or_default(ci->params, ctx->id("USE_DPORT"), false));
        write_bit("ZIS_CLK_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED")));
        write_bit("ZIS_CARRYIN_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CARRYIN_INVERTED")));
        pop(2);

        // The tieoff enters the site ahead of the pin's inverter, which is
        // kept off above, so it has to carry the pin's logical value: the
        // constant the pin was on, flipped when the netlist inverts the pin.
        // A bussed pin's inversion is per bit -- "INMODE1" is bit 1 of
        // IS_INMODE_INVERTED -- so the bit index is split off its name.
        for (auto &tied : tile_tied_pins) {
            const std::string &pin = tied.first;
            const size_t digits_at = pin.find_last_not_of("0123456789") + 1;
            const bool pin_is_a_bus_bit = digits_at < pin.size();
            const bool inverted = pin_is_a_bus_bit
                                          ? pin_inverted(pin.substr(0, digits_at), std::stoi(pin.substr(digits_at)))
                                          : bool_or_default(ci->params, ctx->id("IS_" + pin + "_INVERTED"), false);
            const bool tied_to_vcc = tied.second == "VCC";
            const char *net_name = (tied_to_vcc != inverted) ? "VCC" : "GND";
            write_bit(stringf("%s_%s.DSP_%s_%c", dsp.c_str(), pin.c_str(), net_name, tile_side));
        }

        pop();
    }

    // A placed BUFR is configured from the cell, not from the route: the
    // pp_config entries only fire when a BUFR site is traversed as routing,
    // and they hardcode BUFR_DIVIDE.BYPASS.  Emitting from the cell makes the
    // configuration follow the instance, which is where the parameter lives.
    // (Port of nextpnr-xilinx 0b914578.)
    void write_bufr(CellInfo *ci)
    {
        // Site name is BUFR_X0Y<y> and the feature is BUFR_Y<y>, so the
        // site's y within the tile is the slot index.
        auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));

        // Vivado's BUFR_DIVIDE is a string: "BYPASS" or "1".."8".
        std::string divide = str_or_default(ci->params, ctx->id("BUFR_DIVIDE"), "BYPASS");
        std::string divide_feature;
        bool is_bypass = divide == "BYPASS" || divide.empty();
        bool is_divide_1_to_8 = divide.size() == 1 && divide[0] >= '1' && divide[0] <= '8';
        if (is_bypass) {
            divide_feature = "BYPASS";
        } else if (is_divide_1_to_8) {
            divide_feature = std::string("D") + divide[0];
        } else {
            // Emitting an undocumented feature instead would fail later in
            // fasm2frames with a FasmLookupError naming a bit.
            log_error("BUFR '%s' has BUFR_DIVIDE=\"%s\"; supported are BYPASS and 1..8\n",
                      ci->name.c_str(ctx), divide.c_str());
        }

        push(uarch->tile_name(ci->bel.tile));
        push("BUFR_Y" + std::to_string(xy.y));
        write_bit("IN_USE");
        push("BUFR_DIVIDE");
        write_bit(divide_feature);
        pop();
        pop();
        pop();
    }

    // A placed BUFIO emits nothing by default, so the buffer is never switched
    // on.  The clock reaches the site and leaves it through ordinary tile
    // routing (which write_pip already emits), but the enable is missing:
    // prjxray-db carries BUFIO_Y0..Y3.IN_USE (two bits per slot) and without
    // them the bitstream assembles with the bits clear -- dead on silicon, the
    // same silent failure shape as the RCLK2IO leaf on the BUFR side.  Emitted
    // from the cell for the same reason as write_bufr: the configuration
    // belongs to the instance.  (Port of nextpnr-xilinx c52d41b6.)
    void write_bufio(CellInfo *ci)
    {
        // Site name is BUFIO_X0Y<y> and the feature is BUFIO_Y<y>, where the
        // index is the site's y within its tile -- the same convention BUFR
        // uses above.
        auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));

        push(uarch->tile_name(ci->bel.tile));
        push("BUFIO_Y" + std::to_string(xy.y));
        write_bit("IN_USE");
        pop();
        pop();
    }

    void write_cfg()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel == BelId())
                continue;
            std::string tile_name = uarch->tile_name(ci->bel.tile);
            if (!boost::starts_with(tile_name, "CFG_CENTER_"))
                continue;

            push(tile_name);
            if (ci->type == id_BSCAN) {
                push("BSCAN");
                int chain = int_or_default(ci->params, id_JTAG_CHAIN, 1);
                bool invalid_chain = chain < 1 || 4 < chain;
                if (invalid_chain)
                    log_error("Invalid JTAG_CHAIN number of '%d'. Allowed values are: 1-4.\n", chain);
                write_bit("JTAG_CHAIN_" + std::to_string(chain));
                pop();
            }
            if (ci->type == id_DCIRESET_DCIRESET)
                write_bit("DCIRESET.ENABLED");
            if (ci->type == id_ICAP_ICAP) {
                push("ICAP");
                std::string width = str_or_default(ci->params, id_ICAP_WIDTH, "X32");
                bool invalid_width = width != "X32" && width != "X16" && width != "X8";
                if (invalid_width)
                    log_error("Unknown ICAP_WIDTH of '%s'. Allowed values are: X32, X16 and X8.\n", width.c_str());
                if (width == "X16")
                    write_bit("ICAP_WIDTH_X16");
                if (width == "X8")
                    write_bit("ICAP_WIDTH_X8");
                pop();
            }
            if (ci->type == id_STARTUP_STARTUP) {
                std::string prog_usr = str_or_default(ci->params, id_PROG_USR, "FALSE");
                bool invalid_prog_usr = prog_usr != "TRUE" && prog_usr != "FALSE";
                if (invalid_prog_usr)
                    log_error("Invalid PROG_USR attribute in STARTUPE2 of '%s'. Allowed values are: TRUE, FALSE.\n",
                              prog_usr.c_str());
                write_bit("STARTUP.PROG_USR", prog_usr == "TRUE");
                NetInfo *usrclk = ci->getPort(id_USRCCLKO);
                bool usrcclko_connected =
                        usrclk != nullptr && usrclk->driver.cell != nullptr &&
                        !usrclk->driver.cell->type.in(id_PSEUDO_GND, id_PSEUDO_VCC);
                write_bit("STARTUP.USRCCLKO_CONNECTED", usrcclko_connected);
            }
            pop();
        }
    }

    void write_ibufds_gte2(CellInfo * ci)
    {
        const std::string tile_name = uarch->tile_name(ci->bel.tile);
        push(tile_name);
        Loc siteLoc = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
        push("IBUFDS_GTE2_Y" + std::to_string(siteLoc.y));
        // Vivado sets IN_USE only when the reference clock goes to a
        // transceiver and nowhere else: O to GTREFCLK0/1 of the COMMON or of a
        // channel of this quad, and neither O nor ODIV2 to a fabric clock
        // buffer.  With a fabric load it stays clear, at any CEB (#97), and
        // writing it there is what silenced the factory UART in #86.  pack_gt
        // has already disconnected the hardwired GTREFCLK inputs and recorded
        // them on the GT cells, so what is left on O and ODIV2 are the fabric
        // loads.  Xilinx numbers the sites of a quad per column: COMMON
        // X<n>Y<q>, IBUFDS_GTE2 X<n>Y<2q> and Y<2q+1>, channels X<n>Y<4q> to
        // Y<4q+3>.
        const SiteIndex buf_site = uarch->get_bel_site(ci->bel);
        const auto &buf_site_data = uarch->tile_extra_data(buf_site.tile)->sites[buf_site.site];
        const IdString refclk_used_attr = ctx->id(siteLoc.y == 0 ? "_GTREFCLK0_USED" : "_GTREFCLK1_USED");
        bool drives_gt_refclk = false;
        for (auto &cell : ctx->cells) {
            const CellInfo *gt = cell.second.get();
            const bool is_placed_gt =
                    gt->type.in(id_GTPE2_COMMON, id_GTXE2_COMMON, id_GTPE2_CHANNEL, id_GTXE2_CHANNEL) &&
                    gt->bel != BelId();
            if (!is_placed_gt)
                continue;
            const SiteIndex gt_site = uarch->get_bel_site(gt->bel);
            const auto &gt_site_data = uarch->tile_extra_data(gt_site.tile)->sites[gt_site.site];
            const int gt_quad =
                    gt->type.in(id_GTPE2_CHANNEL, id_GTXE2_CHANNEL) ? gt_site_data.site_y / 4 : gt_site_data.site_y;
            const bool takes_this_refclk = gt_site_data.site_x == buf_site_data.site_x &&
                                           gt_quad == buf_site_data.site_y / 2 &&
                                           bool_or_default(gt->params, refclk_used_attr, false);
            if (takes_this_refclk)
                drives_gt_refclk = true;
        }
        const NetInfo *o = ci->getPort(id_O);
        const NetInfo *odiv2 = ci->getPort(ctx->id("ODIV2"));
        const bool drives_fabric = (o != nullptr && !o->users.empty()) || (odiv2 != nullptr && !odiv2->users.empty());
        write_bit("IN_USE", drives_gt_refclk && !drives_fabric);
        // CLKCM_CFG and CLKRCV_TRST are the bits that follow the site.
        // Xilinx sources write these as the STRINGS "TRUE"/"FALSE", which is
        // what the LiteEth PCS instantiates them with; bool_or_default wants a
        // number and aborts the whole fasm write with "Expecting numeric value
        // but got 'TRUE'".  The buffer then emitted IN_USE and nothing else,
        // which is a reference clock that is switched on and unconfigured.
        auto yes_no = [&](const char *name, bool dflt) {
            auto it = ci->params.find(ctx->id(name));
            if (it == ci->params.end())
                return dflt;
            if (it->second.is_string) {
                const std::string &v = it->second.as_string();
                if (v == "TRUE")
                    return true;
                if (v == "FALSE")
                    return false;
                log_error("%s: %s must be TRUE or FALSE, not '%s'\n", ci->name.c_str(ctx), name, v.c_str());
            }
            return it->second.as_int64() != 0;
        };
        auto clkcm_cfg = yes_no("CLKCM_CFG", true);
        if (!clkcm_cfg) log_warning("%s/%s: According to ug482, CLKCM_CFG should always be on\n",
                                    ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
        write_bit("CLKCM_CFG", clkcm_cfg);
        auto clkrcv_trst = yes_no("CLKRCV_TRST", true);
        if (!clkrcv_trst) log_warning("%s/%s: According to ug482, CLKRCV_TRST should always be on\n",
                                       ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
        write_bit("CLKRCV_TRST", clkrcv_trst);
        pop();
        // The input swing setting lives in the tile under the COMMON segment
        // prefix even though it belongs to this buffer, and it was only ever
        // written by the GTXE2_COMMON writer.  A design using the per-channel
        // CPLL instantiates no GTXE2_COMMON at all -- which is the whole point
        // of the open SGMII PHY -- so its reference clock came out with no
        // swing configured.
        //
        // The segment is GTXE2_COMMON on a GTX tile but GTPE2_COMMON on a GTP
        // one, and the database has no GTXE2_COMMON key under a GTP_COMMON
        // tile.  Hardcoding the GTX name therefore emitted an undeclared
        // feature for every Artix-7 GTP design, which fasm2frames rejects with
        // FasmLookupError (and fpga-as with an aborted map lookup).
        const bool tile_is_gtp = boost::starts_with(tile_name, "GTP_COMMON");
        push(tile_is_gtp ? "GTPE2_COMMON" : "GTXE2_COMMON");
        write_int_vector("IBUFDS_GTE2.CLKSWING_CFG[1:0]",
                         int_or_default(ci->params, ctx->id("CLKSWING_CFG"), 3), 2);
        pop(2);
    }
    // ENABLE_DRP of a COMMON follows the DRP of the channels of its quad,
    // not its own DRP: Vivado sets it when a channel of the quad drives DRPWE,
    // and leaves it clear for a COMMON whose DRP is in use with idle channels,
    // and for a channel that drives only DRPCLK, DRPEN or the address and
    // data.  Xilinx numbers the sites of a quad per column: COMMON X<n>Y<q>,
    // channels X<n>Y<4q> to Y<4q+3>.
    bool quad_channel_drp_write_used(const CellInfo *common)
    {
        const SiteIndex common_site = uarch->get_bel_site(common->bel);
        const auto &common_site_data = uarch->tile_extra_data(common_site.tile)->sites[common_site.site];
        const IdString drpwe = ctx->id("DRPWE");
        for (auto &cell : ctx->cells) {
            CellInfo *ch = cell.second.get();
            const bool is_placed_channel = ch->type.in(id_GTPE2_CHANNEL, id_GTXE2_CHANNEL) && ch->bel != BelId();
            if (!is_placed_channel)
                continue;
            const SiteIndex ch_site = uarch->get_bel_site(ch->bel);
            const auto &ch_site_data = uarch->tile_extra_data(ch_site.tile)->sites[ch_site.site];
            const NetInfo *we = ch->getPort(drpwe);
            const bool drives_drpwe_in_this_quad = ch_site_data.site_x == common_site_data.site_x &&
                                                   ch_site_data.site_y / 4 == common_site_data.site_y &&
                                                   we != nullptr && we->driver.cell != nullptr &&
                                                   !we->driver.cell->type.in(id_PSEUDO_GND, id_PSEUDO_VCC);
            if (drives_drpwe_in_this_quad)
                return true;
        }
        return false;
    }
    void write_gtp_pll(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));

        push("GTPE2_COMMON");
        write_bit("IN_USE");
        write_bit("ENABLE_DRP", quad_channel_drp_write_used(ci));
        write_bit("BOTH_GTREFCLK_USED", bool_or_default(ci->params, ctx->id("_BOTH_GTREFCLK_USED"), false));
        write_bit("GTREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK0_USED"), false));
        write_bit("GTREFCLK1_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK1_USED"), false));
        write_bit("GTGREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTGREFCLK_USED"), false));
        auto clkswing_cfg = int_or_default(ci->params, ctx->id("CLKSWING_CFG"), 3);
        if (clkswing_cfg != 3) log_warning("%s/%s: According to ug482, CLK should always be 0b11\n",
                                           ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
        write_int_vector("IBUFDS_GTE2.CLKSWING_CFG[1:0]", clkswing_cfg, 2);
        write_bit("INV_DRPCLK", bool_or_default(ci->params, ctx->id("IS_DRPCLK_INVERTED")));
        write_bit("INV_PLL0LOCKDETCLK", bool_or_default(ci->params, ctx->id("IS_PLL0LOCKDETCLK_INVERTED")));
        write_bit("INV_PLL1LOCKDETCLK", bool_or_default(ci->params, ctx->id("IS_PLL1LOCKDETCLK_INVERTED")));

        auto bias_cfg = int_or_default(ci->params, ctx->id("BIAS_CFG"), 0);
        write_int_vector("BIAS_CFG[63:0]", bias_cfg, 64);
        auto common_cfg = int_or_default(ci->params, ctx->id("COMMON_CFG"), 0);
        write_int_vector("COMMON_CFG[31:0]", common_cfg, 32);

        // according to ug482, these attributes contain magic undocumented and reserved wizard values.
        // Vivado writes the instance's value (PCIe asks PLL*_CFG = 0x1F024C); the defaults are unisim's.
        write_int_vector("PLL0_CFG[26:0]", int_or_default(ci->params, ctx->id("PLL0_CFG"), 0x01F03DC), 27);
        write_int_vector("PLL1_CFG[26:0]", int_or_default(ci->params, ctx->id("PLL1_CFG"), 0x01F03DC), 27);
        write_int_vector("PLL0_INIT_CFG[23:0]", int_or_default(ci->params, ctx->id("PLL0_INIT_CFG"), 0x00001E), 24);
        write_int_vector("PLL1_INIT_CFG[23:0]", int_or_default(ci->params, ctx->id("PLL1_INIT_CFG"), 0x00001E), 24);
        write_int_vector("PLL0_LOCK_CFG[8:0]", int_or_default(ci->params, ctx->id("PLL0_LOCK_CFG"), 0x1E8), 9);
        write_int_vector("PLL1_LOCK_CFG[8:0]", int_or_default(ci->params, ctx->id("PLL1_LOCK_CFG"), 0x1E8), 9);

        auto pll0_refclk_div = int_or_default(ci->params, ctx->id("PLL0_REFCLK_DIV"), 1);
        bool invalid_pll0_refclk_div = pll0_refclk_div < 1 || pll0_refclk_div > 2;
        if (invalid_pll0_refclk_div)
            log_error("PLL0_REFCLK_DIV can only be 1 or 2, but is: %d", pll0_refclk_div);
        write_bit("PLL0_REFCLK_DIV[4]", pll0_refclk_div == 1);
        auto pll1_refclk_div = int_or_default(ci->params, ctx->id("PLL1_REFCLK_DIV"), 1);
        bool invalid_pll1_refclk_div = pll1_refclk_div < 1 || pll1_refclk_div > 2;
        if (invalid_pll1_refclk_div)
            log_error("PLL1_REFCLK_DIV can only be 1 or 2, but is: %d", pll1_refclk_div);
        write_bit("PLL1_REFCLK_DIV[4]", pll1_refclk_div == 1);

        auto pll0_fbdiv = int_or_default(ci->params, ctx->id("PLL0_FBDIV"), 1);
        bool invalid_pll0_fbdiv = pll0_fbdiv < 1 || pll0_fbdiv > 5;
        if (invalid_pll0_fbdiv)
            log_error("PLL0_FBDIV can only be 1, 2, 3, 4 or 5, but is: %d", pll0_fbdiv);
        if (pll0_fbdiv == 1) write_bit("PLL0_FBDIV[4]");
        else write_int_vector("PLL0_FBDIV[1:0]", pll0_fbdiv - 2, 2);
        auto pll1_fbdiv = int_or_default(ci->params, ctx->id("PLL1_FBDIV"), 1);
        bool invalid_pll1_fbdiv = pll1_fbdiv < 1 || pll1_fbdiv > 5;
        if (invalid_pll1_fbdiv)
            log_error("PLL1_FBDIV can only be 1, 2, 3, 4 or 5, but is: %d", pll1_fbdiv);
        if (pll1_fbdiv == 1) write_bit("PLL1_FBDIV[4]");
        else write_int_vector("PLL1_FBDIV[1:0]", pll1_fbdiv - 2, 2);

        auto pll0_fbdiv_45 = int_or_default(ci->params, ctx->id("PLL0_FBDIV_45"), 4);
        bool invalid_pll0_fbdiv_45 = pll0_fbdiv_45 < 4 || pll0_fbdiv_45 > 5;
        if (invalid_pll0_fbdiv_45)
            log_error("PLL0_FBDIV_45 can only be 4 or 5, but is: %d", pll0_fbdiv);
        write_bit("PLL0_FBDIV_45[0]", pll0_fbdiv_45 == 5);
        auto pll1_fbdiv_45 = int_or_default(ci->params, ctx->id("PLL1_FBDIV_45"), 4);
        bool invalid_pll1_fbdiv_45 = pll1_fbdiv_45 < 4 || pll1_fbdiv_45 > 5;
        if (invalid_pll1_fbdiv_45)
            log_error("PLL1_FBDIV_45 can only be 4 or 5, but is: %d", pll1_fbdiv);
        write_bit("PLL1_FBDIV_45[0]", pll1_fbdiv_45 == 5);

        auto pll0_dmon_cfg = bool_or_default(ci->params, ctx->id("PLL0_DMON_CFG"), false);
        write_bit("PLL0_DMON_CFG[0]", pll0_dmon_cfg);
        auto pll1_dmon_cfg = bool_or_default(ci->params, ctx->id("PLL1_DMON_CFG"), false);
        write_bit("PLL1_DMON_CFG[0]", pll1_dmon_cfg);

        auto rsvd_attr0 = int_or_default(ci->params, ctx->id("RSVD_ATTR0"), 0);
        write_int_vector("RSVD_ATTR0[15:0]", rsvd_attr0, 16);
        auto rsvd_attr1 = int_or_default(ci->params, ctx->id("RSVD_ATTR1"), 0);
        write_int_vector("RSVD_ATTR1[15:0]", rsvd_attr1, 16);

        auto pll_clkout_cfg = int_or_default(ci->params, ctx->id("PLL_CLKOUT_CFG"), 0);
        write_int_vector("PLL_CLKOUT_CFG[7:0]", pll_clkout_cfg, 8);
        pop(); // GTPE2_COMMON

        pop(); // tile name
    }
    void write_gtp_channel(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("GTPE2_CHANNEL");

        write_bit("IN_USE");

        auto write_str_bool = [&](std::string attribute, std::string bit, std::string deflt = "FALSE") {
            auto val = str_or_default(ci->params, ctx->id(attribute), deflt);
            boost::algorithm::to_upper(val);
            write_bit(bit, val == "TRUE");
        };

        auto acjtag_debug_mode = bool_or_default(ci->params, ctx->id("ACJTAG_DEBUG_MODE"), false);
        write_bit("ACJTAG_DEBUG_MODE[0]", acjtag_debug_mode);
        auto acjtag_mode = bool_or_default(ci->params, ctx->id("ACJTAG_MODE"), false);
        write_bit("ACJTAG_MODE[0]", acjtag_mode);
        auto acjtag_reset = bool_or_default(ci->params, ctx->id("ACJTAG_RESET"), false);
        write_bit("ACJTAG_RESET[0]", acjtag_reset);

        auto adapt_cfg0 = int_or_default(ci->params, ctx->id("ADAPT_CFG0"), 0);
        write_int_vector("ADAPT_CFG0[19:0]", adapt_cfg0, 20);

        write_str_bool("ALIGN_COMMA_DOUBLE", "ALIGN_COMMA_DOUBLE");

        auto align_comma_enable = int_or_default(ci->params, ctx->id("ALIGN_COMMA_ENABLE"), 0b1111111111);
        write_int_vector("ALIGN_COMMA_ENABLE[9:0]", align_comma_enable, 10);

        auto align_comma_word = int_or_default(ci->params, ctx->id("ALIGN_COMMA_WORD"), 1);
        bool invalid_align_comma_word = align_comma_word < 1 || 2 < align_comma_word;
        if (invalid_align_comma_word)
            log_error("ALIGN_COMMA_WORD may only be 1 or 2, but is: %d\n", align_comma_word);
        if (align_comma_word == 1)
            write_bit("ALIGN_COMMA_WORD[0]");
        else
            write_bit("ALIGN_COMMA_WORD[1]");

        write_str_bool("ALIGN_MCOMMA_DET", "ALIGN_MCOMMA_DET");
        auto align_mcomma_value = int_or_default(ci->params, ctx->id("ALIGN_MCOMMA_VALUE"), 0b1010000011);
        write_int_vector("ALIGN_MCOMMA_VALUE[9:0]", align_mcomma_value, 10);

        write_str_bool("ALIGN_PCOMMA_DET", "ALIGN_PCOMMA_DET");
        auto align_pcomma_value = int_or_default(ci->params, ctx->id("ALIGN_PCOMMA_VALUE"), 0b0101111100);
        write_int_vector("ALIGN_PCOMMA_VALUE[9:0]", align_pcomma_value, 10);

        auto cbcc_data_source_sel = str_or_default(ci->params, ctx->id("CBCC_DATA_SOURCE_SEL"), "DECODED");
        if (cbcc_data_source_sel == "DECODED") write_bit("CBCC_DATA_SOURCE_SEL.DECODED");

        auto cfok_cfg = get_or_default(ci->params, ctx->id("CFOK_CFG"), Property(int64_t(0))).as_int64();
        write_int_vector("CFOK_CFG[42:0]", cfok_cfg, 43);
        auto cfok_cfg2 = int_or_default(ci->params, ctx->id("CFOK_CFG2"), 0);
        write_int_vector("CFOK_CFG2[6:0]", cfok_cfg2, 7);
        auto cfok_cfg3 = int_or_default(ci->params, ctx->id("CFOK_CFG3"), 0);
        write_int_vector("CFOK_CFG3[6:0]", cfok_cfg3, 7);
        auto cfok_cfg4 = bool_or_default(ci->params, ctx->id("CFOK_CFG4"), false);
        write_bit("CFOK_CFG4[0]", cfok_cfg4);
        auto cfok_cfg5 = int_or_default(ci->params, ctx->id("CFOK_CFG5"), 0);
        write_int_vector("CFOK_CFG5[1:0]", cfok_cfg5, 2);
        auto cfok_cfg6 = int_or_default(ci->params, ctx->id("CFOK_CFG6"), 0);
        write_int_vector("CFOK_CFG6[3:0]", cfok_cfg6, 4);

        write_str_bool("CHAN_BOND_KEEP_ALIGN", "CHAN_BOND_KEEP_ALIGN");
        auto chan_bond_max_skew = int_or_default(ci->params, ctx->id("CHAN_BOND_MAX_SKEW"), 0);
        bool invalid_chan_bond_max_skew = chan_bond_max_skew < 1 || 14 < chan_bond_max_skew;
        if (invalid_chan_bond_max_skew)
            log_error("CHAN_BOND_MAX_SKEW may only range from 1 to 14, but is: %d\n", chan_bond_max_skew);
        write_int_vector("CHAN_BOND_MAX_SKEW[3:0]", chan_bond_max_skew, 4);

        auto chan_bond_seq_1_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_ENABLE[3:0]", chan_bond_seq_1_enable, 4);
        auto chan_bond_seq_1_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_1[9:0]", chan_bond_seq_1_1, 10);
        auto chan_bond_seq_1_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_2[9:0]", chan_bond_seq_1_2, 10);
        auto chan_bond_seq_1_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_3[9:0]", chan_bond_seq_1_3, 10);
        auto chan_bond_seq_1_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_4[9:0]", chan_bond_seq_1_4, 10);

        write_str_bool("CHAN_BOND_SEQ_2_USE", "CHAN_BOND_SEQ_2_USE");
        auto chan_bond_seq_2_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_ENABLE[3:0]", chan_bond_seq_2_enable, 4);
        auto chan_bond_seq_2_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_1[9:0]", chan_bond_seq_2_1, 10);
        auto chan_bond_seq_2_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_2[9:0]", chan_bond_seq_2_2, 10);
        auto chan_bond_seq_2_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_3[9:0]", chan_bond_seq_2_3, 10);
        auto chan_bond_seq_2_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_4[9:0]", chan_bond_seq_2_4, 10);

        auto chan_bond_seq_len = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_LEN"), 0);
        bool invalid_chan_bond_seq_len = chan_bond_seq_len < 1 || 4 < chan_bond_seq_len;
        if (invalid_chan_bond_seq_len)
            log_error("CHAN_BOND_SEQ_LEN may only range from 1 to 4, but is: %d\n", chan_bond_seq_len);
        write_int_vector("CHAN_BOND_SEQ_LEN[1:0]", chan_bond_seq_len - 1, 2);

        auto clk_common_swing = bool_or_default(ci->params, ctx->id("CLK_COMMON_SWING"), false);
        write_bit("CLK_COMMON_SWING[0]", clk_common_swing);
        write_str_bool("CLK_COR_KEEP_IDLE", "CLK_COR_KEEP_IDLE");
        auto clk_cor_max_lat = int_or_default(ci->params, ctx->id("CLK_COR_MAX_LAT"), 0);
        write_int_vector("CLK_COR_MAX_LAT[5:0]", clk_cor_max_lat, 6);
        auto clk_cor_min_lat = int_or_default(ci->params, ctx->id("CLK_COR_MIN_LAT"), 0);
        write_int_vector("CLK_COR_MIN_LAT[5:0]", clk_cor_min_lat, 6);
        write_str_bool("CLK_COR_PRECEDENCE", "CLK_COR_PRECEDENCE");
        auto clk_cor_repeat_wait = int_or_default(ci->params, ctx->id("CLK_COR_REPEAT_WAIT"), 0);
        write_int_vector("CLK_COR_REPEAT_WAIT[4:0]", clk_cor_repeat_wait, 5);

        auto clk_cor_seq_1_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_1_ENABLE[3:0]", clk_cor_seq_1_enable, 4);
        auto clk_cor_seq_1_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_1"), 0);
        write_int_vector("CLK_COR_SEQ_1_1[9:0]", clk_cor_seq_1_1, 10);
        auto clk_cor_seq_1_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_2"), 0);
        write_int_vector("CLK_COR_SEQ_1_2[9:0]", clk_cor_seq_1_2, 10);
        auto clk_cor_seq_1_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_3"), 0);
        write_int_vector("CLK_COR_SEQ_1_3[9:0]", clk_cor_seq_1_3, 10);
        auto clk_cor_seq_1_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_4"), 0);
        write_int_vector("CLK_COR_SEQ_1_4[9:0]", clk_cor_seq_1_4, 10);

        write_str_bool("CLK_COR_SEQ_2_USE", "CLK_COR_SEQ_2_USE");
        auto clk_cor_seq_2_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_2_ENABLE[3:0]", clk_cor_seq_2_enable, 4);
        auto clk_cor_seq_2_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_1"), 0);
        write_int_vector("CLK_COR_SEQ_2_1[9:0]", clk_cor_seq_2_1, 10);
        auto clk_cor_seq_2_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_2"), 0);
        write_int_vector("CLK_COR_SEQ_2_2[9:0]", clk_cor_seq_2_2, 10);
        auto clk_cor_seq_2_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_3"), 0);
        write_int_vector("CLK_COR_SEQ_2_3[9:0]", clk_cor_seq_2_3, 10);
        auto clk_cor_seq_2_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_4"), 0);
        write_int_vector("CLK_COR_SEQ_2_4[9:0]", clk_cor_seq_2_4, 10);

        auto clk_cor_seq_len = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_LEN"), 0);
        bool invalid_clk_cor_seq_len = clk_cor_seq_len < 1 || 4 < clk_cor_seq_len;
        if (invalid_clk_cor_seq_len)
            log_error("CLK_COR_SEQ_LEN may only range from 1 to 4, but is: %d\n", clk_cor_seq_len);
        write_int_vector("CLK_COR_SEQ_LEN[1:0]", clk_cor_seq_len - 1, 2);

        write_str_bool("CLK_CORRECT_USE", "CLK_CORRECT_USE");

        write_str_bool("DEC_MCOMMA_DETECT", "DEC_MCOMMA_DETECT");
        write_str_bool("DEC_PCOMMA_DETECT", "DEC_PCOMMA_DETECT");
        write_str_bool("DEC_VALID_COMMA_ONLY", "DEC_VALID_COMMA_ONLY");

        auto dmonitor_cfg = int_or_default(ci->params, ctx->id("DMONITOR_CFG"), 0x008101);
        write_int_vector("DMONITOR_CFG[23:0]", dmonitor_cfg, 24);

        auto es_clk_phase_sel = bool_or_default(ci->params, ctx->id("ES_CLK_PHASE_SEL"), false);
        write_bit("ES_CLK_PHASE_SEL[0]", es_clk_phase_sel);
        auto es_control = int_or_default(ci->params, ctx->id("ES_CONTROL"), 0);
        write_int_vector("ES_CONTROL[5:0]", es_control, 6);
        write_str_bool("ES_ERRDET_EN", "ES_ERRDET_EN");
        write_str_bool("ES_EYE_SCAN_EN", "ES_EYE_SCAN_EN");
        auto es_horz_offset = int_or_default(ci->params, ctx->id("ES_HORZ_OFFSET"), 0x010);
        write_int_vector("ES_HORZ_OFFSET[11:0]", es_horz_offset, 12);
        auto es_pma_cfg = int_or_default(ci->params, ctx->id("ES_PMA_CFG"), 0);
        write_int_vector("ES_PMA_CFG[9:0]", es_pma_cfg, 10);
        auto es_prescale = int_or_default(ci->params, ctx->id("ES_PRESCALE"), 0);
        write_int_vector("ES_PRESCALE[4:0]", es_prescale, 5);
        auto es_qual_mask = int_or_default(ci->params, ctx->id("ES_QUAL_MASK"), 0);
        write_int_vector("ES_QUAL_MASK[79:0]", es_qual_mask, 80);
        auto es_qualifier = int_or_default(ci->params, ctx->id("ES_QUALIFIER"), 0);
        write_int_vector("ES_QUALIFIER[79:0]", es_qualifier, 80);
        auto es_sdata_mask = int_or_default(ci->params, ctx->id("ES_SDATA_MASK"), 0);
        write_int_vector("ES_SDATA_MASK[79:0]", es_sdata_mask, 80);
        auto es_vert_offset = int_or_default(ci->params, ctx->id("ES_VERT_OFFSET"), 0);
        write_int_vector("ES_VERT_OFFSET[8:0]", es_vert_offset, 9);

        auto fts_deskew_seq_enable = int_or_default(ci->params, ctx->id("FTS_DESKEW_SEQ_ENABLE"), 0b1111);
        write_int_vector("FTS_DESKEW_SEQ_ENABLE[3:0]", fts_deskew_seq_enable, 4);
        auto fts_lane_deskew_cfg = int_or_default(ci->params, ctx->id("FTS_LANE_DESKEW_CFG"), 0);
        write_int_vector("FTS_LANE_DESKEW_CFG[3:0]", fts_lane_deskew_cfg, 4);
        write_str_bool("FTS_LANE_DESKEW_EN", "FTS_LANE_DESKEW_EN");

        auto gearbox_mode = int_or_default(ci->params, ctx->id("GEARBOX_MODE"), 0);
        write_int_vector("GEARBOX_MODE[2:0]", gearbox_mode, 3);

        auto write_inv = [&](std::string name) {
            write_bit("INV_" + name, bool_or_default(ci->params, ctx->id("IS_" + name + "_INVERTED"), false));
        };
        // only these have been fuzzed yet,
        write_inv("DMONITORCLK");
        write_inv("DRPCLK");
        write_inv("RXUSRCLK");
        write_inv("SIGVALIDCLK");
        write_inv("TXPHDLYTSTCLK");
        write_inv("TXUSRCLK");
        write_inv("CLKRSVD0");
        write_inv("CLKRSVD1");
        write_inv("RXUSRCLK2");
        write_inv("TXUSRCLK2");

        auto loopback_cfg = bool_or_default(ci->params, ctx->id("LOOPBACK_CFG"), false);
        write_bit("LOOPBACK_CFG[0]", loopback_cfg);

        auto outrefclk_sel_inv = int_or_default(ci->params, ctx->id("OUTREFCLK_SEL_INV"), 0);
        write_int_vector("OUTREFCLK_SEL_INV[1:0]", outrefclk_sel_inv, 2);

        write_str_bool("PCS_PCIE_EN", "PCS_PCIE_EN");

        auto rsvd_attr = int_or_default(ci->params, ctx->id("PCS_RSVD_ATTR"), 0);
        write_int_vector("PCS_RSVD_ATTR[47:0]", rsvd_attr, 48);

        auto pd_trans_time_from_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_FROM_P2"), 0);
        write_int_vector("PD_TRANS_TIME_FROM_P2[11:0]", pd_trans_time_from_p2, 12);
        auto pd_trans_time_none_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_NONE_P2"), 0);
        write_int_vector("PD_TRANS_TIME_NONE_P2[7:0]", pd_trans_time_none_p2, 8);
        auto pd_trans_time_to_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_TO_P2"), 0);
        write_int_vector("PD_TRANS_TIME_TO_P2[7:0]", pd_trans_time_to_p2, 8);

        auto pma_loopback_cfg = bool_or_default(ci->params, ctx->id("PMA_LOOPBACK_CFG"), false);
        write_bit("PMA_LOOPBACK_CFG[0]", pma_loopback_cfg);
        auto pma_rsv = int_or_default(ci->params, ctx->id("PMA_RSV"), 0);
        write_int_vector("PMA_RSV[31:0]", pma_rsv, 32);
        auto pma_rsv2 = int_or_default(ci->params, ctx->id("PMA_RSV2"), 0);
        write_int_vector("PMA_RSV2[31:0]", pma_rsv2, 32);
        auto pma_rsv3 = int_or_default(ci->params, ctx->id("PMA_RSV3"), 0);
        write_int_vector("PMA_RSV3[1:0]", pma_rsv3, 2);
        auto pma_rsv4 = int_or_default(ci->params, ctx->id("PMA_RSV4"), 0);
        write_int_vector("PMA_RSV4[3:0]", pma_rsv4, 4);
        auto pma_rsv5 = bool_or_default(ci->params, ctx->id("PMA_RSV5"), false);
        write_bit("PMA_RSV5[0]", pma_rsv5);
        auto pma_rsv6 = bool_or_default(ci->params, ctx->id("PMA_RSV6"), false);
        write_bit("PMA_RSV6[0]", pma_rsv6);
        auto pma_rsv7 = bool_or_default(ci->params, ctx->id("PMA_RSV7"), false);
        write_bit("PMA_RSV7[0]", pma_rsv7);

        auto rx_bias_cfg = int_or_default(ci->params, ctx->id("RX_BIAS_CFG"), 0);
        write_int_vector("RX_BIAS_CFG[15:0]", rx_bias_cfg, 16);

        auto rx_buffer_cfg = int_or_default(ci->params, ctx->id("RX_BUFFER_CFG"), 0);
        write_int_vector("RX_BUFFER_CFG[5:0]", rx_buffer_cfg, 6);
        auto rx_clkmux_en = bool_or_default(ci->params, ctx->id("RX_CLKMUX_EN"), false);
        write_bit("RX_CLKMUX_EN[0]", rx_clkmux_en);
        auto rx_cm_sel = int_or_default(ci->params, ctx->id("RX_CM_SEL"), 0);
        write_int_vector("RX_CM_SEL[1:0]", rx_cm_sel, 2);
        auto rx_cm_trim = int_or_default(ci->params, ctx->id("RX_CM_TRIM"), 0);
        write_int_vector("RX_CM_TRIM[3:0]", rx_cm_trim, 4);
        auto rx_data_width = int_or_default(ci->params, ctx->id("RX_DATA_WIDTH"), 0);
        switch (rx_data_width) {
            case 16:
                rx_data_width = 2; break;
            case 20:
                rx_data_width = 3; break;
            case 32:
                rx_data_width = 4; break;
            case 40:
                rx_data_width = 5; break;
            default:
                log_error("Invalid RX_DATA_WIDTH parameter '%d' for GTPE2_CHANNEL instance %s\n", rx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("RX_DATA_WIDTH[2:0]", rx_data_width, 3);
        auto rx_ddi_sel = int_or_default(ci->params, ctx->id("RX_DDI_SEL"), 0);
        write_int_vector("RX_DDI_SEL[5:0]", rx_ddi_sel, 6);
        auto rx_debug_cfg = int_or_default(ci->params, ctx->id("RX_DEBUG_CFG"), 0);
        write_int_vector("RX_DEBUG_CFG[13:0]", rx_debug_cfg, 14);
        write_str_bool("RX_DEFER_RESET_BUF_EN", "RX_DEFER_RESET_BUF_EN");
        write_str_bool("RX_DISPERR_SEQ_MATCH", "RX_DISPERR_SEQ_MATCH");
        auto rx_os_cfg = int_or_default(ci->params, ctx->id("RX_OS_CFG"), 0);
        write_int_vector("RX_OS_CFG[12:0]", rx_os_cfg, 13);
        auto rx_sig_valid_dly = int_or_default(ci->params, ctx->id("RX_SIG_VALID_DLY"), 0) - 1;
        write_int_vector("RX_SIG_VALID_DLY[4:0]", rx_sig_valid_dly, 5);
        auto rx_xclk_sel = str_or_default(ci->params, ctx->id("RX_XCLK_SEL"), "RXUSR");
        bool invalid_rx_xclk_sel = rx_xclk_sel != "RXUSR" && rx_xclk_sel != "RXREC";
        if (invalid_rx_xclk_sel)
            log_error("RX_XCLK_SEL may only have values 'RXREC' or 'RXUSR' but is: '%s'\n", rx_xclk_sel.c_str());
        write_bit("RX_XCLK_SEL.RXUSR", rx_xclk_sel == "RXUSR");
        auto rx_clk25_div = int_or_default(ci->params, ctx->id("RX_CLK25_DIV"), 0) - 1;
        write_int_vector("RX_CLK25_DIV[4:0]", rx_clk25_div, 5);

        auto rxbuf_addr_mode = str_or_default(ci->params, ctx->id("RXBUF_ADDR_MODE"), "PMA");
        bool invalid_rxbuf_addr_mode = rxbuf_addr_mode != "FULL" && rxbuf_addr_mode != "FAST";
        if (invalid_rxbuf_addr_mode)
            log_error("RXBUF_ADDR_MODE may only have values 'FULL' or 'FAST' but is: '%s'\n", rxbuf_addr_mode.c_str());
        write_bit("RXBUF_ADDR_MODE.FAST", rxbuf_addr_mode == "FAST");
        auto rxbuf_eidle_hi_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_HI_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_HI_CNT[3:0]", rxbuf_eidle_hi_cnt, 4);
        auto rxbuf_eidle_lo_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_LO_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_LO_CNT[3:0]", rxbuf_eidle_lo_cnt, 4);
        write_str_bool("RXBUF_EN", "RXBUF_EN", "TRUE");
        write_str_bool("RXBUF_RESET_ON_CB_CHANGE", "RXBUF_RESET_ON_CB_CHANGE", "TRUE");
        write_str_bool("RXBUF_RESET_ON_COMMAALIGN", "RXBUF_RESET_ON_COMMAALIGN");
        write_str_bool("RXBUF_RESET_ON_EIDLE", "RXBUF_RESET_ON_EIDLE");
        write_str_bool("RXBUF_RESET_ON_RATE_CHANGE", "RXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        write_str_bool("RXBUF_THRESH_OVRD", "RXBUF_THRESH_OVRD");
        auto rxbuf_thresh_ovflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_OVFLW"), 0);
        write_int_vector("RXBUF_THRESH_OVFLW[5:0]", rxbuf_thresh_ovflw, 6);
        auto rxbuf_thresh_undflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_UNDFLW"), 0);
        write_int_vector("RXBUF_THRESH_UNDFLW[5:0]", rxbuf_thresh_undflw, 6);
        auto rxbufreset_time = int_or_default(ci->params, ctx->id("RXBUFRESET_TIME"), 0);
        write_int_vector("RXBUFRESET_TIME[4:0]", rxbufreset_time, 5);

        auto rxcdr_cfg = get_or_default(ci->params, ctx->id("RXCDR_CFG"),
            Property("00000000000000000000000000000000000000000000000000000000000000000000000000000000000"));
        write_vector("RXCDR_CFG[82:0]", bits_from_string_default(rxcdr_cfg, 83));
        auto rxcdr_fr_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_FR_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_FR_RESET_ON_EIDLE[0]", rxcdr_fr_reset_on_eidle);
        auto rxcdr_ph_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_PH_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_PH_RESET_ON_EIDLE[0]", rxcdr_ph_reset_on_eidle);
        auto rxcdr_hold_during_eidle = bool_or_default(ci->params, ctx->id("RXCDR_HOLD_DURING_EIDLE"), false);
        write_bit("RXCDR_HOLD_DURING_EIDLE[0]", rxcdr_hold_during_eidle);
        auto rxcdr_lock_cfg = int_or_default(ci->params, ctx->id("RXCDR_LOCK_CFG"), 0);
        write_int_vector("RXCDR_LOCK_CFG[5:0]", rxcdr_lock_cfg, 6);
        auto rxcdrfreqreset_time = int_or_default(ci->params, ctx->id("RXCDRFREQRESET_TIME"), 0);
        write_int_vector("RXCDRFREQRESET_TIME[4:0]", rxcdrfreqreset_time, 5);
        auto rxcdrphreset_time = int_or_default(ci->params, ctx->id("RXCDRPHRESET_TIME"), 0);
        write_int_vector("RXCDRPHRESET_TIME[4:0]", rxcdrphreset_time, 5);

        auto rxdly_cfg = int_or_default(ci->params, ctx->id("RXDLY_CFG"), 0);
        write_int_vector("RXDLY_CFG[15:0]", rxdly_cfg, 16);
        auto rxdly_lcfg = int_or_default(ci->params, ctx->id("RXDLY_LCFG"), 0);
        write_int_vector("RXDLY_LCFG[8:0]", rxdly_lcfg, 9);
        auto rxdly_tap_cfg = int_or_default(ci->params, ctx->id("RXDLY_TAP_CFG"), 0);
        write_int_vector("RXDLY_TAP_CFG[15:0]", rxdly_tap_cfg, 16);

        write_str_bool("RXGEARBOX_EN", "RXGEARBOX_EN");

        auto rxiscanreset_time = int_or_default(ci->params, ctx->id("RXISCANRESET_TIME"), 0);
        write_int_vector("RXISCANRESET_TIME[4:0]", rxiscanreset_time, 5);

        auto rxlpm_bias_startup_disable = bool_or_default(ci->params, ctx->id("RXLPM_BIAS_STARTUP_DISABLE"), false);
        write_bit("RXLPM_BIAS_STARTUP_DISABLE[0]", rxlpm_bias_startup_disable);
        auto rxlpm_cfg = int_or_default(ci->params, ctx->id("RXLPM_CFG"), 0);
        write_int_vector("RXLPM_CFG[3:0]", rxlpm_cfg, 4);
        auto rxlpm_cfg1 = int_or_default(ci->params, ctx->id("RXLPM_CFG1"), 0);
        write_int_vector("RXLPM_CFG1[0]", rxlpm_cfg1, 1);
        auto rxlpm_cm_cfg = int_or_default(ci->params, ctx->id("RXLPM_CM_CFG"), 0);
        write_int_vector("RXLPM_CM_CFG[0]", rxlpm_cm_cfg, 1);
        auto rxlpm_gc_cfg = int_or_default(ci->params, ctx->id("RXLPM_GC_CFG"), 0);
        write_int_vector("RXLPM_GC_CFG[8:0]", rxlpm_gc_cfg, 9);
        auto rxlpm_gc_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_GC_CFG2"), 0);
        write_int_vector("RXLPM_GC_CFG2[2:0]", rxlpm_gc_cfg2, 3);
        auto rxlpm_hf_cfg = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG"), 0);
        write_int_vector("RXLPM_HF_CFG[13:0]", rxlpm_hf_cfg, 14);
        auto rxlpm_hf_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG2"), 0);
        write_int_vector("RXLPM_HF_CFG2[4:0]", rxlpm_hf_cfg2, 5);
        auto rxlpm_hf_cfg3 = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG3"), 0);
        write_int_vector("RXLPM_HF_CFG3[3:0]", rxlpm_hf_cfg3, 4);
        auto rxlpm_hold_during_eidle = bool_or_default(ci->params, ctx->id("RXLPM_HOLD_DURING_EIDLE"), false);
        write_bit("RXLPM_HOLD_DURING_EIDLE[0]", rxlpm_hold_during_eidle);
        auto rxlpm_incm_cfg = bool_or_default(ci->params, ctx->id("RXLPM_INCM_CFG"), false);
        write_bit("RXLPM_INCM_CFG[0]", rxlpm_incm_cfg);
        auto rxlpm_ipcm_cfg = bool_or_default(ci->params, ctx->id("RXLPM_IPCM_CFG"), false);
        write_bit("RXLPM_IPCM_CFG[0]", rxlpm_ipcm_cfg);
        auto rxlpm_lf_cfg = int_or_default(ci->params, ctx->id("RXLPM_LF_CFG"), 0);
        write_int_vector("RXLPM_LF_CFG[17:0]", rxlpm_lf_cfg, 18);
        auto rxlpm_lf_cfg2 = int_or_default(ci->params, ctx->id("RXLPM_LF_CFG2"), 0);
        write_int_vector("RXLPM_LF_CFG2[4:0]", rxlpm_lf_cfg2, 5);
        auto rxlpm_osint_cfg = int_or_default(ci->params, ctx->id("RXLPM_OSINT_CFG"), 0);
        write_int_vector("RXLPM_OSINT_CFG[2:0]", rxlpm_osint_cfg, 3);
        auto rxlpmreset_time = int_or_default(ci->params, ctx->id("RXLPMRESET_TIME"), 0);
        write_int_vector("RXLPMRESET_TIME[6:0]", rxlpmreset_time, 7);

        auto rxoob_cfg = int_or_default(ci->params, ctx->id("RXOOB_CFG"), 0);
        write_int_vector("RXOOB_CFG[6:0]", rxoob_cfg, 7);
        auto rxoob_clk_cfg = str_or_default(ci->params, ctx->id("RXOOB_CLK_CFG"), "PMA");
        bool invalid_rxoob_clk_cfg = rxoob_clk_cfg != "FABRIC" && rxoob_clk_cfg != "PMA";
        if (invalid_rxoob_clk_cfg)
            log_error("RXOOB_CLK_CFG may only have values 'FABRIC' or 'PMA' but is: '%s'\n", rxoob_clk_cfg.c_str());
        write_bit("RXOOB_CLK_CFG.FABRIC", rxoob_clk_cfg == "FABRIC");

        auto rxoscalreset_time = int_or_default(ci->params, ctx->id("RXOSCALRESET_TIME"), 0);
        write_int_vector("RXOSCALRESET_TIME[4:0]", rxoscalreset_time, 5);
        auto rxoscalreset_timeout = int_or_default(ci->params, ctx->id("RXOSCALRESET_TIMEOUT"), 0);
        write_int_vector("RXOSCALRESET_TIMEOUT[4:0]", rxoscalreset_timeout, 5);

        auto rxout_div = std::log2(int_or_default(ci->params, ctx->id("RXOUT_DIV"), 1));
        write_int_vector("RXOUT_DIV[1:0]", rxout_div, 2);

        auto rxpcsreset_time = int_or_default(ci->params, ctx->id("RXPCSRESET_TIME"), 0);
        write_int_vector("RXPCSRESET_TIME[4:0]", rxpcsreset_time, 5);

        auto rxph_cfg = int_or_default(ci->params, ctx->id("RXPH_CFG"), 0);
        write_int_vector("RXPH_CFG[23:0]", rxph_cfg, 24);
        auto rxph_monitor_sel = int_or_default(ci->params, ctx->id("RXPH_MONITOR_SEL"), 0);
        write_int_vector("RXPH_MONITOR_SEL[4:0]", rxph_monitor_sel, 5);
        auto rxphdly_cfg = int_or_default(ci->params, ctx->id("RXPHDLY_CFG"), 0);
        write_int_vector("RXPHDLY_CFG[23:0]", rxphdly_cfg, 24);

        auto rxpi_cfg0 = int_or_default(ci->params, ctx->id("RXPI_CFG0"), 0);
        write_int_vector("RXPI_CFG0[2:0]", rxpi_cfg0, 3);
        auto rxpi_cfg1 = bool_or_default(ci->params, ctx->id("RXPI_CFG1"), false);
        write_bit("RXPI_CFG1[0]", rxpi_cfg1);
        auto rxpi_cfg2 = bool_or_default(ci->params, ctx->id("RXPI_CFG2"), false);
        write_bit("RXPI_CFG2[0]", rxpi_cfg2);

        auto rxpmareset_time = int_or_default(ci->params, ctx->id("RXPMARESET_TIME"), 0);
        write_int_vector("RXPMARESET_TIME[4:0]", rxpmareset_time, 5);

        auto rxprbs_err_loopback = bool_or_default(ci->params, ctx->id("RXPRBS_ERR_LOOPBACK"), false);
        write_bit("RXPRBS_ERR_LOOPBACK[0]", rxprbs_err_loopback);

        auto rxslide_auto_wait = int_or_default(ci->params, ctx->id("RXSLIDE_AUTO_WAIT"), 7);
        write_int_vector("RXSLIDE_AUTO_WAIT[3:0]", rxslide_auto_wait, 4);
        auto rxslide_mode = str_or_default(ci->params, ctx->id("RXSLIDE_MODE"), "OFF");
        bool invalid_rxslide_mode =
                rxslide_mode != "OFF" && rxslide_mode != "AUTO" && rxslide_mode != "PCS" && rxslide_mode != "PMA";
        if (invalid_rxslide_mode)
            log_error("RXSLIDE_MODE may only have values 'OFF', 'AUTO', 'PCS' or 'PMA' but is: '%s'\n", rxslide_mode.c_str());
        write_bit("RXSLIDE_MODE.AUTO", rxslide_mode == "AUTO");
        write_bit("RXSLIDE_MODE.PCS",  rxslide_mode == "PCS");
        write_bit("RXSLIDE_MODE.PMA",  rxslide_mode == "PMA");

        auto rxsync_multilane = bool_or_default(ci->params, ctx->id("RXSYNC_MULTILANE"), false);
        write_bit("RXSYNC_MULTILANE[0]", rxsync_multilane);
        auto rxsync_ovrd = bool_or_default(ci->params, ctx->id("RXSYNC_OVRD"), false);
        write_bit("RXSYNC_OVRD[0]", rxsync_ovrd);
        auto rxsync_skip_da = bool_or_default(ci->params, ctx->id("RXSYNC_SKIP_DA"), false);
        write_bit("RXSYNC_SKIP_DA[0]", rxsync_skip_da);

        auto sas_max_com = int_or_default(ci->params, ctx->id("SAS_MAX_COM"), 0);
        write_int_vector("SAS_MAX_COM[6:0]", sas_max_com, 7);
        auto sas_min_com = int_or_default(ci->params, ctx->id("SAS_MIN_COM"), 0);
        write_int_vector("SAS_MIN_COM[6:0]", sas_min_com, 7);

        auto sata_burst_seq_len = int_or_default(ci->params, ctx->id("SATA_BURST_SEQ_LEN"), 0);
        write_int_vector("SATA_BURST_SEQ_LEN[3:0]", sata_burst_seq_len, 4);
        auto sata_burst_val = int_or_default(ci->params, ctx->id("SATA_BURST_VAL"), 0);
        write_int_vector("SATA_BURST_VAL[2:0]", sata_burst_val, 3);
        auto sata_eidle_val = int_or_default(ci->params, ctx->id("SATA_EIDLE_VAL"), 0);
        write_int_vector("SATA_EIDLE_VAL[2:0]", sata_eidle_val, 3);
        auto sata_max_burst = int_or_default(ci->params, ctx->id("SATA_MAX_BURST"), 0);
        write_int_vector("SATA_MAX_BURST[5:0]", sata_max_burst, 6);
        auto sata_max_init = int_or_default(ci->params, ctx->id("SATA_MAX_INIT"), 0);
        write_int_vector("SATA_MAX_INIT[5:0]", sata_max_init, 6);
        auto sata_max_wake = int_or_default(ci->params, ctx->id("SATA_MAX_WAKE"), 0);
        write_int_vector("SATA_MAX_WAKE[5:0]", sata_max_wake, 6);
        auto sata_min_burst = int_or_default(ci->params, ctx->id("SATA_MIN_BURST"), 0);
        write_int_vector("SATA_MIN_BURST[5:0]", sata_min_burst, 6);
        auto sata_min_init = int_or_default(ci->params, ctx->id("SATA_MIN_INIT"), 0);
        write_int_vector("SATA_MIN_INIT[5:0]", sata_min_init, 6);
        auto sata_min_wake = int_or_default(ci->params, ctx->id("SATA_MIN_WAKE"), 0);
        write_int_vector("SATA_MIN_WAKE[5:0]", sata_min_wake, 6);
        auto sata_pll_cfg = str_or_default(ci->params, ctx->id("SATA_PLL_CFG"), "VCO_3000MHZ");
        bool invalid_sata_pll_cfg =
                sata_pll_cfg != "VCO_3000MHZ" && sata_pll_cfg != "VCO_1500MHZ" && sata_pll_cfg != "VCO_750MHZ";
        if (invalid_sata_pll_cfg)
            log_error("SATA_PLL_CFG may only have values 'VCO_3000MHZ', 'VCO_1500MHZ' or 'VCO_750MHZ' but is: '%s'\n", sata_pll_cfg.c_str());
        write_bit("SATA_PLL_CFG.VCO_1500MHZ", sata_pll_cfg == "VCO_1500MHZ");
        write_bit("SATA_PLL_CFG.VCO_750MHZ",  sata_pll_cfg == "VCO_750MHZ");

        write_str_bool("SHOW_REALIGN_COMMA", "SHOW_REALIGN_COMMA");

        auto term_rcal_cfg = int_or_default(ci->params, ctx->id("TERM_RCAL_CFG"), 0);
        write_int_vector("TERM_RCAL_CFG[14:0]", term_rcal_cfg, 15);
        auto term_rcal_ovrd = int_or_default(ci->params, ctx->id("TERM_RCAL_OVRD"), 0);
        write_int_vector("TERM_RCAL_OVRD[2:0]", term_rcal_ovrd, 3);

        auto trans_time_rate = int_or_default(ci->params, ctx->id("TRANS_TIME_RATE"), 0);
        write_int_vector("TRANS_TIME_RATE[7:0]", trans_time_rate, 8);

        auto tst_rsv = int_or_default(ci->params, ctx->id("TST_RSV"), 0);
        write_int_vector("TST_RSV[31:0]", tst_rsv, 32);

        auto tx_clkmux_en = bool_or_default(ci->params, ctx->id("TX_CLKMUX_EN"), false);
        write_bit("TX_CLKMUX_EN[0]", tx_clkmux_en);
        auto tx_data_width = int_or_default(ci->params, ctx->id("TX_DATA_WIDTH"), 0);
        switch (tx_data_width) {
            case 16:
                tx_data_width = 2; break;
            case 20:
                tx_data_width = 3; break;
            case 32:
                tx_data_width = 4; break;
            case 40:
                tx_data_width = 5; break;
            default:
                log_error("Invalid TX_DATA_WIDTH parameter '%d' for GTPE2_CHANNEL instance %s\n", tx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("TX_DATA_WIDTH[2:0]", tx_data_width, 3);

        auto tx_drive_mode = str_or_default(ci->params, ctx->id("TX_DRIVE_MODE"), "DIRECT");
        bool invalid_tx_drive_mode = tx_drive_mode != "DIRECT" && tx_drive_mode != "PIPE";
        if (invalid_tx_drive_mode)
            log_error("TX_DRIVE_MODE may only have values 'PIPE' or 'DIRECT' but is: '%s'\n", tx_drive_mode.c_str());
        write_bit("TX_DRIVE_MODE.PIPE", tx_drive_mode == "PIPE");

        auto tx_eidle_assert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_ASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_ASSERT_DELAY[2:0]", tx_eidle_assert_delay, 3);
        auto tx_eidle_deassert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_DEASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_DEASSERT_DELAY[2:0]", tx_eidle_deassert_delay, 3);

        write_str_bool("TX_LOOPBACK_DRIVE_HIZ", "TX_LOOPBACK_DRIVE_HIZ");

        auto tx_maincursor_sel = bool_or_default(ci->params, ctx->id("TX_MAINCURSOR_SEL"), false);
        write_bit("TX_MAINCURSOR_SEL[0]", tx_maincursor_sel);
        auto tx_margin_full_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_0"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_0[6:0]", tx_margin_full_0, 7);
        auto tx_margin_full_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_1"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_1[6:0]", tx_margin_full_1, 7);
        auto tx_margin_full_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_2"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_2[6:0]", tx_margin_full_2, 7);
        auto tx_margin_full_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_3"), 0b1000001);
        write_int_vector("TX_MARGIN_FULL_3[6:0]", tx_margin_full_3, 7);
        auto tx_margin_full_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_4"), 0b1000000);
        write_int_vector("TX_MARGIN_FULL_4[6:0]", tx_margin_full_4, 7);
        auto tx_margin_low_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_0"), 0b1000111);
        write_int_vector("TX_MARGIN_LOW_0[6:0]", tx_margin_low_0, 7);
        auto tx_margin_low_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_1"), 0b1000110);
        write_int_vector("TX_MARGIN_LOW_1[6:0]", tx_margin_low_1, 7);
        auto tx_margin_low_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_2"), 0b1000100);
        write_int_vector("TX_MARGIN_LOW_2[6:0]", tx_margin_low_2, 7);
        auto tx_margin_low_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_3"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_3[6:0]", tx_margin_low_3, 7);
        auto tx_margin_low_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_4"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_4[6:0]", tx_margin_low_4, 7);
        auto tx_predriver_mode = bool_or_default(ci->params, ctx->id("TX_PREDRIVER_MODE"), false);
        write_bit("TX_PREDRIVER_MODE", tx_predriver_mode);
        auto tx_rxdetect_cfg = int_or_default(ci->params, ctx->id("TX_RXDETECT_CFG"), 0);
        write_int_vector("TX_RXDETECT_CFG[13:0]", tx_rxdetect_cfg, 14);
        auto tx_rxdetect_ref = int_or_default(ci->params, ctx->id("TX_RXDETECT_REF"), 0);
        write_int_vector("TX_RXDETECT_REF[2:0]", tx_rxdetect_ref, 3);
        auto tx_xclk_sel = str_or_default(ci->params, ctx->id("TX_XCLK_SEL"), "TXUSR");
        bool invalid_tx_xclk_sel = tx_xclk_sel != "TXUSR" && tx_xclk_sel != "TXOUT";
        if (invalid_tx_xclk_sel)
            log_error("TX_XCLK_SEL may only have values 'TXOUT' or 'TXUSR' but is: '%s'\n", tx_xclk_sel.c_str());
        write_bit("TX_XCLK_SEL.TXUSR", tx_xclk_sel == "TXUSR");
        auto tx_clk25_div = int_or_default(ci->params, ctx->id("TX_CLK25_DIV"), 0) - 1;
        write_int_vector("TX_CLK25_DIV[4:0]", tx_clk25_div, 5);
        auto tx_deemph0 = int_or_default(ci->params, ctx->id("TX_DEEMPH0"), 0);
        write_int_vector("TX_DEEMPH0[5:0]", tx_deemph0, 6);
        auto tx_deemph1 = int_or_default(ci->params, ctx->id("TX_DEEMPH1"), 0);
        write_int_vector("TX_DEEMPH1[5:0]", tx_deemph1, 6);
        write_str_bool("TXBUF_EN", "TXBUF_EN");
        write_str_bool("TXBUF_RESET_ON_RATE_CHANGE", "TXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        auto txdly_cfg = int_or_default(ci->params, ctx->id("TXDLY_CFG"), 0);
        write_int_vector("TXDLY_CFG[15:0]", txdly_cfg, 16);
        auto txdly_lcfg = int_or_default(ci->params, ctx->id("TXDLY_LCFG"), 0);
        write_int_vector("TXDLY_LCFG[8:0]", txdly_lcfg, 9);
        auto txdly_tap_cfg = int_or_default(ci->params, ctx->id("TXDLY_TAP_CFG"), 0);
        write_int_vector("TXDLY_TAP_CFG[15:0]", txdly_tap_cfg, 16);
        write_str_bool("TXGEARBOX_EN", "TXGEARBOX_EN");
        auto txoob_cfg = bool_or_default(ci->params, ctx->id("TXOOB_CFG"), false);
        write_bit("TXOOB_CFG[0]", txoob_cfg);
        auto txout_div = std::log2(int_or_default(ci->params, ctx->id("TXOUT_DIV"), 1));
        write_int_vector("TXOUT_DIV[1:0]", txout_div, 2);
        auto txpcsreset_time = int_or_default(ci->params, ctx->id("TXPCSRESET_TIME"), 0);
        write_int_vector("TXPCSRESET_TIME[4:0]", txpcsreset_time, 5);

        auto txph_cfg = int_or_default(ci->params, ctx->id("TXPH_CFG"), 0);
        write_int_vector("TXPH_CFG[15:0]", txph_cfg, 16);
        auto txph_monitor_sel = int_or_default(ci->params, ctx->id("TXPH_MONITOR_SEL"), 0);
        write_int_vector("TXPH_MONITOR_SEL[4:0]", txph_monitor_sel, 5);
        auto txphdly_cfg = int_or_default(ci->params, ctx->id("TXPHDLY_CFG"), 0);
        write_int_vector("TXPHDLY_CFG[23:0]", txphdly_cfg, 24);

        auto txpi_grey_sel = bool_or_default(ci->params, ctx->id("TXPI_GREY_SEL"), false);
        write_bit("TXPI_GREY_SEL[0]", txpi_grey_sel);
        auto txpi_invstrobe_sel = bool_or_default(ci->params, ctx->id("TXPI_INVSTROBE_SEL"), false);
        write_bit("TXPI_INVSTROBE_SEL[0]", txpi_invstrobe_sel);
        auto txpi_ppm_cfg = int_or_default(ci->params, ctx->id("TXPI_PPM_CFG"), 0);
        write_int_vector("TXPI_PPM_CFG[7:0]", txpi_ppm_cfg, 8);
        auto txpi_ppmclk_sel = str_or_default(ci->params, ctx->id("TXPI_PPMCLK_SEL"), "TXUSRCLK");
        bool invalid_txpi_ppmclk_sel = txpi_ppmclk_sel != "TXUSRCLK" && txpi_ppmclk_sel != "TXUSRCLK2";
        if (invalid_txpi_ppmclk_sel)
            log_error("TXPI_PPMCLK_SEL may only have values 'TXUSRCLK2' or 'TXUSRCLK' but is: '%s'\n", txpi_ppmclk_sel.c_str());
        write_bit("TXPI_PPMCLK_SEL.TXUSRCLK2", txpi_ppmclk_sel == "TXUSRCLK2");
        auto txpi_synfreq_ppm = int_or_default(ci->params, ctx->id("TXPI_SYNFREQ_PPM"), 0);
        // A 3-bit attribute whose default is 0 (unisim); Vivado builds 0 with the
        // three bits clear.
        const bool invalid_txpi_synfreq_ppm = txpi_synfreq_ppm < 0 || txpi_synfreq_ppm > 7;
        if (invalid_txpi_synfreq_ppm)
            log_error("TXPI_SYNFREQ_PPM may only have values from 0 to 7 but is: %d\n", txpi_synfreq_ppm);
        write_int_vector("TXPI_SYNFREQ_PPM[2:0]", txpi_synfreq_ppm, 3);
        auto txpi_cfg0 = int_or_default(ci->params, ctx->id("TXPI_CFG0"), 0);
        write_int_vector("TXPI_CFG0[1:0]", txpi_cfg0, 2);
        auto txpi_cfg1 = int_or_default(ci->params, ctx->id("TXPI_CFG1"), 0);
        write_int_vector("TXPI_CFG1[1:0]", txpi_cfg1, 2);
        auto txpi_cfg2 = int_or_default(ci->params, ctx->id("TXPI_CFG2"), 0);
        write_int_vector("TXPI_CFG2[1:0]", txpi_cfg2, 2);
        auto txpi_cfg3 = bool_or_default(ci->params, ctx->id("TXPI_CFG3"), false);
        write_bit("TXPI_CFG3[0]", txpi_cfg3);
        auto txpi_cfg4 = bool_or_default(ci->params, ctx->id("TXPI_CFG4"), false);
        write_bit("TXPI_CFG4[0]", txpi_cfg4);
        auto txpi_cfg5 = int_or_default(ci->params, ctx->id("TXPI_CFG5"), 0);
        write_int_vector("TXPI_CFG5[1:0]", txpi_cfg5, 2);
        auto txpmareset_time = int_or_default(ci->params, ctx->id("TXPMARESET_TIME"), 0);
        write_int_vector("TXPMARESET_TIME[4:0]", txpmareset_time, 5);

        auto txsync_multilane = bool_or_default(ci->params, ctx->id("TXSYNC_MULTILANE"), false);
        write_bit("TXSYNC_MULTILANE[0]", txsync_multilane);
        auto txsync_ovrd = bool_or_default(ci->params, ctx->id("TXSYNC_OVRD"), false);
        write_bit("TXSYNC_OVRD[0]", txsync_ovrd);
        auto txsync_skip_da = bool_or_default(ci->params, ctx->id("TXSYNC_SKIP_DA"), false);
        write_bit("TXSYNC_SKIP_DA[0]", txsync_skip_da);

        auto ucodeer_clr = bool_or_default(ci->params, ctx->id("UCODEER_CLR"), false);
        write_bit("UCODEER_CLR[0]", ucodeer_clr);

        auto use_pcs_clk_phase_sel = bool_or_default(ci->params, ctx->id("USE_PCS_CLK_PHASE_SEL"), false);
        write_bit("USE_PCS_CLK_PHASE_SEL[0]", use_pcs_clk_phase_sel);

        pop(); // GTPE2_CHANNEL
        pop(); // tile name
    }
    void write_gtx_pll(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));

        push("GTXE2_COMMON");
        write_bit("IN_USE");
        write_bit("ENABLE_DRP", quad_channel_drp_write_used(ci));
        write_bit("BOTH_GTREFCLK_USED", bool_or_default(ci->params, ctx->id("_BOTH_GTREFCLK_USED"), false));
        write_bit("GTREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK0_USED"), false));
        write_bit("GTREFCLK1_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK1_USED"), false));
        bool internal_refclk_used = bool_or_default(ci->params, ctx->id("_GTGREFCLK_USED"), false);
        if (internal_refclk_used) {
            write_bit("GTREFCLK0_USED");
            write_bit("GTREFCLK1_USED");
        }
        auto clkswing_cfg = int_or_default(ci->params, ctx->id("CLKSWING_CFG"), 3);
        if (clkswing_cfg != 3) log_warning("%s/%s: According to ug476, CLK should always be 0b11\n",
                                           ci->hierpath.c_str(ctx), ci->name.c_str(ctx));
        write_int_vector("IBUFDS_GTE2.CLKSWING_CFG[1:0]", clkswing_cfg, 2);
        write_bit("INV_DRPCLK", bool_or_default(ci->params, ctx->id("IS_DRPCLK_INVERTED")));
        write_bit("INV_QPLLLOCKDETCLK", bool_or_default(ci->params, ctx->id("IS_QPLLLOCKDETCLK_INVERTED")));

        write_bit("QPLL_DMONITOR_SEL[0]", bool_or_default(ci->params, ctx->id("QPLL_DMONITOR_SEL")));
        
        auto bias_cfg_found = ci->params.find(ctx->id("BIAS_CFG"));
        auto bias_cfg = bias_cfg_found == ci->params.end() ? 0b1000000000000000000000000000001000000000000UL : bias_cfg_found->second.as_int64();
        write_int_vector("BIAS_CFG[63:0]", bias_cfg, 64);
        auto common_cfg = int_or_default(ci->params, ctx->id("COMMON_CFG"), 0);
        write_int_vector("COMMON_CFG[31:0]", common_cfg, 32);

        // according to ug476, these attributes contain magic undocumented and reserved wizard values
        // TODO: check values
        write_int_vector("QPLL_CFG[26:0]", 0b11010000000000111000001, 27);
        write_int_vector("QPLL_CLKOUT_CFG[3:0]", 0, 4);
        write_int_vector("QPLL_COARSE_FREQ_OVRD[5:0]", 0b10000, 6);
        auto coarse_freq_ovrd_en = int_or_default(ci->params, ctx->id("QPLL_COARSE_FREQ_OVRD_EN"), 0);
        if (coarse_freq_ovrd_en > 0)
            log_warning("According to UG476, the QPLL_COARSE_FREQ_OVRD_EN attribute must be 0, but it is not. Be sure you know what you are doing.");
        write_bit("QPLL_COARSE_FREQ_OVRD_EN[0]", coarse_freq_ovrd_en >= 1);
        write_int_vector("QPLL_INIT_CFG[23:0]", 0b110, 24);
        write_int_vector("QPLL_LOCK_CFG[15:0]", 0b10000111101000, 16);
        write_int_vector("QPLL_LPF[3:0]", 0b1111, 4);
        write_int_vector("QPLL_CP[9:0]", 0b11111, 10);
        auto cp_monitor_en = int_or_default(ci->params, ctx->id("QPLL_CP_MONITOR_EN"), 0);
        if (cp_monitor_en > 0)
            log_warning("According to UG476, the QPLL_CP_MONITOR_EN attribute must be 0, but it is not. Be sure you know what you are doing.");
        write_bit("QPLL_CP_MONITOR_EN[0]", cp_monitor_en >= 1);
        write_bit("QPLL_DMONITOR_SEL[0]", 0); // TODO: find real vivado default value

        auto qpll_refclk_div = int_or_default(ci->params, ctx->id("QPLL_REFCLK_DIV"), 1);
        bool invalid_qpll_refclk_div = qpll_refclk_div < 1 || qpll_refclk_div > 4;
        if (invalid_qpll_refclk_div)
            log_error("QPLL_REFCLK_DIV can only range from 1 to 4, but is: %d", qpll_refclk_div);
        auto real_qpll_refclk_div = qpll_refclk_div == 1 ? 16 : qpll_refclk_div - 2;
        write_int_vector("QPLL_REFCLK_DIV[4:0]", real_qpll_refclk_div, 5);

        auto qpll_fbdiv = int_or_default(ci->params, ctx->id("QPLL_FBDIV"), 1);
        write_int_vector("QPLL_FBDIV[9:0]", qpll_fbdiv, 10);

        auto fbdiv_monitor_en = int_or_default(ci->params, ctx->id("FBDIV_MONITOR_EN"), 0);
        if (fbdiv_monitor_en > 0)
            log_warning("According to UG476, the QPLL_FBDIV_MONITOR_EN attribute must be 0, but it is 1. Be sure you know what you are doing.");
        write_bit("QPLL_FBDIV_MONITOR_EN[0]", fbdiv_monitor_en >= 1);

        auto qpll_fbdiv_ratio = int_or_default(ci->params, ctx->id("QPLL_FBDIV_RATIO"), 1);
        if (qpll_fbdiv_ratio > 1)
            log_error("QPLL_FBDIV_ratio can only be 0 or 1, but is: %d", qpll_fbdiv_ratio);
        write_bit("QPLL_FBDIV_RATIO[0]", qpll_fbdiv_ratio == 1);

        auto pll_clkout_cfg = int_or_default(ci->params, ctx->id("PLL_CLKOUT_CFG"), 0);
        write_int_vector("PLL_CLKOUT_CFG[3:0]", pll_clkout_cfg, 4);

        pop(); // GTXE2_COMMON
        pop(); // tile name
    }
    void write_gtx_channel(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("GTXE2_CHANNEL");

        write_bit("IN_USE");

        auto write_str_bool = [&](std::string attribute, std::string bit, std::string deflt = "FALSE") {
            auto val = str_or_default(ci->params, ctx->id(attribute), deflt);
            boost::algorithm::to_upper(val);
            write_bit(bit, val == "TRUE");
        };

        write_str_bool("ALIGN_COMMA_DOUBLE", "ALIGN_COMMA_DOUBLE");

        auto align_comma_enable = int_or_default(ci->params, ctx->id("ALIGN_COMMA_ENABLE"), 0b1111111111);
        write_int_vector("ALIGN_COMMA_ENABLE[9:0]", align_comma_enable, 10);

        auto align_comma_word = int_or_default(ci->params, ctx->id("ALIGN_COMMA_WORD"), 1);
        bool valid_align_comma_word = align_comma_word == 1 || align_comma_word == 2 || align_comma_word == 4;
        if (!valid_align_comma_word)
            log_error("ALIGN_COMMA_WORD may only be 1, 2 or 4 but is: %d\n", align_comma_word);
        write_int_vector("ALIGN_COMMA_WORD[2:0]", align_comma_word, 3);

        write_str_bool("ALIGN_MCOMMA_DET", "ALIGN_MCOMMA_DET");
        auto align_mcomma_value = int_or_default(ci->params, ctx->id("ALIGN_MCOMMA_VALUE"), 0b1010000011);
        write_int_vector("ALIGN_MCOMMA_VALUE[9:0]", align_mcomma_value, 10);

        write_str_bool("ALIGN_PCOMMA_DET", "ALIGN_PCOMMA_DET");
        auto align_pcomma_value = int_or_default(ci->params, ctx->id("ALIGN_PCOMMA_VALUE"), 0b0101111100);
        write_int_vector("ALIGN_PCOMMA_VALUE[9:0]", align_pcomma_value, 10);

        auto cbcc_data_source_sel = str_or_default(ci->params, ctx->id("CBCC_DATA_SOURCE_SEL"), "DECODED");
        if (cbcc_data_source_sel == "DECODED") write_bit("CBCC_DATA_SOURCE_SEL.DECODED");

        write_str_bool("CHAN_BOND_KEEP_ALIGN", "CHAN_BOND_KEEP_ALIGN");
        auto chan_bond_max_skew = int_or_default(ci->params, ctx->id("CHAN_BOND_MAX_SKEW"), 0);
        bool invalid_chan_bond_max_skew = chan_bond_max_skew < 1 || 14 < chan_bond_max_skew;
        if (invalid_chan_bond_max_skew)
            log_error("CHAN_BOND_MAX_SKEW may only range from 1 to 14, but is: %d\n", chan_bond_max_skew);
        write_int_vector("CHAN_BOND_MAX_SKEW[3:0]", chan_bond_max_skew, 4);

        auto chan_bond_seq_1_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_ENABLE[3:0]", chan_bond_seq_1_enable, 4);
        auto chan_bond_seq_1_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_1[9:0]", chan_bond_seq_1_1, 10);
        auto chan_bond_seq_1_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_2[9:0]", chan_bond_seq_1_2, 10);
        auto chan_bond_seq_1_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_3[9:0]", chan_bond_seq_1_3, 10);
        auto chan_bond_seq_1_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_1_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_1_4[9:0]", chan_bond_seq_1_4, 10);

        write_str_bool("CHAN_BOND_SEQ_2_USE", "CHAN_BOND_SEQ_2_USE");
        auto chan_bond_seq_2_enable = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_ENABLE"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_ENABLE[3:0]", chan_bond_seq_2_enable, 4);
        auto chan_bond_seq_2_1 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_1"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_1[9:0]", chan_bond_seq_2_1, 10);
        auto chan_bond_seq_2_2 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_2"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_2[9:0]", chan_bond_seq_2_2, 10);
        auto chan_bond_seq_2_3 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_3"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_3[9:0]", chan_bond_seq_2_3, 10);
        auto chan_bond_seq_2_4 = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_2_4"), 0);
        write_int_vector("CHAN_BOND_SEQ_2_4[9:0]", chan_bond_seq_2_4, 10);

        auto chan_bond_seq_len = int_or_default(ci->params, ctx->id("CHAN_BOND_SEQ_LEN"), 0);
        bool valid_chan_bond_seq_len = chan_bond_seq_len == 1 || chan_bond_seq_len == 2 || chan_bond_seq_len == 4;
        if (!valid_chan_bond_seq_len)
            log_error("CHAN_BOND_SEQ_LEN may only be 1, 2 or 4, but is: %d\n", chan_bond_seq_len);
        write_int_vector("CHAN_BOND_SEQ_LEN[1:0]", chan_bond_seq_len - 1, 2);

        write_str_bool("CLK_COR_KEEP_IDLE", "CLK_COR_KEEP_IDLE");
        auto clk_cor_max_lat = int_or_default(ci->params, ctx->id("CLK_COR_MAX_LAT"), 0);
        write_int_vector("CLK_COR_MAX_LAT[5:0]", clk_cor_max_lat, 6);
        auto clk_cor_min_lat = int_or_default(ci->params, ctx->id("CLK_COR_MIN_LAT"), 0);
        write_int_vector("CLK_COR_MIN_LAT[5:0]", clk_cor_min_lat, 6);
        write_str_bool("CLK_COR_PRECEDENCE", "CLK_COR_PRECEDENCE");
        auto clk_cor_repeat_wait = int_or_default(ci->params, ctx->id("CLK_COR_REPEAT_WAIT"), 0);
        write_int_vector("CLK_COR_REPEAT_WAIT[4:0]", clk_cor_repeat_wait, 5);

        auto clk_cor_seq_1_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_1_ENABLE[3:0]", clk_cor_seq_1_enable, 4);
        auto clk_cor_seq_1_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_1"), 0);
        write_int_vector("CLK_COR_SEQ_1_1[9:0]", clk_cor_seq_1_1, 10);
        auto clk_cor_seq_1_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_2"), 0);
        write_int_vector("CLK_COR_SEQ_1_2[9:0]", clk_cor_seq_1_2, 10);
        auto clk_cor_seq_1_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_3"), 0);
        write_int_vector("CLK_COR_SEQ_1_3[9:0]", clk_cor_seq_1_3, 10);
        auto clk_cor_seq_1_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_1_4"), 0);
        write_int_vector("CLK_COR_SEQ_1_4[9:0]", clk_cor_seq_1_4, 10);

        write_str_bool("CLK_COR_SEQ_2_USE", "CLK_COR_SEQ_2_USE");
        auto clk_cor_seq_2_enable = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_ENABLE"), 0);
        write_int_vector("CLK_COR_SEQ_2_ENABLE[3:0]", clk_cor_seq_2_enable, 4);
        auto clk_cor_seq_2_1 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_1"), 0);
        write_int_vector("CLK_COR_SEQ_2_1[9:0]", clk_cor_seq_2_1, 10);
        auto clk_cor_seq_2_2 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_2"), 0);
        write_int_vector("CLK_COR_SEQ_2_2[9:0]", clk_cor_seq_2_2, 10);
        auto clk_cor_seq_2_3 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_3"), 0);
        write_int_vector("CLK_COR_SEQ_2_3[9:0]", clk_cor_seq_2_3, 10);
        auto clk_cor_seq_2_4 = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_2_4"), 0);
        write_int_vector("CLK_COR_SEQ_2_4[9:0]", clk_cor_seq_2_4, 10);

        auto clk_cor_seq_len = int_or_default(ci->params, ctx->id("CLK_COR_SEQ_LEN"), 0);
        bool invalid_clk_cor_seq_len = clk_cor_seq_len < 1 || 4 < clk_cor_seq_len;
        if (invalid_clk_cor_seq_len)
            log_error("CLK_COR_SEQ_LEN may only range from 1 to 4, but is: %d\n", clk_cor_seq_len);
        write_int_vector("CLK_COR_SEQ_LEN[1:0]", clk_cor_seq_len - 1, 2);

        write_str_bool("CLK_CORRECT_USE", "CLK_CORRECT_USE");

        auto cpll_cfg = int_or_default(ci->params, ctx->id("CPLL_CFG"), 0b101111000000011111011100);
        write_int_vector("CPLL_CFG[23:0]", cpll_cfg, 24);

        auto cpll_fbdiv = int_or_default(ci->params, ctx->id("CPLL_FBDIV"), 1);
        switch (cpll_fbdiv) {
            case 1: cpll_fbdiv  = 16; break;
            case 2: cpll_fbdiv  =  0; break;
            case 3: cpll_fbdiv  =  1; break;
            case 4: cpll_fbdiv  =  2; break;
            case 5: cpll_fbdiv  =  3; break;
            case 6: cpll_fbdiv  =  5; break;
            case 8: cpll_fbdiv  =  6; break;
            case 10: cpll_fbdiv =  7; break;
            case 12: cpll_fbdiv = 13; break;
            case 16: cpll_fbdiv = 14; break;
            case 20: cpll_fbdiv = 15; break;
            default:
                log_error("CPLL_FBDIV can only be 1, 2, 3, 4, 5, 6, 8, 10, 12, 16 or 20, but is: %d", cpll_fbdiv);
                break;
        }
        write_int_vector("CPLL_FBDIV[4:0]", cpll_fbdiv, 5);

        auto cpll_fbdiv_45 = int_or_default(ci->params, ctx->id("CPLL_FBDIV_45"), 4);
        bool invalid_cpll_fbdiv_45 = cpll_fbdiv_45 < 4 || cpll_fbdiv_45 > 5;
        if (invalid_cpll_fbdiv_45)
            log_error("CPLL_FBDIV_45 can only be 4 or 5, but is: %d", cpll_fbdiv);
        write_bit("CPLL_FBDIV_45[0]", cpll_fbdiv_45 == 5);

        auto cpll_init_cfg = int_or_default(ci->params, ctx->id("CPLL_INIT_CFG"), 0b11110);
        write_int_vector("CPLL_INIT_CFG[4:0]", cpll_init_cfg, 5);
        auto cpll_lock_cfg = int_or_default(ci->params, ctx->id("CPLL_LOCK_CFG"), 0b111101000);
        write_int_vector("CPLL_LOCK_CFG[8:0]", cpll_lock_cfg, 9);

        auto cpll_refclk_div = int_or_default(ci->params, ctx->id("CPLL_REFCLK_DIV"), 1);
        switch (cpll_refclk_div) {
            case 1: cpll_refclk_div  = 16; break;
            case 2: cpll_refclk_div  =  0; break;
            case 3: cpll_refclk_div  =  1; break;
            case 4: cpll_refclk_div  =  2; break;
            case 5: cpll_refclk_div  =  3; break;
            case 6: cpll_refclk_div  =  5; break;
            case 8: cpll_refclk_div  =  6; break;
            case 10: cpll_refclk_div =  7; break;
            case 12: cpll_refclk_div = 13; break;
            case 16: cpll_refclk_div = 14; break;
            case 20: cpll_refclk_div = 15; break;
            default:
                log_error("CPLL_REFCLK_DIV can only be 1, 2, 3, 4, 5, 6, 8, 10, 12, 16 or 20, but is: %d", cpll_refclk_div);
                break;
        }
        write_int_vector("CPLL_REFCLK_DIV[4:0]", cpll_refclk_div, 5);

        write_str_bool("DEC_MCOMMA_DETECT", "DEC_MCOMMA_DETECT");
        write_str_bool("DEC_PCOMMA_DETECT", "DEC_PCOMMA_DETECT");
        write_str_bool("DEC_VALID_COMMA_ONLY", "DEC_VALID_COMMA_ONLY");

        auto dmonitor_cfg = int_or_default(ci->params, ctx->id("DMONITOR_CFG"), 0x008101);
        write_int_vector("DMONITOR_CFG[23:0]", dmonitor_cfg, 24);

        auto es_control = int_or_default(ci->params, ctx->id("ES_CONTROL"), 0);
        write_int_vector("ES_CONTROL[5:0]", es_control, 6);
        write_str_bool("ES_ERRDET_EN", "ES_ERRDET_EN");
        write_str_bool("ES_EYE_SCAN_EN", "ES_EYE_SCAN_EN");
        auto es_pma_cfg = int_or_default(ci->params, ctx->id("ES_PMA_CFG"), 0);
        write_int_vector("ES_PMA_CFG[9:0]", es_pma_cfg, 10);
        auto es_prescale = int_or_default(ci->params, ctx->id("ES_PRESCALE"), 0);
        write_int_vector("ES_PRESCALE[4:0]", es_prescale, 5);
        auto es_qual_mask = int_or_default(ci->params, ctx->id("ES_QUAL_MASK"), 0);
        write_int_vector("ES_QUAL_MASK[79:0]", es_qual_mask, 80);
        auto es_qualifier = int_or_default(ci->params, ctx->id("ES_QUALIFIER"), 0);
        write_int_vector("ES_QUALIFIER[79:0]", es_qualifier, 80);
        auto es_sdata_mask = int_or_default(ci->params, ctx->id("ES_SDATA_MASK"), 0);
        write_int_vector("ES_SDATA_MASK[79:0]", es_sdata_mask, 80);
        auto es_vert_offset = int_or_default(ci->params, ctx->id("ES_VERT_OFFSET"), 0);
        write_int_vector("ES_VERT_OFFSET[8:0]", es_vert_offset, 9);
        auto es_horz_offset = int_or_default(ci->params, ctx->id("ES_HORZ_OFFSET"), 0);
        write_int_vector("ES_HORZ_OFFSET[11:0]", es_horz_offset, 12);

        auto fts_deskew_seq_enable = int_or_default(ci->params, ctx->id("FTS_DESKEW_SEQ_ENABLE"), 0b1111);
        write_int_vector("FTS_DESKEW_SEQ_ENABLE[3:0]", fts_deskew_seq_enable, 4);
        auto fts_lane_deskew_cfg = int_or_default(ci->params, ctx->id("FTS_LANE_DESKEW_CFG"), 0);
        write_int_vector("FTS_LANE_DESKEW_CFG[3:0]", fts_lane_deskew_cfg, 4);
        write_str_bool("FTS_LANE_DESKEW_EN", "FTS_LANE_DESKEW_EN");

        auto gearbox_mode = int_or_default(ci->params, ctx->id("GEARBOX_MODE"), 0);
        write_int_vector("GEARBOX_MODE[2:0]", gearbox_mode, 3);

        auto write_inv = [&](std::string name) {
            write_bit("INV_" + name, bool_or_default(ci->params, ctx->id("IS_" + name + "_INVERTED"), false));
        };
        // only these have been fuzzed yet,
        write_inv("DRPCLK");
        write_inv("RXUSRCLK");
        write_inv("RXUSRCLK2");
        write_inv("TXPHDLYTSTCLK");
        write_inv("TXUSRCLK");
        write_inv("TXUSRCLK2");
        write_inv("RXUSRCLK2");
        write_inv("TXUSRCLK2");
        write_inv("CPLLLOCKDETCLK");
        write_bit("GTREFCLK0_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK0_USED"), false));
        write_bit("GTREFCLK1_USED", bool_or_default(ci->params, ctx->id("_GTREFCLK1_USED"), false));

        auto outrefclk_sel_inv = int_or_default(ci->params, ctx->id("OUTREFCLK_SEL_INV"), 0b10);
        write_int_vector("OUTREFCLK_SEL_INV[1:0]", outrefclk_sel_inv, 2);

        write_str_bool("PCS_PCIE_EN", "PCS_PCIE_EN");

        auto rsvd_attr = int_or_default(ci->params, ctx->id("PCS_RSVD_ATTR"), 0);
        write_int_vector("PCS_RSVD_ATTR[47:0]", rsvd_attr, 48);

        auto pd_trans_time_from_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_FROM_P2"), 0);
        write_int_vector("PD_TRANS_TIME_FROM_P2[11:0]", pd_trans_time_from_p2, 12);
        auto pd_trans_time_none_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_NONE_P2"), 0);
        write_int_vector("PD_TRANS_TIME_NONE_P2[7:0]", pd_trans_time_none_p2, 8);
        auto pd_trans_time_to_p2 = int_or_default(ci->params, ctx->id("PD_TRANS_TIME_TO_P2"), 0);
        write_int_vector("PD_TRANS_TIME_TO_P2[7:0]", pd_trans_time_to_p2, 8);

        auto pma_rsv = int_or_default(ci->params, ctx->id("PMA_RSV"), 0);
        write_int_vector("PMA_RSV[31:0]", pma_rsv, 32);
        auto pma_rsv2 = int_or_default(ci->params, ctx->id("PMA_RSV2"), 0);
        write_int_vector("PMA_RSV2[15:0]", pma_rsv2, 16);
        auto pma_rsv3 = int_or_default(ci->params, ctx->id("PMA_RSV3"), 0);
        write_int_vector("PMA_RSV3[1:0]", pma_rsv3, 2);
        auto pma_rsv4 = int_or_default(ci->params, ctx->id("PMA_RSV4"), 0);
        write_int_vector("PMA_RSV4[3:0]", pma_rsv4, 4);

        auto rx_bias_cfg = int_or_default(ci->params, ctx->id("RX_BIAS_CFG"), 0);
        write_int_vector("RX_BIAS_CFG[11:0]", rx_bias_cfg, 12);

        auto rx_buffer_cfg = int_or_default(ci->params, ctx->id("RX_BUFFER_CFG"), 0);
        write_int_vector("RX_BUFFER_CFG[5:0]", rx_buffer_cfg, 6);
        auto rx_cm_sel = int_or_default(ci->params, ctx->id("RX_CM_SEL"), 0);
        write_int_vector("RX_CM_SEL[1:0]", rx_cm_sel, 2);
        auto rx_cm_trim = int_or_default(ci->params, ctx->id("RX_CM_TRIM"), 0);
        write_int_vector("RX_CM_TRIM[2:0]", rx_cm_trim, 3);
        auto rx_data_width = int_or_default(ci->params, ctx->id("RX_DATA_WIDTH"), 0);
        switch (rx_data_width) {
            case 16:
                rx_data_width = 2; break;
            case 20:
                rx_data_width = 3; break;
            case 32:
                rx_data_width = 4; break;
            case 40:
                rx_data_width = 5; break;
            case 64:
                rx_data_width = 6; break;
            case 80:
                rx_data_width = 7; break;
            default:
                log_error("Invalid RX_DATA_WIDTH parameter '%d' for GTXE2_CHANNEL instance %s\n", rx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("RX_DATA_WIDTH[2:0]", rx_data_width, 3);
        auto rx_int_datawidth = bool_or_default(ci->params, ctx->id("RX_INT_DATAWIDTH"), false);
        write_bit("RX_INT_DATAWIDTH[0]", rx_int_datawidth);
        auto rx_ddi_sel = int_or_default(ci->params, ctx->id("RX_DDI_SEL"), 0);
        write_int_vector("RX_DDI_SEL[5:0]", rx_ddi_sel, 6);
        auto rx_debug_cfg = int_or_default(ci->params, ctx->id("RX_DEBUG_CFG"), 0);
        write_int_vector("RX_DEBUG_CFG[10:0]", rx_debug_cfg, 11);
        write_str_bool("RX_DEFER_RESET_BUF_EN", "RX_DEFER_RESET_BUF_EN");

        // default values, see ug476, default values from wizards not mentioned in the handbook taken from LiteX
        auto rx_dfe_gain_cfg = int_or_default(ci->params, ctx->id("RX_DFE_GAIN_CFG"), 0x020FEA);
        write_int_vector("RX_DFE_GAIN_CFG[17:0]", rx_dfe_gain_cfg, 18);
        auto rx_dfe_h2_cfg = int_or_default(ci->params, ctx->id("RX_DFE_H2_CFG"), 0x000);
        write_int_vector("RX_DFE_H2_CFG[11:0]", rx_dfe_h2_cfg, 12);
        auto rx_dfe_h3_cfg = int_or_default(ci->params, ctx->id("RX_DFE_H3_CFG"), 0x040);
        write_int_vector("RX_DFE_H3_CFG[11:0]", rx_dfe_h3_cfg, 12);
        auto rx_dfe_h4_cfg = int_or_default(ci->params, ctx->id("RX_DFE_H4_CFG"), 0x0f0);
        write_int_vector("RX_DFE_H4_CFG[10:0]", rx_dfe_h4_cfg, 11);
        auto rx_dfe_h5_cfg = int_or_default(ci->params, ctx->id("RX_DFE_H5_CFG"), 0x0e0);
        write_int_vector("RX_DFE_H5_CFG[10:0]", rx_dfe_h5_cfg, 11);
        auto rx_dfe_kl_cfg = int_or_default(ci->params, ctx->id("RX_DFE_KL_CFG"), 0b11111110);
        write_int_vector("RX_DFE_KL_CFG[12:0]", rx_dfe_kl_cfg, 13);
        auto rx_dfe_kl_cfg2 = int_or_default(ci->params, ctx->id("RX_DFE_KL_CFG2"), 0b110000000100010100100010101100);
        write_int_vector("RX_DFE_KL_CFG2[31:0]", rx_dfe_kl_cfg2, 32);
        auto rx_dfe_lpm_cfg = int_or_default(ci->params, ctx->id("RX_DFE_LPM_CFG"), 0b100101010100);
        write_int_vector("RX_DFE_LPM_CFG[11:0]", rx_dfe_lpm_cfg, 12);
        auto rx_dfe_lpm_hold_during_eidle = bool_or_default(ci->params, ctx->id("RX_DFE_LPM_HOLD_DURING_EIDLE"), false);
        write_bit("RX_DFE_LPM_HOLD_DURING_EIDLE[0]", rx_dfe_lpm_hold_during_eidle);
        auto rx_dfe_ut_cfg = int_or_default(ci->params, ctx->id("RX_DFE_UT_CFG"), 0x11E00);
        write_int_vector("RX_DFE_UT_CFG[16:0]", rx_dfe_ut_cfg, 17);
        auto rx_dfe_vp_cfg = int_or_default(ci->params, ctx->id("RX_DFE_VP_CFG"), 0x03F03);
        write_int_vector("RX_DFE_VP_CFG[16:0]", rx_dfe_vp_cfg, 17);
        auto rx_dfe_xyd_cfg = int_or_default(ci->params, ctx->id("RX_DFE_XYD_CFG"), 0);
        write_int_vector("RX_DFE_XYD_CFG[12:0]", rx_dfe_xyd_cfg, 13);

        write_str_bool("RX_DISPERR_SEQ_MATCH", "RX_DISPERR_SEQ_MATCH");
        auto rx_os_cfg = int_or_default(ci->params, ctx->id("RX_OS_CFG"), 0);
        write_int_vector("RX_OS_CFG[12:0]", rx_os_cfg, 13);
        auto rx_sig_valid_dly = int_or_default(ci->params, ctx->id("RX_SIG_VALID_DLY"), 0) - 1;
        write_int_vector("RX_SIG_VALID_DLY[4:0]", rx_sig_valid_dly, 5);
        auto rx_xclk_sel = str_or_default(ci->params, ctx->id("RX_XCLK_SEL"), "RXUSR");
        bool invalid_rx_xclk_sel = rx_xclk_sel != "RXUSR" && rx_xclk_sel != "RXREC";
        if (invalid_rx_xclk_sel)
            log_error("RX_XCLK_SEL may only have values 'RXREC' or 'RXUSR' but is: '%s'\n", rx_xclk_sel.c_str());
        write_bit("RX_XCLK_SEL.RXUSR", rx_xclk_sel == "RXUSR");
        auto rx_clk25_div = int_or_default(ci->params, ctx->id("RX_CLK25_DIV"), 0) - 1;
        write_int_vector("RX_CLK25_DIV[4:0]", rx_clk25_div, 5);

        auto rxbuf_addr_mode = str_or_default(ci->params, ctx->id("RXBUF_ADDR_MODE"), "PMA");
        bool invalid_rxbuf_addr_mode = rxbuf_addr_mode != "FULL" && rxbuf_addr_mode != "FAST";
        if (invalid_rxbuf_addr_mode)
            log_error("RXBUF_ADDR_MODE may only have values 'FULL' or 'FAST' but is: '%s'\n", rxbuf_addr_mode.c_str());
        write_bit("RXBUF_ADDR_MODE.FAST", rxbuf_addr_mode == "FAST");
        auto rxbuf_eidle_hi_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_HI_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_HI_CNT[3:0]", rxbuf_eidle_hi_cnt, 4);
        auto rxbuf_eidle_lo_cnt = int_or_default(ci->params, ctx->id("RXBUF_EIDLE_LO_CNT"), 0);
        write_int_vector("RXBUF_EIDLE_LO_CNT[3:0]", rxbuf_eidle_lo_cnt, 4);
        write_str_bool("RXBUF_EN", "RXBUF_EN", "TRUE");
        write_str_bool("RXBUF_RESET_ON_CB_CHANGE", "RXBUF_RESET_ON_CB_CHANGE", "TRUE");
        write_str_bool("RXBUF_RESET_ON_COMMAALIGN", "RXBUF_RESET_ON_COMMAALIGN");
        write_str_bool("RXBUF_RESET_ON_EIDLE", "RXBUF_RESET_ON_EIDLE");
        write_str_bool("RXBUF_RESET_ON_RATE_CHANGE", "RXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        write_str_bool("RXBUF_THRESH_OVRD", "RXBUF_THRESH_OVRD");
        auto rxbuf_thresh_ovflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_OVFLW"), 0);
        write_int_vector("RXBUF_THRESH_OVFLW[5:0]", rxbuf_thresh_ovflw, 6);
        auto rxbuf_thresh_undflw = int_or_default(ci->params, ctx->id("RXBUF_THRESH_UNDFLW"), 0);
        write_int_vector("RXBUF_THRESH_UNDFLW[5:0]", rxbuf_thresh_undflw, 6);
        auto rxbufreset_time = int_or_default(ci->params, ctx->id("RXBUFRESET_TIME"), 0);
        write_int_vector("RXBUFRESET_TIME[4:0]", rxbufreset_time, 5);

        auto rxcdr_cfg = get_or_default(ci->params, ctx->id("RXCDR_CFG"),
            Property("00000000000000000000000000000000000000000000000000000000000000000000000000000000000"));
        write_vector("RXCDR_CFG[71:0]", bits_from_string_default(rxcdr_cfg, 72));
        auto rxcdr_fr_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_FR_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_FR_RESET_ON_EIDLE[0]", rxcdr_fr_reset_on_eidle);
        auto rxcdr_ph_reset_on_eidle = bool_or_default(ci->params, ctx->id("RXCDR_PH_RESET_ON_EIDLE"), false);
        write_bit("RXCDR_PH_RESET_ON_EIDLE[0]", rxcdr_ph_reset_on_eidle);
        auto rxcdr_hold_during_eidle = bool_or_default(ci->params, ctx->id("RXCDR_HOLD_DURING_EIDLE"), false);
        write_bit("RXCDR_HOLD_DURING_EIDLE[0]", rxcdr_hold_during_eidle);
        auto rx_clkmux_pd = bool_or_default(ci->params, ctx->id("RX_CLKMUX_PD"), true);
        write_bit("RX_CLKMUX_PD[0]", rx_clkmux_pd);
        auto rxcdr_lock_cfg = int_or_default(ci->params, ctx->id("RXCDR_LOCK_CFG"), 0);
        write_int_vector("RXCDR_LOCK_CFG[5:0]", rxcdr_lock_cfg, 6);
        auto rxcdrfreqreset_time = int_or_default(ci->params, ctx->id("RXCDRFREQRESET_TIME"), 0);
        write_int_vector("RXCDRFREQRESET_TIME[4:0]", rxcdrfreqreset_time, 5);
        auto rxcdrphreset_time = int_or_default(ci->params, ctx->id("RXCDRPHRESET_TIME"), 0);
        write_int_vector("RXCDRPHRESET_TIME[4:0]", rxcdrphreset_time, 5);

        auto rxdfelpmreset_time = int_or_default(ci->params, ctx->id("RXDFELPMRESET_TIME"), 0);
        write_int_vector("RXDFELPMRESET_TIME[6:0]", rxdfelpmreset_time, 7);

        auto rxdly_cfg = int_or_default(ci->params, ctx->id("RXDLY_CFG"), 0);
        write_int_vector("RXDLY_CFG[15:0]", rxdly_cfg, 16);
        auto rxdly_lcfg = int_or_default(ci->params, ctx->id("RXDLY_LCFG"), 0);
        write_int_vector("RXDLY_LCFG[8:0]", rxdly_lcfg, 9);
        auto rxdly_tap_cfg = int_or_default(ci->params, ctx->id("RXDLY_TAP_CFG"), 0);
        write_int_vector("RXDLY_TAP_CFG[15:0]", rxdly_tap_cfg, 16);

        write_str_bool("RXGEARBOX_EN", "RXGEARBOX_EN");

        auto rxiscanreset_time = int_or_default(ci->params, ctx->id("RXISCANRESET_TIME"), 0);
        write_int_vector("RXISCANRESET_TIME[4:0]", rxiscanreset_time, 5);

        auto rxlpm_hf_cfg = int_or_default(ci->params, ctx->id("RXLPM_HF_CFG"), 0);
        write_int_vector("RXLPM_HF_CFG[13:0]", rxlpm_hf_cfg, 14);
        auto rxlpm_lf_cfg = int_or_default(ci->params, ctx->id("RXLPM_LF_CFG"), 0);
        write_int_vector("RXLPM_LF_CFG[13:0]", rxlpm_lf_cfg, 14);

        auto rxoob_cfg = int_or_default(ci->params, ctx->id("RXOOB_CFG"), 0);
        write_int_vector("RXOOB_CFG[6:0]", rxoob_cfg, 7);

        auto rxout_div = std::log2(int_or_default(ci->params, ctx->id("RXOUT_DIV"), 1));
        write_int_vector("RXOUT_DIV[1:0]", rxout_div, 2);

        auto rxpcsreset_time = int_or_default(ci->params, ctx->id("RXPCSRESET_TIME"), 0);
        write_int_vector("RXPCSRESET_TIME[4:0]", rxpcsreset_time, 5);

        auto rxph_cfg = int_or_default(ci->params, ctx->id("RXPH_CFG"), 0);
        write_int_vector("RXPH_CFG[23:0]", rxph_cfg, 24);
        auto rxph_monitor_sel = int_or_default(ci->params, ctx->id("RXPH_MONITOR_SEL"), 0);
        write_int_vector("RXPH_MONITOR_SEL[4:0]", rxph_monitor_sel, 5);
        auto rxphdly_cfg = int_or_default(ci->params, ctx->id("RXPHDLY_CFG"), 0);
        write_int_vector("RXPHDLY_CFG[23:0]", rxphdly_cfg, 24);

        auto rxpmareset_time = int_or_default(ci->params, ctx->id("RXPMARESET_TIME"), 0);
        write_int_vector("RXPMARESET_TIME[4:0]", rxpmareset_time, 5);

        auto rxprbs_err_loopback = bool_or_default(ci->params, ctx->id("RXPRBS_ERR_LOOPBACK"), false);
        write_bit("RXPRBS_ERR_LOOPBACK[0]", rxprbs_err_loopback);

        auto rxslide_auto_wait = int_or_default(ci->params, ctx->id("RXSLIDE_AUTO_WAIT"), 7);
        write_int_vector("RXSLIDE_AUTO_WAIT[3:0]", rxslide_auto_wait, 4);
        auto rxslide_mode = str_or_default(ci->params, ctx->id("RXSLIDE_MODE"), "OFF");
        bool invalid_rxslide_mode =
                rxslide_mode != "OFF" && rxslide_mode != "AUTO" && rxslide_mode != "PCS" && rxslide_mode != "PMA";
        if (invalid_rxslide_mode)
            log_error("RXSLIDE_MODE may only have values 'OFF', 'AUTO', 'PCS' or 'PMA' but is: '%s'\n", rxslide_mode.c_str());
        write_bit("RXSLIDE_MODE.AUTO", rxslide_mode == "AUTO");
        write_bit("RXSLIDE_MODE.PCS",  rxslide_mode == "PCS");
        write_bit("RXSLIDE_MODE.PMA",  rxslide_mode == "PMA");

        auto sas_max_com = int_or_default(ci->params, ctx->id("SAS_MAX_COM"), 0);
        write_int_vector("SAS_MAX_COM[6:0]", sas_max_com, 7);
        auto sas_min_com = int_or_default(ci->params, ctx->id("SAS_MIN_COM"), 0);
        write_int_vector("SAS_MIN_COM[5:0]", sas_min_com, 6);

        auto sata_burst_seq_len = int_or_default(ci->params, ctx->id("SATA_BURST_SEQ_LEN"), 0);
        write_int_vector("SATA_BURST_SEQ_LEN[3:0]", sata_burst_seq_len, 4);
        auto sata_burst_val = int_or_default(ci->params, ctx->id("SATA_BURST_VAL"), 0);
        write_int_vector("SATA_BURST_VAL[2:0]", sata_burst_val, 3);
        auto sata_eidle_val = int_or_default(ci->params, ctx->id("SATA_EIDLE_VAL"), 0);
        write_int_vector("SATA_EIDLE_VAL[2:0]", sata_eidle_val, 3);
        auto sata_max_burst = int_or_default(ci->params, ctx->id("SATA_MAX_BURST"), 0);
        write_int_vector("SATA_MAX_BURST[5:0]", sata_max_burst, 6);
        auto sata_max_init = int_or_default(ci->params, ctx->id("SATA_MAX_INIT"), 0);
        write_int_vector("SATA_MAX_INIT[5:0]", sata_max_init, 6);
        auto sata_max_wake = int_or_default(ci->params, ctx->id("SATA_MAX_WAKE"), 0);
        write_int_vector("SATA_MAX_WAKE[5:0]", sata_max_wake, 6);
        auto sata_min_burst = int_or_default(ci->params, ctx->id("SATA_MIN_BURST"), 0);
        write_int_vector("SATA_MIN_BURST[5:0]", sata_min_burst, 6);
        auto sata_min_init = int_or_default(ci->params, ctx->id("SATA_MIN_INIT"), 0);
        write_int_vector("SATA_MIN_INIT[5:0]", sata_min_init, 6);
        auto sata_min_wake = int_or_default(ci->params, ctx->id("SATA_MIN_WAKE"), 0);
        write_int_vector("SATA_MIN_WAKE[5:0]", sata_min_wake, 6);
        auto sata_cpll_cfg = str_or_default(ci->params, ctx->id("SATA_CPLL_CFG"), "VCO_3000MHZ");
        bool invalid_sata_cpll_cfg =
                sata_cpll_cfg != "VCO_3000MHZ" && sata_cpll_cfg != "VCO_1500MHZ" && sata_cpll_cfg != "VCO_750MHZ";
        if (invalid_sata_cpll_cfg)
            log_error("SATA_CPLL_CFG may only have values 'VCO_3000MHZ', 'VCO_1500MHZ' or 'VCO_750MHZ' but is: '%s'\n", sata_cpll_cfg.c_str());
        write_bit("SATA_CPLL_CFG.VCO_1500MHZ", sata_cpll_cfg == "VCO_1500MHZ");
        write_bit("SATA_CPLL_CFG.VCO_750MHZ",  sata_cpll_cfg == "VCO_750MHZ");

        write_str_bool("SHOW_REALIGN_COMMA", "SHOW_REALIGN_COMMA");

        auto term_rcal_cfg = int_or_default(ci->params, ctx->id("TERM_RCAL_CFG"), 0);
        write_int_vector("TERM_RCAL_CFG[4:0]", term_rcal_cfg, 5);
        auto term_rcal_ovrd = int_or_default(ci->params, ctx->id("TERM_RCAL_OVRD"), 0);
        write_bit("TERM_RCAL_OVRD[0]", term_rcal_ovrd);

        auto trans_time_rate = int_or_default(ci->params, ctx->id("TRANS_TIME_RATE"), 0);
        write_int_vector("TRANS_TIME_RATE[7:0]", trans_time_rate, 8);

        auto tst_rsv = int_or_default(ci->params, ctx->id("TST_RSV"), 0);
        write_int_vector("TST_RSV[31:0]", tst_rsv, 32);

        auto tx_data_width = int_or_default(ci->params, ctx->id("TX_DATA_WIDTH"), 0);
        switch (tx_data_width) {
            case 16:
                tx_data_width = 2; break;
            case 20:
                tx_data_width = 3; break;
            case 32:
                tx_data_width = 4; break;
            case 40:
                tx_data_width = 5; break;
            case 64:
                tx_data_width = 6; break;
            case 80:
                tx_data_width = 7; break;
            default:
                log_error("Invalid TX_DATA_WIDTH parameter '%d' for GTXE2_CHANNEL instance %s\n", tx_data_width, ci->name.c_str(ctx));
        }
        write_int_vector("TX_DATA_WIDTH[2:0]", tx_data_width, 3);
        auto tx_int_datawidth = bool_or_default(ci->params, ctx->id("TX_INT_DATAWIDTH"), false);
        write_bit("TX_INT_DATAWIDTH[0]", tx_int_datawidth);

        auto tx_drive_mode = str_or_default(ci->params, ctx->id("TX_DRIVE_MODE"), "DIRECT");
        bool invalid_tx_drive_mode =
                tx_drive_mode != "DIRECT" && tx_drive_mode != "PIPE" && tx_drive_mode != "PIPEGEN3";
        if (invalid_tx_drive_mode)
            log_error("TX_DRIVE_MODE may only have values 'PIPE',  'PIPEGEN3' or 'DIRECT' but is: '%s'\n", tx_drive_mode.c_str());
        write_bit("TX_DRIVE_MODE.PIPE", tx_drive_mode == "PIPE");
        write_bit("TX_DRIVE_MODE.PIPEGEN3", tx_drive_mode == "PIPEGEN3");

        auto tx_eidle_assert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_ASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_ASSERT_DELAY[2:0]", tx_eidle_assert_delay, 3);
        auto tx_eidle_deassert_delay = int_or_default(ci->params, ctx->id("TX_EIDLE_DEASSERT_DELAY"), 0);
        write_int_vector("TX_EIDLE_DEASSERT_DELAY[2:0]", tx_eidle_deassert_delay, 3);

        write_str_bool("LOOPBACK_DRIVE_HIZ", "LOOPBACK_DRIVE_HIZ");

        auto tx_maincursor_sel = bool_or_default(ci->params, ctx->id("TX_MAINCURSOR_SEL"), false);
        write_bit("TX_MAINCURSOR_SEL[0]", tx_maincursor_sel);
        auto tx_margin_full_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_0"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_0[6:0]", tx_margin_full_0, 7);
        auto tx_margin_full_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_1"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_1[6:0]", tx_margin_full_1, 7);
        auto tx_margin_full_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_2"), 0b1001111);
        write_int_vector("TX_MARGIN_FULL_2[6:0]", tx_margin_full_2, 7);
        auto tx_margin_full_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_3"), 0b1000001);
        write_int_vector("TX_MARGIN_FULL_3[6:0]", tx_margin_full_3, 7);
        auto tx_margin_full_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_FULL_4"), 0b1000000);
        write_int_vector("TX_MARGIN_FULL_4[6:0]", tx_margin_full_4, 7);
        auto tx_margin_low_0 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_0"), 0b1000111);
        write_int_vector("TX_MARGIN_LOW_0[6:0]", tx_margin_low_0, 7);
        auto tx_margin_low_1 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_1"), 0b1000110);
        write_int_vector("TX_MARGIN_LOW_1[6:0]", tx_margin_low_1, 7);
        auto tx_margin_low_2 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_2"), 0b1000100);
        write_int_vector("TX_MARGIN_LOW_2[6:0]", tx_margin_low_2, 7);
        auto tx_margin_low_3 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_3"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_3[6:0]", tx_margin_low_3, 7);
        auto tx_margin_low_4 = int_or_default(ci->params, ctx->id("TX_MARGIN_LOW_4"), 0b1000000);
        write_int_vector("TX_MARGIN_LOW_4[6:0]", tx_margin_low_4, 7);
        auto tx_predriver_mode = bool_or_default(ci->params, ctx->id("TX_PREDRIVER_MODE"), false);
        write_bit("TX_PREDRIVER_MODE[0]", tx_predriver_mode);
        auto tx_rxdetect_cfg = int_or_default(ci->params, ctx->id("TX_RXDETECT_CFG"), 0);
        write_int_vector("TX_RXDETECT_CFG[13:0]", tx_rxdetect_cfg, 14);
        auto tx_rxdetect_ref = int_or_default(ci->params, ctx->id("TX_RXDETECT_REF"), 0);
        write_int_vector("TX_RXDETECT_REF[2:0]", tx_rxdetect_ref, 3);
        auto tx_xclk_sel = str_or_default(ci->params, ctx->id("TX_XCLK_SEL"), "TXUSR");
        bool invalid_tx_xclk_sel = tx_xclk_sel != "TXUSR" && tx_xclk_sel != "TXOUT";
        if (invalid_tx_xclk_sel)
            log_error("TX_XCLK_SEL may only have values 'TXOUT' or 'TXUSR' but is: '%s'\n", tx_xclk_sel.c_str());
        write_bit("TX_XCLK_SEL.TXUSR", tx_xclk_sel == "TXUSR");
        auto tx_clk25_div = int_or_default(ci->params, ctx->id("TX_CLK25_DIV"), 0) - 1;
        write_int_vector("TX_CLK25_DIV[4:0]", tx_clk25_div, 5);
        auto tx_clkmux_pd = bool_or_default(ci->params, ctx->id("TX_CLKMUX_PD"), true);
        write_bit("TX_CLKMUX_PD[0]", tx_clkmux_pd);
        auto tx_deemph0 = int_or_default(ci->params, ctx->id("TX_DEEMPH0"), 0);
        write_int_vector("TX_DEEMPH0[5:0]", tx_deemph0, 6);
        auto tx_deemph1 = int_or_default(ci->params, ctx->id("TX_DEEMPH1"), 0);
        write_int_vector("TX_DEEMPH1[5:0]", tx_deemph1, 6);
        write_str_bool("TXBUF_EN", "TXBUF_EN");
        write_str_bool("TXBUF_RESET_ON_RATE_CHANGE", "TXBUF_RESET_ON_RATE_CHANGE", "TRUE");
        auto txdly_cfg = int_or_default(ci->params, ctx->id("TXDLY_CFG"), 0);
        write_int_vector("TXDLY_CFG[15:0]", txdly_cfg, 16);
        auto txdly_lcfg = int_or_default(ci->params, ctx->id("TXDLY_LCFG"), 0);
        write_int_vector("TXDLY_LCFG[8:0]", txdly_lcfg, 9);
        auto txdly_tap_cfg = int_or_default(ci->params, ctx->id("TXDLY_TAP_CFG"), 0);
        write_int_vector("TXDLY_TAP_CFG[15:0]", txdly_tap_cfg, 16);
        write_str_bool("TXGEARBOX_EN", "TXGEARBOX_EN");
        auto txout_div = std::log2(int_or_default(ci->params, ctx->id("TXOUT_DIV"), 1));
        write_int_vector("TXOUT_DIV[1:0]", txout_div, 2);
        auto txpcsreset_time = int_or_default(ci->params, ctx->id("TXPCSRESET_TIME"), 0);
        write_int_vector("TXPCSRESET_TIME[4:0]", txpcsreset_time, 5);

        auto txph_cfg = int_or_default(ci->params, ctx->id("TXPH_CFG"), 0);
        write_int_vector("TXPH_CFG[15:0]", txph_cfg, 16);
        auto txph_monitor_sel = int_or_default(ci->params, ctx->id("TXPH_MONITOR_SEL"), 0);
        write_int_vector("TXPH_MONITOR_SEL[4:0]", txph_monitor_sel, 5);
        auto txphdly_cfg = int_or_default(ci->params, ctx->id("TXPHDLY_CFG"), 0);
        write_int_vector("TXPHDLY_CFG[23:0]", txphdly_cfg, 24);

        auto txpmareset_time = int_or_default(ci->params, ctx->id("TXPMARESET_TIME"), 0);
        write_int_vector("TXPMARESET_TIME[4:0]", txpmareset_time, 5);

        auto tx_qpi_status_en = bool_or_default(ci->params, ctx->id("TX_QPI_STATUS_EN"), false);
        write_bit("TX_QPI_STATUS_EN[0]", tx_qpi_status_en);

        auto ucodeer_clr = bool_or_default(ci->params, ctx->id("UCODEER_CLR"), false);
        write_bit("UCODEER_CLR[0]", ucodeer_clr);

        pop(); // GTXE2_CHANNEL
        pop(); // tile name
    }

    std::vector<bool> bits_from_string_default(const Property &p, size_t n)
    {
        std::vector<bool> r;
        if (p.is_string) {
            for (char c : p.as_string())
                r.push_back(c == '1');
        } else {
            r = p.as_bits();
        }
        r.resize(n, false);
        return r;
    }

    void write_pcie_2_1(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("PCIE");

        auto write_str_bool = [&](std::string attribute, std::string deflt = "FALSE") {
            auto val = str_or_default(ci->params, ctx->id(attribute), deflt);
            boost::algorithm::to_upper(val);
            write_bit(attribute, val == "TRUE");
        };

        // PG054, page 145
        auto aer_base_ptr = int_or_default(ci->params, ctx->id("AER_BASE_PTR"), 0x140);
        write_int_vector("AER_BASE_PTR[11:0]", aer_base_ptr, 12);

        write_str_bool("AER_CAP_ECRC_CHECK_CAPABLE");
        write_str_bool("DEV_CAP_ROLE_BASED_ERROR", "TRUE");
        write_str_bool("LINK_CAP_SURPRISE_DOWN_ERROR_CAPABLE");
        write_str_bool("AER_CAP_ECRC_GEN_CAPABLE");

        auto aer_cap_id = int_or_default(ci->params, ctx->id("AER_CAP_ID"), 0x1);
        write_int_vector("AER_CAP_ID[15:0]", aer_cap_id, 16);

        write_str_bool("AER_CAP_MULTIHEADER");

        auto aer_cap_nextptr = int_or_default(ci->params, ctx->id("AER_CAP_NEXTPTR"), 0x178);
        write_int_vector("AER_CAP_NEXTPTR[11:0]", aer_cap_nextptr, 12);

        write_str_bool("AER_CAP_ON");

        auto aer_cap_optional_err_support = int_or_default(ci->params, ctx->id("AER_CAP_OPTIONAL_ERR_SUPPORT"), 0x0);
        write_int_vector("AER_CAP_OPTIONAL_ERR_SUPPORT[23:0]", aer_cap_optional_err_support, 24);

        write_str_bool("AER_CAP_PERMIT_ROOTERR_UPDATE", "TRUE");

        auto aer_cap_version = int_or_default(ci->params, ctx->id("AER_CAP_VERSION"), 0x1);
        write_int_vector("AER_CAP_VERSION[3:0]", aer_cap_version, 4);

        write_str_bool("ALLOW_X8_GEN2");

        auto bar0 = int_or_default(ci->params, ctx->id("BAR0"), 0xFFFFFF00);
        write_int_vector("BAR0[31:0]", bar0, 32);

        auto bar1 = int_or_default(ci->params, ctx->id("BAR1"), 0xFFFF0000);
        write_int_vector("BAR1[31:0]", bar1, 32);

        auto bar2 = int_or_default(ci->params, ctx->id("BAR2"), 0xFFFF000C);
        write_int_vector("BAR2[31:0]", bar2, 32);

        auto bar3 = int_or_default(ci->params, ctx->id("BAR3"), 0xFFFFFFFF);
        write_int_vector("BAR3[31:0]", bar3, 32);

        auto bar4 = int_or_default(ci->params, ctx->id("BAR4"), 0x0);
        write_int_vector("BAR4[31:0]", bar4, 32);

        auto bar5 = int_or_default(ci->params, ctx->id("BAR5"), 0x0);
        write_int_vector("BAR5[31:0]", bar5, 32);

        auto capabilities_ptr = int_or_default(ci->params, ctx->id("CAPABILITIES_PTR"), 0x40);
        write_int_vector("CAPABILITIES_PTR[7:0]", capabilities_ptr, 8);

        auto cardbus_cis_pointer = int_or_default(ci->params, ctx->id("CARDBUS_CIS_POINTER"), 0x0);
        write_int_vector("CARDBUS_CIS_POINTER[31:0]", cardbus_cis_pointer, 32);

        auto class_code = int_or_default(ci->params, ctx->id("CLASS_CODE"), 0x0);
        write_int_vector("CLASS_CODE[23:0]", class_code, 24);

        auto cfg_ecrc_err_cplstat = int_or_default(ci->params, ctx->id("CFG_ECRC_ERR_CPLSTAT"), 0);
        write_int_vector("CFG_ECRC_ERR_CPLSTAT[1:0]", cfg_ecrc_err_cplstat, 2);

        write_str_bool("CMD_INTX_IMPLEMENTED", "TRUE");
        write_str_bool("CPL_TIMEOUT_DISABLE_SUPPORTED");

        auto cpl_timeout_ranges_supported = int_or_default(ci->params, ctx->id("CPL_TIMEOUT_RANGES_SUPPORTED"), 0x0);
        write_int_vector("CPL_TIMEOUT_RANGES_SUPPORTED[3:0]", cpl_timeout_ranges_supported, 4);

        auto crm_module_rsts = int_or_default(ci->params, ctx->id("CRM_MODULE_RSTS"), 0x0);
        write_int_vector("CRM_MODULE_RSTS[6:0]", crm_module_rsts, 7);

        write_str_bool("DEV_CAP2_ARI_FORWARDING_SUPPORTED");
        write_str_bool("DEV_CAP2_ATOMICOP32_COMPLETER_SUPPORTED");
        write_str_bool("DEV_CAP2_ATOMICOP64_COMPLETER_SUPPORTED");
        write_str_bool("DEV_CAP2_ATOMICOP_ROUTING_SUPPORTED");
        write_str_bool("DEV_CAP2_CAS128_COMPLETER_SUPPORTED");
        write_str_bool("DEV_CAP2_ENDEND_TLP_PREFIX_SUPPORTED");
        write_str_bool("DEV_CAP2_EXTENDED_FMT_FIELD_SUPPORTED");
        write_str_bool("DEV_CAP2_LTR_MECHANISM_SUPPORTED");

        auto dev_cap2_max_endend_tlp_prefixes = int_or_default(ci->params, ctx->id("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES"), 0x0);
        write_int_vector("DEV_CAP2_MAX_ENDEND_TLP_PREFIXES[1:0]", dev_cap2_max_endend_tlp_prefixes, 2);

        write_str_bool("DEV_CAP2_NO_RO_ENABLED_PRPR_PASSING");

        auto dev_cap2_tph_completer_supported = int_or_default(ci->params, ctx->id("DEV_CAP2_TPH_COMPLETER_SUPPORTED"), 0x0);
        write_int_vector("DEV_CAP2_TPH_COMPLETER_SUPPORTED[1:0]", dev_cap2_tph_completer_supported, 2);

        write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_SCALE", "TRUE");
        write_str_bool("DEV_CAP_ENABLE_SLOT_PWR_LIMIT_VALUE", "TRUE");

        auto dev_cap_endpoint_l0s_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L0S_LATENCY"), 0);
        write_int_vector("DEV_CAP_ENDPOINT_L0S_LATENCY[2:0]", dev_cap_endpoint_l0s_latency, 3);

        auto dev_cap_endpoint_l1_latency = int_or_default(ci->params, ctx->id("DEV_CAP_ENDPOINT_L1_LATENCY"), 0);
        write_int_vector("DEV_CAP_ENDPOINT_L1_LATENCY[2:0]", dev_cap_endpoint_l1_latency, 3);

        write_str_bool("DEV_CAP_EXT_TAG_SUPPORTED", "TRUE");
        write_str_bool("DEV_CAP_FUNCTION_LEVEL_RESET_CAPABLE");

        auto dev_cap_max_payload_supported = int_or_default(ci->params, ctx->id("DEV_CAP_MAX_PAYLOAD_SUPPORTED"), 2);
        write_int_vector("DEV_CAP_MAX_PAYLOAD_SUPPORTED[2:0]", dev_cap_max_payload_supported, 3);

        auto dev_cap_phantom_functions_support = int_or_default(ci->params, ctx->id("DEV_CAP_PHANTOM_FUNCTIONS_SUPPORT"), 0);
        write_int_vector("DEV_CAP_PHANTOM_FUNCTIONS_SUPPORT[1:0]", dev_cap_phantom_functions_support, 2);

        auto dev_cap_rsvd_14_12 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_14_12"), 0);
        write_int_vector("DEV_CAP_RSVD_14_12[2:0]", dev_cap_rsvd_14_12, 3);

        auto dev_cap_rsvd_17_16 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_17_16"), 0);
        write_int_vector("DEV_CAP_RSVD_17_16[1:0]", dev_cap_rsvd_17_16, 2);

        auto dev_cap_rsvd_31_29 = int_or_default(ci->params, ctx->id("DEV_CAP_RSVD_31_29"), 0);
        write_int_vector("DEV_CAP_RSVD_31_29[2:0]", dev_cap_rsvd_31_29, 3);

        write_str_bool("DEV_CONTROL_AUX_POWER_SUPPORTED");
        write_str_bool("DEV_CONTROL_EXT_TAG_DEFAULT");
        write_str_bool("DISABLE_ASPM_L1_TIMER");
        write_str_bool("DISABLE_BAR_FILTERING");
        write_str_bool("DISABLE_ERR_MSG");
        write_str_bool("DISABLE_ID_CHECK");
        write_str_bool("DISABLE_LANE_REVERSAL");
        write_str_bool("DISABLE_LOCKED_FILTER");
        write_str_bool("DISABLE_PPM_FILTER");
        write_str_bool("DISABLE_RX_POISONED_RESP");
        write_str_bool("DISABLE_RX_TC_FILTER");
        write_str_bool("DISABLE_SCRAMBLING");

        auto dnstream_link_num = int_or_default(ci->params, ctx->id("DNSTREAM_LINK_NUM"), 0x0);
        write_int_vector("DNSTREAM_LINK_NUM[7:0]", dnstream_link_num, 8);

        auto dsn_base_ptr = int_or_default(ci->params, ctx->id("DSN_BASE_PTR"), 0x100);
        write_int_vector("DSN_BASE_PTR[11:0]", dsn_base_ptr, 12);

        auto dsn_cap_id = int_or_default(ci->params, ctx->id("DSN_CAP_ID"), 0x3);
        write_int_vector("DSN_CAP_ID[15:0]", dsn_cap_id, 16);

        auto dsn_cap_nextptr = int_or_default(ci->params, ctx->id("DSN_CAP_NEXTPTR"), 0x10C);
        write_int_vector("DSN_CAP_NEXTPTR[11:0]", dsn_cap_nextptr, 12);

        write_str_bool("DSN_CAP_ON", "TRUE");

        auto dsn_cap_version = int_or_default(ci->params, ctx->id("DSN_CAP_VERSION"), 0x1);
        write_int_vector("DSN_CAP_VERSION[3:0]", dsn_cap_version, 4);

        auto enable_msg_route = int_or_default(ci->params, ctx->id("ENABLE_MSG_ROUTE"), 0x0);
        write_int_vector("ENABLE_MSG_ROUTE[10:0]", enable_msg_route, 11);

        write_str_bool("ENABLE_RX_TD_ECRC_TRIM");
        write_str_bool("ENDEND_TLP_PREFIX_FORWARDING_SUPPORTED");
        write_str_bool("ENTER_RVRY_EI_L0", "TRUE");
        write_str_bool("EXIT_LOOPBACK_ON_EI", "TRUE");

        auto expansion_rom = int_or_default(ci->params, ctx->id("EXPANSION_ROM"), 0xFFFFF001);
        write_int_vector("EXPANSION_ROM[31:0]", expansion_rom, 32);

        auto ext_cfg_cap_ptr = int_or_default(ci->params, ctx->id("EXT_CFG_CAP_PTR"), 0x3F);
        write_int_vector("EXT_CFG_CAP_PTR[5:0]", ext_cfg_cap_ptr, 6);

        auto ext_cfg_xp_cap_ptr = int_or_default(ci->params, ctx->id("EXT_CFG_XP_CAP_PTR"), 0x3FF);
        write_int_vector("EXT_CFG_XP_CAP_PTR[9:0]", ext_cfg_xp_cap_ptr, 10);

        auto header_type = int_or_default(ci->params, ctx->id("HEADER_TYPE"), 0x0);
        write_int_vector("HEADER_TYPE[7:0]", header_type, 8);

        auto infer_ei = int_or_default(ci->params, ctx->id("INFER_EI"), 0x0);
        write_int_vector("INFER_EI[4:0]", infer_ei, 5);

        auto interrupt_pin = int_or_default(ci->params, ctx->id("INTERRUPT_PIN"), 0x1);
        write_int_vector("INTERRUPT_PIN[7:0]", interrupt_pin, 8);

        write_str_bool("INTERRUPT_STAT_AUTO", "TRUE");
        write_str_bool("IS_SWITCH");

        auto last_config_dword = int_or_default(ci->params, ctx->id("LAST_CONFIG_DWORD"), 0x3FF);
        write_int_vector("LAST_CONFIG_DWORD[9:0]", last_config_dword, 10);

        write_str_bool("LINK_CAP_ASPM_OPTIONALITY", "TRUE");

        auto link_cap_aspm_support = int_or_default(ci->params, ctx->id("LINK_CAP_ASPM_SUPPORT"), 1);
        write_int_vector("LINK_CAP_ASPM_SUPPORT[1:0]", link_cap_aspm_support, 2);

        write_str_bool("LINK_CAP_CLOCK_POWER_MANAGEMENT");
        write_str_bool("LINK_CAP_DLL_LINK_ACTIVE_REPORTING_CAP");

        auto link_cap_l0s_exit_latency_comclk_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN1"), 7);
        write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN1[2:0]", link_cap_l0s_exit_latency_comclk_gen1, 3);

        auto link_cap_l0s_exit_latency_comclk_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN2"), 7);
        write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN2[2:0]", link_cap_l0s_exit_latency_comclk_gen2, 3);

        auto link_cap_l0s_exit_latency_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_GEN1"), 7);
        write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_GEN1[2:0]", link_cap_l0s_exit_latency_gen1, 3);

        auto link_cap_l0s_exit_latency_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L0S_EXIT_LATENCY_GEN2"), 7);
        write_int_vector("LINK_CAP_L0S_EXIT_LATENCY_GEN2[2:0]", link_cap_l0s_exit_latency_gen2, 3);

        auto link_cap_l1_exit_latency_comclk_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN1"), 7);
        write_int_vector("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN1[2:0]", link_cap_l1_exit_latency_comclk_gen1, 3);

        auto link_cap_l1_exit_latency_comclk_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN2"), 7);
        write_int_vector("LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN2[2:0]", link_cap_l1_exit_latency_comclk_gen2, 3);

        auto link_cap_l1_exit_latency_gen1 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_GEN1"), 7);
        write_int_vector("LINK_CAP_L1_EXIT_LATENCY_GEN1[2:0]", link_cap_l1_exit_latency_gen1, 3);

        auto link_cap_l1_exit_latency_gen2 = int_or_default(ci->params, ctx->id("LINK_CAP_L1_EXIT_LATENCY_GEN2"), 7);
        write_int_vector("LINK_CAP_L1_EXIT_LATENCY_GEN2[2:0]", link_cap_l1_exit_latency_gen2, 3);

        write_str_bool("LINK_CAP_LINK_BANDWIDTH_NOTIFICATION_CAP");

        auto link_cap_max_link_speed = int_or_default(ci->params, ctx->id("LINK_CAP_MAX_LINK_SPEED"), 0x1);
        write_int_vector("LINK_CAP_MAX_LINK_SPEED[3:0]", link_cap_max_link_speed, 4);

        auto link_cap_max_link_width = int_or_default(ci->params, ctx->id("LINK_CAP_MAX_LINK_WIDTH"), 0x8);
        write_int_vector("LINK_CAP_MAX_LINK_WIDTH[5:0]", link_cap_max_link_width, 6);

        auto link_cap_rsvd_23 = int_or_default(ci->params, ctx->id("LINK_CAP_RSVD_23"), 0);
        write_int_vector("LINK_CAP_RSVD_23[15:0]", link_cap_rsvd_23, 16);

        auto link_control_rcb = int_or_default(ci->params, ctx->id("LINK_CONTROL_RCB"), 0);
        write_int_vector("LINK_CONTROL_RCB[1:0]", link_control_rcb, 2);

        write_str_bool("LINK_CTRL2_DEEMPHASIS");
        write_str_bool("LINK_CTRL2_HW_AUTONOMOUS_SPEED_DISABLE");

        auto link_ctrl2_target_link_speed = int_or_default(ci->params, ctx->id("LINK_CTRL2_TARGET_LINK_SPEED"), 0x2);
        write_int_vector("LINK_CTRL2_TARGET_LINK_SPEED[3:0]", link_ctrl2_target_link_speed, 4);

        write_str_bool("LINK_STATUS_SLOT_CLOCK_CONFIG", "TRUE");

        auto ll_ack_timeout = int_or_default(ci->params, ctx->id("LL_ACK_TIMEOUT"), 0x0);
        write_int_vector("LL_ACK_TIMEOUT[14:0]", ll_ack_timeout, 15);

        write_str_bool("LL_ACK_TIMEOUT_EN");

        auto ll_ack_timeout_func = int_or_default(ci->params, ctx->id("LL_ACK_TIMEOUT_FUNC"), 0);
        write_int_vector("LL_ACK_TIMEOUT_FUNC[1:0]", ll_ack_timeout_func, 2);

        auto ll_replay_timeout = int_or_default(ci->params, ctx->id("LL_REPLAY_TIMEOUT"), 0x0);
        write_int_vector("LL_REPLAY_TIMEOUT[14:0]", ll_replay_timeout, 15);

        write_str_bool("LL_REPLAY_TIMEOUT_EN");

        auto ll_replay_timeout_func = int_or_default(ci->params, ctx->id("LL_REPLAY_TIMEOUT_FUNC"), 0);
        write_int_vector("LL_REPLAY_TIMEOUT_FUNC[1:0]", ll_replay_timeout_func, 2);

        auto ltssm_max_link_width = int_or_default(ci->params, ctx->id("LTSSM_MAX_LINK_WIDTH"), 0x1);
        write_int_vector("LTSSM_MAX_LINK_WIDTH[5:0]", ltssm_max_link_width, 6);

        write_str_bool("MPS_FORCE");

        auto msix_base_ptr = int_or_default(ci->params, ctx->id("MSIX_BASE_PTR"), 0x9C);
        write_int_vector("MSIX_BASE_PTR[7:0]", msix_base_ptr, 8);

        auto msix_cap_id = int_or_default(ci->params, ctx->id("MSIX_CAP_ID"), 0x11);
        write_int_vector("MSIX_CAP_ID[7:0]", msix_cap_id, 8);

        auto msix_cap_nextptr = int_or_default(ci->params, ctx->id("MSIX_CAP_NEXTPTR"), 0x0);
        write_int_vector("MSIX_CAP_NEXTPTR[7:0]", msix_cap_nextptr, 8);

        write_str_bool("MSIX_CAP_ON");

        auto msix_cap_pba_bir = int_or_default(ci->params, ctx->id("MSIX_CAP_PBA_BIR"), 0);
        write_int_vector("MSIX_CAP_PBA_BIR[2:0]", msix_cap_pba_bir, 3);

        auto msix_cap_pba_offset = int_or_default(ci->params, ctx->id("MSIX_CAP_PBA_OFFSET"), 0x50);
        write_int_vector("MSIX_CAP_PBA_OFFSET[28:0]", msix_cap_pba_offset, 29);

        auto msix_cap_table_bir = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_BIR"), 0);
        write_int_vector("MSIX_CAP_TABLE_BIR[2:0]", msix_cap_table_bir, 3);

        auto msix_cap_table_offset = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_OFFSET"), 0x40);
        write_int_vector("MSIX_CAP_TABLE_OFFSET[28:0]", msix_cap_table_offset, 29);

        auto msix_cap_table_size = int_or_default(ci->params, ctx->id("MSIX_CAP_TABLE_SIZE"), 0x0);
        write_int_vector("MSIX_CAP_TABLE_SIZE[10:0]", msix_cap_table_size, 11);

        auto msi_base_ptr = int_or_default(ci->params, ctx->id("MSI_BASE_PTR"), 0x48);
        write_int_vector("MSI_BASE_PTR[7:0]", msi_base_ptr, 8);

        write_str_bool("MSI_CAP_64_BIT_ADDR_CAPABLE", "TRUE");

        auto msi_cap_id = int_or_default(ci->params, ctx->id("MSI_CAP_ID"), 0x5);
        write_int_vector("MSI_CAP_ID[7:0]", msi_cap_id, 8);

        auto msi_cap_multimsgcap = int_or_default(ci->params, ctx->id("MSI_CAP_MULTIMSGCAP"), 0);
        write_int_vector("MSI_CAP_MULTIMSGCAP[2:0]", msi_cap_multimsgcap, 3);

        auto msi_cap_multimsg_extension = int_or_default(ci->params, ctx->id("MSI_CAP_MULTIMSG_EXTENSION"), 0);
        write_int_vector("MSI_CAP_MULTIMSG_EXTENSION[8:0]", msi_cap_multimsg_extension, 9);

        auto msi_cap_nextptr = int_or_default(ci->params, ctx->id("MSI_CAP_NEXTPTR"), 0x60);
        write_int_vector("MSI_CAP_NEXTPTR[7:0]", msi_cap_nextptr, 8);

        write_str_bool("MSI_CAP_ON");
        write_str_bool("MSI_CAP_PER_VECTOR_MASKING_CAPABLE", "TRUE");

        auto n_fts_comclk_gen1 = int_or_default(ci->params, ctx->id("N_FTS_COMCLK_GEN1"), 255);
        write_int_vector("N_FTS_COMCLK_GEN1[7:0]", n_fts_comclk_gen1, 8);

        auto n_fts_comclk_gen2 = int_or_default(ci->params, ctx->id("N_FTS_COMCLK_GEN2"), 255);
        write_int_vector("N_FTS_COMCLK_GEN2[7:0]", n_fts_comclk_gen2, 8);

        auto n_fts_gen1 = int_or_default(ci->params, ctx->id("N_FTS_GEN1"), 255);
        write_int_vector("N_FTS_GEN1[7:0]", n_fts_gen1, 8);

        auto n_fts_gen2 = int_or_default(ci->params, ctx->id("N_FTS_GEN2"), 255);
        write_int_vector("N_FTS_GEN2[7:0]", n_fts_gen2, 8);

        auto pcie_base_ptr = int_or_default(ci->params, ctx->id("PCIE_BASE_PTR"), 0x60);
        write_int_vector("PCIE_BASE_PTR[7:0]", pcie_base_ptr, 8);

        auto pcie_cap_capability_id = int_or_default(ci->params, ctx->id("PCIE_CAP_CAPABILITY_ID"), 0x10);
        write_int_vector("PCIE_CAP_CAPABILITY_ID[7:0]", pcie_cap_capability_id, 8);

        auto pcie_cap_capability_version = int_or_default(ci->params, ctx->id("PCIE_CAP_CAPABILITY_VERSION"), 0x2);
        write_int_vector("PCIE_CAP_CAPABILITY_VERSION[3:0]", pcie_cap_capability_version, 4);

        auto pcie_cap_device_port_type = int_or_default(ci->params, ctx->id("PCIE_CAP_DEVICE_PORT_TYPE"), 0x0);
        write_int_vector("PCIE_CAP_DEVICE_PORT_TYPE[3:0]", pcie_cap_device_port_type, 4);

        auto pcie_cap_nextptr = int_or_default(ci->params, ctx->id("PCIE_CAP_NEXTPTR"), 0x9C);
        write_int_vector("PCIE_CAP_NEXTPTR[7:0]", pcie_cap_nextptr, 8);

        write_str_bool("PCIE_CAP_ON", "TRUE");

        auto pcie_cap_rsvd_15_14 = int_or_default(ci->params, ctx->id("PCIE_CAP_RSVD_15_14"), 0);
        write_int_vector("PCIE_CAP_RSVD_15_14[1:0]", pcie_cap_rsvd_15_14, 2);

        write_str_bool("PCIE_CAP_SLOT_IMPLEMENTED");

        auto pcie_revision = int_or_default(ci->params, ctx->id("PCIE_REVISION"), 2);
        write_int_vector("PCIE_REVISION[3:0]", pcie_revision, 4);

        auto pl_auto_config = int_or_default(ci->params, ctx->id("PL_AUTO_CONFIG"), 0);
        write_int_vector("PL_AUTO_CONFIG[2:0]", pl_auto_config, 3);

        write_str_bool("PL_FAST_TRAIN", "TRUE");

        auto pm_aspml0s_timeout = int_or_default(ci->params, ctx->id("PM_ASPML0S_TIMEOUT"), 0x0);
        write_int_vector("PM_ASPML0S_TIMEOUT[14:0]", pm_aspml0s_timeout, 15);

        write_str_bool("PM_ASPML0S_TIMEOUT_EN");

        auto pm_aspml0s_timeout_func = int_or_default(ci->params, ctx->id("PM_ASPML0S_TIMEOUT_FUNC"), 0);
        write_int_vector("PM_ASPML0S_TIMEOUT_FUNC[1:0]", pm_aspml0s_timeout_func, 2);

        write_str_bool("PM_ASPM_FASTEXIT");

        auto pm_base_ptr = int_or_default(ci->params, ctx->id("PM_BASE_PTR"), 0x40);
        write_int_vector("PM_BASE_PTR[7:0]", pm_base_ptr, 8);

        auto pm_cap_auxcurrent = int_or_default(ci->params, ctx->id("PM_CAP_AUXCURRENT"), 0);
        write_int_vector("PM_CAP_AUXCURRENT[2:0]", pm_cap_auxcurrent, 3);

        write_str_bool("PM_CAP_D1SUPPORT", "TRUE");
        write_str_bool("PM_CAP_D2SUPPORT", "TRUE");
        write_str_bool("PM_CAP_DSI");

        auto pm_cap_id = int_or_default(ci->params, ctx->id("PM_CAP_ID"), 0x1);
        write_int_vector("PM_CAP_ID[7:0]", pm_cap_id, 8);

        auto pm_cap_nextptr = int_or_default(ci->params, ctx->id("PM_CAP_NEXTPTR"), 0x48);
        write_int_vector("PM_CAP_NEXTPTR[7:0]", pm_cap_nextptr, 8);

        write_str_bool("PM_CAP_ON", "TRUE");

        auto pm_cap_pmesupport = int_or_default(ci->params, ctx->id("PM_CAP_PMESUPPORT"), 0xF);
        write_int_vector("PM_CAP_PMESUPPORT[4:0]", pm_cap_pmesupport, 5);

        write_str_bool("PM_CAP_PME_CLOCK");

        auto pm_cap_rsvd_04 = int_or_default(ci->params, ctx->id("PM_CAP_RSVD_04"), 0);
        write_int_vector("PM_CAP_RSVD_04[15:0]", pm_cap_rsvd_04, 16);

        auto pm_cap_version = int_or_default(ci->params, ctx->id("PM_CAP_VERSION"), 3);
        write_int_vector("PM_CAP_VERSION[2:0]", pm_cap_version, 3);

        write_str_bool("PM_CSR_B2B3");
        write_str_bool("PM_CSR_BPCCEN");
        write_str_bool("PM_CSR_NOSOFTRST", "TRUE");

        auto pm_data0 = int_or_default(ci->params, ctx->id("PM_DATA0"), 0x1);
        write_int_vector("PM_DATA0[7:0]", pm_data0, 8);
        auto pm_data1 = int_or_default(ci->params, ctx->id("PM_DATA1"), 0x1);
        write_int_vector("PM_DATA1[7:0]", pm_data1, 8);
        auto pm_data2 = int_or_default(ci->params, ctx->id("PM_DATA2"), 0x1);
        write_int_vector("PM_DATA2[7:0]", pm_data2, 8);
        auto pm_data3 = int_or_default(ci->params, ctx->id("PM_DATA3"), 0x1);
        write_int_vector("PM_DATA3[7:0]", pm_data3, 8);
        auto pm_data4 = int_or_default(ci->params, ctx->id("PM_DATA4"), 0x1);
        write_int_vector("PM_DATA4[7:0]", pm_data4, 8);
        auto pm_data5 = int_or_default(ci->params, ctx->id("PM_DATA5"), 0x1);
        write_int_vector("PM_DATA5[7:0]", pm_data5, 8);
        auto pm_data6 = int_or_default(ci->params, ctx->id("PM_DATA6"), 0x1);
        write_int_vector("PM_DATA6[7:0]", pm_data6, 8);
        auto pm_data7 = int_or_default(ci->params, ctx->id("PM_DATA7"), 0x1);
        write_int_vector("PM_DATA7[7:0]", pm_data7, 8);

        auto pm_data_scale0 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE0"), 0x1);
        write_int_vector("PM_DATA_SCALE0[1:0]", pm_data_scale0, 2);
        auto pm_data_scale1 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE1"), 0x1);
        write_int_vector("PM_DATA_SCALE1[1:0]", pm_data_scale1, 2);
        auto pm_data_scale2 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE2"), 0x1);
        write_int_vector("PM_DATA_SCALE2[1:0]", pm_data_scale2, 2);
        auto pm_data_scale3 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE3"), 0x1);
        write_int_vector("PM_DATA_SCALE3[1:0]", pm_data_scale3, 2);
        auto pm_data_scale4 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE4"), 0x1);
        write_int_vector("PM_DATA_SCALE4[1:0]", pm_data_scale4, 2);
        auto pm_data_scale5 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE5"), 0x1);
        write_int_vector("PM_DATA_SCALE5[1:0]", pm_data_scale5, 2);
        auto pm_data_scale6 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE6"), 0x1);
        write_int_vector("PM_DATA_SCALE6[1:0]", pm_data_scale6, 2);
        auto pm_data_scale7 = int_or_default(ci->params, ctx->id("PM_DATA_SCALE7"), 0x1);
        write_int_vector("PM_DATA_SCALE7[1:0]", pm_data_scale7, 2);

        write_str_bool("PM_MF");

        auto rbar_base_ptr = int_or_default(ci->params, ctx->id("RBAR_BASE_PTR"), 0x178);
        write_int_vector("RBAR_BASE_PTR[11:0]", rbar_base_ptr, 12);

        auto rbar_cap_control_encodedbar0 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR0"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR0[4:0]", rbar_cap_control_encodedbar0, 5);

        auto rbar_cap_control_encodedbar1 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR1"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR1[4:0]", rbar_cap_control_encodedbar1, 5);

        auto rbar_cap_control_encodedbar2 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR2"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR2[4:0]", rbar_cap_control_encodedbar2, 5);

        auto rbar_cap_control_encodedbar3 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR3"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR3[4:0]", rbar_cap_control_encodedbar3, 5);

        auto rbar_cap_control_encodedbar4 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR4"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR4[4:0]", rbar_cap_control_encodedbar4, 5);

        auto rbar_cap_control_encodedbar5 = int_or_default(ci->params, ctx->id("RBAR_CAP_CONTROL_ENCODEDBAR5"), 0x0);
        write_int_vector("RBAR_CAP_CONTROL_ENCODEDBAR5[4:0]", rbar_cap_control_encodedbar5, 5);

        auto rbar_cap_id = int_or_default(ci->params, ctx->id("RBAR_CAP_ID"), 0x15);
        write_int_vector("RBAR_CAP_ID[15:0]", rbar_cap_id, 16);

        auto rbar_cap_index0 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX0"), 0x0);
        write_int_vector("RBAR_CAP_INDEX0[2:0]", rbar_cap_index0, 3);

        auto rbar_cap_index1 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX1"), 0x0);
        write_int_vector("RBAR_CAP_INDEX1[2:0]", rbar_cap_index1, 3);

        auto rbar_cap_index2 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX2"), 0x0);
        write_int_vector("RBAR_CAP_INDEX2[2:0]", rbar_cap_index2, 3);

        auto rbar_cap_index3 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX3"), 0x0);
        write_int_vector("RBAR_CAP_INDEX3[2:0]", rbar_cap_index3, 3);

        auto rbar_cap_index4 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX4"), 0x0);
        write_int_vector("RBAR_CAP_INDEX4[2:0]", rbar_cap_index4, 3);

        auto rbar_cap_index5 = int_or_default(ci->params, ctx->id("RBAR_CAP_INDEX5"), 0x0);
        write_int_vector("RBAR_CAP_INDEX5[2:0]", rbar_cap_index5, 3);

        auto rbar_cap_nextptr = int_or_default(ci->params, ctx->id("RBAR_CAP_NEXTPTR"), 0x0);
        write_int_vector("RBAR_CAP_NEXTPTR[11:0]", rbar_cap_nextptr, 12);

        write_str_bool("RBAR_CAP_ON");

        auto rbar_cap_sup0 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP0"), 0x0);
        write_int_vector("RBAR_CAP_SUP0[31:0]", rbar_cap_sup0, 32);

        auto rbar_cap_sup1 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP1"), 0x0);
        write_int_vector("RBAR_CAP_SUP1[31:0]", rbar_cap_sup1, 32);

        auto rbar_cap_sup2 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP2"), 0x0);
        write_int_vector("RBAR_CAP_SUP2[31:0]", rbar_cap_sup2, 32);

        auto rbar_cap_sup3 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP3"), 0x0);
        write_int_vector("RBAR_CAP_SUP3[31:0]", rbar_cap_sup3, 32);

        auto rbar_cap_sup4 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP4"), 0x0);
        write_int_vector("RBAR_CAP_SUP4[31:0]", rbar_cap_sup4, 32);

        auto rbar_cap_sup5 = int_or_default(ci->params, ctx->id("RBAR_CAP_SUP5"), 0x0);
        write_int_vector("RBAR_CAP_SUP5[31:0]", rbar_cap_sup5, 32);

        auto rbar_cap_version = int_or_default(ci->params, ctx->id("RBAR_CAP_VERSION"), 0x1);
        write_int_vector("RBAR_CAP_VERSION[3:0]", rbar_cap_version, 4);

        auto rbar_num = int_or_default(ci->params, ctx->id("RBAR_NUM"), 0x1);
        write_int_vector("RBAR_NUM[2:0]", rbar_num, 3);

        auto recrc_chk = int_or_default(ci->params, ctx->id("RECRC_CHK"), 0);
        write_int_vector("RECRC_CHK[1:0]", recrc_chk, 2);

        write_str_bool("RECRC_CHK_TRIM");
        write_str_bool("ROOT_CAP_CRS_SW_VISIBILITY");

        auto rp_auto_spd = int_or_default(ci->params, ctx->id("RP_AUTO_SPD"), 0x1);
        write_int_vector("RP_AUTO_SPD[1:0]", rp_auto_spd, 2);

        auto rp_auto_spd_loopcnt = int_or_default(ci->params, ctx->id("RP_AUTO_SPD_LOOPCNT"), 0x1F);
        write_int_vector("RP_AUTO_SPD_LOOPCNT[4:0]", rp_auto_spd_loopcnt, 5);

        write_str_bool("SELECT_DLL_IF");
        write_str_bool("SLOT_CAP_ATT_BUTTON_PRESENT");
        write_str_bool("SLOT_CAP_ATT_INDICATOR_PRESENT");
        write_str_bool("SLOT_CAP_ELEC_INTERLOCK_PRESENT");
        write_str_bool("SLOT_CAP_HOTPLUG_CAPABLE");
        write_str_bool("SLOT_CAP_HOTPLUG_SURPRISE");
        write_str_bool("SLOT_CAP_MRL_SENSOR_PRESENT");
        write_str_bool("SLOT_CAP_NO_CMD_COMPLETED_SUPPORT");

        auto slot_cap_physical_slot_num = int_or_default(ci->params, ctx->id("SLOT_CAP_PHYSICAL_SLOT_NUM"), 0x0);
        write_int_vector("SLOT_CAP_PHYSICAL_SLOT_NUM[12:0]", slot_cap_physical_slot_num, 13);

        write_str_bool("SLOT_CAP_POWER_CONTROLLER_PRESENT");
        write_str_bool("SLOT_CAP_POWER_INDICATOR_PRESENT");

        auto slot_cap_slot_power_limit_scale = int_or_default(ci->params, ctx->id("SLOT_CAP_SLOT_POWER_LIMIT_SCALE"), 0);
        write_int_vector("SLOT_CAP_SLOT_POWER_LIMIT_SCALE[1:0]", slot_cap_slot_power_limit_scale, 2);

        auto slot_cap_slot_power_limit_value = int_or_default(ci->params, ctx->id("SLOT_CAP_SLOT_POWER_LIMIT_VALUE"), 0x0);
        write_int_vector("SLOT_CAP_SLOT_POWER_LIMIT_VALUE[7:0]", slot_cap_slot_power_limit_value, 8);

        write_str_bool("SSL_MESSAGE_AUTO");
        write_str_bool("TECRC_EP_INV");
        write_str_bool("TL_RBYPASS");

        auto tl_rx_ram_raddr_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_RADDR_LATENCY"), 0);
        write_int_vector("TL_RX_RAM_RADDR_LATENCY[1:0]", tl_rx_ram_raddr_latency, 2);

        auto tl_rx_ram_rdata_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_RDATA_LATENCY"), 2);
        write_int_vector("TL_RX_RAM_RDATA_LATENCY[1:0]", tl_rx_ram_rdata_latency, 2);

        auto tl_rx_ram_write_latency = int_or_default(ci->params, ctx->id("TL_RX_RAM_WRITE_LATENCY"), 0);
        write_int_vector("TL_RX_RAM_WRITE_LATENCY[4:0]", tl_rx_ram_write_latency, 5);

        write_str_bool("TL_TFC_DISABLE");
        write_str_bool("TL_TX_CHECKS_DISABLE");

        auto tl_tx_ram_raddr_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_RADDR_LATENCY"), 0);
        write_int_vector("TL_TX_RAM_RADDR_LATENCY[13:0]", tl_tx_ram_raddr_latency, 14);

        auto tl_tx_ram_rdata_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_RDATA_LATENCY"), 2);
        write_int_vector("TL_TX_RAM_RDATA_LATENCY[1:0]", tl_tx_ram_rdata_latency, 2);

        auto tl_tx_ram_write_latency = int_or_default(ci->params, ctx->id("TL_TX_RAM_WRITE_LATENCY"), 0);
        write_int_vector("TL_TX_RAM_WRITE_LATENCY[0:0]", tl_tx_ram_write_latency, 1);

        write_str_bool("TRN_DW");
        write_str_bool("TRN_NP_FC");
        write_str_bool("UPCONFIG_CAPABLE", "TRUE");
        write_str_bool("UPSTREAM_FACING", "TRUE");
        write_str_bool("UR_ATOMIC", "TRUE");
        write_str_bool("UR_CFG1", "TRUE");
        write_str_bool("UR_INV_REQ", "TRUE");
        write_str_bool("UR_PRS_RESPONSE", "TRUE");
        write_str_bool("USER_CLK2_DIV2");

        auto user_clk_freq = int_or_default(ci->params, ctx->id("USER_CLK_FREQ"), 3);
        write_int_vector("USER_CLK_FREQ[2:0]", user_clk_freq, 3);

        write_str_bool("USE_RID_PINS");
        write_str_bool("VC0_CPL_INFINITE", "TRUE");

        auto vc0_rx_ram_limit = int_or_default(ci->params, ctx->id("VC0_RX_RAM_LIMIT"), 0x3FF);
        write_int_vector("VC0_RX_RAM_LIMIT[12:0]", vc0_rx_ram_limit, 13);

        auto vc0_total_credits_cd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_CD"), 127);
        write_int_vector("VC0_TOTAL_CREDITS_CD[10:0]", vc0_total_credits_cd, 11);

        auto vc0_total_credits_ch = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_CH"), 31);
        write_int_vector("VC0_TOTAL_CREDITS_CH[6:0]", vc0_total_credits_ch, 7);

        auto vc0_total_credits_npd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_NPD"), 24);
        write_int_vector("VC0_TOTAL_CREDITS_NPD[10:0]", vc0_total_credits_npd, 11);

        auto vc0_total_credits_nph = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_NPH"), 12);
        write_int_vector("VC0_TOTAL_CREDITS_NPH[6:0]", vc0_total_credits_nph, 7);

        auto vc0_total_credits_pd = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_PD"), 288);
        write_int_vector("VC0_TOTAL_CREDITS_PD[10:0]", vc0_total_credits_pd, 11);

        auto vc0_total_credits_ph = int_or_default(ci->params, ctx->id("VC0_TOTAL_CREDITS_PH"), 32);
        write_int_vector("VC0_TOTAL_CREDITS_PH[6:0]", vc0_total_credits_ph, 7);

        auto vc0_tx_lastpacket = int_or_default(ci->params, ctx->id("VC0_TX_LASTPACKET"), 31);
        write_int_vector("VC0_TX_LASTPACKET[4:0]", vc0_tx_lastpacket, 5);

        auto vc_base_ptr = int_or_default(ci->params, ctx->id("VC_BASE_PTR"), 0x10C);
        write_int_vector("VC_BASE_PTR[11:0]", vc_base_ptr, 12);

        auto vc_cap_id = int_or_default(ci->params, ctx->id("VC_CAP_ID"), 0x2);
        write_int_vector("VC_CAP_ID[15:0]", vc_cap_id, 16);

        auto vc_cap_nextptr = int_or_default(ci->params, ctx->id("VC_CAP_NEXTPTR"), 0x0);
        write_int_vector("VC_CAP_NEXTPTR[11:0]", vc_cap_nextptr, 12);

        write_str_bool("VC_CAP_ON");
        write_str_bool("VC_CAP_REJECT_SNOOP_TRANSACTIONS");

        auto vc_cap_version = int_or_default(ci->params, ctx->id("VC_CAP_VERSION"), 0x1);
        write_int_vector("VC_CAP_VERSION[3:0]", vc_cap_version, 4);

        auto vsec_base_ptr = int_or_default(ci->params, ctx->id("VSEC_BASE_PTR"), 0x128);
        write_int_vector("VSEC_BASE_PTR[11:0]", vsec_base_ptr, 12);

        auto vsec_cap_hdr_id = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_ID"), 0x1234);
        write_int_vector("VSEC_CAP_HDR_ID[15:0]", vsec_cap_hdr_id, 16);

        auto vsec_cap_hdr_length = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_LENGTH"), 0x18);
        write_int_vector("VSEC_CAP_HDR_LENGTH[11:0]", vsec_cap_hdr_length, 12);

        auto vsec_cap_hdr_revision = int_or_default(ci->params, ctx->id("VSEC_CAP_HDR_REVISION"), 0x1);
        write_int_vector("VSEC_CAP_HDR_REVISION[3:0]", vsec_cap_hdr_revision, 4);

        auto vsec_cap_id = int_or_default(ci->params, ctx->id("VSEC_CAP_ID"), 0xB);
        write_int_vector("VSEC_CAP_ID[15:0]", vsec_cap_id, 16);

        write_str_bool("VSEC_CAP_IS_LINK_VISIBLE", "TRUE");

        auto vsec_cap_nextptr = int_or_default(ci->params, ctx->id("VSEC_CAP_NEXTPTR"), 0x140);
        write_int_vector("VSEC_CAP_NEXTPTR[11:0]", vsec_cap_nextptr, 12);

        write_str_bool("VSEC_CAP_ON");

        auto vsec_cap_version = int_or_default(ci->params, ctx->id("VSEC_CAP_VERSION"), 0x1);
        write_int_vector("VSEC_CAP_VERSION[3:0]", vsec_cap_version, 4);

        write_str_bool("ENABLE_JTAG_DBG");

        pop(); // PCIE_2_1
        pop(); // tile name
    }

    void write_ip()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            bool is_placed = ci->bel != BelId();
            if (ci->type == id_DSP48E1_DSP48E1) {
                write_dsp_cell(ci);
                blank();
            } else if (ci->type == id_BUFR_BUFR && is_placed) {
                write_bufr(ci);
                blank();
            } else if (ci->type == id_BUFIO_BUFIO && is_placed) {
                write_bufio(ci);
                blank();
            } else if (ci->type == id_GTPE2_COMMON) {
                write_gtp_pll(ci);
                blank();
            } else if (ci->type == id_GTPE2_CHANNEL) {
                write_gtp_channel(ci);
                blank();
            } else if (ci->type == id_GTXE2_COMMON) {
                write_gtx_pll(ci);
                blank();
            } else if (ci->type == id_GTXE2_CHANNEL) {
                write_gtx_channel(ci);
                blank();
            } else if (ci->type == id_IBUFDS_GTE2 && is_placed) {
                write_ibufds_gte2(ci);
                blank();
            } else if (ci->type == id_PCIE_2_1_PCIE_2_1 && is_placed) {
                write_pcie_2_1(ci);
                blank();
            }
        }
    }

    void write_fasm()
    {
        // Run-identity header: these comment lines are dropped by fasm2frames,
        // so the resulting bitstream hash is unaffected; they record the exact
        // toolchain/chipdb/placer provenance of this FASM file for
        // reproducibility (port of nextpnr-xilinx 7037c948).
        out << "# nextpnr-himbaechel " << GIT_DESCRIBE_STR << "\n";
        out << "# chipdb " << ctx->chip_info->name.get() << " version " << ctx->chip_info->version << " generator "
            << ctx->chip_info->generator.get() << "\n";
        out << "# placer seed " << ctx->rngstate << "\n";
        get_invertible_pins(ctx, invertible_pins);
        populate_bufgctrl_bound_slots(); // must run before any pip emission
        write_logic();
        write_cfg();
        write_io();
        write_routing();
        // After write_routing: it is write_pip that fills pips_by_tile, and
        // this pass reads the routing to find which ILOGIC halves an input
        // passes through.
        write_ilogic_bypass_inversion();
        write_bram();
        write_clocking();
        write_ip();
    }
};

} // namespace

void XilinxImpl::write_fasm(const std::string &filename)
{
    // A log_error during the walk throws, and anything already flushed would
    // otherwise stay behind under the name the user asked for. A later
    // assembler treats that partial text as a complete design and emits a
    // normal-sized bitstream. Write a temporary file in the same directory
    // and rename it into place only after the walk finishes, so a failure
    // leaves the previous file untouched and no partial file under that name.
    // std::filesystem::rename replaces an existing destination on Linux and
    // on Windows, which is what a rebuild finds already in place.
    const std::string tmp = filename + ".nextpnr-tmp";
    struct PendingFasm
    {
        std::string path;
        bool commit = false;
        ~PendingFasm()
        {
            const bool discard = !commit;
            if (discard) {
                std::error_code remove_ec;
                std::filesystem::remove(path, remove_ec);
            }
        }
    } pending{tmp};

    {
        auto out = open_ofstream_and_log_error(tmp, "FASM file");
        FasmBackend be(this->ctx, this, out);
        be.write_fasm();
        out.flush();
        const bool write_failed = !out.good();
        if (write_failed)
            log_error("Failed to write FASM file '%s'.\n", tmp.c_str());
        out.close();
    }

    std::error_code rename_ec;
    std::filesystem::rename(tmp, filename, rename_ec);
    const bool rename_failed = static_cast<bool>(rename_ec);
    if (rename_failed)
        log_error("Failed to rename FASM file '%s' to '%s'.\n", tmp.c_str(), filename.c_str());
    pending.commit = true;
}

// Placement dump: for every cell that ended up on a bel, the physical
// (tile, site, bel) it landed on and its type, keyed by the cell's own name
// from the input netlist.
//
// This exists for external LVS / equivalence checking.  A tool that
// reconstructs a netlist from a bitstream can only name its cells after the
// physical site it found them in, so relating those names back to a gold
// netlist's names needs the placement as ground truth.  A routed JSON is not
// enough: its NEXTPNR_BEL attribute carries himbaechel grid coordinates and a
// site index ("X79Y221/SLICE_X1Y0.B5FF"), and nothing outside nextpnr can turn
// those into the prjxray tile and site names that appear in the FASM
// ("CLBLM_R_X31Y139", "SLICE_X46Y134") that an extraction working from the
// database can derive for itself.  Those are the names written here.
void XilinxImpl::write_placement(const std::string &filename)
{
    auto out = open_ofstream_and_log_error(filename, "placement file");

    auto escape = [](const std::string &in) {
        std::string r;
        for (char c : in) {
            bool needs_escape = c == '"' || c == '\\';
            if (needs_escape)
                r.push_back('\\');
            r.push_back(c);
        }
        return r;
    };

    // pack_idelayctrl() turns the one synthesis IDELAYCTRL into a copy per
    // IOCTRL column -- named <orig>$intcell$CTRL_DUP_<n> -- and drops the
    // original, because the silicon needs one per column while the netlist
    // asks for one.  An external LVS census works from the synthesis and so
    // knows only that one cell; left as-is it reports it dropped.  Collapse the
    // copies back to a single entry under the original name, once verified they
    // all distribute the same REFCLK and RST, so the two descriptions agree.
    const std::string kDupTag = "$intcell$CTRL_DUP_";
    std::map<std::string, CellInfo *> idelay_orig;  // original name -> representative copy
    std::set<IdString> idelay_dup;                  // copies to emit under the merged name
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel == BelId())
            continue;
        // Placement legalisation retypes the cell to its site/bel form
        // (IDELAYCTRL_IDELAYCTRL), so match on the copy's name -- the signature
        // pack_idelayctrl() gives it -- and confirm the bel to be safe.
        std::string n = ci->name.str(ctx);
        auto at = n.find(kDupTag);
        if (at == std::string::npos)
            continue;
        bool on_idelayctrl_bel = bel_name_in_site(ci->bel).str(ctx).find("IDELAYCTRL") != std::string::npos;
        if (!on_idelayctrl_bel)
            continue;
        std::string orig = n.substr(0, at);
        idelay_dup.insert(ci->name);
        auto it = idelay_orig.find(orig);
        if (it == idelay_orig.end()) {
            idelay_orig[orig] = ci;
            continue;
        }
        bool copies_disagree = ci->getPort(id_REFCLK) != it->second->getPort(id_REFCLK) ||
                               ci->getPort(id_RST) != it->second->getPort(id_RST);
        if (copies_disagree) {
            log_error("IDELAYCTRL copies of '%s' distribute different REFCLK/RST; "
                      "cannot merge them for the placement dump\n",
                      orig.c_str());
        }
    }

    out << "{\n";
    bool first = true;
    auto emit = [&](const std::string &name, CellInfo *ci) {
        SiteIndex site = get_bel_site(ci->bel);
        // PSEUDO_VCC / PSEUDO_GND and friends sit on bels with no site at all,
        // so the site index cannot be trusted to be in range here.
        const auto &sites = tile_extra_data(site.tile)->sites;
        bool bel_has_site = site.site >= 0 && site.site < sites.ssize();
        std::string site_str = bel_has_site ? get_site_name(site).str(ctx) : std::string();

        if (!first)
            out << ",\n";
        first = false;
        out << "  \"" << escape(name) << "\": {"
             << "\"tile\": \"" << escape(tile_name(ci->bel.tile)) << "\", "
             << "\"site\": \"" << escape(site_str) << "\", "
             << "\"bel\": \"" << escape(bel_name_in_site(ci->bel).str(ctx)) << "\", "
             // nextpnr's own name for the bel: what a BEL attribute on the cell
             // must say to put it back there (a frozen placement)
             << "\"nextpnr_bel\": \"" << escape(ctx->nameOfBel(ci->bel)) << "\", "
             << "\"type\": \"" << escape(ci->type.str(ctx)) << "\"}";
    };
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->bel == BelId())
            continue;
        // A duplicated IDELAYCTRL is emitted once, below, under its original
        // name rather than per copy.
        if (idelay_dup.count(ci->name))
            continue;
        emit(ci->name.str(ctx), ci);
    }
    for (const auto &kv : idelay_orig)
        emit(kv.first, kv.second);
    out << (first ? "" : "\n") << "}\n";
    log_info("Wrote placement of %d cells to %s\n", int(ctx->cells.size()), filename.c_str());
}

NEXTPNR_NAMESPACE_END
