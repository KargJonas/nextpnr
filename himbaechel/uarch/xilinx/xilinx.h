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

#ifndef HIMBAECHEL_XILINX_H
#define HIMBAECHEL_XILINX_H

#include "extra_data.h"
#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "util.h"

#include "himbaechel_helpers.h"

NEXTPNR_NAMESPACE_BEGIN

struct XilinxCellTags
{
    union
    {
        struct
        {
            bool is_memory, is_srl;
            bool wclk_inv;
            int input_count, output_count;
            int memory_group;
            bool only_drives_carry;
            NetInfo *input_sigs[6], *output_sigs[2];
            NetInfo *address_msb[3];
            NetInfo *di1_net, *di2_net, *wclk, *we;
        } lut;
        struct
        {
            bool is_latch, is_clkinv, is_srinv, ffsync;
            bool is_paired;
            NetInfo *clk, *sr, *ce, *d;
            int32_t control_set;
        } ff;
        struct
        {
            NetInfo *out_sigs[8], *cout_sigs[8], *x_sigs[8];
        } carry;
        struct
        {
            NetInfo *sel, *out;
        } mux;
    };
};

struct SiteIndex
{
    SiteIndex() : tile(-1), site(-1) {};
    SiteIndex(int32_t tile, int32_t site) : tile(tile), site(site) {};

    int32_t tile;
    int32_t site;
    bool operator==(const SiteIndex &other) const { return tile == other.tile && site == other.site; }
    bool operator!=(const SiteIndex &other) const { return tile != other.tile || site != other.site; }
    bool operator<(const SiteIndex &other) const
    {
        return (tile < other.tile) || (tile == other.tile && site < other.site);
    }
    unsigned hash() const { return mkhash(tile, site); }
};

// Key into the hand-maintained pseudo-pip table in fasm.cc.  Shared with
// XilinxImpl::is_pip_unavail, which must not reject a pip the fasm writer can
// in fact emit.
struct PseudoPipKey
{
    IdString tileType;
    IdString dest;
    IdString source;

    bool operator==(const PseudoPipKey &b) const
    {
        return std::tie(this->tileType, this->dest, this->source) == std::tie(b.tileType, b.dest, b.source);
    }

    unsigned int hash() const { return mkhash(mkhash(tileType.hash(), source.hash()), dest.hash()); }
};

void xlnx_build_pseudo_pip_config(Context *ctx, dict<PseudoPipKey, std::vector<std::string>> &pp_config);

struct XilinxImpl : HimbaechelAPI
{

    struct LogicTileStatus
    {
        // z -> cell
        CellInfo *cells[128];

        // Eight-tile valid and dirty status
        struct EigthTileStatus
        {
            mutable bool valid = true, dirty = true;
        } eights[8];
        struct HalfTileStatus
        {
            mutable bool valid = true, dirty = true;
        } halfs[8];
    };

    struct BRAMTileStatus
    {
        CellInfo *cells[12] = {nullptr};
    };

    struct TileStatus
    {
        std::unique_ptr<LogicTileStatus> lts;
        std::unique_ptr<BRAMTileStatus> bts;
        std::vector<int> site_variant;
    };

    ~XilinxImpl();
    po::options_description getUArchOptions() override;
    void init_database(Arch *arch) override;

    void init(Context *ctx) override;

    // Chipdb bel types that repeat their primitive name (RAMB18E1_RAMB18E1,
    // from prjxray's "<site type>_<bel name>" naming) bucket as the primitive,
    // so utilisation, reports and placer logs name the primitive.
    IdString getBelBucketForCellType(IdString cell_type) const override;
    IdString getBelBucketForBel(BelId bel) const override;

    // Bels
    void notifyBelChange(BelId bel, CellInfo *cell) override;
    void update_logic_bel(BelId bel, CellInfo *cell);
    void update_bram_bel(BelId bel, CellInfo *cell);

    bool isBelLocationValid(BelId bel, bool explain_invalid = false) const override;
    bool xc7_logic_tile_valid(IdString tileType, const LogicTileStatus &lts) const;
    void explain_invalid_logic_tile(BelId bel) const;
    // A tile is explained once: the final check of the placer asks every bel of it in turn
    mutable pool<int> explained_tiles;

    // Clusters: a RAMB36E1 cascade chain is placed bel by bel along the
    // chipdb's CASCADEOUT -> CASCADEIN wires; everything else is BaseArch's.
    bool getClusterPlacement(ClusterId cluster, BelId root_bel,
                             std::vector<std::pair<CellInfo *, BelId>> &placement) const override;
    Loc getClusterOffset(const CellInfo *cell) const override;
    BelId bram_cascade_next(BelId bel) const;

    // Pips
    // Lazily-built copy of fasm.cc's pseudo-pip table, so is_pip_unavail can
    // tell "no bits, and no hand-written fasm either" (a trap) from "no bits,
    // but fasm.cc emits it anyway" (fine).  Built once on first use.
    mutable dict<PseudoPipKey, std::vector<std::string>> pseudo_pip_config;
    mutable bool pseudo_pip_keys_valid = false;
    bool is_pip_unavail(PipId pip) const;
    // Does this design instantiate a BUFR?  The regional-clock datapath
    // (RCLK_BEFORE_DIV -> RCLK_OUT -> RCLK2RCLK -> CK_BUFRCLK) runs through a
    // BUFR, so those wires are a buffer's output rather than general routing.
    // Cached: is_pip_unavail is on the router's hot path.
    mutable bool design_has_bufr = false;
    mutable bool design_has_bufr_valid = false;
    bool checkPipAvail(PipId pip) const override { return !is_pip_unavail(pip); }
    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override { return !is_pip_unavail(pip); }

    // Flow management
    void parse_xdc(const std::string &filename);
    void pack() override;
    void apply_loc_constraints(const pool<IdString> &only_types = {});
    void prePlace() override;
    void preRoute() override;
    void apply_prerouted();
    void apply_preplaced(bool verbose = true);
    void apply_holdbufs();
    void close_routed_tiles();
    pool<int> frozen_tiles; // tiles holding -o preplaced cells: no other cell may go there
    void postPlace() override;
    void postRoute() override;
    void postRouteArchInfo() override;
    void write_fasm(const std::string &filename);
    void write_placement(const std::string &filename);

    void configurePlacerHeap(PlacerHeapCfg &cfg) override;
    void configurePlacerStatic(PlacerStaticCfg &cfg) override;
    void configureRouter2(Router2Cfg &cfg) override;

    void fixup_placement();
    void fixup_routing();
    void fixup_hold();
    void route_clocks();

    virtual std::string getDefaultRouter() const override { return "router2"; };

    // Misc utility functions
    const XlnxTileInstExtraDataPOD *tile_extra_data(int tile) const;
    IdString bel_tile_type(BelId bel) const;
    bool is_logic_tile(BelId bel) const;
    bool is_bram_tile(BelId bel) const;

    SiteIndex get_bel_site(BelId bel) const;
    SiteIndex rel_site(SiteIndex site, int dx, int dy) const;
    Loc rel_site_loc(SiteIndex site) const;
    IdString get_site_name(SiteIndex site) const;
    IdString bel_name_in_site(BelId bel) const;
    IdStringList get_site_bel_name(BelId bel) const;
    BelId get_site_bel(SiteIndex site, IdString bel_name) const;
    WireId lookup_wire(int tile, IdString wire_name) const;

    int hclk_for_iob(BelId pad) const;
    int hclk_for_ioi(int tile) const;

    std::string tile_name(int tile) const;
    // The die whose chipdb is loaded (xc7a35t is "xc7a50t", and so on)
    std::string chipdb_die;

    std::vector<XilinxCellTags> cell_tags;
    const XilinxCellTags *get_tags(const CellInfo *cell) const
    {
        return cell ? &cell_tags.at(cell->flat_index) : nullptr;
    }

    std::vector<TileStatus> tile_status;

    bool cell_tags_set = false;

    // Improved delay predictions where sites are located far from their associated interconnect
    dict<WireId, Loc> source_locs, sink_locs;
    bool is_general_routing(WireId wire) const;
    void find_source_sink_locs();

    // Measured interconnect delay by tile offset; see delay_matrix.cc.
    std::vector<delay_t> dm_delay;
    int dm_window = 24;
    int dm_max_explore = 4000000;
    bool dm_valid = false;
    // Out-of-window connections are extrapolated from the measured window
    // edge at these marginal rates (ps per tile), derived from dm_delay
    // itself -- so a loaded matrix and a freshly built one price long
    // connections identically, and there is no arbitrary formula left in
    // the path.
    float dm_rate_x = 0.0f, dm_rate_y = 0.0f;
    size_t dm_index(int dx, int dy) const
    {
        return size_t((dy + dm_window) * (2 * dm_window + 1) + (dx + dm_window));
    }
    int measure_from(WireId src, int sx, int sy, std::vector<delay_t> &out) const;
    std::vector<std::pair<WireId, Loc>> pick_delay_sources(int count) const;
    void build_delay_matrix();
    delay_t delay_matrix_lookup(int dx, int dy) const;
    void compute_edge_rates();
    bool load_delay_matrix(const std::string &path);
    void save_delay_matrix(const std::string &path) const;

    // The global clock tree's insertion delay; see clock_tree.cc
    struct ClockTreeRates
    {
        const char *die;
        delay_t ps_per_region_row, ps_per_10_columns, bram_clock_pin_ps, flat_route_ps;
    };
    ClockTreeRates ct_rates{};
    bool ct_enabled = false;
    int ct_spine_x = -1;
    float ct_centre_y = 0;
    std::vector<int> ct_region_rows; // the centre (HROW) row of each clock region row
    pool<IdString> ct_bufg_bel_types;
    void init_clock_tree();
    bool clock_rates_measured() const;
    bool is_global_clock_bel(BelId bel) const;
    delay_t clock_tree_delay_at(int x, int y, bool bram_clock_pin) const;
    delay_t clock_tree_delay(BelId sink) const;
    delay_t predict_clock_tree_delay(BelId src_bel, BelId dst_bel) const;
    bool getArcDelayOverride(const NetInfo *net_info, const PortRef &sink, DelayQuad &delay) const override;

    // Block RAMs placed by read-mux group, then everything else around them;
    // see bram_place.cc
    bool is_movable_bram(const CellInfo *ci) const;
    bool place_brams_by_group();
    void replace_around_brams();

    delay_t predictDelay(BelId src_bel, IdString src_pin, BelId dst_bel, IdString dst_pin) const override;
    delay_t estimateDelay(WireId src, WireId dst) const override;
    BoundingBox getRouteBoundingBox(WireId src, WireId dst) const override;

  private:
    HimbaechelHelpers h;
    // Bel types (bel_type -> primitive name) whose chipdb name repeats the
    // primitive; lookups happen per cell on the placer's hot path.
    dict<IdString, IdString> primitive_bucket_for_bel_type;
    void assign_cell_tags();
    void index_control_sets();
};

NEXTPNR_NAMESPACE_END
#endif
