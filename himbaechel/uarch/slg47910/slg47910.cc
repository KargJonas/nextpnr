/*
 * nextpnr-himbaechel uarch for the Renesas SLG47910 (ForgeFPGA).
 *
 * The chipdb (slg47910_gen.py) is the vendor's routing graph with the 140 L0
 * logic sites (four slices of a LUT6, two flip-flops and a bit of the carry
 * chain), the 184 L6 pads and the four dedicated clock inputs as bels. This
 * file is what nextpnr does with it:
 *
 *   pack       pads, clock inputs and the PLL's controls from the ports and a
 *              pin file, the synthesiser's LUTs and flip-flops as SLG_LUT and
 *              SLG_FF, a LUT with its single flip-flop as one cluster,
 *              constants as LUTs, CARRY4 chains as one cluster of carry bits
 *              up a column, a MUXF7 or a MUXF8 as one cluster with the LUTs
 *              it takes, two LUTs of five inputs in all as one slice's two
 *              halves
 *   validity   the flip-flops of a site share one enable and one reset; a
 *              site's four I5 pins bring in at most four nets; as many
 *              slices of a site in use as the design needs (two while its
 *              logic fits in the sites with room to spare, up to four), but
 *              where a carry chain fills it
 *   postRoute  what the bitstream tools take: where every cell sits and what
 *              each net's route is, as JSON (placement.json, routing.json),
 *              in the names of the chipdb, which the bitstream tools
 *              (hb_emit.py) turn into the graph's node ids and the sites'
 *              coordinates
 *
 * The memory sites are logic sites here: no LUT RAM yet.
 */

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

#include "himbaechel_api.h"
#include "himbaechel_helpers.h"
#include "log.h"
#include "nextpnr.h"
#include "placer_heap.h"
#include "util.h"

#define GEN_INIT_CONSTIDS
#define HIMBAECHEL_CONSTIDS "uarch/slg47910/constids.inc"
#define HIMBAECHEL_UARCH slg47910
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

constexpr int GRID = 32;
constexpr int FF_Z = 4;      // a slice's flip-flop sits 4 above its LUT
constexpr int CARRY_Z = 8;   // and its bit of the carry chain 8 above
constexpr int OFF_Z = 12;    // and its second flip-flop (OXQ) 12 above
constexpr int MUXF_Z = 16;   // the site's F7A, F7C and F8 (gen)
constexpr int LUTL_Z = 20;   // a slice's lower half, a LUT of its own
// The slice whose XI selects each wide mux, and whose xMUX pin it leaves by.
constexpr int MUXF_SLICE[3] = {0, 2, 1};
constexpr int CLKIN_Z = 100; // W0, N0, N1, W1, E0, E1

// The package's pads: GPIO name -> the vendor's name for its IOB, RBB_<row>_<col>.
const std::map<std::string, std::pair<int, int>> &gpio_names()
{
    static const std::map<std::string, std::pair<int, int>> t = {
            {"GPIO0", {0, 6}},    {"GPIO1", {0, 7}},    {"GPIO2", {0, 8}},    {"GPIO3", {0, 9}},    {"GPIO4", {0, 10}},
            {"GPIO5", {0, 22}},   {"GPIO6", {0, 23}},   {"GPIO7", {0, 24}},   {"GPIO8", {31, 27}},  {"GPIO9", {31, 26}},
            {"GPIO10", {31, 25}}, {"GPIO11", {31, 24}}, {"GPIO12", {31, 23}}, {"GPIO13", {31, 22}}, {"GPIO14", {31, 9}},
            {"GPIO15", {31, 8}},  {"GPIO16", {31, 6}},  {"GPIO17", {31, 5}},  {"GPIO18", {31, 4}},
    };
    return t;
}

// The chip's own inputs the fabric drives through a pad's two outputs, by the
// vendor's names (its io planner's, app note AN-FG-006): the PLL's controls ->
// RBB_<row>_<col> and which output. The oscillator's enable, OSC_CTRL_EN, is
// a pad of its own, (0,25).
struct Control
{
    int row, col, out;
};
const std::map<std::string, Control> &control_names()
{
    static const std::map<std::string, Control> t = [] {
        std::map<std::string, Control> m = {{"PLL_REF_CLK_SELECTION", {0, 11, 0}},
                                            {"PLL_CTRL_BYPASS", {0, 11, 1}},
                                            {"PLL_CTRL_EN", {0, 18, 0}},
                                            {"PLL_CTRL_POSTDIV1[0]", {0, 19, 0}},
                                            {"PLL_CTRL_POSTDIV1[1]", {0, 19, 1}},
                                            {"PLL_CTRL_POSTDIV1[2]", {0, 20, 0}},
                                            {"PLL_CTRL_POSTDIV2[0]", {0, 20, 1}},
                                            {"PLL_CTRL_POSTDIV2[1]", {0, 21, 0}},
                                            {"PLL_CTRL_POSTDIV2[2]", {0, 21, 1}}};
        for (int k = 0; k < 12; k++)
            m["PLL_CTRL_FBDIV[" + std::to_string(k) + "]"] = {0, 12 + k / 2, k % 2};
        for (int k = 0; k < 6; k++)
            m["PLL_CTRL_REFDIV[" + std::to_string(k) + "]"] = {31, 19 + k / 2, k % 2};
        return m;
    }();
    return t;
}

// What the chip states to the fabric at a pad of its own, by the vendor's
// names (app note AN-FG-019): RBB_<row>_<col> and which of the pad's two
// inputs. Its routes take them from that input's pin and nowhere else, and
// it writes nothing at the pad for them.
const std::map<std::string, Control> &status_names()
{
    static const std::map<std::string, Control> t = {{"PLL_LOCK", {0, 12, 1}}, {"OSC_READY", {0, 25, 0}}};
    return t;
}

struct ClockToken
{
    int z, domain;
    const char *side;
};
const std::map<std::string, ClockToken> &clock_tokens()
{
    static const std::map<std::string, ClockToken> t = {{"W0", {0, 0, "W"}}, {"N0", {1, 0, "N"}}, {"N1", {2, 1, "N"}},
                                                          {"W1", {3, 0, "W"}}, {"E0", {4, 1, "E"}}, {"E1", {5, 1, "E"}}};
    return t;
}

std::string json_escape(const std::string &s)
{
    std::string out;
    for (char c : s) {
        if (c == '\\' || c == '"')
            out += '\\', out += c;
        else if (c == '\n')
            out += "\\n";
        else if ((unsigned char)c < 0x20)
            out += ' ';
        else
            out += c;
    }
    return out;
}

struct SlgImpl : HimbaechelAPI
{
    po::options_description getUArchOptions() override
    {
        po::options_description specific("SLG47910 specific options");
        specific.add_options()("pins", po::value<std::string>(),
                               "pin file: `set_io <port> GPIO<n>|<row> <col>|OSC_CTRL_EN|<PLL control>|PLL_LOCK|OSC_READY`, "
                               "`set_clk <port> W0|W1|N0|N1`");
        specific.add_options()("speed", po::value<std::string>(),
                               "the speed grade, a condition the vendor times under (its builds' when not given)");
        specific.add_options()("placement", po::value<std::string>(), "where to write the placement, as JSON");
        specific.add_options()("routing", po::value<std::string>(), "where to write the routing, as JSON");
        return specific;
    }

    void init_database(Arch *arch) override
    {
        init_uarch_constids(arch);
        arch->load_chipdb("slg47910/chipdb-slg47910.bin");
        // The speed grades are the vendor's conditions, the one its builds are
        // timed under first: the default (gen/slg47910_gen.py).
        std::string speed = arch->args.options.count("speed") ? arch->args.options.at("speed").as<std::string>()
                                                             : IdString(arch->chip_info->speed_grades[0].name).str(arch);
        arch->set_speed_grade(speed);
    }

    void init(Context *ctx) override
    {
        h.init(ctx);
        HimbaechelAPI::init(ctx);
        lut_m = ctx->get_cell_timing_idx(id_SLG_LUT_M);
        lutl_m = ctx->get_cell_timing_idx(id_SLG_LUTL_M);
        // The site each site's carry out goes on to: the far end of its CARRY pip.
        for (BelId bel : ctx->getBels()) {
            if (ctx->getBelType(bel) != id_SLG_CARRY || ctx->getBelLocation(bel).z != CARRY_Z + 3)
                continue;
            Loc at = ctx->getBelLocation(bel);
            for (PipId pip : ctx->getPipsDownhill(ctx->getBelPinWire(bel, id_CO)))
                if (IdString(chip_pip_info(ctx->chip_info, pip).type) == id_CARRY) {
                    Loc up = ctx->getPipLocation(pip);
                    site_above[{at.x, at.y}] = {up.x, up.y};
                }
        }
        std::set<std::pair<int, int>> above;
        for (auto &[below, up] : site_above)
            above.insert(up);
        for (auto &[below, up] : site_above)
            if (!above.count(below))
                column_foot = below;
    }

    std::string getDefaultRouter() const override { return "router2"; }

    // Every cell type at once, so a carry chain (the largest cluster) is
    // legalised before the LUTs and flip-flops fill the column it needs.
    void configurePlacerHeap(PlacerHeapCfg &cfg) override
    {
        cfg.placeAllAtOnce = true;
        cfg.chainRipup = true;
    }

    // Site pins sit away from their site and legal routes detour through
    // neighbouring switch boxes: nothing is outside a net's box.
    BoundingBox getRouteBoundingBox(WireId src, WireId dst) const override
    {
        return BoundingBox(0, 0, GRID - 1, GRID - 1);
    }
    delay_t estimateDelay(WireId src, WireId dst) const override { return 0; }

    // ---- packing ----

    std::map<std::string, BelId> io_bels;
    std::map<std::string, std::string> clock_ports; // port -> token
    std::map<std::string, Control> control_ports;    // port -> the chip input it drives
    std::map<std::string, Control> status_ports;     // port -> the pad input the chip states it at
    struct Held
    {
        int x, y, out, value;
    };
    std::vector<Held> held_controls; // a control held at a constant: the pad holds it, nothing routes
    std::map<std::pair<int, int>, CellInfo *> control_cells; // (row, col) -> the pad cell of driven controls

    BelId iob_at(int row, int col, const std::string &where)
    {
        // The two numbers are the vendor's RBB_<row>_<col>: the pad is at x=col, y=row.
        BelId bel = ctx->getBelByLocation(Loc(col, row, 0));
        if (bel == BelId() || ctx->getBelType(bel) != id_SLG_IOB)
            log_error("slg47910: %s: (%d,%d) is not a pad\n", where.c_str(), row, col);
        return bel;
    }

    void read_pins()
    {
        if (!ctx->args.options.count("pins"))
            return;
        std::string path = ctx->args.options.at("pins").as<std::string>();
        std::ifstream f(path);
        if (!f)
            log_error("slg47910: cannot open the pin file %s\n", path.c_str());
        std::string line;
        int lineno = 0;
        std::set<std::tuple<int, int, int>> taken; // (row, col, output): -1 a whole pad
        while (std::getline(f, line)) {
            lineno++;
            line = line.substr(0, line.find('#'));
            std::istringstream in(line);
            std::string keyword, port, where;
            if (!(in >> keyword))
                continue;
            std::string at = path + ":" + std::to_string(lineno);
            if (!(in >> port >> where))
                log_error("slg47910: %s: expected `%s <port> <where>`\n", at.c_str(), keyword.c_str());
            for (auto &c : where)
                c = toupper(c);
            if (keyword == "set_clk") {
                // E0 and E1 take back a clock the fabric makes (pack_fabric_clocks)
                if (!clock_tokens().count(where) || where[0] == 'E')
                    log_error("slg47910: %s: no clock input %s (W0, W1, N0 or N1)\n", at.c_str(), where.c_str());
                clock_ports[port] = where;
            } else if (keyword == "set_io") {
                int row, col;
                auto gpio = gpio_names().find(where);
                auto control = control_names().find(where);
                if (control != control_names().end()) {
                    const Control &c = control->second;
                    if (!taken.insert({c.row, c.col, c.out}).second || taken.count({c.row, c.col, -1}))
                        log_error("slg47910: %s: %s taken twice\n", at.c_str(), where.c_str());
                    control_ports[port] = c;
                    continue;
                }
                auto status = status_names().find(where);
                if (status != status_names().end()) {
                    // a pad's input, beside whatever drives its outputs (2 + the input)
                    const Control &c = status->second;
                    if (!taken.insert({c.row, c.col, 2 + c.out}).second)
                        log_error("slg47910: %s: %s taken twice\n", at.c_str(), where.c_str());
                    status_ports[port] = c;
                    continue;
                }
                if (where == "OSC_CTRL_EN")
                    row = 0, col = 25;
                else if (gpio != gpio_names().end())
                    row = gpio->second.first, col = gpio->second.second;
                else {
                    char *end = nullptr;
                    row = int(strtol(where.c_str(), &end, 10));
                    if (end == where.c_str() || *end != 0 || !(in >> col))
                        log_error("slg47910: %s: %s is no GPIO name, nor a row and a column\n", at.c_str(),
                                  where.c_str());
                }
                if (!taken.insert({row, col, -1}).second || taken.count({row, col, 0}) || taken.count({row, col, 1}))
                    log_error("slg47910: %s: pad (%d,%d) taken twice\n", at.c_str(), row, col);
                io_bels[port] = iob_at(row, col, at);
            } else
                log_error("slg47910: %s: expected set_io or set_clk\n", at.c_str());
        }
    }

    // The two clock domains: mux 8 feeds domain 0 (W0, W1 or N0), mux 9 domain
    // 1 (N1, or W0). The oscillator moves to domain 1 when the PLL or N0 takes
    // domain 0, as the vendor's build of a design on both W0 and W1 has it.
    int clock_domain(const std::string &token) const
    {
        std::set<std::string> used;
        for (auto &[port, t] : clock_ports)
            used.insert(t);
        bool zero = used.count("W1") || used.count("N0");
        if ((used.count("W1") && used.count("N0")) || (zero && used.count("W0") && used.count("N1")))
            log_error("slg47910: more clock inputs than the two clock domains hold\n");
        return token == "W0" && zero ? 1 : clock_tokens().at(token).domain;
    }

    void pack_io()
    {
        std::vector<IdString> gone;
        for (auto &[name, ci] : ctx->cells)
            if (ci->type.in(ctx->id("$nextpnr_ibuf"), ctx->id("$nextpnr_obuf"), ctx->id("$nextpnr_iobuf"))) {
                ci->disconnectPort(id_I);
                ci->disconnectPort(id_O);
                gone.push_back(name);
            }
        for (auto &name : gone)
            ctx->cells.erase(name);

        std::vector<std::unique_ptr<CellInfo>> made;
        std::set<std::string> used;
        std::vector<std::pair<std::string, NetInfo *>> statuses;
        for (auto &[name, port] : ctx->ports) {
            std::string pname = name.str(ctx);
            if (!port.net)
                continue;
            if (status_ports.count(pname)) {
                if (port.type != PORT_IN)
                    log_error("slg47910: %s reads what the chip states: it must be an input\n", pname.c_str());
                statuses.push_back({pname, port.net});
                used.insert(pname);
                continue;
            }
            if (control_ports.count(pname)) {
                // A control held at a constant is the pad's to hold (its own
                // bits, as the vendor writes them): no cell, no route. One the
                // design drives is routed to the pad's first output (PAD) or
                // its second (OE); the two of a pad are one cell.
                const CellInfo *driver = port.net->driver.cell;
                bool vcc = driver && driver->type == ctx->id("VCC"), gnd = driver && driver->type == ctx->id("GND");
                if (port.type != PORT_OUT)
                    log_error("slg47910: %s drives a chip control: it must be an output\n", pname.c_str());
                const Control &c = control_ports.at(pname);
                used.insert(pname);
                if (vcc || gnd) {
                    held_controls.push_back({c.col, c.row, c.out, vcc ? 1 : 0});
                    continue;
                }
                CellInfo *&pad = control_cells[{c.row, c.col}];
                if (pad == nullptr) {
                    auto cell =
                            std::make_unique<CellInfo>(ctx, ctx->idf("$ctl_%d_%d", c.row, c.col), id_SLG_IOB);
                    cell->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(iob_at(c.row, c.col, pname)));
                    cell->attrs[ctx->id("CONTROL")] = Property(1);
                    pad = cell.get();
                    made.push_back(std::move(cell));
                }
                IdString pin = c.out ? id_OE : id_PAD;
                pad->addInput(pin);
                pad->connectPort(pin, port.net);
                continue;
            }
            if (clock_ports.count(pname)) {
                if (port.type != PORT_IN)
                    log_error("slg47910: the clock port %s must be an input\n", pname.c_str());
                const std::string &token = clock_ports.at(pname);
                auto cell = std::make_unique<CellInfo>(ctx, ctx->idf("$clk_%s", pname.c_str()), id_SLG_CLKIN);
                cell->addOutput(id_PAD);
                cell->connectPort(id_PAD, port.net);
                BelId bel = ctx->getBelByLocation(Loc(0, 0, CLKIN_Z + clock_tokens().at(token).z));
                cell->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(bel));
                cell->params[id_CLK_TOKEN] = Property(token);
                cell->params[id_CLK_SIDE] = Property(std::string(clock_tokens().at(token).side));
                cell->params[id_CLK_DOMAIN] = Property(clock_domain(token));
                used.insert(pname);
                made.push_back(std::move(cell));
                continue;
            }
            auto cell = std::make_unique<CellInfo>(ctx, ctx->idf("$io_%s", pname.c_str()), id_SLG_IOB);
            if (port.type == PORT_IN)
                cell->addOutput(id_PAD);
            else if (port.type == PORT_OUT)
                cell->addInput(id_PAD);
            else
                cell->addInout(id_PAD);
            // A pad held at a constant has a constant of its own, which the
            // placer can put beside it: the design's one would be wherever its
            // many flip-flops' enables want it.
            const CellInfo *driver = port.net->driver.cell;
            bool vcc = driver && driver->type == ctx->id("VCC"), gnd = driver && driver->type == ctx->id("GND");
            if (port.type == PORT_OUT && (vcc || gnd)) {
                auto lut = std::make_unique<CellInfo>(ctx, ctx->idf("$const_%s", pname.c_str()), id_SLG_LUT);
                lut->params[id_INIT] = Property(vcc ? 0xFFFFFFFFFFFFFFFFull : 0ull, 64);
                lut->params[id_LUT_MODE] = Property("LUT5");
                lut->addOutput(id_O);
                NetInfo *held = ctx->createNet(ctx->idf("$const_%s_o", pname.c_str()));
                lut->connectPort(id_O, held);
                cell->connectPort(id_PAD, held);
                made.push_back(std::move(lut));
            } else
                cell->connectPort(id_PAD, port.net);
            if (io_bels.count(pname)) {
                cell->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(io_bels.at(pname)));
                used.insert(pname);
            }
            made.push_back(std::move(cell));
        }
        // What the chip states leaves its pad by one of the pad's inputs: an
        // output of the cell that holds the pad (its controls, the oscillator's
        // enable), or of one of its own.
        for (auto &[pname, net] : statuses) {
            const Control &c = status_ports.at(pname);
            std::string bel = ctx->nameOfBel(iob_at(c.row, c.col, pname));
            CellInfo *pad = nullptr;
            for (auto &cell : made)
                if (cell->attrs.count(ctx->id("BEL")) && cell->attrs.at(ctx->id("BEL")).as_string() == bel)
                    pad = cell.get();
            if (pad == nullptr) {
                made.push_back(std::make_unique<CellInfo>(ctx, ctx->idf("$st_%d_%d", c.row, c.col), id_SLG_IOB));
                pad = made.back().get();
                pad->attrs[ctx->id("BEL")] = bel;
            }
            IdString pin = c.out ? id_IN1 : id_IN0;
            pad->addOutput(pin);
            pad->connectPort(pin, net);
        }
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
        for (auto &[port, bel] : io_bels)
            if (!used.count(port))
                log_error("slg47910: the pin file names a port %s the design has not\n", port.c_str());
        for (auto &[port, c] : control_ports)
            if (!used.count(port))
                log_error("slg47910: the pin file names a port %s the design has not\n", port.c_str());
        for (auto &[port, token] : clock_ports)
            if (!used.count(port))
                log_error("slg47910: the pin file names a clock port %s the design has not\n", port.c_str());
    }

    static int lut_inputs(IdString type)
    {
        if (type == id_INV || type == id_LUT1)
            return 1;
        if (type == id_LUT2)
            return 2;
        if (type == id_LUT3)
            return 3;
        if (type == id_LUT4)
            return 4;
        if (type == id_LUT5)
            return 5;
        return 6;
    }

    void pack_luts()
    {
        for (auto &[name, ci] : ctx->cells) {
            if (!ci->type.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4, id_LUT5, id_LUT6, id_INV))
                continue;
            int n = lut_inputs(ci->type);
            bool inv = ci->type == id_INV;
            auto init = ci->params.find(id_INIT);
            ci->type = id_SLG_LUT;
            if (inv)
                ci->params[id_INIT] = Property(0x1ull, 64);
            else if (init != ci->params.end())
                ci->params[id_INIT] = Property(uint64_t(init->second.as_int64()), 64);
            // LUT5 mode: the LUT6's mux passes the LUT5 half unchanged when I5 is unused
            bool lut5 = n <= 5;
            if (!lut5 && init != ci->params.end()) {
                uint64_t raw = uint64_t(init->second.as_int64());
                lut5 = (raw >> 32) == (raw & 0xFFFFFFFFull);
            }
            ci->params[id_LUT_MODE] = lut5 ? Property("LUT5") : Property("LUT6");
            if (n == 1 && ci->ports.count(id_I))
                ci->renamePort(id_I, id_I0);
            if (n == 1 && ci->ports.count(id_A))
                ci->renamePort(id_A, id_I0);
        }
    }

    // The fabric's flip-flops have no inverting D: IS_D_INVERTED becomes an inverting LUT.
    void pack_ffs()
    {
        std::vector<std::unique_ptr<CellInfo>> made;
        for (auto &[name, ci] : ctx->cells) {
            if (!ci->type.in(id_FDRE, id_FDSE, id_FDCE, id_FDPE))
                continue;
            auto dinv = ci->params.find(id_IS_D_INVERTED);
            if (dinv != ci->params.end() && dinv->second.as_int64() != 0) {
                NetInfo *d = ci->getPort(id_D);
                auto lut = std::make_unique<CellInfo>(ctx, ctx->idf("%s$dinv", name.c_str(ctx)), id_SLG_LUT);
                lut->params[id_INIT] = Property(0x1ull, 64);
                lut->params[id_LUT_MODE] = Property("LUT5");
                NetInfo *inverted = ctx->createNet(ctx->idf("%s$dinv_o", name.c_str(ctx)));
                lut->addInput(id_I0);
                lut->addOutput(id_O);
                if (d) {
                    ci->disconnectPort(id_D);
                    lut->connectPort(id_I0, d);
                }
                lut->connectPort(id_O, inverted);
                ci->connectPort(id_D, inverted);
                ci->params.erase(id_IS_D_INVERTED);
                made.push_back(std::move(lut));
            }
            ci->params[id_FF_TYPE] = Property(ci->type.str(ctx));
            ci->type = id_SLG_FF;
            if (ci->ports.count(id_C))
                ci->renamePort(id_C, id_CLK);
            // every kind of reset or set is the bel's one R
            for (IdString p : {id_S, id_CLR, id_PRE})
                if (ci->ports.count(p))
                    ci->renamePort(p, id_R);
            // A site selects a constant enable or reset in its own MUX_CE and
            // MUX_SR: it takes no route, and the constant need not reach it.
            for (auto [port, held, other, key] :
                 {std::tuple<IdString, const char *, const char *, const char *>{id_CE, "VCC", "GND", "CE_CONST"},
                  {id_R, "GND", "VCC", "R_CONST"}}) {
                NetInfo *net = ci->getPort(port);
                const CellInfo *driver = net ? net->driver.cell : nullptr;
                if (driver == nullptr)
                    continue;
                if (driver->type == ctx->id(other))
                    log_error("slg47910: %s holds its %s at %s: a flip-flop that never runs or is always "
                              "reset is not supported\n",
                              name.c_str(ctx), port.c_str(ctx), other);
                if (driver->type == ctx->id(held)) {
                    ci->disconnectPort(port);
                    ci->params[ctx->id(key)] = Property(std::string("$PACKER_") + held);
                }
            }
        }
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
    }

    // A flip-flop whose clock the fabric makes (a LUT, another flip-flop, a
    // pad that is no clock input) takes it back by the vendor's logic as clock
    // (app notes AN-FG-009, AN-FG-019): the net leaves at pad (31,11)'s first
    // output, the enable of that lane, (31,12)'s first output, held at 1, and
    // comes back as clock input E0, which only domain 1 takes (mux 9: the
    // vendor's builds of a counter on each lane, 2026-11-17; with W0, E0 and
    // E1 its placer stops before placing). One clock a design, then.
    void pack_fabric_clocks()
    {
        std::map<IdString, std::vector<CellInfo *>> clocked;
        for (auto &[name, ci] : ctx->cells) {
            NetInfo *clk = ci->type == id_SLG_FF ? ci->getPort(id_CLK) : nullptr;
            if (clk != nullptr && clk->driver.cell != nullptr && clk->driver.cell->type != id_SLG_CLKIN)
                clocked[clk->name].push_back(ci.get());
        }
        if (clocked.empty())
            return;
        if (clocked.size() > 1)
            log_error("slg47910: %zu clocks are made in the fabric (%s, %s, ...): the chip takes back one\n",
                      clocked.size(), clocked.begin()->first.c_str(ctx), std::next(clocked.begin())->first.c_str(ctx));
        for (auto &[port, token] : clock_ports)
            if (clock_domain(token) == 1)
                log_error("slg47910: the clock %s made in the fabric needs domain 1, which %s (%s) takes\n",
                          clocked.begin()->first.c_str(ctx), port.c_str(), token.c_str());
        auto &[name, ffs] = *clocked.begin();
        NetInfo *made = ctx->nets.at(name).get();

        auto pad = std::make_unique<CellInfo>(ctx, ctx->id("$lac_31_11"), id_SLG_IOB);
        pad->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(iob_at(31, 11, "logic as clock")));
        pad->attrs[ctx->id("CONTROL")] = Property(1);
        pad->addInput(id_PAD);
        pad->connectPort(id_PAD, made);
        held_controls.push_back({12, 31, 0, 1});

        auto clkin = std::make_unique<CellInfo>(ctx, ctx->id("$clk_$lac"), id_SLG_CLKIN);
        const ClockToken &e0 = clock_tokens().at("E0");
        clkin->attrs[ctx->id("BEL")] = std::string(ctx->nameOfBel(ctx->getBelByLocation(Loc(0, 0, CLKIN_Z + e0.z))));
        clkin->params[id_CLK_TOKEN] = Property(std::string("E0"));
        clkin->params[id_CLK_SIDE] = Property(std::string(e0.side));
        clkin->params[id_CLK_DOMAIN] = Property(e0.domain);
        NetInfo *back = ctx->createNet(ctx->id("$lac_clk"));
        clkin->addOutput(id_PAD);
        clkin->connectPort(id_PAD, back);
        for (CellInfo *ff : ffs) {
            ff->disconnectPort(id_CLK);
            ff->connectPort(id_CLK, back);
        }
        log_info("slg47910: %zu flip-flops on %s, a clock made in the fabric: out at pad (31,11), back on E0\n",
                 ffs.size(), name.c_str(ctx));
        ctx->cells[pad->name] = std::move(pad);
        ctx->cells[clkin->name] = std::move(clkin);
    }

    // A flip-flop driven by a LUT goes in its LUT's cluster and takes D by the
    // dedicated path. One whose D is any other net (another flip-flop, a net a
    // clustered LUT drives) takes it from the fabric (XI); a constant D, which
    // is not routed, gets a LUT of its own that holds it.
    void pair_ffs()
    {
        std::vector<std::unique_ptr<CellInfo>> made;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_FF)
                continue;
            NetInfo *d = ci->getPort(id_D);
            if (d == nullptr)
                continue;
            CellInfo *lut = d->driver.cell;
            bool constant = lut != nullptr && lut->type.in(ctx->id("VCC"), ctx->id("GND"));
            if (!constant && (lut == nullptr || lut->type != id_SLG_LUT || d->driver.port != id_O ||
                              lut->cluster != ClusterId()))
                continue;
            if (constant) {
                auto buf = std::make_unique<CellInfo>(ctx, ctx->idf("%s$buf", name.c_str(ctx)), id_SLG_LUT);
                buf->params[id_INIT] = Property(0x2ull, 64);
                buf->params[id_LUT_MODE] = Property("LUT5");
                NetInfo *out = ctx->createNet(ctx->idf("%s$buf_o", name.c_str(ctx)));
                buf->addInput(id_I0);
                buf->addOutput(id_O);
                ci->disconnectPort(id_D);
                buf->connectPort(id_I0, d);
                buf->connectPort(id_O, out);
                ci->connectPort(id_D, out);
                lut = buf.get();
                made.push_back(std::move(buf));
            }
            lut->cluster = lut->name;
            lut->constr_abs_z = false;
            lut->constr_z = 0;
            ci->cluster = lut->name;
            ci->constr_abs_z = false;
            ci->constr_z = FF_Z;
            lut->constr_children.push_back(ci.get());
        }
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
        log_info("slg47910: %zu flip-flops hold a constant D in a LUT of their own\n", made.size());
    }

    // A CARRY4 chain (each one's CO[3] the next one's CI) becomes a bit of the
    // fabric's chain per slice, SLG_CARRY: P from the LUT that drives S, in its
    // slice; G on DI (XI, or a constant MUX_xI holds); the carry in from the bit
    // below, or a constant (CIN_SEL) at the chain's foot; O the sum. A chain is
    // one cluster rooted at its first bit, with the bits' LUTs and the
    // flip-flops that take a sum and nothing else, up one column of sites
    // (getClusterPlacement).
    void pack_carry()
    {
        auto bus = [&](CellInfo *ci, const char *port, int i) { return ci->getPort(ctx->idf("%s[%d]", port, i)); };
        // 0 or 1 for a constant (no net is 0), -1 for a signal
        auto constant = [&](const NetInfo *net) {
            const CellInfo *d = net ? net->driver.cell : nullptr;
            return net == nullptr || (d && d->type == ctx->id("GND")) ? 0 : d && d->type == ctx->id("VCC") ? 1 : -1;
        };
        std::vector<CellInfo *> carry4s;
        std::map<CellInfo *, CellInfo *> next;
        std::set<CellInfo *> continued;
        for (auto &[name, ci] : ctx->cells)
            if (ci->type == id_CARRY4)
                carry4s.push_back(ci.get());
        for (CellInfo *c : carry4s) {
            const NetInfo *ci = c->getPort(id_CI);
            if (ci && ci->driver.cell && ci->driver.cell->type == id_CARRY4 && ci->driver.port == ctx->id("CO[3]")) {
                next[ci->driver.cell] = c;
                continued.insert(c);
            }
        }
        struct Bit
        {
            NetInfo *s, *di, *o, *co;
        };
        int chains = 0;
        for (CellInfo *head : carry4s) {
            if (continued.count(head))
                continue;
            // The carry in: CI | CYINIT of the first.
            NetInfo *ci = head->getPort(id_CI), *cyinit = head->getPort(id_CYINIT);
            int ci_const = constant(ci), cyinit_const = constant(cyinit);
            NetInfo *carry_in = nullptr;
            if (ci_const == 1 || cyinit_const == 1)
                ci_const = 1;
            else if (ci_const == -1 && cyinit_const == -1)
                log_error("slg47910: %s takes a carry in on CI and on CYINIT\n", head->name.c_str(ctx));
            else if (ci_const == -1 || cyinit_const == -1)
                carry_in = ci_const == -1 ? ci : cyinit, ci_const = 0;
            std::vector<Bit> bits;
            // A carry in from logic enters as the generate of a bit that propagates nothing.
            if (carry_in)
                bits.push_back({nullptr, carry_in, nullptr, nullptr});
            for (CellInfo *c = head; c; c = next.count(c) ? next.at(c) : nullptr) {
                for (int i = 0; i < 4; i++)
                    bits.push_back({bus(c, "S", i), bus(c, "DI", i), bus(c, "O", i), bus(c, "CO", i)});
                for (auto port : std::vector<IdString>{id_CI, id_CYINIT})
                    c->disconnectPort(port);
                for (int i = 0; i < 4; i++)
                    for (const char *port : {"S", "DI", "O", "CO"})
                        c->disconnectPort(ctx->idf("%s[%d]", port, i));
            }
            // Bits past the last that anything takes are not built.
            auto used = [](const NetInfo *net) { return net != nullptr && net->users.entries() > 0; };
            while (!bits.empty() && !used(bits.back().o) && !used(bits.back().co))
                bits.pop_back();
            if (bits.empty())
                continue;
            // A bit leaves its site by one pin (its slice's xMUX): the last
            // one's sum and carry out, both taken, are one too many. A bit past
            // it that propagates nothing (P 0) has the carry out as its sum.
            if (used(bits.back().o) && used(bits.back().co)) {
                bits.push_back({nullptr, nullptr, bits.back().co, nullptr});
                bits[bits.size() - 2].co = nullptr;
            }
            if (bits.size() > 40)
                log_error("slg47910: the carry chain from %s is %zu bits long: a column holds 40\n",
                          head->name.c_str(ctx), bits.size());
            chains++;
            CellInfo *root = nullptr;
            NetInfo *below = nullptr;
            std::vector<CellInfo *> luts, ffs;
            std::vector<const NetInfo *> gs;
            auto join = [&](CellInfo *cell, int k, int z) {
                cell->cluster = root->name;
                cell->constr_abs_z = true;
                cell->constr_z = z;
                cell->constr_x = 0;
                cell->constr_y = k / 4; // the site up the column (getClusterPlacement)
                if (cell != root)
                    root->constr_children.push_back(cell);
            };
            for (int k = 0; k < int(bits.size()); k++) {
                Bit &b = bits[k];
                CellInfo *bit = ctx->createCell(ctx->idf("%s$bit%d", head->name.c_str(ctx), k), id_SLG_CARRY);
                for (IdString port : {id_P, id_DI, id_CI})
                    bit->addInput(port);
                bit->addOutput(id_O);
                bit->addOutput(id_CO);
                if (root == nullptr)
                    root = bit;
                join(bit, k, CARRY_Z + k % 4);
                if (k == 0)
                    bit->params[id_CI_CONST] = Property(ci_const);
                else
                    bit->connectPort(id_CI, below);
                below = b.co ? b.co : k + 1 < int(bits.size()) ? ctx->createNet(ctx->idf("%s$co", bit->name.c_str(ctx))) : nullptr;
                if (below)
                    bit->connectPort(id_CO, below);
                if (b.o)
                    bit->connectPort(id_O, b.o);
                if (constant(b.di) >= 0)
                    bit->params[id_DI_CONST] = Property(constant(b.di));
                else
                    bit->connectPort(id_DI, b.di);
                // P: the LUT that drives S, or one that holds S (a buffer, a
                // constant). Not one that takes I5: with four routed G a block
                // has no I5 pin to spare for it.
                CellInfo *lut = b.s ? b.s->driver.cell : nullptr;
                if (lut && lut->type == id_SLG_LUT && b.s->driver.port == id_O && lut->cluster == ClusterId() &&
                    lut->getPort(id_I5) == nullptr)
                    bit->connectPort(id_P, b.s);
                else {
                    lut = ctx->createCell(ctx->idf("%s$p", bit->name.c_str(ctx)), id_SLG_LUT);
                    int held = constant(b.s);
                    lut->params[id_INIT] = Property(held == 1 ? 0xFFFFFFFFFFFFFFFFull : held == 0 ? 0ull : 0x2ull, 64);
                    lut->params[id_LUT_MODE] = Property("LUT5");
                    lut->addOutput(id_O);
                    if (held < 0) {
                        lut->addInput(id_I0);
                        lut->connectPort(id_I0, b.s);
                    }
                    NetInfo *p = ctx->createNet(ctx->idf("%s$p", bit->name.c_str(ctx)));
                    lut->connectPort(id_O, p);
                    bit->connectPort(id_P, p);
                }
                join(lut, k, k % 4);
                luts.push_back(lut);
                ffs.push_back(nullptr);
                // a flip-flop that takes the sum and is all the sum drives
                if (b.o && b.o->users.entries() == 1) {
                    const PortRef &usr = *b.o->users.begin();
                    if (usr.cell->type == id_SLG_FF && usr.port == id_D && usr.cell->cluster == ClusterId())
                        ffs.back() = usr.cell;
                }
                gs.push_back(bit->getPort(id_DI));
            }
            // A site's four I5 pins bring in its bits' G, its LUTs' I5 and its
            // flip-flops' enable and reset, and its flip-flops share one enable
            // and one set/reset: a site whose flip-flops would want more leaves
            // them out, to take the sum by xMUX where they are.
            for (int k0 = 0; k0 < int(bits.size()); k0 += 4) {
                std::set<const NetInfo *> nets;
                for (int k = k0; k < std::min(k0 + 4, int(bits.size())); k++) {
                    nets.insert(gs[k]);
                    nets.insert(luts[k]->getPort(id_I5));
                }
                std::set<const NetInfo *> with_ffs = nets;
                // and one enable, one set/reset, acting at once or on the clock
                std::set<std::tuple<const NetInfo *, const NetInfo *, bool>> controls;
                for (int k = k0; k < std::min(k0 + 4, int(bits.size())); k++)
                    if (ffs[k]) {
                        with_ffs.insert(ffs[k]->getPort(id_CE)), with_ffs.insert(ffs[k]->getPort(id_R));
                        controls.insert({ffs[k]->getPort(id_CE), ffs[k]->getPort(id_R),
                                         ff_type_is(ffs[k], "FDCE", "FDPE")});
                    }
                with_ffs.erase(nullptr);
                if (with_ffs.size() > 4 || controls.size() > 1)
                    continue;
                for (int k = k0; k < std::min(k0 + 4, int(bits.size())); k++)
                    if (ffs[k])
                        join(ffs[k], k, FF_Z + k % 4);
            }
        }
        for (CellInfo *c : carry4s)
            ctx->cells.erase(c->name);
        if (chains)
            log_info("slg47910: %d carry chains\n", chains);
    }

    // A MUXF8 with the two MUXF7 it takes, or a MUXF7 alone, is one cluster
    // with the LUTs they take: the site's F7A (A or B, selected by A's XI),
    // F7C (C or D, by C's XI) and F8 (F7A or F7C, by B's XI) take only their
    // own slices' LUTs (gen). An input no LUT of its own drives (a LUT taken
    // by another cluster, a constant) gets a LUT that passes it on.
    void pack_muxf()
    {
        int muxes = 0;
        auto lut_for = [&](CellInfo *mux, IdString port) {
            NetInfo *net = mux->getPort(port);
            CellInfo *lut = net ? net->driver.cell : nullptr;
            if (lut && lut->type == id_SLG_LUT && net->driver.port == id_O && lut->cluster == ClusterId())
                return lut;
            lut = ctx->createCell(ctx->idf("%s$%s", mux->name.c_str(ctx), port.c_str(ctx)), id_SLG_LUT);
            lut->params[id_INIT] = Property(net ? 0x2ull : 0ull, 64);
            lut->params[id_LUT_MODE] = Property("LUT5");
            lut->addOutput(id_O);
            NetInfo *passed = ctx->createNet(ctx->idf("%s$%s", mux->name.c_str(ctx), port.c_str(ctx)));
            if (net) {
                lut->addInput(id_I0);
                mux->disconnectPort(port);
                lut->connectPort(id_I0, net);
            }
            lut->connectPort(id_O, passed);
            mux->connectPort(port, passed);
            return lut;
        };
        auto join = [&](CellInfo *root, CellInfo *cell, int z) {
            cell->cluster = root->name;
            cell->constr_abs_z = true;
            cell->constr_z = z;
            if (cell != root)
                root->constr_children.push_back(cell);
        };
        std::vector<CellInfo *> f7s, f8s;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type.in(id_MUXF7, id_MUXF8)) {
                const NetInfo *sel = ci->getPort(id_S);
                if (sel == nullptr || sel->driver.cell == nullptr ||
                    sel->driver.cell->type.in(ctx->id("VCC"), ctx->id("GND")))
                    log_error("slg47910: %s (%s) has a constant select\n", name.c_str(ctx), ci->type.c_str(ctx));
                (ci->type == id_MUXF7 ? f7s : f8s).push_back(ci.get());
                ci->type = ci->type == id_MUXF7 ? id_SLG_MUXF7 : id_SLG_MUXF8;
            }
        }
        // A MUXF8 takes two MUXF7 that nothing else has taken: F7A on I0, F7C on I1.
        for (CellInfo *f8 : f8s) {
            join(f8, f8, MUXF_Z + 2);
            for (auto [port, z] : {std::pair<IdString, int>{id_I0, 0}, {id_I1, 1}}) {
                NetInfo *net = f8->getPort(port);
                CellInfo *f7 = net ? net->driver.cell : nullptr;
                if (f7 == nullptr || f7->type != id_SLG_MUXF7 || f7->cluster != ClusterId())
                    log_error("slg47910: %s takes %s from no MUXF7 of its own\n", f8->name.c_str(ctx),
                              port.c_str(ctx));
                join(f8, f7, MUXF_Z + z);
                join(f8, lut_for(f7, id_I0), 2 * z);
                join(f8, lut_for(f7, id_I1), 2 * z + 1);
            }
            muxes++;
        }
        // A MUXF7 alone is F7A or F7C: its LUTs' slices follow where it is
        // placed (getClusterPlacement), the one on I0 below the one on I1.
        for (CellInfo *f7 : f7s) {
            if (f7->cluster != ClusterId())
                continue;
            f7->cluster = f7->name;
            f7->constr_abs_z = false;
            for (auto [port, z] : {std::pair<IdString, int>{id_I0, 0}, {id_I1, 1}}) {
                CellInfo *lut = lut_for(f7, port);
                lut->cluster = f7->name;
                lut->constr_abs_z = false;
                lut->constr_z = z;
                f7->constr_children.push_back(lut);
            }
            muxes++;
        }
        if (muxes)
            log_info("slg47910: %zu MUXF7, %zu MUXF8\n", f7s.size(), f8s.size());
    }

    bool getClusterPlacement(ClusterId cluster, BelId root_bel,
                             std::vector<std::pair<CellInfo *, BelId>> &placement) const override
    {
        CellInfo *root = getClusterRootCell(cluster);
        if (root->type == id_SLG_MUXF7 && !root->constr_abs_z) {
            // F7A takes slices A and B, F7C slices C and D.
            Loc at = ctx->getBelLocation(root_bel);
            if (ctx->getBelType(root_bel) != id_SLG_MUXF7)
                return false;
            placement.clear();
            placement.emplace_back(root, root_bel);
            for (CellInfo *lut : root->constr_children)
                placement.emplace_back(lut, ctx->getBelByLocation(Loc(at.x, at.y, 2 * (at.z - MUXF_Z) + lut->constr_z)));
            return true;
        }
        if (root->type != id_SLG_CARRY)
            return HimbaechelAPI::getClusterPlacement(cluster, root_bel, placement);
        Loc at = ctx->getBelLocation(root_bel);
        if (ctx->getBelType(root_bel) != id_SLG_CARRY || at.z != CARRY_Z)
            return false;
        std::vector<Loc> column{at};
        placement.clear();
        placement.emplace_back(root, root_bel);
        for (CellInfo *child : root->constr_children) {
            while (int(column.size()) <= child->constr_y) {
                auto up = site_above.find({column.back().x, column.back().y});
                if (up == site_above.end())
                    return false;
                column.push_back(Loc(up->second.first, up->second.second, 0));
            }
            Loc l = column.at(child->constr_y);
            l.z = child->constr_z;
            BelId bel = ctx->getBelByLocation(l);
            if (bel == BelId() || !isValidBelForCellType(child->type, bel))
                return false;
            placement.emplace_back(child, bel);
        }
        return true;
    }

    // A carry bit, its LUT and its flip-flop sit constr_y sites up the column
    // from the chain's foot: sites are not evenly spaced, so the offset is the
    // placed root's, or a column foot's before it is placed.
    Loc getClusterOffset(const CellInfo *cell) const override
    {
        const CellInfo *root = getClusterRootCell(cell->cluster);
        if (root->type == id_SLG_MUXF7 && !root->constr_abs_z)
            return Loc(0, 0, 0);
        if (root->type != id_SLG_CARRY)
            return HimbaechelAPI::getClusterOffset(cell);
        std::pair<int, int> foot = column_foot;
        if (root->bel != BelId())
            foot = {ctx->getBelLocation(root->bel).x, ctx->getBelLocation(root->bel).y};
        std::pair<int, int> site = foot;
        for (int k = 0; k < cell->constr_y && site_above.count(site); k++)
            site = site_above.at(site);
        return Loc(site.first - foot.first, site.second - foot.second, cell->constr_z);
    }

    // Every output pad is driven by a LUT of its own. On silicon a pad shows
    // the complement of what a flip-flop sends it, and a LUT's output arrives
    // complemented wherever it leaves its site, so through a LUT a pad reads
    // true. The level-13 lines a route takes invert too; the bitstream tools
    // take each sink's count back (a pad's in its LUT's table).
    static bool pad_only_lut(const NetInfo *net)
    {
        return net->driver.cell != nullptr && net->driver.cell->type == id_SLG_LUT && net->users.entries() == 1 &&
               (*net->users.begin()).cell->type == id_SLG_IOB;
    }

    void buffer_pads()
    {
        std::vector<std::unique_ptr<CellInfo>> made;
        for (auto &[name, ci] : ctx->cells)
            for (IdString pin : {id_PAD, id_OE}) {
                if (ci->type != id_SLG_IOB || !ci->ports.count(pin) || ci->ports.at(pin).type != PORT_IN)
                    continue;
                NetInfo *net = ci->getPort(pin);
                if (net == nullptr || net->driver.cell == nullptr || pad_only_lut(net))
                    continue;
                auto buf = std::make_unique<CellInfo>(ctx, ctx->idf("%s$buf_%s", name.c_str(ctx), pin.c_str(ctx)),
                                                      id_SLG_LUT);
                buf->params[id_INIT] = Property(0x2ull, 64);
                buf->params[id_LUT_MODE] = Property("LUT5");
                NetInfo *out = ctx->createNet(ctx->idf("%s$buf_%s_o", name.c_str(ctx), pin.c_str(ctx)));
                buf->addInput(id_I0);
                buf->addOutput(id_O);
                ci->disconnectPort(pin);
                buf->connectPort(id_I0, net);
                buf->connectPort(id_O, out);
                ci->connectPort(pin, out);
                made.push_back(std::move(buf));
            }
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
    }

    // A LUT input on a constant (a buffer for a constant D, an input Yosys left
    // tied) is folded into the table and disconnected: constants are not routed.
    void fold_constant_inputs()
    {
        const IdString inputs[] = {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_LUT)
                continue;
            uint64_t init = uint64_t(param(ci.get(), id_INIT));
            for (int k = 0; k < 6; k++) {
                const NetInfo *net = ci->getPort(inputs[k]);
                const CellInfo *driver = net ? net->driver.cell : nullptr;
                if (driver == nullptr || !driver->type.in(ctx->id("VCC"), ctx->id("GND")))
                    continue;
                bool one = driver->type == ctx->id("VCC");
                uint64_t folded = 0;
                for (int addr = 0; addr < 64; addr++)
                    folded |= (init >> (one ? addr | (1 << k) : addr & ~(1 << k)) & 1) << addr;
                init = folded;
                ci->disconnectPort(inputs[k]);
            }
            ci->params[id_INIT] = Property(init, 64);
        }
    }

    // A slice's LUT6 is two LUT5 on its first five inputs (gen): with the
    // sixth tied to 1 (x6_TIE 0) the slice's output is the upper half and its
    // xMUX pin the lower (MUX_X's OX0), as the vendor's packer pairs LUTs (its
    // inst_LUT4O, in 2,165 slices of its builds). Two LUTs that share an
    // input and take five in all become one slice's halves, both rewritten
    // onto the same five: the upper one may hold its flip-flop, the lower one
    // nothing else. A LUT that drives a pad is left alone: it is the pad's
    // own, out by the slice's output.
    void pair_luts()
    {
        static const IdString ports[] = {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
        auto inputs = [&](CellInfo *c) {
            std::vector<NetInfo *> nets;
            for (IdString port : ports)
                if (NetInfo *net = c->getPort(port); net && std::find(nets.begin(), nets.end(), net) == nets.end())
                    nets.push_back(net);
            return nets;
        };
        auto own_ff_only = [&](const CellInfo *c) {
            if (c->cluster == ClusterId())
                return true;
            if (c->cluster != c->name)
                return false;
            for (const CellInfo *child : c->constr_children)
                if (child->type != id_SLG_FF)
                    return false;
            return true;
        };
        auto pairable = [&](CellInfo *c) {
            if (c->type != id_SLG_LUT || !own_ff_only(c) || inputs(c).empty() || inputs(c).size() > 5)
                return false;
            if (c->attrs.count(ctx->id("BEL")))
                return false; // placed by the user where it is: not moved into a pair
            const NetInfo *out = c->getPort(id_O);
            if (out == nullptr || out->users.entries() == 0)
                return false;
            for (auto &usr : out->users)
                if (usr.cell->type == id_SLG_IOB)
                    return false;
            return true;
        };
        // Each LUT on the nets it takes, in the order of the union.
        auto rewrite = [&](CellInfo *c, const std::vector<NetInfo *> &onto) {
            uint64_t init = uint64_t(param(c, id_INIT)), made = 0;
            std::vector<int> at(6, -1);
            for (int k = 0; k < 6; k++)
                if (NetInfo *net = c->getPort(ports[k]))
                    at[k] = int(std::find(onto.begin(), onto.end(), net) - onto.begin());
            for (int addr = 0; addr < 64; addr++) {
                int from = 0;
                for (int k = 0; k < 6; k++)
                    if (at[k] >= 0 && (addr >> at[k] & 1))
                        from |= 1 << k;
                made |= (init >> from & 1) << addr;
            }
            std::vector<NetInfo *> taken;
            for (int k = 0; k < 6; k++)
                if (NetInfo *net = c->getPort(ports[k])) {
                    taken.push_back(net);
                    c->disconnectPort(ports[k]);
                }
            for (int j = 0; j < int(onto.size()); j++)
                if (std::find(taken.begin(), taken.end(), onto[j]) != taken.end()) {
                    if (!c->ports.count(ports[j]))
                        c->addInput(ports[j]);
                    c->connectPort(ports[j], onto[j]);
                }
            c->params[id_INIT] = Property(made, 64);
        };
        std::set<const CellInfo *> paired;
        int pairs = 0;
        std::vector<CellInfo *> luts;
        for (auto &[name, ci] : ctx->cells)
            if (pairable(ci.get()))
                luts.push_back(ci.get());
        for (CellInfo *a : luts) {
            if (paired.count(a))
                continue;
            std::vector<NetInfo *> a_in = inputs(a);
            CellInfo *best = nullptr;
            int best_shared = 0;
            for (NetInfo *net : a_in) {
                if (net->users.entries() > 64)
                    continue;
                for (auto &usr : net->users) {
                    CellInfo *b = usr.cell;
                    if (b == a || paired.count(b) || !pairable(b) ||
                        (a->cluster != ClusterId() && b->cluster != ClusterId()))
                        continue;
                    std::vector<NetInfo *> b_in = inputs(b);
                    int shared = 0;
                    for (NetInfo *n : b_in)
                        shared += std::find(a_in.begin(), a_in.end(), n) != a_in.end();
                    if (int(a_in.size() + b_in.size()) - shared <= 5 && shared > best_shared)
                        best = b, best_shared = shared;
                }
            }
            if (best == nullptr)
                continue;
            CellInfo *upper = a->cluster != ClusterId() || best->cluster == ClusterId() ? a : best;
            CellInfo *lower = upper == a ? best : a;
            std::vector<NetInfo *> onto = inputs(upper);
            for (NetInfo *n : inputs(lower))
                if (std::find(onto.begin(), onto.end(), n) == onto.end())
                    onto.push_back(n);
            rewrite(upper, onto);
            rewrite(lower, onto);
            if (upper->cluster == ClusterId()) {
                upper->cluster = upper->name;
                upper->constr_abs_z = false;
                upper->constr_z = 0;
            }
            lower->type = id_SLG_LUTL;
            lower->cluster = upper->name;
            lower->constr_abs_z = false;
            lower->constr_z = LUTL_Z;
            upper->constr_children.push_back(lower);
            paired.insert(upper);
            paired.insert(lower);
            pairs++;
        }
        if (pairs)
            log_info("slg47910: %d pairs of LUTs share a slice\n", pairs);
    }

    void pack() override
    {
        for (auto &[name, ci] : ctx->cells) {
            std::string type = ci->type.str(ctx);
            if ((type.rfind("MUXF", 0) == 0 && type != "MUXF7" && type != "MUXF8") || type.rfind("RAM", 0) == 0 ||
                type.rfind("SRL", 0) == 0 || type.rfind("RAMB", 0) == 0 || type.rfind("PLL", 0) == 0 ||
                type.rfind("MMCM", 0) == 0)
                log_error("slg47910: %s (%s) is not supported yet: the chipdb has neither MUXF5/6, memory nor a PLL "
                          "(synthesise with -nobram -nodsp -nolutram)\n",
                          name.c_str(ctx), type.c_str());
        }
        read_pins();
        pack_io();
        pack_luts();
        pack_ffs();
        pack_fabric_clocks();
        pack_muxf();
        pack_carry();
        pair_ffs();
        buffer_pads();
        fold_constant_inputs();
        h.replace_constants(CellTypePort(id_SLG_LUT, id_O), CellTypePort(id_SLG_LUT, id_O),
                            {{id_INIT, Property(0xFFFFFFFFFFFFFFFFull, 64)}}, {{id_INIT, Property(0x0ull, 64)}});
        pair_luts();
        cap_slices();
    }

    // The four nodes of a slice's input pool each feed several slices of the
    // sites around: a site with every slice full leaves its neighbours fewer to
    // route through. So the placer fills sites as far as the design needs, as
    // the vendor's does (its packer packs densely past half the chip,
    // HIGH_DENSITY_PACKING_RBB_THR 50): two slices a site while the logic
    // takes at most four fifths of them, then three or four. A slice holds a
    // LUT, or two flip-flops of their own; a carry chain or a MUXF8 fills
    // its sites, four bits a site.
    void cap_slices()
    {
        int luts = 0, ffs = 0, carry = 0, muxf8 = 0, sites = 0;
        for (BelId bel : ctx->getBels())
            sites += ctx->getBelType(bel) == id_SLG_CARRY && ctx->getBelLocation(bel).z == CARRY_Z;
        for (auto &[name, ci] : ctx->cells) {
            carry += ci->type == id_SLG_CARRY;
            muxf8 += ci->type == id_SLG_MUXF8;
            if (ci->cluster != ClusterId() && getClusterRootCell(ci->cluster)->type.in(id_SLG_CARRY, id_SLG_MUXF8))
                continue;
            luts += ci->type == id_SLG_LUT;
            ffs += ci->type == id_SLG_FF &&
                   (ci->cluster == ClusterId() || getClusterRootCell(ci->cluster)->type != id_SLG_LUT);
        }
        int demand = luts + (ffs + 1) / 2, free = sites - (carry + 3) / 4 - muxf8;
        slices = std::clamp((5 * demand + 4 * free - 1) / (4 * std::max(free, 1)), 2, 4);
        log_info("slg47910: %d slices of logic in %d sites: at most %d slices a site\n", demand, free, slices);
    }

    // ---- validity ----

    bool isValidBelForCellType(IdString cell_type, BelId bel) const override
    {
        return ctx->getBelType(bel) == cell_type;
    }

    // ---- timing ----

    // A bel of a memory site's logic (gen/slg47910_gen.py's BEL_FLAG_MEMORY).
    static constexpr uint32_t BEL_FLAG_MEMORY = 0x100;
    int lut_m = -1, lutl_m = -1;

    // A LUT placed in a memory site takes the memory layer's arcs: the vendor
    // times its LUTs 0.4-1.1 ns slower after routing (timing.json's LUT_M).
    // A LUT whose output feeds back into it is a latch (the flow's
    // latch_map.v, as the vendor builds one): that input times no path.
    bool getCellDelay(const CellInfo *cell, IdString fromPort, IdString toPort, DelayQuad &delay) const override
    {
        if (cell->type.in(id_SLG_LUT, id_SLG_LUTL) && cell->getPort(fromPort) != nullptr &&
            cell->getPort(fromPort) == cell->getPort(toPort))
            return false;
        if (cell->bel != BelId() && (cell->type == id_SLG_LUT || cell->type == id_SLG_LUTL) &&
            (chip_bel_info(ctx->chip_info, cell->bel).flags & BEL_FLAG_MEMORY))
            return ctx->lookup_cell_delay(cell->type == id_SLG_LUT ? lut_m : lutl_m, fromPort, toPort, delay);
        return HimbaechelAPI::getCellDelay(cell, fromPort, toPort, delay);
    }
    IdString getBelBucketForCellType(IdString cell_type) const override { return cell_type; }

    static bool ff_type_is(const CellInfo *ci, const char *a, const char *b)
    {
        auto it = ci->params.find(id_FF_TYPE);
        return it != ci->params.end() && (it->second.as_string() == a || it->second.as_string() == b);
    }

    const CellInfo *bound_at(Loc at, int z) const
    {
        return ctx->getBoundBelCell(ctx->getBelByLocation(Loc(at.x, at.y, z)));
    }

    bool isBelLocationValid(BelId bel, bool explain_invalid) const override
    {
        // A bel left empty is checked too: a flip-flop moved away can leave
        // the other of its slice taking D from the fabric, one more net on
        // the I5 pins.
        if (!ctx->getBelType(bel).in(id_SLG_LUT, id_SLG_LUTL, id_SLG_FF, id_SLG_CARRY, id_SLG_MUXF7, id_SLG_MUXF8))
            return true;
        Loc at = ctx->getBelLocation(bel);
        auto fail = [&](const char *why) {
            if (explain_invalid)
                log_warning("slg47910: (%d,%d): %s\n", at.x, at.y, why);
            return false;
        };
        int busy = 0;
        bool logic = false;
        std::set<const NetInfo *> i5;
        std::vector<const CellInfo *> ffs;
        const CellInfo *mux_of[4] = {};
        for (int k = 0; k < 3; k++)
            mux_of[MUXF_SLICE[k]] = bound_at(at, MUXF_Z + k);
        for (int sl = 0; sl < 4; sl++) {
            const CellInfo *lut = bound_at(at, sl), *ff = bound_at(at, FF_Z + sl), *off = bound_at(at, OFF_Z + sl),
                           *carry = bound_at(at, CARRY_Z + sl), *mux = mux_of[sl], *lower = bound_at(at, LUTL_Z + sl);
            // A carry chain or a MUXF8 fills its site.
            for (const CellInfo *c : {lut, ff, off, carry})
                logic |= c && (c->cluster == ClusterId() ||
                               !getClusterRootCell(c->cluster)->type.in(id_SLG_CARRY, id_SLG_MUXF8));
            busy += lut || ff || off;
            // XI (MUX_xI) brings in one thing: a wide mux's select, a carry's G
            // (a constant one too: MUX_xI holds it), or the flip-flops' D from
            // the fabric -- the second's but where it takes the first's Q --
            // when both take the same net (emit_sdc.py).
            std::set<const NetInfo *> ff_xi;
            if (ff && ff->cluster == ClusterId())
                ff_xi.insert(ff->getPort(id_D));
            if (off && !(ff && off->getPort(id_D) == ff->getPort(id_Q)))
                ff_xi.insert(off->getPort(id_D));
            ff_xi.erase(nullptr);
            if (ff_xi.size() > 1 || int(carry != nullptr) + int(mux != nullptr) + int(!ff_xi.empty()) > 1)
                return fail("two cells want one slice's XI");
            std::set<const NetInfo *> xi = ff_xi;
            if (carry)
                xi.insert(carry->getPort(id_DI));
            if (mux)
                xi.insert(mux->getPort(id_S));
            xi.erase(nullptr);
            // one xMUX pin: the second flip-flop's Q, the carry's sum or
            // carry, a wide mux, or the LUT's lower half
            if (int(off != nullptr) + int(carry != nullptr) + int(mux != nullptr) + int(lower != nullptr) > 1)
                return fail("two cells want one slice's xMUX pin");
            i5.insert(xi.begin(), xi.end());
            if (lut)
                i5.insert(lut->getPort(id_I5));
            for (const CellInfo *f : {ff, off})
                if (f) {
                    ffs.push_back(f);
                    i5.insert(f->getPort(id_CE));
                    i5.insert(f->getPort(id_R));
                }
        }
        // As many slices as the design needs (cap_slices); a carry chain
        // fills its sites (its bits take few inputs each).
        if (logic && busy > slices)
            return fail("more slices in use than the design needs");
        // The site's four I5 pins bring in every LUT's I5, every XI, the
        // enable and the set/reset.
        i5.erase(nullptr);
        if (i5.size() > 4)
            return fail("more than four nets on the I5 pins");
        // One clock, one enable and one set/reset for the site's eight
        // flip-flops, acting on the clock or at once (QasyncSRen) for all.
        for (const CellInfo *f : ffs) {
            if (f->getPort(id_CLK) != ffs.front()->getPort(id_CLK))
                return fail("two clocks");
            if (f->getPort(id_CE) != ffs.front()->getPort(id_CE))
                return fail("two enables");
            if (f->getPort(id_R) != ffs.front()->getPort(id_R))
                return fail("two set/resets");
            if (f->getPort(id_R) && ff_type_is(f, "FDCE", "FDPE") != ff_type_is(ffs.front(), "FDCE", "FDPE"))
                return fail("a synchronous and an asynchronous set/reset");
        }
        return true;
    }

    // A pad is only the source of the net its input cell drives and the
    // sink of the net its output cell takes: not a way through the pad. And
    // no net takes a line an output pad's enable rests on (gen: REST pips):
    // there the enable follows no net, and moved off it a pad lets go.
    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override
    {
        if (resting.count(IdString(chip_pip_info(ctx->chip_info, pip).type)))
            return false;
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

    void preRoute() override
    {
        // The pip types of the lines the output pads' enables rest on.
        std::set<std::pair<int, int>> outputs;
        for (auto &[name, ci] : ctx->cells)
            if (ci->type == id_SLG_IOB && ci->bel != BelId() && ci->ports.count(id_PAD) &&
                ci->ports.at(id_PAD).type == PORT_IN) {
                Loc at = ctx->getBelLocation(ci->bel);
                outputs.insert({at.x, at.y});
            }
        pool<IdString> seen;
        for (PipId pip : ctx->getPips()) {
            IdString type(chip_pip_info(ctx->chip_info, pip).type);
            if (!seen.insert(type).second)
                continue;
            std::string s = type.str(ctx);
            if (s.compare(0, 5, "REST:") != 0)
                continue;
            for (size_t i = 5; i < s.size();) {
                size_t comma = s.find(',', i), end = std::min(s.find(';', i), s.size());
                if (outputs.count({std::stoi(s.substr(i, comma - i)), std::stoi(s.substr(comma + 1, end - comma - 1))}))
                    resting.insert(type);
                i = end + 1;
            }
        }
        // A pad on the edge can need a detour far out of its net's box.
        ctx->settings[ctx->id("router2/bbMargin/x")] = std::string("8");
        ctx->settings[ctx->id("router2/bbMargin/y")] = std::string("8");
    }

    // ---- what the bitstream tools take ----

    // Which site pin (`SPI_<k>` in, `SPO_<k>` out) a wire is, or -1.
    int pin_of(WireId w, const char *prefix) const
    {
        if (w == WireId())
            return -1;
        std::string type = ctx->getWireType(w).str(ctx);
        size_t n = strlen(prefix);
        return type.compare(0, n, prefix) == 0 ? std::stoi(type.substr(n)) : -1;
    }
    WireId uphill_bound(WireId sink) const
    {
        if (sink == WireId())
            return WireId();
        for (PipId pip : ctx->getPipsUphill(sink))
            if (ctx->getBoundPipNet(pip) != nullptr)
                return ctx->getPipSrcWire(pip);
        return WireId();
    }
    WireId downhill_bound(WireId source) const
    {
        if (source == WireId())
            return WireId();
        for (PipId pip : ctx->getPipsDownhill(source))
            if (ctx->getBoundPipNet(pip) != nullptr)
                return ctx->getPipDstWire(pip);
        return WireId();
    }
    // The site pin a bel's input comes in by (through XI, for a D or a carry's G).
    int in_pin(BelId bel, IdString port) const
    {
        WireId w = uphill_bound(ctx->getBelPinWire(bel, port));
        if (w != WireId() && ctx->getWireType(w) == id_XI)
            w = uphill_bound(w);
        return pin_of(w, "SPI_");
    }
    // The net on a port, or the constant the cell holds it at (`key`)
    std::string net_of(const CellInfo *ci, IdString port, const char *key = nullptr) const
    {
        const NetInfo *net = ci->getPort(port);
        if (net == nullptr && key != nullptr)
            return text(ci, ctx->id(key));
        return net ? net->name.str(ctx) : "";
    }
    int64_t param(const CellInfo *ci, IdString key, int64_t otherwise = 0) const
    {
        auto it = ci->params.find(key);
        return it == ci->params.end() ? otherwise : it->second.as_int64();
    }
    std::string text(const CellInfo *ci, IdString key) const
    {
        auto it = ci->params.find(key);
        return it == ci->params.end() ? "" : it->second.as_string();
    }

    // The clock domain a pad's output is retimed on: a row-31 pad's output
    // passes a register on one of the two domains (bl4 wl26). The vendor puts
    // it on the domain of the flip-flops behind the pad, on domain 1 when they
    // are of both (its build of counters on W0 and W1, 2026-11-17); on
    // silicon an output on the other domain's clock stops while it does. With
    // none behind it (a pad, a constant, what the chip states) it goes on the
    // oscillator's, the one clock that always runs: the vendor writes domain
    // 1 there, where in a design clocked from W0 alone an output of ours
    // stays 0 (PLL_LOCK to GPIO8, 2026-11-17).
    int output_domain(const CellInfo *iob, const std::map<std::string, int> &domains) const
    {
        std::vector<const NetInfo *> todo{iob->getPort(id_PAD)};
        pool<IdString> seen;
        int domain = -1;
        while (!todo.empty()) {
            const NetInfo *net = todo.back();
            todo.pop_back();
            if (net == nullptr || net->driver.cell == nullptr || !seen.insert(net->driver.cell->name).second)
                continue;
            const CellInfo *ci = net->driver.cell;
            if (ci->type == id_SLG_IOB)
                continue;
            if (ci->type == id_SLG_FF) {
                auto it = domains.find(net_of(ci, id_CLK));
                domain = std::max(domain, it == domains.end() ? 0 : it->second);
                continue;
            }
            for (auto &[name, port] : ci->ports)
                if (port.type == PORT_IN)
                    todo.push_back(port.net);
        }
        return domain < 0 ? clock_domain("W0") : domain;
    }

    void emit_placement()
    {
        std::ofstream f(ctx->args.options.at("placement").as<std::string>());
        struct Clock
        {
            std::string net, token, side;
            int domain = 0;
        };
        std::map<std::string, Clock> clocks;
        for (auto &[name, ci] : ctx->cells)
            if (ci->type == id_SLG_CLKIN && ci->bel != BelId()) {
                Clock c;
                c.net = net_of(ci.get(), id_PAD);
                c.token = text(ci.get(), id_CLK_TOKEN);
                c.side = text(ci.get(), id_CLK_SIDE);
                c.domain = int(param(ci.get(), id_CLK_DOMAIN));
                clocks[c.net] = c;
            }

        f << "{\n\"luts\": [";
        bool first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_LUT || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"x\":" << loc.x
              << ",\"y\":" << loc.y << ",\"slice\":" << loc.z << ",\"init\":" << (uint64_t)param(ci.get(), id_INIT);
            // the lower half's LUT, on the same pins (I5 tied to 1 picks this one)
            if (const CellInfo *lower = bound_at(loc, LUTL_Z + loc.z))
                f << ",\"lower\":\"" << json_escape(lower->name.str(ctx))
                  << "\",\"init_lower\":" << (uint64_t)param(lower, id_INIT);
            f << ",\"pins\":[";
            first = false;
            static const IdString pins[] = {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
            for (int k = 0; k < 6; k++)
                f << (k ? "," : "") << in_pin(ci->bel, pins[k]);
            f << "]}";
        }

        f << "\n],\n\"ffs\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_FF || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            std::string clk_net = net_of(ci.get(), id_CLK);
            Clock clock = clocks.count(clk_net) ? clocks.at(clk_net) : Clock();
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"x\":" << loc.x
              << ",\"y\":" << loc.y << ",\"slice\":" << loc.z % 4 << ",\"slot\":\""
              << (loc.z >= OFF_Z ? "OXQ" : "XQ") << "\",\"ff_type\":\""
              << text(ci.get(), id_FF_TYPE) << "\",\"init\":" << param(ci.get(), id_INIT)
              << ",\"d_pin\":" << in_pin(ci->bel, id_D) << ",\"clk_pin\":" << in_pin(ci->bel, id_CLK)
              << ",\"ce_pin\":" << in_pin(ci->bel, id_CE) << ",\"sr_pin\":" << in_pin(ci->bel, id_R)
              << ",\"clk_domain\":" << clock.domain << ",\"clk_token\":\"" << clock.token << "\",\"clk_side\":\""
              << clock.side << "\",\"clk_net\":\"" << json_escape(clk_net) << "\",\"ce_net\":\""
              << json_escape(net_of(ci.get(), id_CE, "CE_CONST")) << "\",\"sr_net\":\""
              << json_escape(net_of(ci.get(), id_R, "R_CONST"))
              << "\",\"is_c_inverted\":" << param(ci.get(), id_IS_C_INVERTED)
              << ",\"is_r_inverted\":" << param(ci.get(), id_IS_R_INVERTED)
              << ",\"is_s_inverted\":" << param(ci.get(), id_IS_S_INVERTED)
              << ",\"is_d_inverted\":" << param(ci.get(), id_IS_D_INVERTED);
            // The one-hot select of what drives D. MUX_XQ (aq1_sel): the slice's
            // LUT (X_RAW, the vendor's default), the carry's sum (S_BIT) or XI;
            // slice D drops EXTRA_IN, so its bits sit one lower. MUX_OXQ
            // (oaq_sel) of a second flip-flop: XI, the first flip-flop's Q or the
            // lower half (OX0).
            WireId src = uphill_bound(ctx->getBelPinWire(ci->bel, id_D));
            IdString from = src != WireId() ? ctx->getWireType(src) : IdString();
            if (loc.z >= OFF_Z)
                f << ",\"oaq_sel\":" << (from == ctx->id("FFQ") ? 2 : from == id_LUTLO ? 4 : 1) << "}";
            else {
                int abc_bit = from == id_LUTO ? 5 : from == id_CYO ? 4 : 1;
                int field_bit = (loc.z % 4 == 3) ? abc_bit - 1 : abc_bit;
                f << ",\"aq1_sel\":" << (1 << field_bit) << "}";
            }
            first = false;
        }

        f << "\n],\n\"carries\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_CARRY || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            // What leaves by the slice's xMUX pin (MUX_X): the sum, the carry, or neither.
            auto out = [&](IdString port) {
                WireId w = downhill_bound(ctx->getBelPinWire(ci->bel, port));
                return w != WireId() && pin_of(w, "SPO_") >= 0;
            };
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"x\":" << loc.x
              << ",\"y\":" << loc.y << ",\"slice\":" << loc.z - CARRY_Z
              << ",\"ci\":" << param(ci.get(), id_CI_CONST, -1) << ",\"di\":" << param(ci.get(), id_DI_CONST, -1)
              << ",\"di_pin\":" << in_pin(ci->bel, id_DI) << ",\"xmux\":\""
              << (out(id_O) ? "S" : out(id_CO) ? "C" : "") << "\"}";
            first = false;
        }

        f << "\n],\n\"muxfs\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (!ci->type.in(id_SLG_MUXF7, id_SLG_MUXF8) || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            WireId out = downhill_bound(ctx->getBelPinWire(ci->bel, id_O));
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"x\":" << loc.x
              << ",\"y\":" << loc.y << ",\"slice\":" << MUXF_SLICE[loc.z - MUXF_Z]
              << ",\"sel_pin\":" << in_pin(ci->bel, id_S) << ",\"xmux\":" << (pin_of(out, "SPO_") >= 0) << "}";
            first = false;
        }

        f << "\n],\n\"clocks\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_CLKIN || ci->bel == BelId())
                continue;
            std::string port = name.str(ctx).substr(5);
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"port\":\""
              << json_escape(port) << "\",\"net\":\"" << json_escape(net_of(ci.get(), id_PAD)) << "\",\"token\":\""
              << text(ci.get(), id_CLK_TOKEN) << "\",\"side\":\"" << text(ci.get(), id_CLK_SIDE)
              << "\",\"domain\":" << param(ci.get(), id_CLK_DOMAIN) << "}";
            first = false;
        }

        f << "\n],\n\"held_controls\": [";
        first = true;
        for (auto &h : held_controls) {
            f << (first ? "\n" : ",\n") << "  {\"x\":" << h.x << ",\"y\":" << h.y << ",\"out\":" << h.out
              << ",\"value\":" << h.value << "}";
            first = false;
        }
        std::map<std::string, int> domains;
        for (auto &[net, c] : clocks)
            domains[net] = c.domain;
        f << "\n],\n\"iobs\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_IOB || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            auto port = ci->ports.find(id_PAD);
            // a pad of driven chip controls may drive only its second output
            // (OE), and one of what the chip states only the fabric (IN0, IN1)
            PortType type = port != ci->ports.end() ? port->second.type
                            : ci->ports.count(id_OE)  ? PORT_IN
                                                      : PORT_OUT;
            const char *dir = type == PORT_IN ? "output" : type == PORT_OUT ? "input" : "inout";
            WireId pad = ctx->getBelPinWire(ci->bel, id_PAD);
            WireId route = uphill_bound(uphill_bound(pad));
            WireId oe = uphill_bound(uphill_bound(ctx->getBelPinWire(ci->bel, id_OE)));
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"gx\":" << loc.x
              << ",\"gy\":" << loc.y << ",\"dir\":\"" << dir << "\",\"pad_in_pin\":" << in_pin(ci->bel, id_PAD)
              << ",\"pad_out_pin\":" << pin_of(downhill_bound(pad), "SPO_") << ",\"pad_in_sm\":\""
              << (route == WireId() ? "" : ctx->getWireName(route).str(ctx)) << "\",\"net\":\""
              << json_escape(net_of(ci.get(), id_PAD)) << "\",\"control\":" << int(ci->attrs.count(ctx->id("CONTROL")))
              << ",\"oe_sm\":\"" << (oe == WireId() ? "" : ctx->getWireName(oe).str(ctx)) << "\",\"oe_net\":\""
              << json_escape(net_of(ci.get(), id_OE)) << "\",\"domain\":"
              << (type == PORT_IN ? output_domain(ci.get(), domains) : 0) << ",\"status\":[";
            // the pad's inputs that carry what the chip states (status_names)
            bool any = false;
            for (int k = 0; k < 2; k++)
                if (ci->getPort(k ? id_IN1 : id_IN0) != nullptr)
                    f << (any ? "," : "") << k, any = true;
            f << "]}";
            first = false;
        }
        f << "\n]\n}\n";
    }

    void emit_routing()
    {
        std::ofstream f(ctx->args.options.at("routing").as<std::string>());
        f << "{\"nets\": [";
        bool first = true;
        for (auto &[name, net] : ctx->nets) {
            std::string pips;
            for (auto &w : net->wires) {
                PipId pip = w.second.pip;
                if (pip == PipId())
                    continue;
                pips += (pips.empty() ? "" : ",") + std::string("[\"") +
                        json_escape(ctx->getWireName(ctx->getPipSrcWire(pip)).str(ctx)) + "\",\"" +
                        json_escape(ctx->getWireName(ctx->getPipDstWire(pip)).str(ctx)) + "\",\"" +
                        IdString(chip_pip_info(ctx->chip_info, pip).type).str(ctx) + "\"]";
            }
            if (pips.empty())
                continue;
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"pips\":[" << pips
              << "]}";
            first = false;
        }
        f << "\n]}\n";
    }

    void postRoute() override
    {
        if (ctx->args.options.count("placement"))
            emit_placement();
        if (ctx->args.options.count("routing"))
            emit_routing();
    }

  private:
    HimbaechelHelpers h;
    pool<IdString> resting;
    int slices = 2;
    std::map<std::pair<int, int>, std::pair<int, int>> site_above;
    std::pair<int, int> column_foot;
};

struct SlgArch : HimbaechelArch
{
    SlgArch() : HimbaechelArch("slg47910"){};
    bool match_device(const std::string &device) override { return device == "slg47910"; }
    std::unique_ptr<HimbaechelAPI> create(const std::string &device) override { return std::make_unique<SlgImpl>(); }
} slgArch;

} // namespace

NEXTPNR_NAMESPACE_END
