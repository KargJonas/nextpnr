/*
 * nextpnr-himbaechel uarch for the Renesas SLG47910 (ForgeFPGA).
 *
 * The chipdb (slg47910_gen.py) is the vendor's routing graph with the 140 L0
 * logic sites (four slices of a LUT6 and the flip-flop on its output), the
 * 184 L6 pads and the three dedicated clock inputs as bels. This file is what
 * nextpnr does with it:
 *
 *   pack       pads and clock inputs from the ports and a pin file, the
 *              synthesiser's LUTs and flip-flops as SLG_LUT and SLG_FF, a LUT
 *              with its single flip-flop as one cluster, constants as LUTs
 *   validity   the flip-flops of a site share one enable and one reset
 *   postRoute  what the bitstream tools take: where every cell sits and what
 *              each net's route is, as JSON (placement.json, routing.json),
 *              in the names of the chipdb, which the bitstream tools
 *              (hb_emit.py) turn into the graph's node ids and the sites'
 *              coordinates
 *
 * Not yet: the carry chain, the memory sites and the second flip-flop of a slice.
 */

#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

#include "himbaechel_api.h"
#include "himbaechel_helpers.h"
#include "log.h"
#include "nextpnr.h"
#include "util.h"

#define GEN_INIT_CONSTIDS
#define HIMBAECHEL_CONSTIDS "uarch/slg47910/constids.inc"
#define HIMBAECHEL_UARCH slg47910
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

constexpr int GRID = 32;
constexpr int FF_Z = 4;      // a slice's flip-flop sits 4 above its LUT
constexpr int CLKIN_Z = 100; // W0, N0, N1

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

struct ClockToken
{
    int z, domain;
    const char *side;
};
const std::map<std::string, ClockToken> &clock_tokens()
{
    static const std::map<std::string, ClockToken> t = {{"W0", {0, 0, "W"}}, {"N0", {1, 0, "N"}}, {"N1", {2, 1, "N"}}};
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
                               "pin file: `set_io <port> GPIO<n>|<row> <col>`, `set_clk <port> W0|N0|N1`");
        specific.add_options()("slices", po::value<int>()->default_value(2),
                               "slices (LUT and flip-flop pairs) a placer may use in one site");
        specific.add_options()("placement", po::value<std::string>(), "where to write the placement, as JSON");
        specific.add_options()("routing", po::value<std::string>(), "where to write the routing, as JSON");
        return specific;
    }

    void init_database(Arch *arch) override
    {
        init_uarch_constids(arch);
        arch->load_chipdb("slg47910/chipdb-slg47910.bin");
        arch->set_speed_grade("DEFAULT");
    }

    void init(Context *ctx) override
    {
        h.init(ctx);
        HimbaechelAPI::init(ctx);
        slices = ctx->args.options.count("slices") ? ctx->args.options.at("slices").as<int>() : 2;
    }

    std::string getDefaultRouter() const override { return "router2"; }

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
        std::set<std::pair<int, int>> taken;
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
                if (!clock_tokens().count(where))
                    log_error("slg47910: %s: no clock input %s (W0, N0 or N1)\n", at.c_str(), where.c_str());
                clock_ports[port] = where;
            } else if (keyword == "set_io") {
                int row, col;
                auto gpio = gpio_names().find(where);
                if (gpio != gpio_names().end())
                    row = gpio->second.first, col = gpio->second.second;
                else {
                    char *end = nullptr;
                    row = int(strtol(where.c_str(), &end, 10));
                    if (end == where.c_str() || *end != 0 || !(in >> col))
                        log_error("slg47910: %s: %s is no GPIO name, nor a row and a column\n", at.c_str(),
                                  where.c_str());
                }
                if (!taken.insert({row, col}).second)
                    log_error("slg47910: %s: pad (%d,%d) taken twice\n", at.c_str(), row, col);
                io_bels[port] = iob_at(row, col, at);
            } else
                log_error("slg47910: %s: expected set_io or set_clk\n", at.c_str());
        }
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
        for (auto &[name, port] : ctx->ports) {
            std::string pname = name.str(ctx);
            if (!port.net)
                continue;
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
                cell->params[id_CLK_DOMAIN] = Property(clock_tokens().at(token).domain);
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
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
        for (auto &[port, bel] : io_bels)
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

    // A flip-flop takes D from the LUT of its slice, by the dedicated path: it
    // goes in its LUT's cluster, and where no LUT of its own drives it (another
    // flip-flop, a constant, a net several flip-flops take) a buffer LUT is its own.
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
            if (lut == nullptr || lut->type != id_SLG_LUT || d->driver.port != id_O || lut->cluster != ClusterId()) {
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
        log_info("slg47910: flip-flops share a slice with their LUT, %zu with a buffer of their own\n", made.size());
    }

    // Every output pad is driven by a LUT of its own. On silicon a pad shows
    // the complement of what a flip-flop sends it, and a LUT's output arrives
    // complemented wherever it leaves its site, so through a LUT a pad reads
    // true (the bitstream tools take the complement back at a LUT's input). A
    // LUT that drives nothing but the pad may also take a level-13 line, which
    // inverts: the bitstream tools invert that LUT's table for it.
    static bool pad_only_lut(const NetInfo *net)
    {
        return net->driver.cell != nullptr && net->driver.cell->type == id_SLG_LUT && net->users.entries() == 1 &&
               (*net->users.begin()).cell->type == id_SLG_IOB;
    }

    void buffer_pads()
    {
        std::vector<std::unique_ptr<CellInfo>> made;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_IOB || !ci->ports.count(id_PAD) || ci->ports.at(id_PAD).type != PORT_IN)
                continue;
            NetInfo *net = ci->getPort(id_PAD);
            if (net == nullptr || net->driver.cell == nullptr || pad_only_lut(net))
                continue;
            auto buf = std::make_unique<CellInfo>(ctx, ctx->idf("%s$buf", name.c_str(ctx)), id_SLG_LUT);
            buf->params[id_INIT] = Property(0x2ull, 64);
            buf->params[id_LUT_MODE] = Property("LUT5");
            NetInfo *out = ctx->createNet(ctx->idf("%s$buf_o", name.c_str(ctx)));
            buf->addInput(id_I0);
            buf->addOutput(id_O);
            ci->disconnectPort(id_PAD);
            buf->connectPort(id_I0, net);
            buf->connectPort(id_O, out);
            ci->connectPort(id_PAD, out);
            made.push_back(std::move(buf));
        }
        for (auto &cell : made)
            ctx->cells[cell->name] = std::move(cell);
    }

    void pack() override
    {
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type == id_LUT5 || ci->type == id_LUT6)
                log_error("slg47910: %s is a %s: a LUT's fifth and sixth inputs are not routable yet; "
                          "map to four-input LUTs (abc -lut 4)\n",
                          name.c_str(ctx), ci->type.c_str(ctx));
            std::string type = ci->type.str(ctx);
            if (type == "CARRY4" || type.rfind("MUXF", 0) == 0 || type.rfind("RAM", 0) == 0 ||
                type.rfind("SRL", 0) == 0 || type.rfind("RAMB", 0) == 0 || type.rfind("PLL", 0) == 0 ||
                type.rfind("MMCM", 0) == 0)
                log_error("slg47910: %s (%s) is not supported yet: the chipdb has neither the carry chain, MUXF, "
                          "memory nor a PLL (synthesise with -nobram -nodsp -nolutram, and no carry)\n",
                          name.c_str(ctx), type.c_str());
        }
        read_pins();
        pack_io();
        pack_luts();
        pack_ffs();
        pair_ffs();
        buffer_pads();
        h.replace_constants(CellTypePort(id_SLG_LUT, id_O), CellTypePort(id_SLG_LUT, id_O),
                            {{id_INIT, Property(0xFFFFFFFFFFFFFFFFull, 64)}}, {{id_INIT, Property(0x0ull, 64)}});
    }

    // ---- validity ----

    bool isValidBelForCellType(IdString cell_type, BelId bel) const override
    {
        return ctx->getBelType(bel) == cell_type;
    }
    IdString getBelBucketForCellType(IdString cell_type) const override { return cell_type; }

    static bool set_type(const Context *ctx, const CellInfo *ci)
    {
        auto it = ci->params.find(id_FF_TYPE);
        return it != ci->params.end() && (it->second.as_string() == "FDSE" || it->second.as_string() == "FDPE");
    }

    bool isBelLocationValid(BelId bel, bool explain_invalid) const override
    {
        const CellInfo *bound = ctx->getBoundBelCell(bel);
        if (bound == nullptr)
            return true;
        if (bound->type == id_SLG_LUT || bound->type == id_SLG_FF) {
            Loc at = ctx->getBelLocation(bel);
            // The four nodes of a slice's input pool each feed several slices of
            // the sites around: a site with every slice full leaves its neighbours
            // none to route through, and no router can untangle what that costs.
            int busy = 0;
            for (int sl = 0; sl < 4; sl++)
                busy += ctx->getBoundBelCell(ctx->getBelByLocation(Loc(at.x, at.y, sl))) != nullptr ||
                        ctx->getBoundBelCell(ctx->getBelByLocation(Loc(at.x, at.y, FF_Z + sl))) != nullptr;
            if (busy > slices) {
                if (explain_invalid)
                    log_warning("slg47910: more than %d slices in use at (%d,%d)\n", slices, at.x, at.y);
                return false;
            }
        }
        if (bound->type != id_SLG_FF)
            return true;
        Loc loc = ctx->getBelLocation(bel);
        const NetInfo *my_ce = bound->getPort(id_CE);
        const NetInfo *my_r = bound->getPort(id_R);
        for (int sl = 0; sl < 4; sl++) {
            if (FF_Z + sl == loc.z)
                continue;
            const CellInfo *other = ctx->getBoundBelCell(ctx->getBelByLocation(Loc(loc.x, loc.y, FF_Z + sl)));
            if (other == nullptr)
                continue;
            // one enable and one set/reset per site, of one polarity (the XOR is the site's)
            if (other->getPort(id_R) != my_r) {
                if (explain_invalid)
                    log_warning("slg47910: %s and %s need different resets in one site\n", ctx->nameOf(bound),
                                ctx->nameOf(other));
                return false;
            }
            if (other->getPort(id_CE) != my_ce) {
                if (explain_invalid)
                    log_warning("slg47910: %s and %s need different enables in one site\n", ctx->nameOf(bound),
                                ctx->nameOf(other));
                return false;
            }
            if (my_r && other->getPort(id_R) && set_type(ctx, other) != set_type(ctx, bound)) {
                if (explain_invalid)
                    log_warning("slg47910: %s and %s mix set and reset in one site\n", ctx->nameOf(bound),
                                ctx->nameOf(other));
                return false;
            }
        }
        return true;
    }

    // A pad is only the source of the net its input cell drives and the
    // sink of the net its output cell takes: not a way through the pad.
    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override
    {
        WireId src = ctx->getPipSrcWire(pip), dst = ctx->getPipDstWire(pip);
        // A hop up a level-13 line inverts the signal. A LUT's INIT can take
        // that back (the bitstream tools flip the input, or the output of a LUT
        // that drives only a pad), nothing else can.
        if (ctx->getWireType(dst) == id_L13) {
            if (net == nullptr)
                return false;
            if (pad_only_lut(net))
                return true;
            for (auto &usr : net->users)
                if (usr.cell->type != id_SLG_LUT)
                    return false;
        }
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
    int in_pin(BelId bel, IdString port) const { return pin_of(uphill_bound(ctx->getBelPinWire(bel, port)), "SPI_"); }
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
              << ",\"y\":" << loc.y << ",\"slice\":" << loc.z << ",\"init\":" << (uint64_t)param(ci.get(), id_INIT)
              << ",\"pins\":[";
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
              << ",\"y\":" << loc.y << ",\"slice\":" << loc.z - FF_Z << ",\"slot\":\"XQ\",\"ff_type\":\""
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
            // MUX_XQ's one-hot select, by what drives D: the slice's LUT (X_RAW,
            // the vendor's default) or a site input (XI). Slice D drops EXTRA_IN, so its bits sit one lower.
            WireId src = uphill_bound(ctx->getBelPinWire(ci->bel, id_D));
            int abc_bit = (src != WireId() && ctx->getWireType(src) == ctx->id("LUTO")) ? 5 : 1;
            int field_bit = (loc.z - FF_Z == 3) ? abc_bit - 1 : abc_bit;
            f << ",\"aq1_sel\":" << (1 << field_bit) << "}";
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

        f << "\n],\n\"iobs\": [";
        first = true;
        for (auto &[name, ci] : ctx->cells) {
            if (ci->type != id_SLG_IOB || ci->bel == BelId())
                continue;
            Loc loc = ctx->getBelLocation(ci->bel);
            auto port = ci->ports.find(id_PAD);
            PortType type = port != ci->ports.end() ? port->second.type : PORT_INOUT;
            const char *dir = type == PORT_IN ? "output" : type == PORT_OUT ? "input" : "inout";
            WireId pad = ctx->getBelPinWire(ci->bel, id_PAD);
            WireId route = uphill_bound(uphill_bound(pad));
            f << (first ? "\n" : ",\n") << "  {\"name\":\"" << json_escape(name.str(ctx)) << "\",\"gx\":" << loc.x
              << ",\"gy\":" << loc.y << ",\"dir\":\"" << dir << "\",\"pad_in_pin\":" << in_pin(ci->bel, id_PAD)
              << ",\"pad_out_pin\":" << pin_of(downhill_bound(pad), "SPO_") << ",\"pad_in_sm\":\""
              << (route == WireId() ? "" : ctx->getWireName(route).str(ctx)) << "\",\"net\":\""
              << json_escape(net_of(ci.get(), id_PAD)) << "\"}";
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
    int slices = 2;
};

struct SlgArch : HimbaechelArch
{
    SlgArch() : HimbaechelArch("slg47910"){};
    bool match_device(const std::string &device) override { return device == "slg47910"; }
    std::unique_ptr<HimbaechelAPI> create(const std::string &device) override { return std::make_unique<SlgImpl>(); }
} slgArch;

} // namespace

NEXTPNR_NAMESPACE_END
