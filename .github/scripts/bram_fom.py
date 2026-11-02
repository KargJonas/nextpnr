#!/usr/bin/env python3
"""A figure of merit for a placement's block RAMs: how late each one's read
data can reach the registers it feeds, from the placement alone.

For each block RAM, its read cone is followed from its data outputs through
LUTs, muxes and carries (up to 24 cells, nets of fanout 64 or less) to the
registers it ends in.  Its score is the worst, over those registers r, of

    route(b -> r) + skew(b, r)

with route the measured delay matrix's estimate for the grid offset (nextpnr's
-o delay-matrix file, extrapolated at its edge rates as nextpnr does), and
skew the clock tree's extra insertion delay at the block RAM over a flip-flop
at r (nextpnr's clock_tree.cc model: 170 ps per clock region row from the
BUFGs, 1.7 ps per column from the spine, 430 ps for a block RAM clock pin).
The mux logic between them is the same in every placement of a netlist, so
the score ranks placements; it is not a slack.

    bram_fom.py --netlist xilinx_vc707.json --tilegrid tilegrid.json \\
        --delay-matrix delay-matrix.txt --budget 3000 \\
        NAME=placement.json NAME=vivado:cell_sites.txt ...

A nextpnr placement is its -o placement=... JSON; a Vivado one is "cell site"
lines (get_cells ... SITE).  --mark names block RAMs to report the rank of
(ones known to fail, say).  --max-worst and --max-p95 make it a gate: exit 1
if any placement scores over them."""
import argparse, collections, json, os, re, statistics


def load_netlist(path):
    mods = json.load(open(path))["modules"]
    top = max(mods.values(), key=lambda m: len(m["cells"]))
    return top["cells"]


def read_cones(cells, depth=24, max_fanout=64):
    users = collections.defaultdict(list)
    for name, c in cells.items():
        for port, bits in c["connections"].items():
            if c.get("port_directions", {}).get(port) == "input":
                for b in bits:
                    if not isinstance(b, str):
                        users[b].append(name)

    def outputs(name, data_only):
        c = cells[name]
        for port, bits in c["connections"].items():
            if c["port_directions"].get(port) != "output":
                continue
            if data_only and not port.startswith("DO"):
                continue
            for b in bits:
                if not isinstance(b, str) and len(users[b]) <= max_fanout:
                    yield from users[b]

    is_comb = lambda t: t.startswith(("LUT", "MUXF", "CARRY"))
    is_reg = lambda t: t.startswith("FD")
    cones = {}
    for b, c in cells.items():
        if not c["type"].startswith("RAMB"):
            continue
        regs, seen, front = set(), set(), list(outputs(b, True))
        for d in range(1, depth + 1):
            nxt = []
            for u in front:
                if u in seen:
                    continue
                seen.add(u)
                t = cells[u]["type"]
                if is_reg(t):
                    regs.add(u)
                elif is_comb(t) and d < depth:
                    nxt += list(outputs(u, False))
            front = nxt
        cones[b] = regs
    return cones


class DelayMatrix:
    def __init__(self, path):
        vals = open(path).read().split()
        self.w = int(vals[0])
        side = 2 * self.w + 1
        self.m = [int(v) for v in vals[1:1 + side * side]]
        band = 4
        at = lambda x, y: self.m[(y + self.w) * side + (x + self.w)]
        sx = [(at(s * self.w, y) - at(s * (self.w - band), y)) / band for y in range(-self.w, self.w + 1) for s in (1, -1)]
        sy = [(at(x, s * self.w) - at(x, s * (self.w - band))) / band for x in range(-self.w, self.w + 1) for s in (1, -1)]
        self.rx, self.ry = max(0.0, statistics.mean(sx)), max(0.0, statistics.mean(sy))
        self.at = at

    def __call__(self, dx, dy):
        ex, ey = max(-self.w, min(self.w, dx)), max(-self.w, min(self.w, dy))
        return self.at(ex, ey) + (abs(dx) - abs(ex)) * self.rx + (abs(dy) - abs(ey)) * self.ry


class FormulaDelay:
    """nextpnr's fallback when no delay matrix is measured (predictDelay)"""

    def __call__(self, dx, dy):
        x, y = abs(dx), abs(dy)
        return 1.5 * (30 * min(x, 18) + 10 * max(x - 18, 0) + 60 * min(y, 6) + 20 * max(y - 6, 0) + 300)


class ClockTree:
    def __init__(self, tilegrid):
        rows, bufg, self.spine = set(), [], None
        for name, t in tilegrid.items():
            if t["type"].startswith("CLK_HROW_"):
                rows.add(t["grid_y"])
                self.spine = t["grid_x"]
            if t["type"].startswith(("CLK_BUFG_TOP", "CLK_BUFG_BOT")):
                bufg.append(t["grid_y"])
        self.rows, self.centre = sorted(rows), sum(bufg) / len(bufg)

    def __call__(self, x, y, bram):
        r = min(self.rows, key=lambda r: abs(r - y))
        off = r - self.centre
        away = sum(1 for q in self.rows if ((q - self.centre) > 0) == (off > 0) and abs(q - self.centre) < abs(off))
        return 170 * away + 1.7 * abs(x - self.spine) + (430 if bram else 0)


def load_placement(spec, site_xy):
    if spec.startswith("vivado:"):
        # Vivado's names keep the netlist's hierarchy and name registers
        # x_reg[i]; the flat netlist's are x_i
        out = {}
        for line in open(spec[len("vivado:"):]):
            cell, site = line.split()
            if site not in site_xy:
                continue
            flat = re.sub(r"_reg(\[(\d+)\])?$", lambda m: "_" + m[2] if m[2] else "", cell.replace("/", "_"))
            out[flat] = out[flat + "_0"] = site_xy[site]
        return out
    out = {}
    for cell, v in json.load(open(spec)).items():
        m = re.match(r"X(\d+)Y(\d+)/", v.get("nextpnr_bel", ""))
        if m:
            out[cell] = (int(m[1]), int(m[2]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--netlist", required=True)
    ap.add_argument("--tilegrid", required=True)
    ap.add_argument("--delay-matrix", required=True, help="nextpnr's -o delay-matrix file, or 'formula'")
    ap.add_argument("--budget", type=float, default=3000, help="score (ps) beyond which a block RAM counts as over")
    ap.add_argument("--mark", default="", help="comma-separated block RAMs to rank")
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--max-worst", type=float, help="fail (exit 1) if a placement's worst score is over this")
    ap.add_argument("--max-p95", type=float, help="fail (exit 1) if a placement's 95th percentile is over this")
    ap.add_argument("placements", nargs="+", help="NAME=placement.json or NAME=vivado:cell_sites.txt")
    a = ap.parse_args()

    cells = load_netlist(a.netlist)
    cones = read_cones(cells)
    tg = json.load(open(a.tilegrid))
    site_xy = {s: (t["grid_x"], t["grid_y"]) for t in tg.values() for s in t.get("sites", {})}
    # "formula" (or no file): nextpnr's fallback formula, whose scores are not
    # comparable with a measured matrix's
    use_formula = a.delay_matrix == "formula" or not os.path.isfile(a.delay_matrix)
    dm, ct = (FormulaDelay() if use_formula else DelayMatrix(a.delay_matrix)), ClockTree(tg)
    if use_formula:
        print("delay model: nextpnr's formula (no delay matrix file)")
    marks = [m for m in a.mark.split(",") if m]

    failed = []
    gate = a.max_worst is not None or a.max_p95 is not None
    print("%-22s %6s %6s %6s %6s %8s  %s" % ("placement", "worst", "p95", "p50", "over", "unplaced", "marked ranks"))
    for spec in a.placements:
        name, path = spec.split("=", 1)
        pl = load_placement(path, site_xy)
        if not cones:  # a design without block RAMs has nothing to score
            print("%-22s no block RAMs" % name)
            continue
        scores, missing_brams, missing_regs = {}, 0, 0
        for b, regs in cones.items():
            if b not in pl:
                missing_brams += 1
                continue
            bx, by = pl[b]
            worst = None
            for r in regs:
                if r not in pl:
                    missing_regs += 1
                    continue
                rx, ry = pl[r]
                s = dm(rx - bx, ry - by) + ct(bx, by, True) - ct(rx, ry, False)
                worst = s if worst is None else max(worst, s)
            if worst is not None:
                scores[b] = worst
        missing = missing_brams + missing_regs
        if not scores:
            # Block RAMs in the netlist but nothing scored: the placement does
            # not name them, so this is a mismatch, not a clean design.
            print("%-22s %d block RAMs, none scored" % (name, len(cones)))
            failed.append("%s: %d block RAMs and no scores; the placement does not name the netlist's cells"
                          % (name, len(cones)))
            continue
        if gate and missing:
            # A path the placement leaves out cannot be scored, and the worst
            # path is the test, so an omission is a failure rather than a gap.
            failed.append("%s: the placement omits %d block RAM(s) and %d register(s) of their read cones"
                          % (name, missing_brams, missing_regs))
        ranked = sorted(scores, key=scores.get, reverse=True)
        v = sorted(scores.values())
        p = lambda q: v[min(len(v) - 1, int(q * len(v)))]
        over = sum(1 for s in v if s > a.budget)
        ranks = " ".join("%s:#%d" % (m.replace("rom_inst_", ""), ranked.index(m) + 1) for m in marks if m in scores)
        print("%-22s %6.0f %6.0f %6.0f %6d %8d  %s" % (name, v[-1], p(0.95), p(0.5), over, missing, ranks))
        for b in ranked[:a.top]:
            print("    %-24s %6.0f  at X%dY%d" % (b, scores[b], *pl[b]))
        if a.max_worst is not None and v[-1] > a.max_worst:
            failed.append("%s: worst %.0f ps is over %.0f" % (name, v[-1], a.max_worst))
        if a.max_p95 is not None and p(0.95) > a.max_p95:
            failed.append("%s: p95 %.0f ps is over %.0f" % (name, p(0.95), a.max_p95))
    for f in failed:
        print("FAIL " + f)
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
