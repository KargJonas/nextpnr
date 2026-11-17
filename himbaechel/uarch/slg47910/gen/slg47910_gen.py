#!/usr/bin/env python3
"""The SLG47910's chipdb for nextpnr-himbaechel, from its Project Inkwell database.

    slg47910_gen.py --db <forgefpga checkout> --bba chipdb-slg47910.bba

The database's slg47910/graph.json is the vendor's routing graph: 364 sites,
48,544 routing nodes (each at a grid position, each named there), 310,016 pips
between nodes. It is a flat graph, so the chipdb is a thin layer over it:

  * one tile type per grid position (32 x 32), holding the nodes that sit
    there under their own names, so a node's name in the chipdb is its name in
    the graph (`U0B0N0` at (3,2) is X3/Y2/U0B0N0);
  * a pip lives in the tile of the wire it drives; its source, where it sits
    in another tile, is an alias there (`@<x>_<y>_<name>`) that a himbaechel
    node merges with the wire it stands for. The owner is always the node's
    first wire, so every router and every report names the owner;
  * the logic of an L0 site (4 slices: a LUT6, two flip-flops, a bit of the
    carry chain) and an L6 site (an IOB) sit in their tile as bels, their
    inputs fed from the site's pin nodes by pips; the 40 L0 sites that have an
    L1 memory view are logic sites too (BEL_FLAG_MEMORY), without it;
  * each slice's LUT6 as two LUTs: the upper half, and the lower half out by
    the slice's xMUX pin;
  * the site's wide muxes: MUXF7 (A or B, C or D) and MUXF8 (of the two);
  * the carry chain runs from slice A to D of a site and on, by a dedicated
    pip, to slice A of the next site up its column (the vendor's CIN/COUT);
  * the dedicated clock inputs (W0 the oscillator, W1 the PLL, N0, N1, and
    E0, E1 that take back a clock the fabric makes) are bels at (0, 0) whose
    pad wire reaches every L0 site's CLK through one GCLK pip.
  * a pip into a line an output pad's enable rests on (graph.json's `rest`)
    has the type `REST:x,y;...`, the pads whose it is.

A wire's type says what the emitter needs of it: `SPI_<k>` and `SPO_<k>` are
the site pins' k-th input and output, `SM` any other node.
"""

import argparse
from collections import defaultdict
import json
import os
import re
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
from himbaechel_dbgen.chip import BEL_FLAG_GLOBAL, Chip, ClockEdge, NodeWire, PinType, TimingValue


# A bel of a memory site's logic (slg47910.cc reads it).
BEL_FLAG_MEMORY = 0x100


def cells(chip, grade, arcs):
    """The bels' arcs under one grade, as timing.json fits them to the units'."""
    value = lambda pair: TimingValue(pair[0], pair[1])
    lut = chip.timing.add_cell_variant(grade, "SLG_LUT")
    for pin, delay in arcs["LUT"].items():
        lut.add_comb_arc(pin, "O", value(delay))
    # the lower half, out by the xMUX pin: taken as the LUT's (no unit's arc
    # has been fitted to it)
    lower = chip.timing.add_cell_variant(grade, "SLG_LUTL")
    for pin, delay in arcs["LUT"].items():
        if pin != "I5":
            lower.add_comb_arc(pin, "O", value(delay))
    # A LUT in a memory site, as the vendor times it after routing: the uarch
    # takes these for a LUT placed there (BEL_FLAG_MEMORY).
    lut_m = chip.timing.add_cell_variant(grade, "SLG_LUT_M")
    lower_m = chip.timing.add_cell_variant(grade, "SLG_LUTL_M")
    for pin, delay in arcs["LUT_M"].items():
        lut_m.add_comb_arc(pin, "O", value(delay))
        if pin != "I5":
            lower_m.add_comb_arc(pin, "O", value(delay))
    carry = chip.timing.add_cell_variant(grade, "SLG_CARRY")
    c = arcs["CARRY"]
    carry.add_comb_arc("CI", "CO", value(c["CI_CO"]))
    carry.add_comb_arc("P", "CO", value(c["P_CO"]))
    carry.add_comb_arc("DI", "CO", value(c["P_CO"]))
    carry.add_comb_arc("CI", "O", value(c["CI_O"]))
    carry.add_comb_arc("P", "O", value(c["CI_O"]))
    ff = chip.timing.add_cell_variant(grade, "SLG_FF")
    f = arcs["FF"]
    for pin in ("D", "CE", "R"):
        ff.add_setup_hold("CLK", pin, ClockEdge.RISING, value(f["SETUP"]), TimingValue(0))
    ff.add_clock_out("CLK", "Q", ClockEdge.RISING, value(f["CLK_Q"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", required=True, help="Project Inkwell forgefpga checkout")
    ap.add_argument("--bba", required=True, help="bba file to write")
    args = ap.parse_args()

    db = json.load(open(os.path.join(args.db, "slg47910", "graph.json")))
    width, height = db["grid"]["width"], db["grid"]["height"]
    wires = {w["id"]: w for w in db["wires"]}
    sites = [s for s in db["sites"] if s["type"] in ("L0", "L6")]

    # A site pin node's type: the pin it is, by its name (I<k> in, O<k> out).
    pin_type = {}
    for s in sites:
        for p in s["pins"]:
            pin_type[p["node"]] = ("SPI_" if p["dir"] == "I" else "SPO_") + p["name"][1:]

    # The pins an enable or a reset arrives by: I5 of each slice, which share
    # four feeders (every one of 14,000 routed enables and resets of the
    # vendor's builds comes in by them, none by the I4 pins).
    spare = {str(6 * sl + 5) for sl in range(4)}
    chip = Chip("slg47910", "slg47910", width, height)
    chip.strs.read_constids(os.path.join(os.path.dirname(__file__), "..", "constids.inc"))
    by_tile = {(x, y): chip.create_tile_type(f"T_{x}_{y}") for x in range(width) for y in range(height)}
    aliases = {}  # node id -> [(x, y, alias name)]
    clock_pads = {}

    def type_of(nid):
        return pin_type.get(nid) or "SM"

    def local(x, y, nid):
        """The name of node `nid` in tile (x, y): its own, or an alias there."""
        w = wires[nid]
        if (w["x"], w["y"]) == (x, y):
            return w["name"]
        name = f"@{w['x']}_{w['y']}_{w['name']}"
        tt = by_tile[(x, y)]
        if not tt.has_wire(name):
            tt.create_wire(name, type_of(nid))
            aliases.setdefault(nid, []).append((x, y, name))
        return name

    # The fabric's timing as the vendor states it, read by Kraken and checked
    # against the vendor's routes (timing.json, from Kraken's export by
    # tools/kraken_timing.py): per pip a class, its delay and the wire's it
    # drives together, per speed grade; the timed grade first, the default.
    timing = json.load(open(os.path.join(args.db, "slg47910", "timing.json")))
    grades = timing["grades"]
    chip.set_speed_grades(grades)
    pip_class = timing["pips"]

    # The way out of a logic site: from an output pin to the first wire a
    # net takes, its delay as the vendor's routes show it (Kraken's rule #34,
    # `out:<layer>/<type>/o<port>><wire>`, timing.json's `outputs`), the
    # pin's own and the wire's together. The site's outputs are O26 on, port 0
    # first.
    outputs = timing["outputs"]
    out_port = {p["node"]: int(p["name"][1:]) - 26 for s in sites if s["type"] == "L0" for p in s["pins"] if p["dir"] == "O"}

    def pip(tt, src, dst, kind, stated=None):
        """A pip; `stated` the key of its delay in timing.json -- a pip's,
        or a way out of a site's -- where one is stated: the others -- into
        sites, and inside them -- take none, the cells' arcs holding the way
        to a site's pins."""
        made = tt.create_pip(src, dst, pip_class.get(stated) or (stated if stated in outputs else "T0"))
        made.pip_type = chip.strs.id(kind)

    for w in db["wires"]:
        by_tile[(w["x"], w["y"])].create_wire(w["name"], type_of(w["id"]))

    # The graph's own pips, in the tile of the wire each one drives, each
    # timed as the vendor states (timing.json): its delay is the
    # connection's and the resource it drives together.
    # Only the kinds of hop (which mux, fed from which, how far away) the
    # vendor's router takes in its own builds (hops.json): it never takes four
    # in ten of the graph's mux-to-mux edges, and on silicon a counter routed
    # over them showed one net on another's pad.
    hops = {(tuple(k), tuple(s), ox, oy) for k, s, ox, oy, _ in json.load(open(os.path.join(args.db, "slg47910", "hops.json")))}

    def key(nid):
        m = re.fullmatch(r"([UD])(\d+)B(\d+)N(\d+)", wires[nid]["name"])
        return (int(m[2]), m[1], int(m[3]), int(m[4])) if m else None

    # The pips into the lines an output pad's enable rests on (graph.json's
    # `rest`) say whose they are, `REST:x,y;...`: no net takes them while
    # that pad is an output (the uarch's checkPipAvailForNet).
    rests = defaultdict(list)
    for s in sites:
        for nid in s.get("rest", []):
            rests[nid].append(f"{s['x']},{s['y']}")

    seen = set()
    for p in db["pips"]:
        if (p["src"], p["dst"]) in seen:
            continue
        a, b = wires[p["src"]], wires[p["dst"]]
        if key(p["src"]) and key(p["dst"]) and (key(p["dst"]), key(p["src"]), b["x"] - a["x"], b["y"] - a["y"]) not in hops:
            continue
        seen.add((p["src"], p["dst"]))
        d = wires[p["dst"]]
        stated = f"out:L/RBB_6L/o{out_port[p['src']]}>{d['name']}" if p["src"] in out_port else f"{d['x']},{d['y']},{a['name']},{d['name']}"
        kind = "REST:" + ";".join(rests[p["dst"]]) if p["dst"] in rests else "PIP"
        pip(by_tile[(d["x"], d["y"])], local(d["x"], d["y"], p["src"]), d["name"], kind, stated)

    # The dedicated clock inputs, at (0, 0): the pad wire of each is a node
    # that every L0 site's CLK is one pip from. E0 and E1 take back what the
    # fabric sends out at pad (31,11) (logic as clock, app note AN-FG-019).
    tt0 = by_tile[(0, 0)]
    for z, (token, _d) in enumerate([("W0", 0), ("N0", 0), ("N1", 1), ("W1", 0), ("E0", 1), ("E1", 1)]):
        wire = f"CLKPAD_{token}"
        tt0.create_wire(wire, "CLKPAD")
        bel = tt0.create_bel(f"CLKIN_{token}", "SLG_CLKIN", z=100 + z)
        bel.flags = BEL_FLAG_GLOBAL  # its net is the clock tree: no placer pulls cells to (0, 0)
        tt0.add_bel_pin(bel, "PAD", wire, PinType.OUTPUT)
        clock_pads[token] = wire

    # Every logic site, those of the memory rows too: a memory site used as
    # logic is a logic site (its own way into its I4 pins is the bitstream
    # tools').
    site_at = {(s["sx"], s["sy"]): (s["x"], s["y"]) for s in sites if s["type"] == "L0"}
    memory = {(s["x"], s["y"]) for s in db["sites"] if s["type"] == "L1"}
    for s in sites:
        x, y = s["x"], s["y"]
        tt = by_tile[(x, y)]
        pins = {p["name"]: p for p in s["pins"]}
        ins = [p["node"] for p in s["pins"] if p["dir"] == "I"]
        if s["type"] == "L6":
            tt.create_wire("PAD", "PAD")
            bel = tt.create_bel("IOB", "SLG_IOB", z=0)
            tt.add_bel_pin(bel, "PAD", "PAD", PinType.INPUT)
            # A pad's second output (the vendor's Output=1): a GPIO's enable,
            # or the second of the chip controls a pad carries (the PLL's).
            tt.create_wire("OE", "OE")
            tt.add_bel_pin(bel, "OE", "OE", PinType.INPUT)
            # What the chip states at a pad of its own (PLL_LOCK, OSC_READY),
            # by the pad's input it names: the vendor's Input=0 leaves by O4,
            # Input=1 by O5 (its routes of them, AN-FG-019's pads). What a
            # GPIO reads is its Input=0, IN0 (by O5 it reads 0 on silicon).
            for k in range(2):
                tt.create_wire(f"IN{k}", "IN")
                tt.add_bel_pin(bel, f"IN{k}", f"IN{k}", PinType.OUTPUT)
            for p in s["pins"]:
                w = local(x, y, p["node"])
                if p["dir"] == "O":
                    pip(tt, f"IN{int(p['name'][1:]) - 4}", w, "IOPAD")
                else:
                    pip(tt, w, "PAD", "IOPAD")
                    pip(tt, w, "OE", "IOPAD")
            continue

        # L0: the clock, enable and reset every flip-flop of the site shares.
        for name in ("CLK", "CE", "SR"):
            tt.create_wire(name, name)
        # Not from a LUT's own four pins (I0-I3 of a slice share the four
        # nodes of its pool): an enable or a reset taking one would leave the
        # slice a LUT input short.
        for nid in ins:
            if pin_type[nid][4:] in spare:
                w = local(x, y, nid)
                pip(tt, w, "CE", "MUX")
                pip(tt, w, "SR", "MUX")
        # The carry into slice A: the carry out of the site below in the column.
        tt.create_wire("CIN", "CIN")
        below = site_at.get((s["sx"], s["sy"] - 1))
        if below is not None:
            alias = f"@{below[0]}_{below[1]}_COUT3"
            tt.create_wire(alias, "COUT")
            aliases.setdefault(("cout",) + below, []).append((x, y, alias))
            pip(tt, alias, "CIN", "CARRY")
        for sl in range(4):
            luto, ffd, ffq, xi = f"LUTO{sl}", f"FFD{sl}", f"FFQ{sl}", f"XI{sl}"
            cyo, cout = f"CYO{sl}", f"COUT{sl}"
            for w, kind in ((luto, "LUTO"), (ffd, "FFD"), (ffq, "FFQ"), (xi, "XI"), (cyo, "CYO"), (cout, "COUT")):
                tt.create_wire(w, kind)
            # XI: one of the site's I5 pins through MUX_xI, the flip-flop's D
            # from the fabric and the carry's generate input both.
            for j in range(4):
                pip(tt, local(x, y, pins[f"I{j * 6 + 5}"]["node"]), xi, "MUX")
            lut = tt.create_bel(f"LUT{sl}", "SLG_LUT", z=sl)
            if (x, y) in memory:
                lut.flags |= BEL_FLAG_MEMORY
            tt.add_bel_pin(lut, "O", luto, PinType.OUTPUT)
            for k in range(6):
                li = f"LUTI{sl}_{k}"
                tt.create_wire(li, "LUTI")
                tt.add_bel_pin(lut, f"I{k}", li, PinType.INPUT)
                # I0-I3 have the slice's own pins (A 0-3, B 6-9, ...). I4 and I5
                # take any of the site's four shared pins of their kind (4, 10,
                # 16, 22 and 5, 11, 17, 23) through MUX_x5 and MUX_x6.
                for j in ([sl] if k < 4 else range(4)):
                    pip(tt, local(x, y, pins[f"I{j * 6 + k}"]["node"]), li, "MUX")
            # a LUT leaves by its own slice's O pin only (arch_init.cc)
            pip(tt, luto, local(x, y, pins[f"O{26 + 3 * sl}"]["node"]), "OMUX")
            # The LUT6 is two LUT5 on I0-I4, I5 choosing: with I5 tied to 1
            # (x6_TIE 0) the LUT is the upper half, and the lower half a LUT of
            # its own, out by the slice's xMUX pin (MUX_X's OX0).
            lower = tt.create_bel(f"LUTL{sl}", "SLG_LUTL", z=20 + sl)
            if (x, y) in memory:
                lower.flags |= BEL_FLAG_MEMORY
            tt.create_wire(f"LUTLO{sl}", "LUTLO")
            tt.add_bel_pin(lower, "O", f"LUTLO{sl}", PinType.OUTPUT)
            for k in range(5):
                tt.add_bel_pin(lower, f"I{k}", f"LUTI{sl}_{k}", PinType.INPUT)
            pip(tt, f"LUTLO{sl}", local(x, y, pins[f"O{27 + 3 * sl}"]["node"]), "XMUX")
            ff = tt.create_bel(f"FF{sl}", "SLG_FF", z=4 + sl)
            tt.add_bel_pin(ff, "D", ffd, PinType.INPUT)
            tt.add_bel_pin(ff, "Q", ffq, PinType.OUTPUT)
            tt.add_bel_pin(ff, "CLK", "CLK", PinType.INPUT)
            tt.add_bel_pin(ff, "CE", "CE", PinType.INPUT)
            tt.add_bel_pin(ff, "R", "SR", PinType.INPUT)
            # A flip-flop takes its D from its slice's LUT, the carry's sum, or
            # XI (MUX_XQ).
            for w in (luto, cyo, xi):
                pip(tt, w, ffd, "FFDMUX")
            pip(tt, ffq, local(x, y, pins[f"O{28 + 3 * sl}"]["node"]), "OMUX")
            # A bit of the carry chain: P is the LUT's output, G is XI;
            # O = P ^ CI, CO = P ? CI : G. Its sum or its carry leaves the
            # site by the slice's xMUX pin (MUX_X).
            # The second flip-flop (OXQ): D from XI or the first flip-flop's Q
            # (MUX_OXQ), Q out by the slice's xMUX pin only (MUX_X).
            offd, offq = f"OFFD{sl}", f"OFFQ{sl}"
            tt.create_wire(offd, "OFFD")
            tt.create_wire(offq, "OFFQ")
            off = tt.create_bel(f"OFF{sl}", "SLG_FF", z=12 + sl)
            tt.add_bel_pin(off, "D", offd, PinType.INPUT)
            tt.add_bel_pin(off, "Q", offq, PinType.OUTPUT)
            for pin, wire in (("CLK", "CLK"), ("CE", "CE"), ("R", "SR")):
                tt.add_bel_pin(off, pin, wire, PinType.INPUT)
            # MUX_OXQ: XI, the first flip-flop's Q, or the lower half (OX0, the
            # vendor's OAQ_SEL 100: 329 of its 645 second flip-flops).
            for w in (xi, ffq, f"LUTLO{sl}"):
                pip(tt, w, offd, "FFDMUX")
            pip(tt, offq, local(x, y, pins[f"O{27 + 3 * sl}"]["node"]), "XMUX")
            carry = tt.create_bel(f"CARRY{sl}", "SLG_CARRY", z=8 + sl)
            tt.add_bel_pin(carry, "P", luto, PinType.INPUT)
            tt.add_bel_pin(carry, "DI", xi, PinType.INPUT)
            tt.add_bel_pin(carry, "CI", "CIN" if sl == 0 else f"COUT{sl - 1}", PinType.INPUT)
            tt.add_bel_pin(carry, "O", cyo, PinType.OUTPUT)
            tt.add_bel_pin(carry, "CO", cout, PinType.OUTPUT)
            for w in (cyo, cout):
                pip(tt, w, local(x, y, pins[f"O{27 + 3 * sl}"]["node"]), "XMUX")
        # The site's wide muxes (the vendor's MUXF, its EXTRA_IN): F7A = A or
        # B's LUT, F7C = C or D's, F8 = F7A or F7C, each selected by one
        # slice's XI (A, C, B) and leaving by that slice's xMUX pin (MUX_X).
        for name, kind, z, ins, sel, sl in (("F7A", "SLG_MUXF7", 16, ("LUTO0", "LUTO1"), "XI0", 0),
                                            ("F7C", "SLG_MUXF7", 17, ("LUTO2", "LUTO3"), "XI2", 2),
                                            ("F8", "SLG_MUXF8", 18, ("F7A", "F7C"), "XI1", 1)):
            tt.create_wire(name, "MUXF")
            mux = tt.create_bel(name, kind, z=z)
            tt.add_bel_pin(mux, "I0", ins[0], PinType.INPUT)
            tt.add_bel_pin(mux, "I1", ins[1], PinType.INPUT)
            tt.add_bel_pin(mux, "S", sel, PinType.INPUT)
            tt.add_bel_pin(mux, "O", name, PinType.OUTPUT)
            pip(tt, name, local(x, y, pins[f"O{27 + 3 * sl}"]["node"]), "XMUX")
        # the global clock: one pip from each pad's wire
        for token, pad in clock_pads.items():
            alias = f"@0_0_{pad}"
            tt.create_wire(alias, "CLKPAD")
            aliases.setdefault(("clk", token), []).append((x, y, alias))
            pip(tt, alias, "CLK", "GCLK")

    for (x, y) in by_tile:
        chip.set_tile_type(x, y, f"T_{x}_{y}")
    for i, grade in enumerate(grades):
        chip.timing.set_pip_class(grade, "T0", TimingValue(0))
        for name, by in timing["classes"].items():
            chip.timing.set_pip_class(grade, name, TimingValue(*by[i]))
        for name, by in outputs.items():
            chip.timing.set_pip_class(grade, name, TimingValue(*(by[i] or [0, 0])))
        cells(chip, grade, timing["cells"][grade])

    # Merge each node's aliases into it, its owner first.
    for nid, where in aliases.items():
        if isinstance(nid, tuple) and nid[0] == "clk":
            owner = NodeWire(0, 0, clock_pads[nid[1]])
        elif isinstance(nid, tuple):
            owner = NodeWire(nid[1], nid[2], "COUT3")
        else:
            w = wires[nid]
            owner = NodeWire(w["x"], w["y"], w["name"])
        chip.add_node([owner] + [NodeWire(x, y, name) for x, y, name in where])

    chip.write_bba(args.bba)


if __name__ == "__main__":
    main()
