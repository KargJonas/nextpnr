#!/usr/bin/env python3
"""The SLG47910's chipdb for nextpnr-himbaechel, from its Project Inkwell database.

    slg47910_gen.py --db <forgefpga checkout> --bba chipdb-slg47910.bba

The database's slg47910/graph.json is the vendor's routing graph: 364 sites,
48,544 routing nodes (each at a grid position, each named there), 243,456 pips
between nodes. It is a flat graph, so the chipdb is a thin layer over it:

  * one tile type per grid position (32 x 32), holding the nodes that sit
    there under their own names, so a node's name in the chipdb is its name in
    the graph (`U0B0N0` at (3,2) is X3/Y2/U0B0N0);
  * a pip lives in the tile of the wire it drives; its source, where it sits
    in another tile, is an alias there (`@<x>_<y>_<name>`) that a himbaechel
    node merges with the wire it stands for. The owner is always the node's
    first wire, so every router and every report names the owner;
  * the logic of an L0 site (4 slices: a LUT6, a flip-flop on the LUT's own
    output) and an L6 site (an IOB) sit in their tile as bels, their inputs
    fed from the site's pin nodes by pips, without the L1 memory views, the
    second flip-flop, the MUXF and the carry chain;
  * the three dedicated clock inputs (W0, N0, N1) are bels at (0, 0) whose
    pad wire reaches every L0 site's CLK through one GCLK pip.

A wire's type says what the emitter needs of it: `SPI_<k>` and `SPO_<k>` are
the site pins' k-th input and output, `SM` any other node.
"""

import argparse
import json
import os
import re
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
from himbaechel_dbgen.chip import BEL_FLAG_GLOBAL, Chip, NodeWire, PinType, TimingValue


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
        return pin_type.get(nid) or ("L13" if wires[nid]["name"].startswith("U13") else "SM")

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

    chip.set_speed_grades(["DEFAULT"])
    classes = set()

    def pip(tt, src, dst, kind, cost=0):
        # The graph's costs read as femtoseconds (median 2.2e6, a plausible
        # 2.2 ns): a pip's delay is its cost in ps, to the nearest 100, and at
        # least 100: congestion is priced by delay, so a wire that costs nothing
        # is one a router never moves a net off.
        bucket = max(1, round(cost / 100000))
        name = f"P{bucket}"
        classes.add(bucket)
        made = tt.create_pip(src, dst, name)
        made.pip_type = chip.strs.id(kind)

    for w in db["wires"]:
        by_tile[(w["x"], w["y"])].create_wire(w["name"], type_of(w["id"]))

    # The graph's own pips, in the tile of the wire each one drives. A hop up
    # a level-13 line inverts the signal, so they are typed `L13` for the uarch
    # to give only to nets whose inversion a LUT can take back, and they cost
    # 3 ns more: their eight wires at the chip's middle are what a router would
    # take across the chip, and fight over.
    # Only the kinds of hop (which mux, fed from which, how far away) the
    # vendor's router takes in its own builds (hops.json): it never takes four
    # in ten of the graph's mux-to-mux edges, and on silicon a counter routed
    # over them showed one net on another's pad.
    hops = {(tuple(k), tuple(s), ox, oy) for k, s, ox, oy, _ in json.load(open(os.path.join(args.db, "slg47910", "hops.json")))}

    def key(nid):
        m = re.fullmatch(r"([UD])(\d+)B(\d+)N(\d+)", wires[nid]["name"])
        return (int(m[2]), m[1], int(m[3]), int(m[4])) if m else None

    seen = set()
    for p in db["pips"]:
        if (p["src"], p["dst"]) in seen:
            continue
        a, b = wires[p["src"]], wires[p["dst"]]
        if key(p["src"]) and key(p["dst"]) and (key(p["dst"]), key(p["src"]), b["x"] - a["x"], b["y"] - a["y"]) not in hops:
            continue
        seen.add((p["src"], p["dst"]))
        d = wires[p["dst"]]
        pip(by_tile[(d["x"], d["y"])], local(d["x"], d["y"], p["src"]), d["name"], "PIP", p["cost"] + (3000000 if d["name"].startswith("U13") else 0))

    # The dedicated clock inputs, at (0, 0): the pad wire of each is a node
    # that every L0 site's CLK is one pip from.
    tt0 = by_tile[(0, 0)]
    for z, (token, _d) in enumerate([("W0", 0), ("N0", 0), ("N1", 1)]):
        wire = f"CLKPAD_{token}"
        tt0.create_wire(wire, "CLKPAD")
        bel = tt0.create_bel(f"CLKIN_{token}", "SLG_CLKIN", z=100 + z)
        bel.flags = BEL_FLAG_GLOBAL  # its net is the clock tree: no placer pulls cells to (0, 0)
        tt0.add_bel_pin(bel, "PAD", wire, PinType.OUTPUT)
        clock_pads[token] = wire

    # A memory site is one of the L0 sites with an L1 view: the writer has no
    # LUT_IPT_SEL for the slices there yet (the 40 of them are the M units), so
    # they hold no logic.
    memory = {(s["x"], s["y"]) for s in db["sites"] if s["type"] == "L1"}
    for s in sites:
        x, y = s["x"], s["y"]
        if s["type"] == "L0" and (x, y) in memory:
            continue
        tt = by_tile[(x, y)]
        pins = {p["name"]: p for p in s["pins"]}
        ins = [p["node"] for p in s["pins"] if p["dir"] == "I"]
        if s["type"] == "L6":
            tt.create_wire("PAD", "PAD")
            bel = tt.create_bel("IOB", "SLG_IOB", z=0)
            tt.add_bel_pin(bel, "PAD", "PAD", PinType.INOUT)
            if x == 31:  # east-edge output pads have an enable of their own
                tt.create_wire("OE", "OE")
                tt.add_bel_pin(bel, "OE", "OE", PinType.INPUT)
            for p in s["pins"]:
                w = local(x, y, p["node"])
                if p["dir"] == "O":
                    pip(tt, "PAD", w, "IOPAD")
                else:
                    pip(tt, w, "PAD", "IOPAD")
                    if x == 31:
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
        for sl in range(4):
            luto, ffd, ffq = f"LUTO{sl}", f"FFD{sl}", f"FFQ{sl}"
            tt.create_wire(luto, "LUTO")
            tt.create_wire(ffd, "FFD")
            tt.create_wire(ffq, "FFQ")
            lut = tt.create_bel(f"LUT{sl}", "SLG_LUT", z=sl)
            tt.add_bel_pin(lut, "O", luto, PinType.OUTPUT)
            for k in range(6):
                li = f"LUTI{sl}_{k}"
                tt.create_wire(li, "LUTI")
                tt.add_bel_pin(lut, f"I{k}", li, PinType.INPUT)
                # I0-I3 have the slice's own pins (A 0-3, B 6-9, ...). I4 and I5
                # are muxed from the site's inputs by a one-hot select over four
                # shared pins that the encoder does not write yet: no pip, so
                # no route the bitstream would get wrong.
                if k < 4:
                    pip(tt, local(x, y, pins[f"I{sl * 6 + k}"]["node"]), li, "MUX")
            # a LUT leaves by its own slice's O pin only (arch_init.cc)
            pip(tt, luto, local(x, y, pins[f"O{26 + 3 * sl}"]["node"]), "OMUX")
            ff = tt.create_bel(f"FF{sl}", "SLG_FF", z=4 + sl)
            tt.add_bel_pin(ff, "D", ffd, PinType.INPUT)
            tt.add_bel_pin(ff, "Q", ffq, PinType.OUTPUT)
            tt.add_bel_pin(ff, "CLK", "CLK", PinType.INPUT)
            tt.add_bel_pin(ff, "CE", "CE", PinType.INPUT)
            tt.add_bel_pin(ff, "R", "SR", PinType.INPUT)
            # A flip-flop takes its D from its slice's LUT. (From a site input
            # it would go through the same shared pins as I4 and I5.)
            pip(tt, luto, ffd, "FFDMUX")
            pip(tt, ffq, local(x, y, pins[f"O{28 + 3 * sl}"]["node"]), "OMUX")
        # the global clock: one pip from each pad's wire
        for token, pad in clock_pads.items():
            alias = f"@0_0_{pad}"
            tt.create_wire(alias, "CLKPAD")
            aliases.setdefault(("clk", token), []).append((x, y, alias))
            pip(tt, alias, "CLK", "GCLK")

    for (x, y) in by_tile:
        chip.set_tile_type(x, y, f"T_{x}_{y}")
    for bucket in sorted(classes - {0}):
        chip.timing.set_pip_class("DEFAULT", f"P{bucket}", TimingValue(bucket * 100))

    # Merge each node's aliases into it, its owner first.
    for nid, where in aliases.items():
        if isinstance(nid, tuple):
            owner = NodeWire(0, 0, clock_pads[nid[1]])
        else:
            w = wires[nid]
            owner = NodeWire(w["x"], w["y"], w["name"])
        chip.add_node([owner] + [NodeWire(x, y, name) for x, y, name in where])

    chip.write_bba(args.bba)


if __name__ == "__main__":
    main()
