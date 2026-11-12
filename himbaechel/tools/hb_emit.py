#!/usr/bin/env python3
"""nextpnr-himbaechel's placement and routing, in the form the bitstream tools take.

    hb_emit.py chipdb.json placement_raw.json routing_raw.json placement.json routing.json

The uarch writes what it knows in the chipdb's own names: a cell at a grid
position, a net's pips as `X<x>/Y<y>/<wire>` pairs. The tools (route_bits.py,
emit_sdc.py, pack.py) were written for the Viaduct uarch's output and take the
vendor graph's node ids and a site's (sx, sy), so this turns the one into the
other: a wire of the graph becomes its node id; any other (a pad, a LUT input)
a virtual id above them, named in `pad_nodes`, as the Viaduct uarch numbered
them.
"""

import json
import re
import sys


def main():
    chipdb, placement_in, routing_in, placement_out, routing_out = sys.argv[1:]
    db = json.load(open(chipdb))
    site = {(s["x"], s["y"]): s for s in db["sites"] if s["type"] == "L0"}
    node = {(w["x"], w["y"], w["name"]): w["id"] for w in db["wires"]}

    placement = json.load(open(placement_in))
    # A constant held at a pad has a LUT of its own in nextpnr (so the placer can
    # put it beside the pad). At the oscillator-enable pad (0,25) the tools know
    # such a net by the constant's name and keep no LUT for it (its encoding is
    # the vendor's for a constant). Anywhere else the LUT stays and drives the
    # pad as any other: the tools hold no constant encoding for a row-31 pad.
    held = {iob["net"] for iob in placement["iobs"] if (iob["gx"], iob["gy"]) == (25, 0)}
    constant = {}
    for lut in placement["luts"]:
        if lut["name"].startswith("$const_") and lut["name"] + "_o" in held:
            constant[lut["name"] + "_o"] = "$PACKER_VCC" if lut["init"] else "$PACKER_GND"
    placement["luts"] = [lut for lut in placement["luts"] if lut["name"] + "_o" not in constant]
    for iob in placement["iobs"]:
        iob["net"] = constant.get(iob["net"], iob["net"])
    luts_at = {(c["x"], c["y"], c["slice"]): c for c in placement["luts"]}
    for cell in placement["luts"] + placement["ffs"]:
        s = site[(cell.pop("x"), cell.pop("y"))]
        cell.update(site_type="L0", sx=s["sx"], sy=s["sy"])
    placement["slice_muxes"] = [
        {"sx": ff["sx"], "sy": ff["sy"], "site_type": "L0", "slice": ff["slice"], "oaq_sel": 2, "aq1_sel": ff["aq1_sel"]}
        for ff in placement["ffs"]
    ]
    for iob in placement["iobs"]:
        branch = re.search(r"D0B(\d+)N", iob["pad_in_sm"])
        iob.update(pad_in_d0_branch=int(branch.group(1)) if branch else -1, oe_pin=-1, oe_sm="", oe_d0_branch=-1)
    routing = json.load(open(routing_in))
    for net in routing["nets"]:
        net["name"] = constant.get(net["name"], net["name"])
    base = len(db["wires"])
    virtual = {}
    pad_nodes = []

    def ident(name):
        x, y, wire = re.fullmatch(r"X(\d+)Y(\d+)/(.*)", name).groups()
        assert not wire.startswith("@"), f"{name}: the router names an alias, not its node"
        if (int(x), int(y), wire) in node:
            return node[(int(x), int(y), wire)]
        if name not in virtual:
            virtual[name] = base + len(pad_nodes)
            pad_nodes.append([virtual[name], name])
        return virtual[name]

    nets, data, clock = [], [], []
    for net in routing["nets"]:
        pips = []
        for src, dst, kind in net["pips"]:
            s, d = ident(src), ident(dst)
            pips.append([s, d])
            if s < base and d < base:
                {"PIP": data, "CLKPIP": clock}.get(kind, []).append([s, d])
        nets.append({"name": net["name"], "pips": pips})
    # A hop up a level-13 line inverts the signal, and so does a LUT's output
    # leaving its site (on silicon: build/sv9/t1-t3, comb, duty): a LUT input
    # reached through an odd number of these is inverted in the LUT's table,
    # which takes it back.
    compensated, checked, lut_nets = [], [], set()
    for net in routing["nets"]:
        parent = {dst: src for src, dst, _kind in net["pips"]}
        from_lut = any(kind == "OMUX" and re.search(r"/LUTO\d$", src) for src, _dst, kind in net["pips"])
        if from_lut:
            lut_nets.add(net["name"])
        for dst in list(parent):
            m = re.fullmatch(r"X(\d+)Y(\d+)/LUTI(\d)_(\d)", dst)
            if not m:
                continue
            checked.append(dst)
            hops, wire = int(from_lut), dst
            while wire in parent:
                wire = parent[wire]
                hops += wire.rsplit("/", 1)[1].startswith("U13")
            if hops % 2:
                x, y, slice_, k = (int(v) for v in m.groups())
                lut = luts_at[(x, y, slice_)]
                n = max(i for i, pin in enumerate(lut["pins"]) if pin >= 0) + 1
                init = lut["init"]
                flipped = init & ~((1 << (1 << n)) - 1)
                for i in range(1 << n):
                    flipped |= (init >> (i ^ (1 << k)) & 1) << i
                lut["init"] = flipped
                compensated.append(dst)
    # A LUT that drives only a pad (the uarch gives every pad one) may take
    # level-13 lines: an odd number of them on the way is taken back by
    # inverting the LUT's output.
    for net in routing["nets"]:
        parent = {dst: src for src, dst, _kind in net["pips"]}
        pads = [dst for _src, dst, kind in net["pips"] if kind == "IOPAD" and dst.endswith("/PAD")]
        drivers = [re.fullmatch(r"X(\d+)Y(\d+)/LUTO(\d)", src) for src, _dst, kind in net["pips"] if kind == "OMUX"]
        if len(pads) != 1 or len(drivers) != 1 or drivers[0] is None:
            continue
        hops, wire = 0, pads[0]
        while wire in parent:
            wire = parent[wire]
            hops += wire.rsplit("/", 1)[1].startswith("U13")
        if hops % 2:
            lut = luts_at[tuple(int(v) for v in drivers[0].groups())]
            n = max([i for i, pin in enumerate(lut["pins"]) if pin >= 0], default=-1) + 1
            lut["init"] ^= (1 << (1 << n)) - 1 if n else (1 << 64) - 1
            compensated.append(pads[0])
    # An enable or a reset a LUT drives arrives complemented too: the site's
    # MUX_CE / MUX_SR take it back with their XOR (emit_sdc.py).
    for ff in placement["ffs"]:
        ff["ce_inverted"] = int(ff.get("ce_net") in lut_nets)
        ff["sr_inverted"] = int(ff.get("sr_net") in lut_nets)
    json.dump(placement, open(placement_out, "w"), indent=1)
    json.dump({"pips": data, "clock_pips": clock, "pad_nodes": pad_nodes, "nets": nets, "compensated": compensated,
               "checked": checked},
              open(routing_out, "w"))


if __name__ == "__main__":
    main()
