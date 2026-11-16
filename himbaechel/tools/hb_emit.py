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
    # A setting flip-flop (FDSE, FDPE) is a clearing one whose D xQpol inverts:
    # it holds its value inverted, and every sink of its Q takes that back.
    setting = {(ff["x"], ff["y"], ff["slice"], ff["slot"]) for ff in placement["ffs"] if ff["ff_type"] in ("FDSE", "FDPE")}
    for cell in placement["luts"] + placement["ffs"] + placement["carries"] + placement["muxfs"]:
        s = site[(cell.pop("x"), cell.pop("y"))]
        cell.update(site_type="L0", sx=s["sx"], sy=s["sy"])
    # MUX_XQ (aq1_sel) by what drives a flip-flop, MUX_X (amx_sel) by what
    # leaves by the xMUX pin: S_BIT or C_BIT (slice D drops EXTRA_IN, one lower).
    # A second flip-flop (OXQ) takes MUX_OXQ (oaq_sel) and leaves by xMUX (MUX_X's OXQ).
    muxes = {}
    for ff in placement["ffs"]:
        m = muxes.setdefault((ff["sx"], ff["sy"], ff["slice"]), {})
        if ff["slot"] == "OXQ":
            m.update(oaq_sel=ff["oaq_sel"], amx_sel=1 if ff["slice"] == 3 else 2)
        else:
            m["aq1_sel"] = ff["aq1_sel"]
    for c in placement["carries"]:
        if c["xmux"]:
            bit = (4 if c["xmux"] == "S" else 3) - (c["slice"] == 3)
            muxes.setdefault((c["sx"], c["sy"], c["slice"]), {})["amx_sel"] = 1 << bit
    # A slice's lower half leaves by its xMUX pin as MUX_X's OX0 (the vendor's
    # AMX_SEL 00100, slice D 0010, in all 369 slices of its builds that pair LUTs).
    # Not where the slice's second flip-flop leaves by it: then the lower half
    # feeds that flip-flop (MUX_OXQ's OX0) and nothing else.
    for lut in placement["luts"]:
        if "lower" in lut:
            muxes.setdefault((lut["sx"], lut["sy"], lut["slice"]), {}).setdefault("amx_sel", 4 if lut["slice"] < 3 else 2)
    # A wide mux (F7A, F8, F7C) leaves by its slice's xMUX pin as MUX_X's EXTRA_IN
    # (the vendor's AMX_SEL 00001 at every one of 208 in its builds).
    for m in placement["muxfs"]:
        if m["xmux"]:
            muxes.setdefault((m["sx"], m["sy"], m["slice"]), {})["amx_sel"] = 1
    placement["slice_muxes"] = [dict(sx=sx, sy=sy, site_type="L0", slice=sl, **{"oaq_sel": 2, **m})
                                for (sx, sy, sl), m in muxes.items()]
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
    # leaving its site; a pad shows the complement of what reaches it, which the
    # uarch takes back by giving every pad a LUT of its own. Each sink takes an
    # odd count back where it can: a LUT input in its LUT's table, a pad in its
    # LUT's output, a site's enable or reset in MUX_CE / MUX_SR's XOR, XI (a
    # flip-flop's D from the fabric, a carry's G) in MUX_xI's (emit_sdc.py).
    def u13_hops(parent, wire):
        hops = 0
        while wire in parent:
            wire = parent[wire]
            hops += wire.rsplit("/", 1)[1].startswith("U13")
        return hops

    compensated, checked, control = [], [], {}
    for net in routing["nets"]:
        if net["name"] in constant.values():
            continue  # the oscillator enable's constant: no LUT, the tools encode it
        parent = {dst: src for src, dst, _kind in net["pips"]}
        drivers = [re.fullmatch(r"X(\d+)Y(\d+)/LUTO(\d)", src) for src, _dst, kind in net["pips"] if kind == "OMUX"]
        drivers = [d for d in drivers if d]
        from_lut = any(drivers)  # a carry's sum or carry leaves by xMUX true (on silicon)
        from_lut ^= any((int(m[1]), int(m[2]), int(m[4]), "OXQ" if m[3] else "XQ") in setting for m in
                        (re.fullmatch(r"X(\d+)Y(\d+)/(O?)FFQ(\d)", src) for src, _dst, _kind in net["pips"]) if m)
        for dst in list(parent):
            m = re.fullmatch(r"X(\d+)Y(\d+)/LUTI(\d)_(\d)", dst)
            if m:
                checked.append(dst)
                # The I4 pins complement what reaches them (on silicon): there a
                # flip-flop's output arrives inverted and a LUT's arrives true.
                # A memory site's do too: pack.py clears the bits that would
                # invert its inputs (memory_clb_logic_bits).
                x, y, slice_, k = (int(v) for v in m.groups())
                if (from_lut + (k == 4) + u13_hops(parent, dst)) % 2:
                    lut = luts_at[(x, y, slice_)]
                    n = max(i for i, pin in enumerate(lut["pins"]) if pin >= 0) + 1
                    # both halves of a slice take the pin (init_lower: the lower one's LUT)
                    for half in ("init", "init_lower"):
                        if half not in lut:
                            continue
                        init = lut[half]
                        flipped = init & ~((1 << (1 << n)) - 1)
                        for i in range(1 << n):
                            flipped |= (init >> (i ^ (1 << k)) & 1) << i
                        lut[half] = flipped
                    compensated.append(dst)
            elif re.fullmatch(r"X\d+Y\d+/O?FFD\d", dst):
                checked.append(dst)  # D in its slice, or XI's (taken back at XI)
            elif re.fullmatch(r"X\d+Y\d+/(CE|SR|XI\d)", dst):
                checked.append(dst)
                control[dst] = (from_lut + u13_hops(parent, dst)) % 2
            elif dst.endswith("/PAD") and u13_hops(parent, dst) % 2:
                lut = luts_at[tuple(int(v) for v in drivers[0].groups())]
                n = max([i for i, pin in enumerate(lut["pins"]) if pin >= 0], default=-1) + 1
                lut["init"] ^= (1 << (1 << n)) - 1 if n else (1 << 64) - 1
                compensated.append(dst)
    tile_of = {(s["sx"], s["sy"]): xy for xy, s in site.items()}
    # A second flip-flop that takes its slice's first flip-flop's Q (MUX_OXQ's
    # XQ) gets it inverted when the first is a setting one: its xQpol takes
    # that back (emit_sdc.py).
    for ff in placement["ffs"]:
        if ff["slot"] == "OXQ" and ff["oaq_sel"] == 2:
            x, y = tile_of[(ff["sx"], ff["sy"])]
            ff["d_parity"] = int((x, y, ff["slice"], "XQ") in setting)
    for ff in placement["ffs"]:
        x, y = tile_of[(ff["sx"], ff["sy"])]
        ff["ce_inverted"] = control.get(f"X{x}Y{y}/CE", 0)
        ff["sr_inverted"] = control.get(f"X{x}Y{y}/SR", 0)
        ff["d_inverted"] = control.get(f"X{x}Y{y}/XI{ff['slice']}", 0)
    for c in placement["carries"]:
        x, y = tile_of[(c["sx"], c["sy"])]
        c["di_inverted"] = control.get(f"X{x}Y{y}/XI{c['slice']}", 0)
    for m in placement["muxfs"]:
        x, y = tile_of[(m["sx"], m["sy"])]
        m["sel_inverted"] = control.get(f"X{x}Y{y}/XI{m['slice']}", 0)
    json.dump(placement, open(placement_out, "w"), indent=1)
    json.dump({"pips": data, "clock_pips": clock, "pad_nodes": pad_nodes, "nets": nets, "compensated": compensated,
               "checked": checked},
              open(routing_out, "w"))


if __name__ == "__main__":
    main()
