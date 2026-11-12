#!/usr/bin/env python3
"""nextpnr routing -> the switch-matrix (SM) bits of a config image.

    route_bits.py routing.json -o route.bin

nextpnr's routing.json names, per net, the pips it used as (src, dst) node-id
pairs over the vendor routing graph (ids < 48544; larger ids are nextpnr's
virtual site wires, named in "pad_nodes"). Each routed SM node becomes a hop
in the form the vendor writes to PNR_ROUTE.log -- the mux at its own tile
(sx, sy), keyed (level, U/D, branch, index), delivering to the tile of what it
feeds (dx, dy) -- and bitgen.sm_emit, the generative encoder transcribed from
the vendor placer, turns the hops into bits on the device template.

On top of the routed nets goes the clock entry: the fixed bits the vendor
writes for a design clocked from W0 (the 50 MHz oscillator, datasheet
Table 10), whatever the design and wherever its flip-flops sit.
"""

import argparse
import json
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from bitgen import rgraph, sm_emit  # noqa: E402

N_GRAPH = 48544                      # vendor routing-graph node count
N_SM = 40960                         # ids below this are SM mux nodes
CONSTANT_NETS = {"$PACKER_GND", "$PACKER_VCC"}


def load_clock_entry():
    """[(word, bit, value)] the vendor writes for a W0-clocked design."""
    return [tuple(x) for x in json.loads((ROOT / "device" / "clock_w0.json").read_text())]


def net_hops(g, pips, exit_select):
    """A net's pip tree as vendor hops, parents before children. Each routed
    (0,'U') exit's select -- which of the two site pins of its fan-in drives it --
    is recorded in `exit_select`."""
    kids, has_parent = {}, set()
    for a, b in pips:
        kids.setdefault(a, []).append(b)
        has_parent.add(b)
    roots = [a for a in kids if a not in has_parent]
    parent = {b: a for a, b in pips}

    def tile(n):
        node = g.nodes[n]
        return node.x, node.y

    hops, order, seen = [], list(roots), set()
    while order:
        n = order.pop(0)
        if n in seen:
            continue
        seen.add(n)
        order.extend(kids.get(n, []))
        if n >= N_SM:
            continue
        node = g.nodes[n]
        sx, sy = node.x, node.y
        if node.level == 0 and node.direction == "U":
            fanin = [p for p, _w in g.radj.get(n, [])]
            if parent.get(n) not in fanin:
                sys.exit(f"{node.sm_name}@({sx},{sy}) is not fed by a pin of its fan-in")
            exit_select[((sx, sy), node.key)] = fanin.index(parent[n])
        dests = {tile(k) for k in kids.get(n, []) if k < N_GRAPH}
        for dx, dy in sorted(dests) or [(sx, sy)]:
            hops.append({"sx": sx, "sy": sy, "dx": dx, "dy": dy, "level": node.level,
                         "branch": node.branch, "index": node.index,
                         "j": int(node.direction == "D")})
    return hops


# The four pool pins X[0..3] of a slice share one symmetric 4:1 fan-in, and
# which LUT input the net lands on is fixed by WHICH of those four nodes
# delivers it, not by the pin the router names: the node at fan-in position i
# lands on LUT input POOL_MAP[i] (eflx-port issue #5: 819/819 vendor LUT lanes
# bit-exact, lut_lab/route_perm.py).
POOL_MAP = (1, 3, 0, 2)


def pool_inputs(g, routing):
    """[[sx, sy, pin, lut_input]] for every routed pool pin (pin % 6 < 4) of
    an L0 site; an L1 half-site's pins each have their own feeder."""
    by_site_pin = {}
    for node in g.nodes[N_SM:]:
        by_site_pin.setdefault((node.site, node.pin), []).append(node.nid)
    out = []
    for net in routing["nets"]:
        parent = {b: a for a, b in net["pips"]}
        for dst, src in parent.items():
            if not (N_SM <= dst < N_GRAPH) or not src < N_SM:
                continue
            pin = g.nodes[dst]
            if pin.pin >= 24 or pin.pin % 6 == 4:
                continue
            site = pin.site
            if pin.pin % 6 == 5:
                # I5 of the four slices share four feeders, and MUX_CE / MUX_SR
                # select among them by the feeder's branch (A-D = D0B0-D0B3: all
                # of 7,400 routed enables and resets of the vendor's builds).
                # Reported as pin 100 + pin, apart from the LUT inputs.
                sx, sy = (int(v) for v in re.match(r"(\d+),(\d+)", site).groups())
                if re.match(r"\d+,\d+L0", site):
                    out.append([sx, sy, 100 + pin.pin, g.nodes[src].branch])
                continue
            fanin = [f for n in by_site_pin[(site, pin.pin - pin.pin % 6)]
                     for f, _w in g.radj.get(n, [])]
            if len(fanin) == 1:
                continue      # an L1 half-site pin has one direct feeder: no pool
            if src not in fanin:
                sys.exit(f"pool pin {pin.pin} of {site} fed by a node outside its fan-in")
            sx, sy = (int(v) for v in re.match(r"(\d+),(\d+)", site).groups())
            out.append([sx, sy, pin.pin, POOL_MAP[fanin.index(src)]])
    return out


def inverting_paths(g, routing, virtual):
    """[(net, sink, n)] for every root->sink path crossing n (odd) level-13-up
    hops. Such a hop inverts the signal: the vendor's hop-polarity leaf
    FUN_00573470 returns 1 for every L13-up hop, and on silicon the ring branch
    crossing U13B1N1 read inverted. The vendor folds this into the LUT init and
    the IOB value bits; this flow does not yet."""
    out = []
    for net in routing["nets"]:
        if net["name"] in CONSTANT_NETS:
            continue
        kids, has_parent = {}, set()
        for a, b in net["pips"]:
            kids.setdefault(a, []).append(b)
            has_parent.add(b)
        stack = [(r, 0) for r in kids if r not in has_parent]
        while stack:
            n, k = stack.pop()
            if n < N_SM and g.nodes[n].level == 13 and g.nodes[n].direction == "U":
                k += 1
            if n not in kids:
                if k % 2:
                    out.append((net["name"], virtual.get(n, str(n)), k))
                continue
            stack.extend((c, k) for c in kids[n])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("routing")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--lut-inputs", required=True,
                    help="where to write which LUT input each routed pool pin lands on")
    args = ap.parse_args()

    routing = json.loads(Path(args.routing).read_text())
    g = rgraph.load()

    nets, exit_select = {}, {}
    for i, net in enumerate(routing["nets"]):
        if net["name"] in CONSTANT_NETS:
            continue                  # constants are set where they are used
        hops = net_hops(g, [tuple(p) for p in net["pips"]], exit_select)
        if hops:
            nets[i] = hops

    virtual = {i: name for i, name in routing["pad_nodes"]}
    # LUT inputs whose polarity the caller has settled in the LUT's table
    compensated = set(routing.get("compensated", [])) | set(routing.get("checked", []))
    bad = False
    for net, sink, k in inverting_paths(g, routing, virtual):
        if sink in compensated:
            continue
        kind = sink.rsplit("/", 1)[-1]
        print(f"route_bits: {net} -> {sink} crosses {k} inverting level-13-up hop(s)"
              + ("" if kind == "PAD" else " -- the LUT would see its input inverted"),
              file=sys.stderr)
        bad |= kind != "PAD"
    if bad:
        sys.exit("route_bits: inverted LUT inputs are not compensated yet")

    image, n = sm_emit.emit_design(g, nets, bound_ports={}, input_tiles=set(),
                                   exit_select=exit_select)
    words = image.words
    for w, b, v in load_clock_entry():
        words[w] = (words[w] | (1 << b)) if v else (words[w] & ~(1 << b))
    Path(args.output).write_bytes(struct.pack(f"<{len(words)}I", *words))
    Path(args.lut_inputs).write_text(json.dumps(pool_inputs(g, routing)))
    print(f"route_bits: {len(nets)} nets, {sum(len(h) for h in nets.values())} hops, "
          f"{n} SM nodes written -> {args.output}")


if __name__ == "__main__":
    main()
