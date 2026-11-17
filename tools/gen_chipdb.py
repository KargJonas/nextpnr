#!/usr/bin/env python3
"""Generate nextpnr chipdb for SLG47910.

Reads the decrypted EFLX routing graph via `routing_graph.build_graph()`
and emits it as one JSON file: the himbaechel chipdb's source (forgefpga's
slg47910/graph.json) and the node ids the bitstream tools name.

Output schema (version 1):
    {
      "version": 1,
      "sites":   [{"id": n, "type": "L0|L1|L6",
                   "x": gx, "y": gy, "z": z,
                   "sx": site_local_x, "sy": site_local_y,
                   "pins": [{"name": str, "node": node_id}, ...],
                   "rest": [node_id, ...]}, ...],   (L6: rest_lines)
      "wires":   [{"id": node_id, "name": str, "x": gx, "y": gy}, ...],
      "pips":    [{"src": node_id, "dst": node_id, "cost": int}, ...]
    }

Sites use global grid coordinates (x, y) on a 32x32 fabric, plus a Z
index to disambiguate L0 and L1 sites that share a grid cell (memory
CLBs expose both a logic and a memory site). The site-local coords
`sx`/`sy` are kept for SDC/bitstream emission later.

Wires cover every routing node AND every sitepin node (48,544 total).
Pips cover every edge (310,016). Sites list all CLB/IOB placement sites
(140 L0 + 40 L1 + 184 L6 = 364) with their pin→node mapping so the uarch
can bind bel pins to wires.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from routing_graph import build_graph  # noqa: E402
from bitgen import bitstream, rgraph, sm_emit  # noqa: E402


SITE_TYPE_NAME = {0: "L0", 1: "L1", 6: "L6"}


def rest_lines(pins):
    """The lines an output pad's enable rests on, of a pad whose input pins
    are `pins`: no net may take them while the pad is an output.

    A pad takes its enable from the first or the second (0,'D') mux of the
    pin its data comes by (route_bits.py), and no net is routed to it: it
    holds while that mux, and the (1,'D') mux it selects, sit where the
    template has them. A net on the (1,'D') line moves the first, a net on
    the level-5 line that one selects the second, and moved, a pad lets go
    (on silicon: the clock enable, the done and the pass pads of dense
    builds). So these are both lines of the first two muxes of every pin."""
    g, words = _template()
    out = set()
    for a in pins:
        for m, _ in g.radj.get(a, []):
            if not g.is_sm(m) or g.nodes[m].key[:2] != (0, "D") or g.nodes[m].key[2] > 1:
                continue
            front = [m]
            for _ in range(2):
                front = [p for n in front for p in _selected(g, words, n)]
                out.update(front)
    return sorted(out)


_TEMPLATE = None


def _template():
    global _TEMPLATE
    if _TEMPLATE is None:
        _TEMPLATE = rgraph.load(), bitstream.Image.from_template(sm_emit._TMPL).words
    return _TEMPLATE


def _selected(g, words, nid):
    """The sources mux `nid` selects in `words`."""
    node = g.nodes[nid]
    held = sm_emit._selval(words, node.x, node.y, node.key)
    return [p for p, _ in g.radj.get(nid, []) if g.is_sm(p)
            and sm_emit.value_for(node.key, g.nodes[p].key, False, tval=held, active=True) == held]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--output", default="chipdb.json",
                    help="Output JSON path (default: chipdb.json)")
    ap.add_argument("--arch-dir", default=None,
                    help="Override decrypted arch directory")
    ap.add_argument("--compact", action="store_true",
                    help="Minified JSON (single line)")
    args = ap.parse_args()

    graph = build_graph(args.arch_dir)

    # --- Sites ---
    # L0 and L1 can share a grid cell (memory CLBs appear twice). Disambiguate
    # via Z: L0=0, L1=1, L6=0 (never overlaps).
    type_z = {0: 0, 1: 1, 6: 0}
    # Pick a representative pin to get the global grid location per site.
    site_grid: dict[tuple[int, int, int], tuple[int, int]] = {}
    for sp in graph.sitepin_nodes.values():
        key = (sp.site_type, sp.site_x, sp.site_y)
        if key not in site_grid:
            site_grid[key] = (sp.grid_x, sp.grid_y)

    # Determine pin direction from routing graph edges.
    # If a sitepin node has in_edges (routing → sitepin), it's a site INPUT.
    # If it has out_edges (sitepin → routing), it's a site OUTPUT.
    pin_dir: dict[int, str] = {}  # node_id -> "I" or "O"
    for nid in graph.sitepin_nodes:
        has_in = len(graph.in_edges.get(nid, [])) > 0
        has_out = len(graph.out_edges.get(nid, [])) > 0
        pin_dir[nid] = "I" if has_in and not has_out else "O"

    sites = []
    sid = 0
    for (sx, sy, st), pin_map in sorted(graph.site_to_pins.items()):
        type_name = SITE_TYPE_NAME.get(st, f"L{st}")
        pins = []
        for pin_idx, node_id in sorted(pin_map.items()):
            d = pin_dir.get(node_id, "I")
            pins.append({"name": f"{d}{pin_idx}", "node": node_id, "dir": d})
        gx, gy = site_grid.get((st, sx, sy), (sx, sy))
        sites.append({
            "id": sid, "type": type_name,
            "x": gx, "y": gy, "z": type_z.get(st, 0),
            "sx": sx, "sy": sy,
            "pins": pins,
        })
        if type_name == "L6":
            sites[-1]["rest"] = rest_lines([p["node"] for p in pins if p["dir"] == "I"])
        sid += 1

    # --- Wires ---
    wires = []
    for nid, rn in sorted(graph.routing_nodes.items()):
        wires.append({
            "id": nid, "name": rn.sm_name,
            "x": rn.grid_x, "y": rn.grid_y,
        })
    for nid, sp in sorted(graph.sitepin_nodes.items()):
        wires.append({
            "id": nid,
            "name": f"SP_L{sp.site_type}_{sp.site_x}_{sp.site_y}_p{sp.pin_index}",
            "x": sp.grid_x, "y": sp.grid_y,
        })

    # --- Pips ---
    pips = [{"src": src, "dst": dst, "cost": cost}
            for src, edges in sorted(graph.out_edges.items()) for dst, cost in edges]

    data = {
        "version": 1,
        "grid": {"width": 32, "height": 32},
        "sites": sites,
        "wires": wires,
        "pips": pips,
    }

    with open(args.output, "w") as f:
        if args.compact:
            json.dump(data, f, separators=(",", ":"))
        else:
            json.dump(data, f, separators=(",", ":"), indent=None)

    size = os.path.getsize(args.output)
    print(f"Wrote {args.output}: {len(sites)} sites, {len(wires)} wires, "
          f"{len(pips)} pips  ({size/1024/1024:.1f} MB)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
