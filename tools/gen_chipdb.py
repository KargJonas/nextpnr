#!/usr/bin/env python3
"""Generate nextpnr chipdb for SLG47910.

Reads the decrypted EFLX routing graph via `routing_graph.build_graph()`
and emits a single JSON file the Viaduct uarch loads at startup.

Output schema (version 1):
    {
      "version": 1,
      "sites":   [{"id": n, "type": "L0|L1|L6",
                   "x": gx, "y": gy, "z": z,
                   "sx": site_local_x, "sy": site_local_y,
                   "pins": [{"name": str, "node": node_id}, ...]}, ...],
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
from routing_graph import build_graph  # noqa: E402


SITE_TYPE_NAME = {0: "L0", 1: "L1", 6: "L6"}

# Two-stage steering muxes whose data-routing encoding is unverified: omitted
# from plain data pips so the data router avoids them. The clock route DOES
# traverse them — `add_clock_fabric_real` re-adds their incoming edges as
# CLKPIP. See docs/clock-routing.md.
_EXCLUDED_DST_NODES = {'D1B1N0', 'D1B2N0', 'D1B3N0', 'D3B4N0'}

# Pin direction determined from routing graph topology later.
# Names are just "P{index}" — direction comes from the "dir" field.
def pin_name(idx: int) -> str:
    return f"P{idx}"


# ---------------------------------------------------------------------------
# Clock-fabric extension (Option A, step A)
# ---------------------------------------------------------------------------
# The decrypted vendor routing graph (`minplacer.rgraph_def`) models only the
# data-routing fabric. The vendor's dedicated clock-distribution layer
# (PAD → CLK_x_y_SEL global muxes → per-row CLK_SEL gates → per-CLB CLK pin,
# per `docs/clock-routing.md`) is not represented in that graph at all.
#
# `--enable-clock-fabric` adds a minimal, additive model of that layer so the
# router can reach every CLB's CLK input through real chipdb edges instead of
# the synthetic `GCLK` pip in `arch_init.cc::init_clock_bels`. The wire/pip
# count and naming here are intentionally coarse: one "CLK" pin per L0/L1
# site, two spine wires (one per clock domain), one per-row distribution
# wire. The router only needs enough granularity to find a path; the
# downstream emitter (Option A step D) derives the per-placement SM bits
# from the placed FF's `(gx, gy)` coordinate via
# `clock_fanout_synth.extract_rules_for_placement`.
#
# All new node IDs are allocated above the highest existing routing/site-pin
# node so the additive extension never conflicts with the data-routing
# graph's 0..48543 id space.
#
# Token naming and `W0`/`N0`/`N1` matches the existing
# `init_clock_bels::ClockBelSpec` table in `arch_init.cc` so step B can
# replace the `GCLK` shortcut with these wires without renaming.

CLOCK_PAD_TOKENS = ["W0", "N0", "N1"]
CLOCK_DOMAINS = [0, 1]
# Per vendor docs: W0 feeds Clock_0; N0 feeds Clock_0; N1 feeds Clock_1.
_PAD_DOMAINS = {"W0": [0], "N0": [0], "N1": [1]}


def add_clock_fabric(data: dict) -> dict:
    """Add a minimal clock-distribution layer to a chipdb dict.

    Adds, in-place:
      - 3 wires for the dedicated clock-input PADs (`CLKPAD_<token>`).
      - 2 wires for the clock-domain spines (`CLKSPINE_<domain>`).
      - One wire per L0/L1 row Y for the per-row distribution (`CLKROW_<y>`).
      - One wire per L0/L1 site for the per-CLB clock input
        (`CLBCLK_<x>_<y>_<z>`), exposed on the site as a new "CLK" input pin.
      - Pips: PAD → spine (per `_PAD_DOMAINS`), spine → row (every domain
        feeds every row, mirroring the vendor's per-row `CLK_SEL` gates),
        row → site-CLK (the final SM-network hop). Every clock-fabric pip
        carries ``"type": "CLKPIP"`` so the C++ side can label it distinctly
        from the unmarked data-routing pips (Option A step C).

    Returns the same dict (mutated) for convenience. Tracks the new wire
    counts under ``data["clock_fabric"]`` for downstream inspection.
    """
    max_node = max((w["id"] for w in data["wires"]), default=-1)
    next_id = max_node + 1

    def alloc_wire(name: str, x: int, y: int) -> int:
        nonlocal next_id
        nid = next_id
        next_id += 1
        data["wires"].append({"id": nid, "name": name, "x": x, "y": y})
        return nid

    pad_wires: dict[str, int] = {
        tok: alloc_wire(f"CLKPAD_{tok}", 0, 0) for tok in CLOCK_PAD_TOKENS
    }
    spine_wires: dict[int, int] = {
        d: alloc_wire(f"CLKSPINE_{d}", 0, 0) for d in CLOCK_DOMAINS
    }

    row_ys = sorted({s["y"] for s in data["sites"] if s["type"] in ("L0", "L1")})
    row_wires: dict[int, int] = {
        y: alloc_wire(f"CLKROW_{y}", 0, y) for y in row_ys
    }

    site_clk_wires: list[int] = []
    for site in data["sites"]:
        if site["type"] not in ("L0", "L1"):
            continue
        x, y, z = site["x"], site["y"], site["z"]
        clk_node = alloc_wire(f"CLBCLK_{x}_{y}_{z}", x, y)
        site["pins"].append({"name": "CLK", "node": clk_node, "dir": "I"})
        data["pips"].append({"src": row_wires[y], "dst": clk_node, "cost": 0,
                             "type": "CLKPIP"})
        site_clk_wires.append(clk_node)

    n_pad_pips = 0
    for tok, domains in _PAD_DOMAINS.items():
        for d in domains:
            data["pips"].append({
                "src": pad_wires[tok], "dst": spine_wires[d], "cost": 0,
                "type": "CLKPIP",
            })
            n_pad_pips += 1
    n_spine_pips = 0
    for d in CLOCK_DOMAINS:
        for y in row_ys:
            data["pips"].append({
                "src": spine_wires[d], "dst": row_wires[y], "cost": 0,
                "type": "CLKPIP",
            })
            n_spine_pips += 1

    data["clock_fabric"] = {
        "enabled": True,
        "pad_wires": pad_wires,
        "spine_wires": spine_wires,
        "row_wires": row_wires,
        "n_site_clk_wires": len(site_clk_wires),
        "n_pad_pips": n_pad_pips,
        "n_spine_pips": n_spine_pips,
        "n_row_pips": len(site_clk_wires),
    }
    return data


# ---------------------------------------------------------------------------
# Clock-fabric extension v2 — route the clock over the REAL SM graph
# ---------------------------------------------------------------------------
# The v1 `add_clock_fabric` (above) models the clock as a fake 3-hop
# abstraction (CLKPAD -> CLKSPINE -> CLKROW -> CLBCLK). nextpnr routes the
# clock through *that* abstraction, never over the real SM data fabric, so
# `emit_route` must rebuild the real ~53-bit SM route from the hand-RE'd
# `clock_fanout` table — which has silicon-dead entries for most placements.
#
# `add_clock_fabric_real` instead connects the clock to the REAL SM fabric so
# nextpnr's router2 (a negotiated-congestion router) routes it over the actual
# graph and picks its own self-consistent two-stage slot assignment (the slot
# is load-bearing AND congestion-dependent — see docs/clock-routing.md). The
# clock then enters the CLB on real input pins exactly like the vendor.
#
# What it adds (all additive, above the 0..48543 real-node id space):
#   - One routable wire per clock-pad token (`CLKPAD_<tok>`), connected by a
#     CLKPIP pip to that pad's REAL SM-injection node(s) (the L6 clock-pad
#     site's output sitepin nodes feed U0B*N0@(grid) — those are the entry).
#   - One `CLBCLK_<x>_<y>_<z>` wire per L0/L1 site, exposed as the site's "CLK"
#     input pin (arch_init binds the FF/RAM CLK bel-pin to it). It is fed by
#     CLKPIP pips from the CLB's clock-capable input sitepin nodes (pins
#     18-21 — the I-pins the vendor lands the clock on; dual-purpose
#     slice-D LUT/clock inputs, selected by config downstream).
#   - The currently-`_EXCLUDED_DST_NODES` two-stage D-node incoming edges,
#     re-added as CLKPIP so the clock can traverse them (they stay excluded
#     from plain data pips, so data routing is unaffected).
#
# The SM-internal hops are the chipdb's ordinary data pips; the clock net
# routes over them like any net. emit.cc emits the clock NET's full routed
# pip chain (by net identity) into routing.json's `clock_pips`, so emit_route
# reads the SM muxes nextpnr actually chose — no `clock_fanout` table.

# Clock-capable CLB input pins (slice-D region I18-21). Verified present on
# all 140 L0 + 40 L1 sites. The vendor lands the clock on these. See
# docs/clock-routing.md "How the clock physically enters the CLB".
#
# SPIKE NOTE (2026-05-31): golden lands the clock on pin 18 specifically (via
# D0B2N0); a spike that let the router land on pin 20 was silicon-DEAD even with
# correct steering+clearing (experiments id=13/16). The CLB CLK_SEL_MUX appears
# to tap ONE specific input pin as the clock, not any of 18-21. Restricting to
# pin 18 forces the router to land where the vendor does — testing whether the
# landing pin is load-bearing. Set via CLB_CLK_INPUT_PINS env-style override.
_CLB_CLK_INPUT_PINS = (18,)


def add_clock_fabric_real(data: dict, graph, excluded_dst_nodes) -> dict:
    """Wire the clock to the REAL SM fabric (v2). See module comment above.

    ``graph`` is the routing_graph.build_graph() object (for injection-node
    and clock-input-pin lookups). ``excluded_dst_nodes`` is the set of SM
    node names excluded from data pips, whose incoming edges are re-added
    here as CLKPIP so the clock can traverse them.

    Mutates ``data`` in place; records counts under ``data["clock_fabric"]``.
    """
    max_node = max((w["id"] for w in data["wires"]), default=-1)
    next_id = max_node + 1

    def alloc_wire(name: str, x: int, y: int) -> int:
        nonlocal next_id
        nid = next_id
        next_id += 1
        data["wires"].append({"id": nid, "name": name, "x": x, "y": y})
        return nid

    def clkpip(src: int, dst: int) -> None:
        data["pips"].append({"src": src, "dst": dst, "cost": 0,
                             "type": "CLKPIP"})

    # --- (1) clock-pad token wires -> real SM injection nodes ---
    # Map each token to the L6 clock-pad site whose output sitepin nodes feed
    # the SM fabric. W0 = site (96,0) -> grid (25,31) per docs. N0/N1 are the
    # north-edge pads; for the spike only W0 is exercised (osc_probe uses W0),
    # but we wire all known tokens we can resolve.
    #
    # CLOCK DELIVERY MODEL (docs/clock-routing.md, datasheet Fig 11 +
    # silicon id=23/24): the clock reaches the FF via a Logic-as-Clock loopback
    # through a Core-IOB. The IOB has fabric-routable INPUT pins (I0/I1) and
    # OUTPUT pins that drive the injection rail; CLK0/CLK1->CLK_SEL->CLB_CLK is
    # INTERNAL (not a routing-graph edge). So we model:
    #   (a) CLKPAD_<tok> -> IOB INPUT nodes (clock enters at a loopback IOB),
    #   (b) loopback CLKPIP: IOB INPUT node -> IOB OUTPUT node (the internal
    #       CLK0/CLK1->CLK_SEL pass-through), so
    #   (c) the existing IOB-OUTPUT -> rail edges carry it onto the injection
    #       rail, and the router continues rail -> spine -> FF.
    # This forces the router to thread through an IOB loopback (it cannot reach
    # the rail any other way), reproducing the vendor branch+spine as ONE path.
    # Replaces the earlier direct CLKPAD->rail injection that produced a
    # spine-only (silicon-dead) route.
    ring = _resolve_clock_ring_iobs(graph)   # token -> {"inputs":[...], "loopback":[(in,out),...]}
    pad_wires: dict[str, int] = {}
    n_inject_pips = 0
    n_loopback_pips = 0
    for tok in CLOCK_PAD_TOKENS:
        w = alloc_wire(f"CLKPAD_{tok}", 0, 0)
        pad_wires[tok] = w
        spec = ring.get(tok, {})
        for in_node in spec.get("inputs", []):
            clkpip(w, in_node)            # (a) pad -> IOB input
            n_inject_pips += 1
        for (in_node, out_node) in spec.get("loopback", []):
            clkpip(in_node, out_node)     # (b) IOB internal loopback
            n_loopback_pips += 1

    # --- (2) per-CLB CLK wire fed from real clock-capable input pin nodes ---
    # Index sitepin nodes by (site_type, sx, sy, pin_index) so we can find the
    # I18-21 nodes for each L0/L1 site.
    pin_node: dict[tuple, int] = {}
    for nid, sp in graph.sitepin_nodes.items():
        pin_node[(sp.site_type, sp.site_x, sp.site_y, sp.pin_index)] = nid
    type_st = {"L0": 0, "L1": 1}

    n_site_clk = 0
    n_exit_pips = 0
    for site in data["sites"]:
        if site["type"] not in ("L0", "L1"):
            continue
        x, y, z = site["x"], site["y"], site["z"]
        st = type_st[site["type"]]
        clk_node = alloc_wire(f"CLBCLK_{x}_{y}_{z}", x, y)
        site["pins"].append({"name": "CLK", "node": clk_node, "dir": "I"})
        n_site_clk += 1
        for p in _CLB_CLK_INPUT_PINS:
            innode = pin_node.get((st, site["sx"], site["sy"], p))
            if innode is None:
                continue
            clkpip(innode, clk_node)
            n_exit_pips += 1

    # --- (3) re-add excluded two-stage D-node incoming edges as CLKPIP ---
    # These were dropped from the plain data pips (their encoding was unverified
    # for data routing) but are exactly the steering muxes the clock route
    # traverses; the clock must be able to enter them.
    n_excl_pips = 0
    for src, edges in graph.out_edges.items():
        for dst, cost in edges:
            rn = graph.routing_nodes.get(dst)
            if rn is not None and rn.sm_name in excluded_dst_nodes:
                clkpip(src, dst)
                n_excl_pips += 1

    data["clock_fabric"] = {
        "enabled": True,
        "model": "real-loopback",
        "pad_wires": pad_wires,
        "n_inject_pips": n_inject_pips,
        "n_loopback_pips": n_loopback_pips,
        "n_site_clk_wires": n_site_clk,
        "n_exit_pips": n_exit_pips,
        "n_excluded_clkpips": n_excl_pips,
    }
    return data


def _resolve_clock_ring_iobs(graph) -> dict:
    """Map each clock-pad token to its loopback-IOB ring entry + loopback edges.

    The clock-pad ring is the 184 L6 (Core-IOB) sites tiled around the fabric
    border (grid rows/cols 0-1, 30-31; interior empty). Each ring IOB has 4
    fabric-routable INPUT sitepin nodes (pins 0-3) and 2 OUTPUT sitepin nodes
    (pins 4-5) that drive the adjacent edge tile's U0B*N0 injection rail. The
    clock's Logic-as-Clock loopback runs INPUT -> (internal CLK0/CLK1->CLK_SEL)
    -> OUTPUT; that internal hop is NOT a routing-graph edge, so we synthesize
    it as a CLKPIP.

    Returns ``{token: {"inputs": [in_node, ...],
                       "loopback": [(in_node, out_node), ...]}}``.
      - ``inputs``  : every ring-IOB input node CLKPAD_<tok> may drive (the
                       clock can enter at any loopback IOB on that token's edge).
      - ``loopback``: input->output node pairs per ring IOB (the internal
                       pass-through the router threads through to reach the rail).

    Edge assignment: "W0" => the NORTH rail (silicon-verified: osc_probe +
    gpio8 both declare ``clk W0`` yet loop through N-edge IOBs near their FF;
    experiments id=19). N0/N1 left empty until a design exercises them.
    """
    def _edge(gx, gy):
        if gy >= 30:
            return "N"
        if gy <= 1:
            return "S"
        if gx <= 1:
            return "W"
        if gx >= 30:
            return "E"
        return None

    # Per IOB site: collect input nodes (fabric-fed) and output nodes (drive
    # rail), grouped by the edge the site sits on.
    site_io: dict[tuple, dict] = {}   # (sx, sy) -> {"edge":e, "in":[...], "out":[...]}
    for nid, sp in graph.sitepin_nodes.items():
        if sp.site_type != 6:
            continue
        edge = _edge(sp.grid_x, sp.grid_y)
        if edge is None:
            continue
        has_in = len(graph.in_edges.get(nid, [])) > 0
        has_out = len(graph.out_edges.get(nid, [])) > 0
        rec = site_io.setdefault((sp.site_x, sp.site_y), {"edge": edge, "in": [], "out": []})
        if has_in and not has_out:
            rec["in"].append(nid)
        elif has_out and not has_in:
            rec["out"].append(nid)

    # Map token -> edge.
    _TOKEN_EDGE = {"W0": "N"}
    out: dict[str, dict] = {tok: {"inputs": [], "loopback": []} for tok in CLOCK_PAD_TOKENS}
    for tok, want_edge in _TOKEN_EDGE.items():
        inputs, loopback = [], []
        for (sx, sy), rec in site_io.items():
            if rec["edge"] != want_edge:
                continue
            inputs.extend(rec["in"])
            for in_node in rec["in"]:
                for out_node in rec["out"]:
                    loopback.append((in_node, out_node))
        out[tok] = {"inputs": inputs, "loopback": loopback}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--output", default="chipdb.json",
                    help="Output JSON path (default: chipdb.json)")
    ap.add_argument("--arch-dir", default=None,
                    help="Override decrypted arch directory")
    ap.add_argument("--compact", action="store_true",
                    help="Minified JSON (single line)")
    ap.add_argument("--enable-clock-fabric", action="store_true",
                    help="Add the v1 fake 3-hop clock abstraction "
                         "(CLKPAD->SPINE->ROW->CLBCLK). Off by default — "
                         "leaves the data-routing chipdb byte-identical.")
    ap.add_argument("--clock-fabric-real", action="store_true",
                    help="Wire the clock to the REAL SM fabric (v2): "
                         "CLKPAD_<tok> -> real SM injection node, CLB "
                         "clock-input pins -> CLBCLK, and the excluded "
                         "two-stage D-nodes re-added as CLKPIP so the router "
                         "routes the clock over the actual graph.")
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
    # Nodes whose encoding is unverified: omit pips targeting them so the
    # router is forced to find alternative paths through known-good nodes.
    pips = []
    for src, edges in sorted(graph.out_edges.items()):
        for dst, cost in edges:
            rn = graph.routing_nodes.get(dst)
            if rn is not None and rn.sm_name in _EXCLUDED_DST_NODES:
                continue
            pips.append({"src": src, "dst": dst, "cost": cost})

    data = {
        "version": 1,
        "grid": {"width": 32, "height": 32},
        "sites": sites,
        "wires": wires,
        "pips": pips,
    }

    if args.enable_clock_fabric and args.clock_fabric_real:
        ap.error("--enable-clock-fabric and --clock-fabric-real are mutually "
                 "exclusive (v1 fake abstraction vs v2 real fabric)")
    if args.enable_clock_fabric:
        add_clock_fabric(data)
        cf = data["clock_fabric"]
        print(f"  + clock fabric: {len(cf['pad_wires'])} pads, "
              f"{len(cf['spine_wires'])} spines, {len(cf['row_wires'])} rows, "
              f"{cf['n_site_clk_wires']} per-CLB CLK wires, "
              f"{cf['n_pad_pips'] + cf['n_spine_pips'] + cf['n_row_pips']} clock pips",
              file=sys.stderr)
    if args.clock_fabric_real:
        add_clock_fabric_real(data, graph, _EXCLUDED_DST_NODES)
        cf = data["clock_fabric"]
        print(f"  + clock fabric (REAL-loopback): {len(cf['pad_wires'])} pad wires, "
              f"{cf['n_inject_pips']} pad->IOB-input pips, "
              f"{cf['n_loopback_pips']} IOB loopback pips, "
              f"{cf['n_site_clk_wires']} per-CLB CLK wires, "
              f"{cf['n_exit_pips']} clk-input->CLBCLK pips, "
              f"{cf['n_excluded_clkpips']} re-added excluded-node CLKPIPs",
              file=sys.stderr)

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
