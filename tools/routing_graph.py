#!/usr/bin/env python3
"""
Parse EFLX routing architecture files into a machine-readable graph.

Input files (decrypted from eda-placer):
  - minplacer.rgraph_def    — routing graph: 48544 nodes, 310016 edges
  - minplacer.rnode2sm_def  — maps routing nodes (0..40959) to grid position + SM name
  - minplacer.rnode2sitepin_def — maps site-pin nodes (40960..48543) to CLB site pins
  - minplacer.timing_def    — pin names per RBB type (F=FF, T=through, O=output, I=input)

Node ID space:
  0 .. 40959   — routing/switch-matrix nodes (40 SMs per grid point, 32x32 grid)
  40960 .. 48543 — site pin nodes (CLB/IOB pin connections)

Output: JSON graph suitable for nextpnr Viaduct backend.
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field, asdict
from typing import Optional


sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from bitgen.database import GRAPH  # noqa: E402

ARCH_DIR = str(GRAPH)

DELIMITER = "@^#$a~%_!D+=/>,-@"


@dataclass
class RoutingNode:
    """A routing/switch-matrix node."""
    node_id: int
    grid_x: int
    grid_y: int
    sm_name: str  # e.g. "U3B1N0" — Up/Down, Level, Branch, Index


@dataclass
class SitePinNode:
    """A site-pin node connecting the routing fabric to a CLB/IOB pin."""
    node_id: int
    grid_x: int  # grid position where this pin connects to routing
    grid_y: int
    site_x: int  # site coordinates
    site_y: int
    site_type: int  # L0=CLB, L1=CLB(?), L6=BRAM/IOB
    pin_index: int


@dataclass
class Edge:
    """A directed edge in the routing graph."""
    src: int
    dst: int
    cost: int


@dataclass
class PinDef:
    """Pin definition from timing_def."""
    kind: str   # F=FF, T=through, O=output, I=input
    rbb_type: int
    pin_num: int


@dataclass
class RBBType:
    """RBB type definition from timing_def."""
    name: str       # e.g. "RBB_4"
    cell_type: str  # e.g. "RBB_6L", "IOB"
    pins: list      # list of PinDef


@dataclass
class RoutingGraph:
    """Complete routing graph for the EFLX fabric."""
    num_nodes: int = 0
    num_edges: int = 0
    num_routing_nodes: int = 0
    num_sitepin_nodes: int = 0

    # Node data
    routing_nodes: dict = field(default_factory=dict)   # node_id -> RoutingNode
    sitepin_nodes: dict = field(default_factory=dict)    # node_id -> SitePinNode

    # Adjacency lists
    out_edges: dict = field(default_factory=dict)  # src_node -> [(dst_node, cost)]
    in_edges: dict = field(default_factory=dict)   # dst_node -> [(src_node, cost)]

    # RBB type definitions
    rbb_types: list = field(default_factory=list)

    # Derived lookups (built after parsing)
    grid_to_routing: dict = field(default_factory=dict)  # (x,y) -> {sm_name: node_id}
    site_to_pins: dict = field(default_factory=dict)     # (sx,sy,type) -> {pin_idx: node_id}


def _strip_encwrap(lines):
    """Skip encwrap header lines, yield data lines between delimiters.
    A .clean dump (forgefpga's slg47910/graph) has no wrapper: every line is data."""
    lines = [l.rstrip('\n').rstrip('\r') for l in lines]
    if not any(l.startswith(DELIMITER) for l in lines):
        yield from lines
        return
    started = False
    for line in lines:
        line = line.rstrip('\n').rstrip('\r')
        if not started:
            if line.startswith(DELIMITER):
                rest = line[len(DELIMITER):]
                started = True
                if rest:
                    yield rest
            continue
        if line.startswith(DELIMITER):
            break
        yield line


def parse_rgraph(path):
    """Parse minplacer.rgraph_def → (num_nodes, num_edges, out_edges, in_edges).

    Format:
      Header: "D <num_nodes> E <num_edges>"
      +N>dest=cost      — start new source node group at node N
      ++dest=cost       — continue to next source node (N+1, N+2, ...)
      dest=cost         — additional edge from same source node
      Multiple dest=cost per line separated by newlines

    dest and cost are integers. dest is the absolute destination node ID.
    """
    out_edges = {}
    in_edges = {}

    with open(path) as f:
        lines = list(_strip_encwrap(f))

    # The vendor header "D <nodes> E <edges>" sits on the encwrap delimiter
    # line, so the decrypted .clean dump (forgefpga's slg47910/graph) starts at the edges.
    num_nodes = num_edges = None
    m = re.match(r'D\s+(\d+)\s+E\s+(\d+)', lines[0]) if lines else None
    if m:
        num_nodes, num_edges = int(m.group(1)), int(m.group(2))
        lines = lines[1:]

    cur_node = -1  # current source node
    edge_count = 0

    for line in lines:
        if line.startswith('++'):
            # Continue to next source node
            cur_node += 1
            rest = line[2:]
        elif line.startswith('+'):
            # New node group: +N>...
            gt = line.index('>')
            new_id = int(line[1:gt])
            # Fill gap: nodes between old cur_node+1 and new_id have no outgoing edges
            cur_node = new_id
            rest = line[gt+1:]
        else:
            # Additional edge line for current source node
            rest = line

        # Parse dest=cost
        m = re.match(r'(\d+)=(-?\d+)', rest)
        if not m:
            continue  # skip empty or malformed
        dst = int(m.group(1))
        cost = int(m.group(2))

        if cur_node not in out_edges:
            out_edges[cur_node] = []
        out_edges[cur_node].append((dst, cost))

        if dst not in in_edges:
            in_edges[dst] = []
        in_edges[dst].append((cur_node, cost))

        edge_count += 1

    if num_edges is not None and edge_count != num_edges:
        print(f"WARNING: parsed {edge_count} edges, header says {num_edges}",
              file=sys.stderr)
    if num_nodes is None:
        num_nodes = 1 + max(max(out_edges), max(in_edges))
        num_edges = edge_count

    return num_nodes, num_edges, out_edges, in_edges


def parse_rnode2sm(path):
    """Parse minplacer.rnode2sm_def → dict of node_id -> RoutingNode.

    Format: one line per node ID (0, 1, 2, ...):
      "x,y SM_NAME"
    Header: "[0,40960)"
    """
    nodes = {}
    with open(path) as f:
        lines = list(_strip_encwrap(f))

    # Parse header
    # The "[start,end)" range header sits on the encwrap delimiter line, so
    # the decrypted .clean dump (forgefpga's slg47910/graph) starts at the first node.
    m = re.match(r'\[(\d+),(\d+)\)', lines[0])
    if m:
        start_id, end_id = int(m.group(1)), int(m.group(2))
        lines = lines[1:]
    else:
        start_id = 0
        end_id = start_id + len([l for l in lines if l.strip()])

    for i, line in enumerate(lines):
        node_id = start_id + i
        if node_id >= end_id:
            break
        parts = line.split()
        if len(parts) != 2:
            continue
        coords, sm_name = parts
        x, y = coords.split(',')
        nodes[node_id] = RoutingNode(
            node_id=node_id,
            grid_x=int(x),
            grid_y=int(y),
            sm_name=sm_name,
        )

    return nodes, start_id, end_id


def parse_rnode2sitepin(path):
    """Parse minplacer.rnode2sitepin_def → dict of node_id -> SitePinNode.

    Format: one line per node ID (starting from offset):
      "gx,gy sx,syLtype pin_index"
    Header: "[40960,48544)"
    """
    nodes = {}
    with open(path) as f:
        lines = list(_strip_encwrap(f))

    # The "[start,end)" range header sits on the encwrap delimiter line, so
    # the decrypted .clean dump (forgefpga's slg47910/graph) starts at the first node.
    m = re.match(r'\[(\d+),(\d+)\)', lines[0])
    if m:
        start_id, end_id = int(m.group(1)), int(m.group(2))
        lines = lines[1:]
    else:
        start_id = 40960
        end_id = start_id + len([l for l in lines if l.strip()])

    for i, line in enumerate(lines):
        node_id = start_id + i
        if node_id >= end_id:
            break
        parts = line.split()
        if len(parts) != 3:
            continue
        grid_coords, site_ref, pin_idx = parts
        gx, gy = grid_coords.split(',')

        # Parse site ref like "0,0L0" or "99,0L6"
        sm = re.match(r'(\d+),(\d+)L(\d+)', site_ref)
        if not sm:
            print(f"WARNING: bad site ref {site_ref!r} at node {node_id}",
                  file=sys.stderr)
            continue

        nodes[node_id] = SitePinNode(
            node_id=node_id,
            grid_x=int(gx),
            grid_y=int(gy),
            site_x=int(sm.group(1)),
            site_y=int(sm.group(2)),
            site_type=int(sm.group(3)),
            pin_index=int(pin_idx),
        )

    return nodes, start_id, end_id


def parse_timing_def(path):
    """Parse minplacer.timing_def → list of RBBType.

    Format:
      I <name> (<cell_type>) ;
      D <kind><rbb_type>,<pin_num> ;
    """
    rbb_types = []
    with open(path) as f:
        lines = list(_strip_encwrap(f))

    current = None
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            if current:
                rbb_types.append(current)
                current = None
            continue

        m = re.match(r'I\s+(\w+)\s+\((\w+)\)\s*;', line)
        if m:
            if current:
                rbb_types.append(current)
            current = RBBType(name=m.group(1), cell_type=m.group(2), pins=[])
            continue

        m = re.match(r'D\s+([FTOI])(\d+),(\d+)\s*;', line)
        if m and current:
            current.pins.append(PinDef(
                kind=m.group(1),
                rbb_type=int(m.group(2)),
                pin_num=int(m.group(3)),
            ))
            continue

    if current:
        rbb_types.append(current)

    return rbb_types


def build_graph(arch_dir=None):
    """Parse all architecture files and build the complete routing graph."""
    if arch_dir is None:
        arch_dir = ARCH_DIR

    rgraph_path = os.path.join(arch_dir, "minplacer.rgraph_def")
    sm_path = os.path.join(arch_dir, "minplacer.rnode2sm_def")
    sitepin_path = os.path.join(arch_dir, "minplacer.rnode2sitepin_def")
    timing_path = os.path.join(arch_dir, "minplacer.timing_def")

    print("Parsing routing graph...", file=sys.stderr)
    num_nodes, num_edges, out_edges, in_edges = parse_rgraph(rgraph_path)
    print(f"  {num_nodes} nodes, {num_edges} edges", file=sys.stderr)

    print("Parsing routing node -> SM mapping...", file=sys.stderr)
    routing_nodes, rn_start, rn_end = parse_rnode2sm(sm_path)
    print(f"  {len(routing_nodes)} routing nodes [{rn_start}..{rn_end})",
          file=sys.stderr)

    print("Parsing site pin nodes...", file=sys.stderr)
    sitepin_nodes, sp_start, sp_end = parse_rnode2sitepin(sitepin_path)
    print(f"  {len(sitepin_nodes)} site pin nodes [{sp_start}..{sp_end})",
          file=sys.stderr)

    print("Parsing timing definitions...", file=sys.stderr)
    rbb_types = parse_timing_def(timing_path)
    print(f"  {len(rbb_types)} RBB types, "
          f"{sum(len(r.pins) for r in rbb_types)} total pins", file=sys.stderr)

    graph = RoutingGraph(
        num_nodes=num_nodes,
        num_edges=num_edges,
        num_routing_nodes=len(routing_nodes),
        num_sitepin_nodes=len(sitepin_nodes),
        routing_nodes=routing_nodes,
        sitepin_nodes=sitepin_nodes,
        out_edges=out_edges,
        in_edges=in_edges,
        rbb_types=rbb_types,
    )

    # Build derived lookups
    _build_lookups(graph)

    return graph


def _build_lookups(graph):
    """Build derived lookup tables from parsed data."""
    # grid_to_routing: (x,y) -> {sm_name: node_id}
    for nid, node in graph.routing_nodes.items():
        key = (node.grid_x, node.grid_y)
        if key not in graph.grid_to_routing:
            graph.grid_to_routing[key] = {}
        graph.grid_to_routing[key][node.sm_name] = nid

    # site_to_pins: (sx,sy,type) -> {pin_idx: node_id}
    for nid, node in graph.sitepin_nodes.items():
        key = (node.site_x, node.site_y, node.site_type)
        if key not in graph.site_to_pins:
            graph.site_to_pins[key] = {}
        graph.site_to_pins[key][node.pin_index] = nid


def print_summary(graph):
    """Print a human-readable summary of the routing graph."""
    print(f"=== EFLX Routing Graph Summary ===")
    print(f"Total nodes: {graph.num_nodes}")
    print(f"  Routing nodes: {graph.num_routing_nodes} (SM switches)")
    print(f"  Site pin nodes: {graph.num_sitepin_nodes} (CLB/IOB pins)")
    print(f"Total edges: {graph.num_edges}")
    print()

    # Grid statistics
    grid_points = set()
    for node in graph.routing_nodes.values():
        grid_points.add((node.grid_x, node.grid_y))
    xs = sorted(set(p[0] for p in grid_points))
    ys = sorted(set(p[1] for p in grid_points))
    print(f"Grid: {len(xs)}x{len(ys)} = {len(grid_points)} points")
    print(f"  X range: {min(xs)}..{max(xs)}")
    print(f"  Y range: {min(ys)}..{max(ys)}")
    print()

    # SM type distribution
    sm_counts = {}
    for node in graph.routing_nodes.values():
        sm_counts[node.sm_name] = sm_counts.get(node.sm_name, 0) + 1
    print(f"SM types: {len(sm_counts)}")
    for name in sorted(sm_counts.keys()):
        print(f"  {name}: {sm_counts[name]} instances")
    print()

    # Site type distribution
    site_types = {}
    for node in graph.sitepin_nodes.values():
        lt = f"L{node.site_type}"
        site_types[lt] = site_types.get(lt, 0) + 1
    print(f"Site pin types:")
    for lt in sorted(site_types.keys()):
        print(f"  {lt}: {site_types[lt]} pins")
    print()

    # Sites per type
    sites_by_type = {}
    for key in graph.site_to_pins:
        sx, sy, st = key
        lt = f"L{st}"
        sites_by_type.setdefault(lt, set()).add((sx, sy))
    for lt in sorted(sites_by_type.keys()):
        sites = sites_by_type[lt]
        print(f"  {lt} sites: {len(sites)}")
    print()

    # Edge statistics
    out_degree = {}
    for src, edges in graph.out_edges.items():
        out_degree[src] = len(edges)
    in_degree = {}
    for dst, edges in graph.in_edges.items():
        in_degree[dst] = len(edges)

    if out_degree:
        avg_out = sum(out_degree.values()) / len(out_degree)
        max_out = max(out_degree.values())
        print(f"Out-degree: avg={avg_out:.1f}, max={max_out}")
    if in_degree:
        avg_in = sum(in_degree.values()) / len(in_degree)
        max_in = max(in_degree.values())
        print(f"In-degree:  avg={avg_in:.1f}, max={max_in}")
    print()

    # Connectivity between routing and site-pin nodes
    routing_to_site = 0
    site_to_routing = 0
    routing_to_routing = 0
    sp_start = min(graph.sitepin_nodes.keys()) if graph.sitepin_nodes else graph.num_nodes

    for src, edges in graph.out_edges.items():
        for dst, cost in edges:
            src_is_routing = src < sp_start
            dst_is_routing = dst < sp_start
            if src_is_routing and dst_is_routing:
                routing_to_routing += 1
            elif src_is_routing and not dst_is_routing:
                routing_to_site += 1
            elif not src_is_routing and dst_is_routing:
                site_to_routing += 1

    print(f"Edge breakdown:")
    print(f"  Routing -> Routing: {routing_to_routing}")
    print(f"  Routing -> SitePin: {routing_to_site}")
    print(f"  SitePin -> Routing: {site_to_routing}")
    print()

    # RBB types
    if graph.rbb_types:
        print(f"RBB types ({len(graph.rbb_types)}):")
        for rbb in graph.rbb_types:
            kinds = {}
            for p in rbb.pins:
                kinds[p.kind] = kinds.get(p.kind, 0) + 1
            kind_str = ", ".join(f"{k}={v}" for k, v in sorted(kinds.items()))
            print(f"  {rbb.name} ({rbb.cell_type}): {len(rbb.pins)} pins ({kind_str})")
    print()

    # Pin name mapping for RBB_6L (CLB)
    for rbb in graph.rbb_types:
        if rbb.cell_type == "RBB_6L":
            print(f"Pin mapping for {rbb.name} ({rbb.cell_type}):")
            by_kind = {}
            for p in rbb.pins:
                by_kind.setdefault(p.kind, []).append(p.pin_num)
            for kind in sorted(by_kind.keys()):
                pins = sorted(by_kind[kind])
                kind_name = {"F": "FF", "T": "Through", "O": "Output", "I": "Input"}
                print(f"  {kind_name.get(kind, kind)} ({kind}): "
                      f"pins {min(pins)}-{max(pins)} ({len(pins)} pins)")
            break


def export_json(graph, path):
    """Export graph as JSON for use by other tools."""
    data = {
        "num_nodes": graph.num_nodes,
        "num_edges": graph.num_edges,
        "num_routing_nodes": graph.num_routing_nodes,
        "num_sitepin_nodes": graph.num_sitepin_nodes,
        "routing_nodes": {
            str(nid): {
                "grid_x": n.grid_x,
                "grid_y": n.grid_y,
                "sm_name": n.sm_name,
            }
            for nid, n in graph.routing_nodes.items()
        },
        "sitepin_nodes": {
            str(nid): {
                "grid_x": n.grid_x,
                "grid_y": n.grid_y,
                "site_x": n.site_x,
                "site_y": n.site_y,
                "site_type": n.site_type,
                "pin_index": n.pin_index,
            }
            for nid, n in graph.sitepin_nodes.items()
        },
        "edges": [
            {"src": src, "dst": dst, "cost": cost}
            for src, edges in sorted(graph.out_edges.items())
            for dst, cost in edges
        ],
        "rbb_types": [
            {
                "name": r.name,
                "cell_type": r.cell_type,
                "pins": [
                    {"kind": p.kind, "rbb_type": p.rbb_type, "pin_num": p.pin_num}
                    for p in r.pins
                ],
            }
            for r in graph.rbb_types
        ],
    }
    with open(path, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    print(f"Exported {path} ({os.path.getsize(path) / 1024 / 1024:.1f} MB)",
          file=sys.stderr)


def cmd_summary(args):
    graph = build_graph(args.arch_dir)
    print_summary(graph)


def cmd_export(args):
    graph = build_graph(args.arch_dir)
    export_json(graph, args.output)


def cmd_node(args):
    """Show details about a specific node."""
    graph = build_graph(args.arch_dir)
    nid = args.node_id
    sp_start = min(graph.sitepin_nodes.keys()) if graph.sitepin_nodes else graph.num_nodes

    if nid in graph.routing_nodes:
        n = graph.routing_nodes[nid]
        print(f"Node {nid}: Routing node")
        print(f"  Grid: ({n.grid_x}, {n.grid_y})")
        print(f"  SM: {n.sm_name}")
    elif nid in graph.sitepin_nodes:
        n = graph.sitepin_nodes[nid]
        print(f"Node {nid}: Site pin node")
        print(f"  Grid: ({n.grid_x}, {n.grid_y})")
        print(f"  Site: ({n.site_x}, {n.site_y}) L{n.site_type}")
        print(f"  Pin: {n.pin_index}")
    else:
        print(f"Node {nid}: unknown (no metadata)")

    out = graph.out_edges.get(nid, [])
    inp = graph.in_edges.get(nid, [])
    print(f"  Out-edges ({len(out)}):")
    for dst, cost in sorted(out):
        label = _node_label(graph, dst, sp_start)
        print(f"    -> {dst} (cost={cost}) {label}")
    print(f"  In-edges ({len(inp)}):")
    for src, cost in sorted(inp):
        label = _node_label(graph, src, sp_start)
        print(f"    <- {src} (cost={cost}) {label}")


def cmd_site(args):
    """Show all pins for a site."""
    graph = build_graph(args.arch_dir)
    sp_start = min(graph.sitepin_nodes.keys()) if graph.sitepin_nodes else graph.num_nodes

    # Parse site spec like "0,0L0"
    m = re.match(r'(\d+),(\d+)L(\d+)', args.site)
    if not m:
        print(f"Bad site spec: {args.site!r} (expected X,YLT e.g. 0,0L0)")
        sys.exit(1)
    sx, sy, st = int(m.group(1)), int(m.group(2)), int(m.group(3))
    key = (sx, sy, st)
    pins = graph.site_to_pins.get(key)
    if not pins:
        print(f"No pins found for site ({sx},{sy})L{st}")
        sys.exit(1)

    print(f"Site ({sx},{sy})L{st}: {len(pins)} pins")
    for pin_idx in sorted(pins.keys()):
        nid = pins[pin_idx]
        node = graph.sitepin_nodes[nid]
        out = graph.out_edges.get(nid, [])
        inp = graph.in_edges.get(nid, [])
        direction = "OUT" if out and not inp else "IN" if inp and not out else "BIDIR"
        print(f"  pin {pin_idx:2d}: node {nid}, grid ({node.grid_x},{node.grid_y}), "
              f"{direction}, {len(out)} out, {len(inp)} in")


def cmd_path(args):
    """Find shortest path between two nodes using BFS (unweighted) or Dijkstra."""
    graph = build_graph(args.arch_dir)
    sp_start = min(graph.sitepin_nodes.keys()) if graph.sitepin_nodes else graph.num_nodes

    import heapq
    src, dst = args.src, args.dst

    # Dijkstra
    dist = {src: 0}
    prev = {}
    heap = [(0, src)]

    while heap:
        d, u = heapq.heappop(heap)
        if u == dst:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v, cost in graph.out_edges.get(u, []):
            nd = d + max(cost, 1)  # treat negative/zero costs as 1
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    if dst not in prev and src != dst:
        print(f"No path from {src} to {dst}")
        return

    # Reconstruct
    path = []
    u = dst
    while u != src:
        path.append(u)
        u = prev[u]
    path.append(src)
    path.reverse()

    print(f"Path from {src} to {dst}: {len(path)} nodes, cost={dist[dst]}")
    for nid in path:
        label = _node_label(graph, nid, sp_start)
        print(f"  {nid} {label}")


def cmd_layout(args):
    """Show ASCII chip floorplan."""
    graph = build_graph(args.arch_dir)

    # Build grid contents
    grid = {}  # (gx, gy) -> label
    for nid, n in graph.sitepin_nodes.items():
        key = (n.site_x, n.site_y, n.site_type)
        gx, gy = n.grid_x, n.grid_y
        if n.site_type == 0:
            grid[(gx, gy)] = f"C"  # CLB
        elif n.site_type == 1:
            grid[(gx, gy)] = f"M"  # Memory
        elif n.site_type == 6:
            grid[(gx, gy)] = f"I"  # IOB

    print("Chip floorplan (32x32 routing grid):")
    print("  C=CLB, M=Memory CLB, I=IOB, .=routing only")
    print()
    print("    " + "".join(f"{x:2d}" for x in range(32)))
    for y in range(32):
        row = f"{y:2d}  "
        for x in range(32):
            ch = grid.get((x, y), ".")
            row += f" {ch}"
        print(row)
    print()

    # Show CLB coordinate mapping
    print("CLB (L0) placement mapping:")
    print("  Placer (X,Y) → Grid (gx,gy)")
    for sy in range(10):
        for sx in range(14):
            key = (sx, sy, 0)
            pins = graph.site_to_pins.get(key, {})
            if pins and 0 in pins:
                n = graph.sitepin_nodes[pins[0]]
                print(f"  CLB({sx:2d},{sy}): grid ({n.grid_x:2d},{n.grid_y:2d})")
        if sy < 9:
            print()

    # SM hierarchy analysis
    print()
    print("SM hierarchy (routing node types):")
    print("  U/D = Up/Down direction")
    print("  Level 0-1: local (within CLB pair)")
    print("  Level 3-5: mid-range")
    print("  Level 13: long-range (chip-spanning)")
    print()

    # Analyze SM connectivity at one grid point
    gx, gy = 16, 16  # center of chip
    sms = graph.grid_to_routing.get((gx, gy), {})
    print(f"SM connectivity at grid ({gx},{gy}):")
    for sm_name in sorted(sms.keys()):
        nid = sms[sm_name]
        out = graph.out_edges.get(nid, [])
        inp = graph.in_edges.get(nid, [])
        out_grids = set()
        for dst, _ in out:
            if dst in graph.routing_nodes:
                n = graph.routing_nodes[dst]
                out_grids.add((n.grid_x, n.grid_y))
        in_grids = set()
        for src, _ in inp:
            if src in graph.routing_nodes:
                n = graph.routing_nodes[src]
                in_grids.add((n.grid_x, n.grid_y))
        # Compute reach
        out_reach = max((abs(g[0]-gx) + abs(g[1]-gy) for g in out_grids), default=0)
        in_reach = max((abs(g[0]-gx) + abs(g[1]-gy) for g in in_grids), default=0)
        print(f"  {sm_name:10s}: {len(out):2d} out (reach {out_reach:2d}), "
              f"{len(inp):2d} in (reach {in_reach:2d})")


def _node_label(graph, nid, sp_start):
    """Short human-readable label for a node."""
    if nid in graph.routing_nodes:
        n = graph.routing_nodes[nid]
        return f"[routing ({n.grid_x},{n.grid_y}) {n.sm_name}]"
    elif nid in graph.sitepin_nodes:
        n = graph.sitepin_nodes[nid]
        return f"[sitepin ({n.grid_x},{n.grid_y}) site=({n.site_x},{n.site_y})L{n.site_type} pin={n.pin_index}]"
    return "[unknown]"


def main():
    parser = argparse.ArgumentParser(description="EFLX Routing Graph Tool")
    parser.add_argument("--arch-dir", default=ARCH_DIR,
                        help="Directory containing decrypted arch files")

    sub = parser.add_subparsers(dest="command")

    sub.add_parser("summary", help="Print routing graph summary")

    p_export = sub.add_parser("export", help="Export graph as JSON")
    p_export.add_argument("-o", "--output", default="routing_graph.json",
                          help="Output JSON path")

    p_node = sub.add_parser("node", help="Show node details")
    p_node.add_argument("node_id", type=int, help="Node ID to inspect")

    p_site = sub.add_parser("site", help="Show site pin details")
    p_site.add_argument("site", help="Site spec (e.g. 0,0L0)")

    p_path = sub.add_parser("path", help="Find shortest path between nodes")
    p_path.add_argument("src", type=int, help="Source node ID")
    p_path.add_argument("dst", type=int, help="Destination node ID")

    sub.add_parser("layout", help="Show chip floorplan and CLB mapping")

    args = parser.parse_args()

    commands = {
        "summary": cmd_summary,
        "export": cmd_export,
        "node": cmd_node,
        "site": cmd_site,
        "path": cmd_path,
        "layout": cmd_layout,
    }

    if args.command in commands:
        commands[args.command](args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
