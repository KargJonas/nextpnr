"""Routing-graph loader: the vendor's chipdb as a directed, weighted graph.

Loads the verbatim minplacer dumps (port/db/graph/, decrypted) into one graph:

    minplacer.rgraph_def         edges  (source->dst, weight=delay)
    minplacer.rnode2sm_def       node -> grid (x,y) + switch-matrix mux name
    minplacer.rnode2sitepin_def  node -> grid (x,y) + CLB/IOB site pin
    minplacer.chip_def           array geometry + fixed-IO list

Node-ID scheme (verified against the dumps): the 48544 routing-node IDs are one
flat space -- IDs [0, 40960) are SM nodes (line i of rnode2sm_def is node i), IDs
[40960, 48544) are site-pin nodes (line j of rnode2sitepin_def is node 40960+j).
A graph edge u->v means "v's mux can select u as its driver"; routing a net over
u->v programs the mux at v to pick input u. Edge weight is the vendor's delay.

rgraph_def line grammar (recovered empirically; 310016 edge lines):
    "+N>dst=w"   set current source node := N (absolute), add edge N->dst (w)
    "++dst=w"    current source := current source + 1, add edge src->dst (w)
    "dst=w"      add another edge from the current source: src->dst (w)

The SM mux name "U0B0N0" parses as direction U/D, level 0, branch 0, node-index
0 -- the same (level, direction, branch, node_index) key sm_routing.SM_BL_WL and
compute_select use, so a routed node maps straight to its select-bit encoding.
"""

import os
import re
from dataclasses import dataclass, field

_SM_NAME = re.compile(r"^([UD])(\d+)B(\d+)N(\d+)$")


def _graph_dir():
    from bitgen.database import GRAPH
    return str(GRAPH)


@dataclass
class SMNode:
    """An SM-mux routing node: grid (x,y) + (level, direction, branch, index)."""
    nid: int
    x: int
    y: int
    direction: str       # 'U' or 'D'
    level: int
    branch: int
    index: int

    @property
    def sm_name(self):
        return f"{self.direction}{self.level}B{self.branch}N{self.index}"

    @property
    def key(self):
        """(level, direction, branch, node_index) -- the sm_routing.SM_BL_WL key."""
        return (self.level, self.direction, self.branch, self.index)


@dataclass
class PinNode:
    """A CLB/IOB site-pin routing node: grid (x,y) + raw site-pin token."""
    nid: int
    x: int
    y: int
    site: str            # e.g. "0,0L0"
    pin: int             # the pin index


@dataclass
class RGraph:
    nodes: list = field(default_factory=list)         # nid -> SMNode | PinNode
    adj: dict = field(default_factory=dict)           # nid -> [(dst_nid, weight), ...]
    radj: dict = field(default_factory=dict)          # dst_nid -> [(src_nid, weight), ...]
    cols: int = 0
    rows: int = 0
    fixed_io: list = field(default_factory=list)      # chip_def FIXEDIO entries
    _n_sm: int = 0

    def is_sm(self, nid):
        return nid < self._n_sm

    def sm_at(self, x, y):
        """All SM nodes at grid (x,y) keyed by (level,dir,branch,index)."""
        return {n.key: n for n in self.nodes[:self._n_sm]
                if n.x == x and n.y == y}

    def pins_at(self, x, y):
        return [n for n in self.nodes[self._n_sm:] if n.x == x and n.y == y]


def _parse_sm_line(nid, line):
    xy, name = line.split(None, 1)
    x, y = (int(v) for v in xy.split(","))
    m = _SM_NAME.match(name.strip())
    if not m:
        raise ValueError(f"bad SM name {name!r} at node {nid}")
    d, lvl, br, idx = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
    return SMNode(nid, x, y, d, lvl, br, idx)


def _parse_pin_line(nid, line):
    parts = line.split()
    x, y = (int(v) for v in parts[0].split(","))
    site = parts[1]
    pin = int(parts[2]) if len(parts) > 2 else -1
    return PinNode(nid, x, y, site, pin)


_LOAD_CACHE = {}


def load(graph_dir=None):
    """Load the full routing graph from the vendor dumps.

    Memoized by graph_dir (the dumps are constant pure artifacts) so repeated
    callers -- e.g. the post move loop scoring many designs -- share one parse."""
    gd = graph_dir or _graph_dir()
    if gd in _LOAD_CACHE:
        return _LOAD_CACHE[gd]

    sm_lines = open(os.path.join(gd, "minplacer.rnode2sm_def")).read().splitlines()
    pin_lines = open(os.path.join(gd, "minplacer.rnode2sitepin_def")).read().splitlines()
    n_sm = len(sm_lines)

    nodes = [None] * (n_sm + len(pin_lines))
    for i, ln in enumerate(sm_lines):
        nodes[i] = _parse_sm_line(i, ln)
    for j, ln in enumerate(pin_lines):
        nodes[n_sm + j] = _parse_pin_line(n_sm + j, ln)

    adj, radj = {}, {}
    src = -1
    with open(os.path.join(gd, "minplacer.rgraph_def")) as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            if line.startswith("++"):
                src += 1
                body = line[2:]
            elif line.startswith("+"):
                head, body = line[1:].split(">", 1)
                src = int(head)
            else:
                body = line
            dst_s, w_s = body.split("=", 1)
            dst, w = int(dst_s), int(w_s)
            adj.setdefault(src, []).append((dst, w))
            radj.setdefault(dst, []).append((src, w))

    g = RGraph(nodes=nodes, adj=adj, radj=radj, _n_sm=n_sm)

    # chip_def: "32x32,1,1;,M;" then FIXEDIO=layer,?,x,y lines.
    chip = open(os.path.join(gd, "minplacer.chip_def")).read().splitlines()
    if chip:
        m = re.match(r"(\d+)x(\d+)", chip[0])
        if m:
            g.cols, g.rows = int(m.group(1)), int(m.group(2))
    for ln in chip[1:]:
        if ln.startswith("FIXEDIO="):
            g.fixed_io.append(tuple(int(v) for v in ln[len("FIXEDIO="):].split(",")))
    _LOAD_CACHE[gd] = g
    return g
