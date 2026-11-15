"""Pack/place slice-output model (the (0,'U') family) -- foundation.

(0,'U',branch) is NOT an SM routing decision: it is the placed cell's slice
OUTPUT-pin assignment, chosen per-output by criticality (decomp FUN @ 461212-257
picks, for each output position, the less-critical of two candidate ports via an
Anet comparison; the bit is then formula(inst_type, port) @ 461398-414). So the
value couples packing + placement + timing + geometry.

This module builds the GEOMETRY foundation that model needs:
  * resolve_site() -- a placed cell (.vm col/row) -> its rnode2sitepin site,
  * output_groups() -- the site's (0,'U') output tiles as 4-branch groups,
  * output_nodes() -- the (0,'U',branch) SM nodes a placed cell drives.

The value layer (the Anet-driven port selection + formula) is built on top of
this; see eda_placer.criticality for the Anet kernel it will consume.
"""

import os
import re

from bitgen.database import GRAPH

_R2SP = str(GRAPH / "minplacer.rnode2sitepin_def")
_LINE = re.compile(r"(\d+),(\d+) (\d+),(\d+)L(\d+) (\d+)")


# The slice's 12 output ports (port/db/library/placetiles.txt SITETYPE L), in the
# port-array order the selection indexes (decomp 461214-221): index 0..11.
OUTPUT_PORTS = ["AO", "AMUX", "AQ", "BO", "BMUX", "BQ",
                "CO", "CMUX", "CQ", "DO", "DMUX", "DQ"]

# Per output position, the two candidate ports the criticality selection chooses
# between (461214-221: (iVar45*3, +2) for pos<4; (1,7)/(4,10) for pos 4,5). The
# less-critical (lower Anet) port wins; the bit is then formula(inst_type, port).
SELECT_PAIRS = [(0, 2), (3, 5), (6, 8), (9, 11), (1, 7), (4, 10)]


def formula_bit_27_14(port):
    """(0,'U') bit for a chosen output port, inst_type 27/14 (decomp 461400-405):
    1 for the registered/mux outputs {AQ,BQ,CQ,DQ,CMUX,DMUX}, 0 for the
    combinational/early-mux {AO,BO,CO,DO,AMUX,BMUX}. I.e. the bit records whether
    the slice output uses its registered (or C/D-mux) path."""
    return 1 if (port % 3 == 2 or port in (7, 10)) else 0


# --- (0,'U') value oracle: the vendor's own per-output write log -------------
# The (cell, port) -> output-tile geometry is the big FUN_0068d5b0 switch and the
# per-position port SELECTION is an Anet (criticality) comparison (decomp
# 461244-249) -- a full generative model needs both, plus the timing model. As an
# intermediate (the same tier route_oracle reads PNR_ROUTE.log), we read the
# placer's -debug_mode 2 "RBB output" log: it prints, per slice output, the chosen
# port and the (x,y,branch) level-0 SM tile it writes (decomp fprintf @ 461466).
# Each (0,'U') tile is written exactly once (verified: 4096 writes = 4096 tiles),
# so the log is a 1:1 (tile -> port) map. The select BIT is then the decompiled
# formula formula_bit_27_14(port) -- verified to reproduce the written value.
_DEBUG = "pnr_debug.log"
_DBG_RX = re.compile(
    r"RBB output \S+ \[\d+ \d+\] port (\d+).*?"
    r"Input x=(\d+) y=(\d+) level=(\d+) branch=(\d+) index=(\d+) downward=(\d+)")


def _debug_path(design):
    p = os.path.join(_ROOT, "vendor", "reference", design, _DEBUG)
    return p if os.path.exists(p) else None


def output_ports(design):
    """{(x,y,branch): port} for the level-0 upward ((0,'U')) SM output writes,
    parsed from the vendor's -debug_mode 2 log (decomp FUN @ 461466). The placer
    writes each (0,'U') tile exactly once, so this is a 1:1 tile->port map."""
    path = _debug_path(design)
    res = {}
    if path is None:
        return res
    for ln in open(path):
        m = _DBG_RX.search(ln)
        if not m:
            continue
        port, x, y, lvl, br, idx, down = (int(v) for v in m.groups())
        if lvl == 0 and down == 0 and idx == 0:
            res[(x, y, br)] = port           # 1:1; last would win if not
    return res


def output_polarity(design):
    """{(x,y,branch): bit} -- the (0,'U') select value the placer wrote at each
    logic slice output tile, = formula_bit_27_14(port) (decomp 461400-405). Keyed
    by SM tile so it drops straight into the (0,'U') emit/validate path."""
    return {k: formula_bit_27_14(p) for k, p in output_ports(design).items()}


def cell_site(col, row):
    """Placed-cell (.vm col,row) -> rnode2sitepin site (sx,sy). Derived from the
    corpus: logic columns are even and site sx = col//2 - 1, sy = row - 1
    (verified: col24->11, col26->12, col6->2; row1->0)."""
    return (col // 2 - 1, row - 1)


def _site_pins(site, level=0):
    """[(pin, x, y)] in file order for a site at the given SM level."""
    sx, sy = site
    pins = []
    with open(_R2SP) as f:
        for ln in f:
            m = _LINE.match(ln)
            if not m:
                continue
            rx, ry, ssx, ssy, lvl, pin = (int(v) for v in m.groups())
            if (ssx, ssy) == (sx, sy) and lvl == level:
                pins.append((pin, rx, ry))
    return pins


def output_groups(site):
    """The site's (0,'U') OUTPUT tiles as ordered 4-branch groups: a list of
    (tile, [pin0,pin1,pin2,pin3]) where 4 consecutive same-tile pins are the
    four branches of that output tile (the cell's slice outputs)."""
    pins = _site_pins(site, 0)
    groups, i = [], 0
    while i < len(pins):
        p0, x, y = pins[i]
        run = [p0]
        j = i + 1
        while j < len(pins) and (pins[j][1], pins[j][2]) == (x, y) and len(run) < 4:
            run.append(pins[j][0])
            j += 1
        if len(run) == 4:                       # a 4-branch output tile
            groups.append(((x, y), run))
        i = j
    return groups

