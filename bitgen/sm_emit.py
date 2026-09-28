"""Generative SM emission: detailed route -> bitstream SM bits.

The validation rules (sm_validate) recover each reprogrammed mux's source and
check its value against the vendor bitstream. This module turns that around: given
the route it PRODUCES the bits -- for every routed mux it computes
the select value from its routed source via the sm_writer encoders and writes the
SM_BL_WL cells into a device-template image. That image's SM bits are the
generative router's routing output; comparing them to the vendor bitstream
measures how much of the SM bitstream we can synthesize (vs merely validate).

value_for() is the deterministic core: (mux key, routed source key) -> value,
with no predecessor search (the route gives the source). It is the single place
the encoder rules are applied generatively.
"""

import os

from . import (sm_routing, sm_writer, bitstream, coords, rgraph,
               slice_output)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TMPL = os.path.join(_ROOT, "device", "template.bin")
_G = None       # cached rgraph (load once)


def _sm_name(key):
    lvl, d, br, idx = key
    return f"{d}{lvl}B{br}N{idx}"


def value_for(key, src, timed, tval=0, has_1u=False, active=True):
    """Select value for mux `key` given its routed source `src` (both
    (level, dir, branch, index)), or None if no rule applies.

    `timed` selects the (4,'D') stage-1 argmin outcome; `tval` is the template
    default (single-stage needs it); `has_1u` flags a (1,'U') routed at the tile
    (the (4,'D') sibling neutral pair); `active` flags a (1,'D')<-(1,'U') mux that
    delivers downward (stage2 enabled) -- bound from the route by the caller.
    Mirrors sm_validate's rule dispatch but driven by the known source.
    """
    level, direction, branch, ni = key
    if src is None:
        return None
    s_lvl, s_dir, s_br, s_idx = src
    s_down = 1 if s_dir == "D" else 0

    if level == 5 and direction == "U":
        return sm_writer.level5_up_value(s_lvl, s_br, s_idx)
    if level == 0 and direction == "D":
        return sm_writer.level0_down_value(s_lvl, s_br, s_idx, s_down)
    if level == 5 and direction == "D":
        return sm_writer.level5_down(branch, s_lvl, s_br, s_idx, s_down)
    if level == 3 and direction == "D" and s_lvl == 5 and s_dir == "D":
        return sm_writer.level3_down(branch, s_lvl, s_br, s_idx, s_down)
    if level == 1 and direction == "D":
        if s_lvl == 1 and s_dir == "U":
            # active (delivers down, stage2 enabled) vs default (source-only) --
            # bound from the route: `active` is set when the mux forwards the net
            # downward to a (0,'D') slice input (computed in emit from the hops).
            return sm_writer.level1_down(s_br, active)
        if s_lvl == 4:
            return sm_writer.level1_down_src4(s_br, s_down)
        if s_lvl == 5 and s_dir == "D":
            return sm_writer.level1_down_src5(s_br, s_idx)
        if s_lvl == 3:
            return sm_writer.level1_down_src3(s_br, s_down, timed)
        return None
    if level == 4 and direction == "D":
        r = sm_writer.level4_down(s_lvl, s_down, s_br, s_idx, branch,
                                  crit_of_edge=(lambda e: 0.0))
        if r is None:
            return None
        s1, s2 = r
        if s_lvl == 5 and s_dir == "D" and s_br > 0:
            s1 = (s_br - 1) if timed else 0
        return sm_writer.level4_down_value(branch, s1, s2)
    # single-stage (1,'U')/(3,'U')/(4,'U')/(13,'U'): geometry-grounded compute
    return sm_routing.compute_select(level, direction, branch, ni, _sm_name(src),
                                     tval)


def resolve_source(g, tile, key, net_set, dest=None, mux2net=None, net=None):
    """A mux's value source = its graph predecessor on the net's route (the
    descriptor source FUN_00573470 carries). Preference order: the route FEEDER
    (a pred whose hop delivers to this tile, e.g. the (1,'D') feeding a (0,'D')),
    then the same-tile delivered node (e.g. the up-root delivered for (5,'D')),
    then the first. When several feeders deliver to the tile, prefer the one that
    is a routed hop on THIS net (mux2net) -- two (1,'D',0,*) feeders can sit at one
    tile but only this net's own hop is the value source (multilut6 intra-tile
    (0,'D') fan-out: pick (1,'D',0,1) on the net, not (1,'D',0,0) from another).
    Returns the source key or None."""
    node = g.sm_at(*tile).get(key)
    if node is None:
        return None
    preds = [g.nodes[p] for p, _ in g.radj.get(node.nid, [])
             if g.is_sm(p) and p in net_set]
    if not preds:
        return None

    def on_net(p):
        return mux2net is not None and mux2net.get(((p.x, p.y), p.key)) == net

    if dest is not None:
        feeders = [p for p in preds if dest.get(((p.x, p.y), p.key)) == tile]
        if feeders:
            return next((p for p in feeders if on_net(p)), feeders[0]).key
    same = [p for p in preds if (p.x, p.y) == tile]
    if same:
        return next((p for p in same if on_net(p)), same[0]).key
    return preds[0].key


def _selval(words, x, y, key):
    v = 0
    for i, (bl, wl) in enumerate(sm_routing.SM_BL_WL[key]):
        wi, bit = coords.trace_to_word_bit(x, y, bl, wl)
        v |= ((words[wi] >> bit) & 1) << i
    return v


def _write_value(img, x, y, key, val):
    for i, (bl, wl) in enumerate(sm_routing.SM_BL_WL[key]):
        img.set_trace_bit(x, y, bl, wl, (val >> i) & 1)


# Generative full-coverage emitter (route -> ALL reprogrammed SM cells).
# emit() above scores the route-hop muxes only. emit_design() is the complete
# writer: it reproduces the vendor's whole SM write pass (verified against the
# single_ff set_bit trace, vendor/reference/single_ff/bw_trace.log) from the
# route + placement alone -- no reference, no debug log:
#
#   1. ROUTE writes -- one value per routed mux (the trace's leading 4 writes
#      per net: exit (0,'U'), up/turn hops, delivery (0,'D')), via value_for.
#   2. FILL recomputation -- the vendor's zero-fill walks the whole device; its
#      value differs from the device template only at DESIGN-AFFECTED tiles:
#        * (1,'U') muxes at a tile with bound slice outputs are cleared to 0
#          (all but the routed one; trace: (24,3) (1,'U',{0,2,3}) -> 0).
#        * a routed (1,'D',0,N1) down-turn co-writes its two-stage apparatus
#          ((3,'U',1/4), (4,'U',2/5), (4,'D',2/5), (5,'U',0,N0/N1)): ENABLED
#          when the turn's routed source comes off the long lines (level >= 3,
#          e.g. the clock corner's (5,'D') arrival), BYPASSED/disabled when the
#          source is level 1 (trace: (25,3) disabled, (0,31) enabled). The
#          state discriminator is the graph structure of the routed source.
#   3. The CLOCK-CORNER SM net -- the always-present clk delivery's fabric
#      block at (0,31)/(1,31). Its 14 nodes/values are corpus-constant (all 42
#      paired references; the same tier as clock_route.CLOCK_CORNER_DELTA) and
#      are emitted through the same turn-apparatus rules (the corner is an
#      ENABLED stage-2 delivery sourced from D5B1).
#   4. (0,'U') slice-pin family: routed input-binding parity (FUN @ 615884,
#      uVar13 % 2 == 1-(y%2)), input-IOB tiles = 1, and the output-pin
#      assignment cleanup (formula_bit_27_14 over the assigned port).

# The clock-corner SM net: the constant fabric tail of the clk delivery at the
# NW corner. Hop shape (from the rgraph + the corpus-constant block): a (5,'D')
# long-line arrival at (0,31) turning down (1,'D',0,1) into the (0,'D',0) pin
# delivery, with the (1,'U',0) up-bind at (0,31) and the (0,'U') IOB binds at
# (1,31). Values are computed by the same rules as any routed net.
CLOCK_CORNER_TILE = (0, 31)
CLOCK_CORNER_OUT_TILE = (1, 31)


def _corner_writes(g):
    """[(tile, key, value)] for the constant clock-corner SM net.

    The corner is an ENABLED two-stage delivery (source = the D5B1 long line):
    the turn + delivery + apparatus values all come from the transcribed rule
    functions over the graph sources; the (0,'U') binds at (1,31) are the IOB
    output-assignment formula over the AO default (formula_bit_27_14(0) == 0).
    Validated: this exact 14-node/value block appears in every paired reference.
    """
    t = CLOCK_CORNER_TILE
    src5 = (5, "D", 1, 0)                       # the D5B1 long-line arrival
    out = []
    # delivery pair: (1,'D',0,1) <- D5B1 (level1_down_src5), (0,'D',0) <- D1B1.
    out.append((t, (1, "D", 0, 1), sm_writer.level1_down_src5(1, 0)))
    out.append((t, (0, "D", 0, 0), sm_writer.level0_down_value(1, 1, 0, 1)))
    # the up-bind at the corner tile (single-stage from the U0B0 pin mux).
    out.append((t, (1, "U", 0, 0), 3))
    # the ENABLED two-stage apparatus (source level >= 3).
    out += _apparatus_writes(t, enabled=True, src=src5)
    # IOB (0,'U') binds at (1,31): assignment formula over the default AO port.
    for br in (0, 1, 3):
        out.append((CLOCK_CORNER_OUT_TILE, (0, "U", br, 0),
                    slice_output.formula_bit_27_14(0)))
    return out


def _apparatus_writes(tile, enabled, src):
    """The two-stage apparatus co-written with a (1,'D',0,N1) down-turn.

    Group members (trace + corpus): (3,'U',{1,4}), (4,'U',{2,5}), (4,'D',{2,5}),
    (5,'U',0,{0,1}). ENABLED (long-line source, level >= 3): the 'U' members
    carry the select 3 and the (5,'U') root the delivered-source value
    (level5_up_value over the source); DISABLED (level-1 source bypasses the
    apparatus): the 'U' members are cleared to 0 and the root reverts to the
    U1B0 default. The (4,'D') siblings take the level-4 encoder with the
    neutral (s1,s2) of their state ((2,4) enabled-with-1U / (0,0) cleared).
    """
    on = 3 if enabled else 0
    out = []
    for br in (1, 4):
        out.append((tile, (3, "U", br, 0), on))
    for br in (2, 5):
        out.append((tile, (4, "U", br, 0), on))
    # (4,'D') stage encodings of the apparatus state (the level-4 encoder over
    # the state's (s1,s2): enabled (0,3), bypassed (1,4) -- single_ff trace +
    # the clock corner, falsified corpus-wide by the emit==reference check).
    s1, s2 = (0, 3) if enabled else (1, 4)
    for br in (2, 5):
        out.append((tile, (4, "D", br, 0), sm_writer.level4_down_value(br, s1, s2)))
    # (5,'U') root select: the delivering U1B0 feed when enabled, the U1B1
    # default when bypassed (both via the transcribed level5_up encoder).
    root = (sm_writer.level5_up_value(1, 0, 0) if enabled
            else sm_writer.level5_up_value(1, 1, 0))
    for ni in (0, 1):
        out.append((tile, (5, "U", 0, ni), root))
    return out


def emit_design(g, nets, bound_ports, input_tiles, timed=False, image=None,
                driver_pin=None, exit_select=None):
    """Generative full-coverage SM emission: route + placement -> image.

    `nets` = {net_id: [hop dict(sx,sy,dx,dy,level,branch,index,j)]} -- the
    PORT'S OWN route (or the oracle's, for coverage validation). `bound_ports` =
    {(x,y,branch): port_index} the pack/output assignment bound on (0,'U')
    positions (generative counterpart of the -debug_mode 2 log). `input_tiles` =
    tiles holding an input IOB. `driver_pin` = {net_id: pin_index} -- which of
    the source tile's output pins drives the net (0 = the O/combinational pin,
    1 = the Q/registered pin): a routed (0,'U') exit's select value IS that pin
    index (the trace: q via DQ -> 1, the D-net via DO -> 0 at the same tile).
    `exit_select` = {(tile, key): value} overrides it per exit: the (0,'U') mux
    picks between the two site pins of its fan-in, so a net leaving one site
    through several pins needs a select per exit, not per net.
    Returns (image, n_written).

    No reference bitstream and no vendor debug log are read: every value is a
    rule function over the route/graph/placement, and the write SET is the
    route-touched + bound + corner neighborhoods (the fill recomputation only
    differs from the template there -- verified corpus-wide).
    """
    global _G
    if g is None:
        if _G is None:
            _G = rgraph.load()
        g = _G
    if image is None:
        image = bitstream.Image.from_template(_TMPL)
    tmpl_words = bitstream.Image.from_template(_TMPL).words

    used, dest, mux2net, net_nodes = {}, {}, {}, {}
    for nid, hops in nets.items():
        delivered = {}
        nodes = set()
        for h in hops:
            ft, tt = (h["sx"], h["sy"]), (h["dx"], h["dy"])
            key = (h["level"], "D" if h["j"] else "U", h["branch"], h["index"])
            used[(ft, key)] = delivered.get(ft)
            delivered[tt] = key
            dest[(ft, key)] = tt
            mux2net[(ft, key)] = nid
            m = g.sm_at(*ft).get(key)
            if m is not None:
                nodes.add(m.nid)
                for a, _ in g.adj.get(m.nid, []) + g.radj.get(m.nid, []):
                    if g.is_sm(a) and (g.nodes[a].x, g.nodes[a].y) == tt:
                        nodes.add(a)
        net_nodes[nid] = net_nodes.get(nid, set()) | nodes

    writes = {}                                  # (tile,key) -> value

    # -- 1. route writes (the per-hop values; value_for over the routed source).
    onetile_1u = {t for (t, k) in used if k[0] == 1 and k[1] == "U"}
    # Two distinct turn discriminators (single_ff trace at (25,3)/(24,1)):
    #  * VALUE-active (the level1_down 32x encodings): the turn delivers to a
    #    (0,'D') at its own tile -- both turns there are value-active (323/320).
    #  * BOUNCE (apparatus bypass): the net went UP and came DOWN at one tile
    #    (the delivering (1,'U') hop has source==dest==the turn tile). Only the
    #    (25,3) turn is a bounce; (24,1)'s (1,'U') arrives from another tile and
    #    leaves the apparatus at its default.
    active_l1d, bounce_l1d = set(), set()
    for nid, hops in nets.items():
        down_tiles = {(h["sx"], h["sy"]) for h in hops
                      if h["level"] == 0 and h["j"] == 1}
        bounce_tiles = {(h["sx"], h["sy"]) for h in hops
                        if h["level"] == 1 and h["j"] == 0
                        and (h["sx"], h["sy"]) == (h["dx"], h["dy"])}
        for h in hops:
            if h["level"] != 1 or h["j"] != 1:
                continue
            k = ((h["sx"], h["sy"]), (1, "D", h["branch"], h["index"]))
            if (h["sx"], h["sy"]) in down_tiles:
                active_l1d.add(k)
            if (h["sx"], h["sy"]) in bounce_tiles:
                bounce_l1d.add(k)
    for (tile, key), rsrc in used.items():
        x, y = tile
        if key not in sm_routing.SM_BL_WL:
            continue
        src = rsrc
        if (key[1] == "D" and key[0] in (0, 5) and rsrc is not None
                and rsrc[1] == "D" and rsrc[2] == 0 and rsrc[0] == key[0]):
            this_net = mux2net.get((tile, key))
            src = resolve_source(g, tile, key, net_nodes.get(this_net, set()),
                                 dest, mux2net, this_net) or rsrc
        if key[0] == 0 and key[1] == "U":
            if tile in input_tiles:
                # input-IOB exit: the select is the pad's driving-pin index,
                # column-parity coded (xor_reg trace: col1 -> 0, col2 -> 1).
                writes[(tile, key)] = 1 - (x % 2)
            else:
                # slice exit: the select picks the driving output pin (O=0, Q=1).
                pin = (exit_select or {}).get((tile, key))
                if pin is None:
                    pin = (driver_pin or {}).get(mux2net.get((tile, key)))
                writes[(tile, key)] = pin if pin is not None else 1 - (y % 2)
            continue
        # a routed (1,'D') turn fed by a (1,'U') is always the ACTIVE
        # level1_down encoding (32x family) -- single_ff/two_ff/xor_reg traces:
        # every such turn carries 32x regardless of where it delivers.
        v = value_for(key, src, timed, tval=_selval(tmpl_words, x, y, key),
                      has_1u=tile in onetile_1u,
                      active=(key[0] == 1 and key[1] == "D" and src is not None
                              and src[0] == 1 and src[1] == "U")
                             or (tile, key) in active_l1d)
        if v is not None:
            writes[(tile, key)] = v

    # -- 2. fill recomputation at affected tiles.
    # (1,'U') clears: at tiles where the net EXITS (routed (0,'U')) AND climbs
    # (routed (1,'U')), the unrouted (1,'U') branches are cleared to 0 (traces:
    # (24,3) and (2,0) cleared; the exit-only tile (1,0) and the transit tile
    # (25,3) keep their template 3).
    exit_tiles = {t for (t, k) in used if k[0] == 0 and k[1] == "U"}
    climb_tiles = {t for (t, k) in used if k[0] == 1 and k[1] == "U"}
    for (x, y) in exit_tiles & climb_tiles:
        for br in range(4):
            k = (1, "U", br, 0)
            if g.sm_at(x, y).get(k) is None or ((x, y), k) in used:
                continue
            writes.setdefault(((x, y), k), 0)
    # down-turn apparatus at routed (1,'D') turn tiles (any branch). Three
    # states (single_ff/xor_reg traces + the clock corner):
    #   * source off the long lines (level >= 3)        -> ENABLED write;
    #   * bounce turn (same-tile (1,'U') source)         -> BYPASS/disable write;
    #   * pass-through turn (source from another tile)   -> default values
    #     (== the device template) -> no write needed.
    for (tile, key), rsrc in used.items():
        if key[:2] != (1, "D"):
            continue
        if rsrc is not None and rsrc[0] >= 3:
            state = True
        elif (tile, key) in bounce_l1d:
            state = False
        else:
            continue
        for (t, k, v) in _apparatus_writes(tile, state, rsrc or (1, "U", 0, 0)):
            if ((t, k)) not in used:
                writes.setdefault((t, k), v)
    # climb apparatus: a tile where the net climbs onto the long lines via a
    # routed (5,'U') slot co-writes the same bypass group -- the (3,'U')/
    # (4,'U') pairs clear and the OTHER (5,'U') slot takes the 32 default
    # (xor_reg trace at (3,3); the routed slot keeps its route value via the
    # `in used` guard below).
    # (the (4,'D') members keep their template at climb tiles -- xor_reg (3,3))
    longline_tiles = {t for (t, k) in used if k[0] == 13}
    for (tile, key) in list(used):
        if key[0] == 5 and key[1] == "U" and tile in longline_tiles:
            for (t, k, v) in _apparatus_writes(tile, False, None):
                if (t, k) not in used and (k[0], k[1]) != (4, "D"):
                    writes.setdefault((t, k), v)
    # (0,'D') sibling rewrites at delivery tiles: the unrouted (0,'D') branches
    # are pointed at the PARTNER lane of the tile's arriving (1,'D') feeder
    # (corpus table: B0N0 -> B0N1, B1 -> B0N0, B2 -> B3, B3 -> B2; a B0N1
    # feeder leaves the template). The rewrite applies only when the feeder
    # turn is itself level-1-fed (a bounce/lateral): a long-line-fed feeder
    # leaves the siblings at the template (xor_reg trace at (24,1) vs the
    # rewritten and_reg (2,3)). With several feeders the lowest-branch applies.
    _PARTNER = {(0, 0): "D1B0N1", (1, 0): "D1B0N0",
                (2, 0): "D1B3N0", (3, 0): "D1B2N0"}
    deliv = {}
    for (tile, key), rsrc in used.items():
        if key[0] == 0 and key[1] == "D" and rsrc is not None \
                and rsrc[0] == 1 and rsrc[1] == "D":
            cur = deliv.get(tile)
            if cur is None or (rsrc[2], rsrc[3]) < (cur[2], cur[3]):
                deliv[tile] = rsrc
    feeder_src = {}                       # (1,'D') key -> its own routed source
    for (tile, key), rsrc in used.items():
        if key[0] == 1 and key[1] == "D":
            feeder_src.setdefault(key, rsrc)
    for (x, y), feeder in deliv.items():
        fsrc = feeder_src.get(feeder)
        if fsrc is not None and fsrc[0] >= 3:
            continue                                  # long-line-fed: template
        partner = _PARTNER.get((feeder[2], feeder[3]))
        if partner is None:
            continue                                  # B0N1 feeder: template
        for br in range(4):
            k = (0, "D", br, 0)
            if ((x, y), k) in used or g.sm_at(x, y).get(k) is None:
                continue
            v = sm_routing.compute_select(0, "D", br, 0, partner,
                                          _selval(tmpl_words, x, y, k))
            if v is not None:
                writes.setdefault(((x, y), k), v)

    # -- 3. the constant clock-corner block.
    for (t, k, v) in _corner_writes(g):
        writes.setdefault((t, k), v)

    # -- 4. (0,'U') output assignment + input IOBs.
    for (x, y, br), port in bound_ports.items():
        k = (0, "U", br, 0)
        if ((x, y), k) in used:
            continue                              # routed bind wins (parity)
        writes.setdefault(((x, y), k), slice_output.formula_bit_27_14(port))
    # input-IOB (0,'U') defaults: column-parity coded (xor_reg trace: the
    # UNROUTED branches at col1 -> 1, col2 -> 0 -- the complement of the routed
    # exit's driving-pin index).
    for (x, y) in input_tiles:
        for br in range(4):
            k = (0, "U", br, 0)
            if g.sm_at(x, y).get(k) is not None:
                writes.setdefault(((x, y), k), x % 2)

    n = 0
    for ((x, y), key), v in writes.items():
        if key not in sm_routing.SM_BL_WL:
            continue
        _write_value(image, x, y, key, v)
        n += 1
    return image, n
