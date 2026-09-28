"""In-port switch-matrix (SM) routing-bit encoder.

This is the pure bit-math reimplementation of the SM routing encoder: given a
routed hop (an SM mux node + the upstream source feeding it), produce the exact
(word_idx, bit, value) config writes. Transcribed in-port (NOT imported) from
the project's verified SM encoding -- the BL/WL tables come from GDB tracing of
FUN_00573110 (level 0) / FUN_00573470 (level 1+), and the select-value toggle
formula is the one cross-checked against the decompiled bit writers.

Two layers, matching the vendor:
  1. node geometry  : (level, dir, branch, node_index) -> list of (BL, WL) the
                      select bits occupy in the per-tile buffer.
  2. select value   : source identity -> integer select value via the XOR/toggle
                      formula `default ^ (1<<default_src_bit) ^ (1<<source_bit)`.

A hop is encoded by mapping each select bit through coords.trace_to_word_bit and
writing (value >> i) & 1. Verified bit-for-bit equivalent to the reference
encoder (tests/test_sm_routing.py) across every SM node and grid point.
"""

from . import coords


# (level, direction, branch, node_index) -> [(BL, WL), ...]  (bit 0 first)
# Recovered from FUN_00573110 / FUN_00573470 traces; write order preserved.
SM_BL_WL = {
    (0, 'D', 0, 0): [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],
    (0, 'D', 1, 0): [(0, 5), (0, 6), (0, 7), (0, 8), (0, 9)],
    (0, 'D', 2, 0): [(0, 10), (0, 11), (0, 12), (0, 13), (0, 14)],
    (0, 'D', 3, 0): [(0, 15), (0, 16), (0, 17), (0, 18), (0, 19)],
    (0, 'U', 0, 0): [(5, 25)],
    (0, 'U', 1, 0): [(4, 23)],
    (0, 'U', 2, 0): [(4, 24)],
    (0, 'U', 3, 0): [(4, 25)],

    (1, 'D', 0, 0): [(6, 0), (6, 1), (6, 2), (6, 3), (6, 4),
                     (5, 0), (5, 1), (5, 2), (5, 3), (5, 4)],
    (1, 'D', 0, 1): [(6, 5), (6, 6), (6, 7), (6, 8), (6, 9),
                     (5, 5), (5, 6), (5, 7), (5, 8), (5, 9)],
    (1, 'D', 1, 0): [(6, 10), (6, 11), (6, 12), (6, 13), (6, 14),
                     (5, 10), (5, 11), (5, 12), (5, 13), (5, 14)],
    (1, 'D', 2, 0): [(6, 15), (6, 16), (6, 17), (6, 18), (6, 19),
                     (5, 15), (5, 16), (5, 17), (5, 18), (5, 19)],
    (1, 'D', 3, 0): [(6, 20), (6, 21), (6, 22), (6, 23), (6, 24),
                     (5, 20), (5, 21), (5, 22), (5, 23), (5, 24)],
    (1, 'U', 0, 0): [(8, 20), (8, 21), (8, 22), (8, 23)],
    (1, 'U', 1, 0): [(7, 20), (7, 21), (7, 22), (7, 23)],
    (1, 'U', 2, 0): [(8, 24), (8, 25), (8, 26), (8, 27)],
    (1, 'U', 3, 0): [(7, 24), (7, 25), (7, 26), (7, 27)],

    (3, 'D', 1, 0): [(6, 26), (6, 27), (6, 28), (6, 29), (6, 30), (6, 31),
                     (8, 28), (8, 29), (8, 30), (8, 31)],
    (3, 'D', 4, 0): [(5, 26), (5, 27), (5, 28), (5, 29), (5, 30), (5, 31),
                     (7, 28), (7, 29), (7, 30), (7, 31)],
    (3, 'U', 1, 0): [(8, 2), (8, 3), (8, 4), (8, 5)],
    (3, 'U', 4, 0): [(7, 2), (7, 3), (7, 4), (7, 5)],

    (4, 'D', 2, 0): [(3, 24), (3, 25), (3, 26), (3, 27),
                     (0, 20), (0, 21), (0, 22), (0, 23), (0, 24), (0, 25)],
    (4, 'D', 5, 0): [(3, 28), (3, 29), (3, 30), (3, 31),
                     (0, 26), (0, 27), (0, 28), (0, 29), (0, 30), (0, 31)],
    (4, 'U', 2, 0): [(8, 6), (8, 7), (8, 8), (8, 9), (8, 10), (8, 11)],
    (4, 'U', 5, 0): [(7, 6), (7, 7), (7, 8), (7, 9), (7, 10), (7, 11)],

    (5, 'D', 0, 0): [(4, 0), (4, 1), (4, 2), (4, 3),
                     (3, 0), (3, 1), (3, 2), (3, 3)],
    (5, 'D', 0, 1): [(4, 4), (4, 5), (4, 6), (4, 7),
                     (3, 4), (3, 5), (3, 6), (3, 7)],
    (5, 'D', 1, 0): [(4, 8), (4, 9), (4, 10), (4, 11), (4, 12),
                     (3, 8), (3, 9), (3, 10), (3, 11), (3, 12)],
    (5, 'D', 2, 0): [(4, 13), (4, 14), (4, 15), (4, 16), (4, 17),
                     (3, 13), (3, 14), (3, 15), (3, 16), (3, 17)],
    (5, 'D', 3, 0): [(4, 18), (4, 19), (4, 20), (4, 21), (4, 22),
                     (3, 18), (3, 19), (3, 20), (3, 21), (3, 22)],
    (5, 'U', 0, 0): [(7, 12), (7, 13), (7, 14), (7, 15),
                     (8, 12), (8, 13), (8, 14), (8, 15)],
    (5, 'U', 0, 1): [(7, 16), (7, 17), (7, 18), (7, 19),
                     (8, 16), (8, 17), (8, 18), (8, 19)],

    (13, 'U', 1, 0): [(1, 24), (1, 25), (1, 26), (1, 27),
                      (2, 24), (2, 25), (2, 26), (2, 27)],
    (13, 'U', 1, 1): [(1, 28), (1, 29), (1, 30), (1, 31),
                      (2, 28), (2, 29), (2, 30), (2, 31)],
    (13, 'U', 2, 0): [(1, 16), (1, 17), (1, 18), (1, 19),
                      (2, 16), (2, 17), (2, 18), (2, 19)],
    (13, 'U', 2, 1): [(1, 20), (1, 21), (1, 22), (1, 23),
                      (2, 20), (2, 21), (2, 22), (2, 23)],
    (13, 'U', 4, 0): [(1, 8), (1, 9), (1, 10), (1, 11),
                      (2, 8), (2, 9), (2, 10), (2, 11)],
    (13, 'U', 4, 1): [(1, 12), (1, 13), (1, 14), (1, 15),
                      (2, 12), (2, 13), (2, 14), (2, 15)],
    (13, 'U', 5, 0): [(1, 0), (1, 1), (1, 2), (1, 3),
                      (2, 0), (2, 1), (2, 2), (2, 3)],
    (13, 'U', 5, 1): [(1, 4), (1, 5), (1, 6), (1, 7),
                      (2, 4), (2, 5), (2, 6), (2, 7)],
}


def select_positions(grid_x, grid_y, level, direction, branch, node_index=0):
    """(word_idx, bit) for each select bit of an SM node at a grid point."""
    bl_wl = SM_BL_WL.get((level, direction, branch, node_index))
    if bl_wl is None:
        return None
    return [coords.trace_to_word_bit(grid_x, grid_y, bl, wl) for bl, wl in bl_wl]


# Select-value formula (single-stage nodes).
_SOURCE_BITS = {
    (0, 'D'): {'D1B0': 0, 'D1B0N1': 1, 'D1B1': 2, 'D1B2': 3, 'D1B3': 4},
    (1, 'U'): {'U0B0': 0, 'U0B1': 1, 'U0B2': 2, 'U0B3': 3},
    (3, 'U'): {'U1B0': 0, 'U1B1': 1, 'U1B2': 2, 'U1B3': 3},
    (4, 'U'): {'U1B0': 0, 'U1B1': 1, 'U1B2': 2, 'U1B3': 3, 'U3B1': 4, 'U3B4': 5},
    (5, 'U'): {'U4B2': 0, 'U1B1': 1, 'U1B2': 2, 'U1B3': 3, 'U3B1': 4,
               'U3B4': 5, 'U1B0': 6, 'U4B5': 7},
    (13, 'U', 1): {'U5B0': 0, 'U5B0N1': 1, 'U13B2': 4, 'U13B2N1': 5},
    (13, 'U', 2): {'U5B0': 0, 'U5B0N1': 1, 'U13B1': 6, 'U13B1N1': 7},
    (5, 'D', 0): {'U13B2': 4, 'U13B2N1': 5, 'U13B1': 6, 'U13B1N1': 7},
}
_ENABLE_BITS = {4: {1}, 5: {1}, 6: {1}, 8: {1, 5}, 10: {1}}


def _source_key(sm_name):
    return sm_name[:-2] if sm_name.endswith('N0') else sm_name


def _default_src_bit(default_value, enable_bits):
    for i in range(16):
        if i in enable_bits:
            continue
        if (default_value >> i) & 1:
            return i
    return 0


def compute_select(level, direction, branch, node_index, source_sm_name,
                   default_value):
    """Select value for a single-stage SM node (XOR/toggle formula).

    `default_value` is the value already present in the empty template at this
    node's bits; the routed source toggles the default source bit off and its own
    source bit on. Returns None if the source is not in the level's source map.
    """
    src_bits = _SOURCE_BITS.get((level, direction, branch)) or \
        _SOURCE_BITS.get((level, direction))
    if src_bits is None:
        return None
    src_bit = src_bits.get(_source_key(source_sm_name))
    if src_bit is None:
        return None
    bl_wl = SM_BL_WL.get((level, direction, branch, node_index))
    nbits = len(bl_wl) if bl_wl else 5
    enable_bits = _ENABLE_BITS.get(nbits, {1})
    base = default_value
    if base == 0:
        # Uninitialized node: the device template holds 0 only at the clock-root
        # corner tile (0,31). Every other tile of a single-stage family encodes
        # the mux's RESET pattern (enable bit(s) + the default source bit) -- for
        # the level-0/1/3/4 families that is 3, verified 1023/1024 across the
        # empty template. The single-stage SM writer toggles from that reset
        # pattern, not from the bitstream's current 0, so the corner needs the
        # reconstructed base. <- FUN_00573470 single-stage select (reset-pattern
        # read-modify-write). Guarded by the caller's `cv == rval` check, so an
        # off-base family can only miss, never false-match.
        base = sum(1 << e for e in enable_bits) | (1 << min(src_bits.values()))
    dsb = _default_src_bit(base, enable_bits)
    return base ^ (1 << dsb) ^ (1 << src_bit)


def write_select(image, grid_x, grid_y, level, direction, branch,
                 select_value, node_index=0):
    """Write a known select value's bits into the image. Returns bits written."""
    positions = select_positions(grid_x, grid_y, level, direction, branch,
                                 node_index)
    if positions is None:
        return 0
    for i, (word_idx, bit) in enumerate(positions):
        image.set_word_bit(word_idx, bit, (select_value >> i) & 1)
    return len(positions)


def write_hop(image, hop):
    """Encode one routed hop (dict) into the image.

    hop = {grid_x, grid_y, level, direction, branch, node_index?,
           select_value?  OR  source + default_value}
    """
    ni = hop.get("node_index", 0)
    if "select_value" in hop:
        val = hop["select_value"]
    else:
        val = compute_select(hop["level"], hop["direction"], hop["branch"], ni,
                             hop["source"], hop.get("default_value", 0))
        if val is None:
            return 0
    return write_select(image, hop["grid_x"], hop["grid_y"], hop["level"],
                        hop["direction"], hop["branch"], val, ni)
