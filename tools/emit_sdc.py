#!/usr/bin/env python3
"""Convert nextpnr placement JSON to EFLX SDC format.

Usage: python3 tools/emit_sdc.py placement.json -o output.sdc

The placement JSON is produced by nextpnr-slg47910 and contains LUT placements
with INIT values and site input pin mappings. This script converts that into
the set_case_analysis format consumed by tools/pack.py.

Physical architecture:
  Each CLB has 24 site input pins and 4 slices (A-D).
  Per slice:
    - I0-I3 (A1-A4): hard-wired to dedicated pins (sl*6+0..sl*6+3).
    - I4 (A5): routable via MUX_x5 (4-bit select from all 24 pins).
    - I5 (A6): routable via MUX_x6 (4-bit select from all 24 pins).
  Since nextpnr routes freely, the SDC emitter must remap the LUT INIT
  truth table to match the physical pin assignment.
"""

import json
import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bitgen import coords  # noqa: E402

# Coordinate mapping: (sx, sy, site_type) → RBB name.
# rbb_x = 3*sy + 1 (interior columns: 1,4,7,10,13,16,19,22,25,28)
# rbb_y = 2*sx + 2 (rows: 2,4,...,28)
# L1 (memory CLB) sites use a different sx space: L1 sx={0,1,2,3} → L0 sx={2,3,10,11}
_L1_SX_TO_L0 = {0: 2, 1: 3, 2: 10, 3: 11}

def site_to_rbb(sx, sy, site_type="L0"):
    if site_type == "L1":
        sx = _L1_SX_TO_L0[sx]
    return (3 * sy + 1, 2 * sx + 2)

# Dedicated pin for LUT input k on slice sl.
def dedicated_pin(sl, k):
    """Return the site input pin index for dedicated input k (0-3) of slice sl."""
    assert 0 <= k <= 3
    return sl * 6 + k

SLICE_LETTER = ['A', 'B', 'C', 'D']


class UnsupportedBackendFeature(RuntimeError):
    """Raised when placement data requires unsupported silicon knowledge."""


def warn(msg):
    print(f"WARNING: {msg}", file=sys.stderr)


def is_sitewide_ram32x1d(ram):
    """Return True for the vendor-shaped single-RBB RAM32X1D placement view."""
    return (ram.get('ram_type') == 'RAM32X1D' and
            ram.get('layout') == 'rbb6m_single')


def is_native_ram32m_half(ram):
    """Return True for the native vendor-shaped half-row RAM32M view."""
    return (ram.get('ram_type') == 'RAM32M' and
            ram.get('layout') == 'ram32m_native_half')


def emit_sitewide_ram32x1d_muxes(f, prefix):
    """Emit vendor-grounded fixed mux settings for RBB6M RAM32X1D mode.

    Across the local vendor corpus, site-wide RAM32X1D consistently programs
    the per-slice input muxes to a fixed shape that is independent of logical
    write/read port routing.  The routed site pins already feed the dedicated
    RBB6M RAM ports directly; these muxes describe the RAM mode plumbing, not
    the old slice-local LUT pin assignment.

    Verified against vendor SDC: MUX_A5=2, MUX_B5=2, MUX_C5=4, MUX_D5=1,
    all x6=1, AI/BI=(16,xor=1), CI/DI=(2,xor=0), CE=(1,xor=0), SR=(1,xor=0).
    """
    emit_mux_sel(f, prefix, "MUX_A5", 2, 4)
    emit_mux_sel(f, prefix, "MUX_B5", 2, 4)
    emit_mux_sel(f, prefix, "MUX_C5", 4, 4)
    emit_mux_sel(f, prefix, "MUX_D5", 1, 4)

    for letter in SLICE_LETTER:
        emit_mux_sel(f, prefix, f"MUX_{letter}6", 1, 4)

    emit_mux_sel_with_xor(f, prefix, "MUX_AI", 16, 1, 5)
    emit_mux_sel_with_xor(f, prefix, "MUX_BI", 16, 1, 5)
    emit_mux_sel_with_xor(f, prefix, "MUX_CI", 2, 0, 5)
    emit_mux_sel_with_xor(f, prefix, "MUX_DI", 2, 0, 5)
    emit_mux_sel_with_xor(f, prefix, "MUX_CE", 1, 0, 5)
    emit_mux_sel_with_xor(f, prefix, "MUX_SR", 1, 0, 5)


def derive_ram32x1d_bc(ram, lut_slices):
    """Build 256-bit BC image for the site-wide RAM32X1D.

    Vendor observations show a fixed pattern:
      - BC[192:255] (slice A, chunk 1): helper LUT A INIT or 0xFFFFFFFF pass-through
      - BC[128:191] (slice B, chunk 1): helper LUT B INIT or 0xFFFFFFFF pass-through
      - BC[64:127]  (slice A, chunk 0 + slice B chunk 0): RAM INIT content
      - BC[0:63]    (slices C/D): all zeros

    The upper halves (chunks 1) of slices A and B carry either colocated
    helper LUT truth tables (if present) or 0xFFFFFFFF (pass-through).
    The lower halves (chunks 0) encode the RAM initial content. Targeted
    vendor probes show that, in this single-RBB mode, only INIT[1:0] affect
    the emitted BC image:
      - INIT[0] drives all even bits of BC[32:63] and BC[96:127]
      - INIT[1] drives all odd bits of BC[32:63] and BC[96:127]
      - INIT[31:2] are ignored by the vendor flow for this mode
    """
    bc = 0

    # Slices A and B upper chunks (bits 192-255 and 128-191):
    # If helper LUTs are colocated, use their remapped INIT.
    # Otherwise, use 0xFFFFFFFF (pass-through).
    for sl in (0, 1):
        lut = lut_slices.get(sl)
        if lut is not None:
            perm, _, _ = assign_pins(sl, lut['pins'])
            chunk = permute_init(lut['init'], perm)
        else:
            # Vendor default: upper 32 bits = all 1s (pass-through)
            chunk = 0xFFFFFFFF << 32
        # Slice A at bits 192-255, slice B at bits 128-191
        bc |= chunk << (128 + (1 - sl) * 64)

    # Lower halves of slices A and B (bits 32-63 and 96-127):
    # Vendor probes across explicit INIT values show a mirrored 32-bit image
    # that only depends on INIT[1:0]. The two live BC windows carry the same
    # pattern.
    init = int(ram.get('init', 0)) & 0xFFFFFFFF
    ram_init_chunk = 0
    if init & 0x1:
        ram_init_chunk |= 0x55555555
    if init & 0x2:
        ram_init_chunk |= 0xAAAAAAAA
    bc |= ram_init_chunk << 32    # Slice A chunk 0 upper half (BC[32:63])
    bc |= ram_init_chunk << 96    # Slice B chunk 0 upper half (BC[96:127])

    return bc


SITEWIDE_RAM32X1D_LUT_IPT_SEL = 8

_CLOCK_MUX_DEFAULTS = {
    8: [0, 0, 0, 0],
    9: [0, 1, 1, 0],
}

_CLOCK_SOURCE_MUX_BITS = {
    "W0": (8, [0, 0, 0, 0]),
    "N0": (8, [0, 0, 1, 0]),
    "N1": (9, [1, 0, 1, 0]),
}

# Route-aware side-edge IOB output emission.
#
# West-edge (gx=0) mux values are determined by the terminal D0 branch feeding
# the PAD input.  Clean vendor single-output probes show:
#   D0B0N0 -> O0=1, O1/SR/CE=2
#   D0B1N0 -> O0=2, O1/SR/CE=1
#   D0B2N0 -> O0=4, O1/SR/CE=1
#   D0B3N0 -> O0=8, O1/SR/CE=1
_EDGE_IOB_OUTPUT_O0_BY_D0_BRANCH = {
    0: 1,
    1: 2,
    2: 4,
    3: 8,
}

_EDGE_IOB_OUTPUT_SHARED_SEL_BY_D0_BRANCH = {
    0: 2,
    1: 1,
    2: 1,
    3: 1,
}

# Vendor-vs-packer clean single-output deltas that are not represented in the
# vendor SDC. These are west-edge side bits keyed by fabric site and, for the
# route-sensitive corner/mid-edge cases, by terminal D0 branch.
_WEST_IOB_OUTPUT_NVM_BITS = {
    (0, 0): {
        0: [(10, 24, 0)],
    },
    (0, 8): {
        0: [(8, 0, 0)],
    },
    (0, 9): {
        0: [(7, 21, 0)],
    },
    (0, 22): {
        None: [(14, 2, 0)],
        0: [(14, 10, 0)],
    },
    (0, 23): {
        None: [(14, 7, 0)],
    },
    (0, 24): {
        None: [(13, 8, 0)],
    },
    # (0, 25) is the dedicated west-edge clk_en pad. NVM_R13 bit 13 (= w13/b13)
    # is part of the chip-level baseline (0xfbdfe1fe) which has it set to 1;
    # vendor clears it whenever this pad is routed. Verified across 3 vendor
    # designs that use (0, 25) and the 184-design corpus where the bit is only
    # 1 in the 3 designs that do not touch RBB_0_25 (see
    # docs/methodology/sym-bucket6-findings.md).
    (0, 25): {
        None: [(13, 13, 0)],
    },
    (0, 30): {
        None: [(11, 10, 0)],
    },
    (0, 31): {
        None: [(11, 5, 0)],
    },
}

# East-edge dynamic-output mode recovered by HITL bit-minimization and a
# 32-variant MODE-bit sweep on 2026-05-19/20.  When an east-edge output IOB
# is fed from fabric (i.e. its net is not $PACKER_GND/$PACKER_VCC), the IOB
# stays tri-stated by default and the pad reads as the external pull-up.
# Clearing the per-gy bit listed here switches the IOB into fabric-drive
# mode.
#
# The MODE truth table (sweep target pad: 0 = driven, 1 = tri-state) is:
#
#     gy   (0,0)  (0,1)  (1,0)  (1,1)
#      8     0      1      0      1
#      9     0      1      0      1
#     22     1      1      0      1
#     23     1      0      1      1
#     24     1      1      0      1
#     25     0      0      1      1
#     26     0      1      0      1
#     27     1      0      1      1
#
# i.e. the IOB drives when its specific (MODE[0], MODE[1]) bit pair matches
# one of the per-gy combinations above.  The encoder clears exactly one bit
# per gy (the one that's part of the drive combo) and leaves the other at
# the empty-bitstream default of 1; that choice always produces a combo
# that the sweep marks as driving.  See docs/iob-debugging.md for the full
# investigation.
#
# gy=9 also needs (11040, 26)=0 — the same physical bit as
# RBB_31_9/MUX_O1/BSEL, which the side-edge mux encoder writes as 1 when
# nextpnr routes OE through any branch other than BSEL.  The override is
# applied via the direct-NVM tier in pack.py so it trumps the conflicting
# edge-mux write.
# Row-31 pad -> (tile, bl) of its enable bits: the pad's own tile at bl9 when
# it is single, else the even tile of its pair (8/9, 22/23, 24/25).
_EAST_PAD_ENABLE_TILE = {6: (6, 9), 8: (8, 9), 9: (8, 10), 22: (22, 9), 23: (22, 10),
                         24: (24, 9), 25: (24, 10), 26: (26, 9), 27: (27, 9)}


def _word_bit(x, y, bl, wl):
    """Trace (x, y, BL, WL) -> (word, bit) of the raw image."""
    return coords.trace_to_word_bit(x, y, bl, wl)



# Vendor-grounded single-RBB RAM32X1D WE cases observed in the local corpus.
# Alias-driven cases are described by logical topology; logically independent
# WE routing splits further by the routed site pin used to feed the RAM mode.
_SITEWIDE_RAM32X1D_WE_BY_WA_INDEX = {
    0: 3,
    1: 2,
    2: 1,
    3: 3,
    4: 1,
}
_SITEWIDE_RAM32X1D_WE_BY_DPRA_INDEX = {
    3: 1,
}
_SITEWIDE_RAM32X1D_WE_BY_INDEPENDENT_PIN = {
    23: 1,
    6: 2,
    9: 3,
}


def derive_sitewide_ram32x1d_we_sel(ram):
    """Derive the supported single-RBB RAM32X1D WE_SEL value.

    Currently modeled cases from the vendor single-RBB corpus:
      - WE reusing the RAM D path (C6-style)   -> 000
      - WE reusing observed WA / DPRA taps     -> table-driven
      - independent WE on routed site pin 23   -> 001
      - independent WE on routed site pin 6    -> 010
      - independent WE on routed site pin 9    -> 011

    This keeps the open emitter honest for the supported site-wide RAM32X1D
    path instead of hardcoding a blanket placeholder. If the placement JSON
    describes a WE alias pattern or independent ingress pin outside the
    locally RE-backed cases, stop and report it rather than silently emitting
    a made-up value.
    """
    we_net = ram.get('we_net', '')
    if not we_net:
        return 1

    d_net = ram.get('d_net', '')
    if d_net and d_net == we_net:
        return 0

    for idx, net in enumerate(ram.get('wa_nets', [])):
        if net and net == we_net:
            value = _SITEWIDE_RAM32X1D_WE_BY_WA_INDEX.get(idx)
            if value is None:
                raise UnsupportedBackendFeature(
                    f"site-wide RAM32X1D WE alias on WA{idx} is not yet modeled"
                )
            return value

    for idx, net in enumerate(ram.get('dpra_nets', [])):
        if net and net == we_net:
            value = _SITEWIDE_RAM32X1D_WE_BY_DPRA_INDEX.get(idx)
            if value is None:
                raise UnsupportedBackendFeature(
                    f"site-wide RAM32X1D WE alias on DPRA{idx} is not yet modeled"
                )
            return value

    we_pin = ram.get('we_pin', -1)
    value = _SITEWIDE_RAM32X1D_WE_BY_INDEPENDENT_PIN.get(we_pin)
    if value is None:
        raise UnsupportedBackendFeature(
            f"site-wide RAM32X1D independent WE on site pin {we_pin} is not yet modeled"
        )
    return value


def derive_sitewide_ram32x1d_lut_ipt_sel(ram):
    """Return the vendor-grounded fixed LUT_IPT_SEL shape for single-RBB RAM."""
    del ram
    return SITEWIDE_RAM32X1D_LUT_IPT_SEL


def derive_sitewide_ram32x1d_clk_sel(ram):
    """Return the vendor-backed RAM clock-domain select."""
    domain = int(ram.get('clk_domain', 0))
    if domain not in (0, 1):
        raise UnsupportedBackendFeature(
            f"site-wide RAM32X1D uses unsupported clock domain {domain}"
        )
    return domain


def validate_sitewide_ram32x1d(ram, prefix):
    """Validate vendor-observed single-RBB RAM32X1D mode constraints."""
    init = int(ram.get('init', 0)) & 0xFFFFFFFF
    if init & ~0x3:
        raise UnsupportedBackendFeature(
            f"{prefix}: site-wide RAM32X1D INIT=0x{init:08X} uses INIT[31:2], "
            "but the vendor-observed single-RBB mode only preserves INIT[1:0]"
        )


def validate_native_ram32m_site(ram_slices, lut_slices, ff_slices, prefix):
    """Validate one native RAM32M half-row."""
    if len(ram_slices) != 4:
        raise UnsupportedBackendFeature(
            f"{prefix}: native RAM32M half must occupy all four slice RAM bels"
        )
    if lut_slices:
        raise UnsupportedBackendFeature(
            f"{prefix}: native RAM32M half cannot share its RBB with LUT cells"
        )
    if ff_slices:
        raise UnsupportedBackendFeature(
            f"{prefix}: native RAM32M half cannot share its RBB with FF cells"
        )

    groups = {ram.get('ram32m_group', '') for ram in ram_slices.values()}
    if len(groups) != 1 or '' in groups:
        raise UnsupportedBackendFeature(
            f"{prefix}: native RAM32M half has inconsistent grouping metadata"
        )

    bitplanes = {int(ram.get('ram32m_bitplane', -1)) for ram in ram_slices.values()}
    if len(bitplanes) != 1 or next(iter(bitplanes)) not in (0, 1):
        raise UnsupportedBackendFeature(
            f"{prefix}: native RAM32M half has inconsistent bitplane metadata"
        )


def remap_lut(init, pins):
    """Remap a LUT INIT to match the physical pin assignment.

    The nextpnr router freely assigns site input pins to LUT inputs.
    The physical chip has:
      - I0-I3: dedicated pins (hard-wired, no mux)
      - I4: MUX_x5 (any pin)
      - I5: MUX_x6 (any pin)

    We need to permute the truth table so that:
      - Signals on dedicated pins go to the correct LUT address bits.
      - Signals on non-dedicated pins go to I4/I5 positions.

    Returns: (new_init, mux5_pin, mux6_pin) where mux5_pin/mux6_pin are
    the site input pin indices for MUX_x5 and MUX_x6 (-1 if unused).
    """
    # Identify which LUT inputs are actually used.
    used = [(k, pins[k]) for k in range(6) if pins[k] >= 0]

    if not used:
        # No inputs used (constant LUT). Return as-is.
        return (init & 0xFFFFFFFFFFFFFFFF, -1, -1)

    sl = None  # Will be set from context by caller.
    return (init & 0xFFFFFFFFFFFFFFFF, -1, -1)  # Placeholder


def permute_init(init, perm):
    """Permute a 64-bit LUT INIT truth table.

    perm[i] = j means physical address bit i gets its value from logical bit j.
    If perm[i] = -1, that address bit is tied to 0.

    For a LUT6: O = INIT[{A6, A5, A4, A3, A2, A1}]
    A1 = bit 0 of address, A6 = bit 5.
    """
    new_init = 0
    for addr in range(64):
        # Map physical address to logical address.
        log_addr = 0
        for bit in range(6):
            if perm[bit] >= 0 and (addr >> bit) & 1:
                log_addr |= (1 << perm[bit])
        if (init >> log_addr) & 1:
            new_init |= (1 << addr)
    return new_init


def assign_pins(sl, pins):
    """Assign LUT input signals to physical inputs, respecting constraints.

    Returns: (new_init_perm, mux5_pin, mux6_pin)
      - new_init_perm: a permutation array [0..5] mapping physical LUT
        address bits to logical input indices.
      - mux5_pin: site input pin selected by MUX_x5 (-1 if unused).
      - mux6_pin: site input pin selected by MUX_x6 (-1 if unused).
    """
    # Physical assignment plan:
    # For each used input, determine if it's on a dedicated pin or not.
    # Dedicated pins: sl*6+0..sl*6+3 → map to A1-A4 (bit positions 0-3).
    # Non-dedicated → must go to A5 or A6 (bit positions 4 or 5).

    ded_pins = set(dedicated_pin(sl, k) for k in range(4))

    # Separate used inputs into dedicated and non-dedicated.
    on_dedicated = []   # (logical_input_idx, physical_pin, dedicated_slot)
    on_nondedicated = [] # (logical_input_idx, physical_pin)

    for k in range(6):
        if pins[k] < 0:
            continue
        pin = pins[k]
        if pin in ded_pins:
            # Which dedicated slot (0-3)?
            ded_slot = pin - sl * 6
            on_dedicated.append((k, pin, ded_slot))
        else:
            on_nondedicated.append((k, pin))

    if len(on_nondedicated) > 2:
        # Can't map more than 2 signals to muxed inputs.
        # This is a routing error — the design can't be implemented.
        print(f"WARNING: slice {SLICE_LETTER[sl]}: {len(on_nondedicated)} signals "
              f"on non-dedicated pins (max 2). Design may be incorrect.",
              file=sys.stderr)

    # Build permutation: perm[physical_bit] = logical_input_idx
    perm = [-1] * 6

    # Map dedicated signals.
    for log_k, pin, ded_slot in on_dedicated:
        perm[ded_slot] = log_k

    # Map non-dedicated signals to A5 (bit 4) and A6 (bit 5).
    mux5_pin = -1
    mux6_pin = -1
    if len(on_nondedicated) >= 1:
        perm[4] = on_nondedicated[0][0]
        mux5_pin = on_nondedicated[0][1]
    if len(on_nondedicated) >= 2:
        perm[5] = on_nondedicated[1][0]
        mux6_pin = on_nondedicated[1][1]

    return (perm, mux5_pin, mux6_pin)


def emit_mux_sel(f, prefix, mux_name, value, n_bits=4):
    """Emit set_case_analysis lines for a MUX select."""
    sel_names = ['ASEL', 'BSEL', 'CSEL', 'DSEL', 'ESEL']
    for i in range(n_bits):
        bit = (value >> i) & 1
        f.write(f"set_case_analysis {bit} {prefix}/{mux_name}/{sel_names[i]}\n")
        f.write(f"set_case_analysis {1-bit} {prefix}/{mux_name}/{sel_names[i]}b\n")


def emit_mux_sel_with_xor(f, prefix, mux_name, value, xor_val=0, n_bits=5):
    """Emit set_case_analysis lines for a MUX select with XOR bit."""
    emit_mux_sel(f, prefix, mux_name, value, n_bits)
    f.write(f"set_case_analysis {xor_val} {prefix}/{mux_name}/XOR\n")
    f.write(f"set_case_analysis {1-xor_val} {prefix}/{mux_name}/XORb\n")


def emit_direct_nvm_bit(f, reg, bit, value):
    """Emit a direct NVM-bit override for pack.py's synthetic mapping path."""
    if reg < 15:
        f.write(f"set_case_analysis {value} TILE_00/NVM_R{reg}_B{bit}\n")
    else:
        f.write(f"set_case_analysis {value} TILE_00/RAW_W{reg}_B{bit}\n")


def derive_edge_iob_output_muxes(iob):
    """Return the recovered edge-output mux tuple for one routed side-edge IOB."""
    gx = int(iob.get('gx', -1))
    gy = int(iob.get('gy', -1))
    net = iob.get('net', '')

    # East-edge IOBs (gx=31): O0 selects the D0 branch the data came through
    # (branch k -> one-hot 1<<k, as on the west edge). The output enable is a
    # constant that route_bits routes to nothing, so O1/SR/CE must not name a
    # fabric branch: the vendor sets them to A, and to B when O0 itself is A
    # (read from its builds of one probe on every row-31 pad; with both on A
    # the pad sits static high).
    if gx == 31:
        if net in ('$PACKER_GND', '$PACKER_VCC'):
            raise UnsupportedBackendFeature(
                f"east-edge constant output at (31,{gy}) is not supported")
        data_branch = int(iob.get('pad_in_d0_branch', -1))
        if data_branch < 0:
            raise UnsupportedBackendFeature(
                f"east-edge IOB at (31,{gy}) has no pad_in_d0_branch in placement"
            )
        return 1 << data_branch, 2 if data_branch == 0 else 1

    branch = int(iob.get('pad_in_d0_branch', -1))
    o0 = _EDGE_IOB_OUTPUT_O0_BY_D0_BRANCH.get(branch)
    shared = _EDGE_IOB_OUTPUT_SHARED_SEL_BY_D0_BRANCH.get(branch)
    if o0 is None or shared is None:
        raise UnsupportedBackendFeature(
            f"edge output IOB at ({gx},{gy}) uses unsupported "
            f"terminal D0 branch {branch}"
        )
    return o0, shared


def derive_edge_iob_output_nvm_bits(iob):
    """Return recovered direct NVM overrides for supported edge outputs."""
    key = (int(iob.get('gx', -1)), int(iob.get('gy', -1)))
    gx, gy = key
    net = iob.get('net', '')

    # Fabric-driven outputs on the row-31 pads (the vendor's RBB_31_<col>; the
    # IOB is at graph (col, 31)). Read from the vendor placer's own builds of
    # one probe on each pad (Kraken harness eda-placer, design set pads):
    #   own tile   bl7 wl1 = 0   enables the output driver
    #              bl8 wl1 = 1   bypasses the IOB's output flip-flop (template)
    #              bl4 wl26 = 0
    #   enable tile bl9|10, the "lane 4" of the three shared muxes and wl25:
    #              shared A: wl14 = 0      shared B: wl22 = 1, wl30 = 0
    #              wl25 = 0 either way
    # The enable tile is the pad's own for a single pad (6, 26, 27) and the
    # even tile of its pair for 8/9, 22/23, 24/25 (bl9 for the even pad, bl10
    # for the odd one).
    if gx == 31:
        if gy not in _EAST_PAD_ENABLE_TILE:
            raise UnsupportedBackendFeature(
                f"east-edge output at (31,{gy}) is not RE-backed (GPIO16, 8-13 are)")
        et, ebl = _EAST_PAD_ENABLE_TILE[gy]
        lane4 = [(14, 0)] if derive_edge_iob_output_muxes(iob)[1] == 1 else [(22, 1), (30, 0)]
        return ([(*_word_bit(gy, 31, 7, 1), 0), (*_word_bit(gy, 31, 8, 1), 1),
                 (*_word_bit(gy, 31, 4, 26), 0), (*_word_bit(et, 31, ebl, 25), 0)] +
                [(*_word_bit(et, 31, ebl, wl), v) for wl, v in lane4])

    # West IOB constant outputs ($PACKER_GND / $PACKER_VCC): the
    # _WEST_IOB_OUTPUT_NVM_BITS entries were recovered from fabric-driven
    # single-output probes (fuzz.py make_iobmux), where the bits at
    # (0,22)/(0,23)/(0,24) drop to 0 because the IOB output path is
    # active. For constant outputs the vendor leaves those bits at the
    # empty-bitstream default of 1 (verified against iob_const_00, which
    # has $PACKER_VCC on every west q_en pad). Suppressing the clear here
    # closes Bucket 1 bits 13/8, 14/2, 14/7 on osc_probe.
    #
    # Exception: (0, 25) is the dedicated clock-enable pad — vendor clears
    # w13/b13 whenever it is used, even with a constant net. Verified across
    # all 3 of the 184-design corpus designs that don't touch RBB_0_25 vs the
    # 181 that do (see docs/methodology/sym-bucket6-findings.md).
    if net in ('$PACKER_GND', '$PACKER_VCC') and key != (0, 25):
        return []

    branch = int(iob.get('pad_in_d0_branch', -1))
    table = _WEST_IOB_OUTPUT_NVM_BITS.get(key)
    if table is None:
        return []

    bits = []
    bits.extend(table.get(None, []))
    bits.extend(table.get(branch, []))
    return bits


def ff_is_set_type(ff_type):
    """Return True if the FF type has SET/PRESET behavior (Q→1 on SR assertion)."""
    return ff_type in ('FDSE', 'FDPE')


def validate_ff(ff, prefix):
    """Validate a single FF placement entry.

    Raises UnsupportedBackendFeature for unsupported configurations.
    """
    ff_type = ff.get('ff_type') or 'FDRE'
    if ff_type not in ('FDRE', 'FDSE', 'FDCE', 'FDPE'):
        raise UnsupportedBackendFeature(
            f"{prefix}: unknown FF type {ff_type}"
        )

    # IS_D_INVERTED is absorbed into the colocated LUT INIT during BC
    # emission (see emit_sdc per-RBB loop).  Validation that a colocated
    # LUT exists happens there, not here.

    if ff.get('is_c_inverted', 0):
        raise UnsupportedBackendFeature(
            f"{prefix}: IS_C_INVERTED is not supported; vendor RE shows the "
            "fabric FFs ignore per-FF clock inversion"
        )

    # INIT validation: the hardware always resets FFs to 0 on power-on.
    # The XOR on MUX_SR controls set vs reset behavior when SR is asserted.
    # Non-default INIT values (FDRE with INIT=1 or FDSE with INIT=0) would
    # require a separate configuration bit that hasn't been located yet.
    init_val = int(ff.get('init', 0))
    default_init = 1 if ff_is_set_type(ff_type) else 0
    if init_val != default_init:
        raise UnsupportedBackendFeature(
            f"{prefix}: FF INIT={init_val} differs from default for {ff_type} "
            f"(expected {default_init}); non-default INIT not yet supported"
        )


def resolve_shared_ff_control(ffs, prefix, pin_key, net_key, default_pin, mux_name):
    """Validate one shared CLB control and return the selected site pin."""
    if not ffs:
        return default_pin

    pins = sorted({ff.get(pin_key, -1) for ff in ffs if ff.get(pin_key, -1) >= 0})
    nets = sorted({ff.get(net_key, '') for ff in ffs if ff.get(net_key)})

    if len(pins) > 1:
        raise UnsupportedBackendFeature(
            f"{prefix}: conflicting shared {mux_name} selections {pins} across slices"
        )
    if len(nets) > 1:
        raise UnsupportedBackendFeature(
            f"{prefix}: conflicting shared {mux_name} nets {nets} across slices"
        )

    if nets == ['$PACKER_VCC'] and mux_name == "MUX_CE":
        return 16
    if nets == ['$PACKER_GND'] and mux_name == "MUX_SR":
        return 1

    pin = pins[0] if pins else default_pin
    if mux_name == "MUX_CE" and pin >= 16:
        warn(f"{prefix}: {mux_name} selects pin {pin}; ESEL coverage is still sparse on this architecture")
    return pin


def collect_clock_sources(placement):
    """Return explicit dedicated clock sources keyed by clock domain."""
    sources = {}
    for clock in placement.get('clocks', []):
        token = clock.get('token', '')
        if not token:
            continue
        observed = _CLOCK_SOURCE_MUX_BITS.get(token)
        if observed is None:
            raise UnsupportedBackendFeature(
                f"unsupported dedicated clock token {token!r} in placement data"
            )
        expected_domain = 0 if token in ('W0', 'N0') else 1
        domain = int(clock.get('domain', expected_domain))
        if domain != expected_domain:
            raise UnsupportedBackendFeature(
                f"clock source {token!r} reports domain {domain}, expected {expected_domain}"
            )
        current = sources.get(domain)
        if current is not None:
            same_net = current.get('net', '') == clock.get('net', '')
            same_token = current.get('token', '') == token
            if not (same_net and same_token):
                raise UnsupportedBackendFeature(
                    f"multiple dedicated clock sources target Clock_{domain}: "
                    f"{current.get('token', '')} and {token}"
                )
            continue
        sources[domain] = clock
    return sources


def resolve_shared_clock_domain(clk_items, prefix):
    """Validate the shared CLB clock selection and return its domain."""
    if not clk_items:
        return 0

    domains = sorted({int(item.get('clk_domain', 0)) for item in clk_items})
    nets = sorted({item.get('clk_net', '') for item in clk_items if item.get('clk_net')})

    if len(domains) > 1:
        raise UnsupportedBackendFeature(
            f"{prefix}: conflicting shared clock domains {domains} across slices"
        )
    if len(nets) > 1:
        raise UnsupportedBackendFeature(
            f"{prefix}: conflicting shared CLK nets {nets} across slices"
        )

    domain = domains[0] if domains else 0
    if domain not in (0, 1):
        raise UnsupportedBackendFeature(
            f"{prefix}: unsupported clock domain {domain}"
        )
    return domain


def emit_sdc(placement, out_file):
    """Generate SDC file from placement data."""
    with open(out_file, 'w') as f:
        clock_sources = collect_clock_sources(placement)

        # --- Per-RBB configuration ---
        # Collect all used RBBs from placement data.
        # Each RBB entry tracks LUTs, FFs, and RAMs per slice.
        used_rbbs = {}  # (rbb_x, rbb_y) → {"luts": {sl: lut}, "ffs": {sl: ff}, "rams": {sl: ram}}

        def ensure_rbb(sx, sy, site_type="L0"):
            rbb_x, rbb_y = site_to_rbb(sx, sy, site_type)
            key = (rbb_x, rbb_y)
            if key not in used_rbbs:
                used_rbbs[key] = {"luts": {}, "ffs": {}, "rams": {}}
            return key

        # Skip nextpnr's $PACKER_VCC_DRV / $PACKER_GND_DRV constant-driver LUTs.
        # These are synthesis artifacts: nextpnr-generic inserts a 0-input LUT
        # driving the constant net so the IOB has a fabric input to route from.
        # The vendor flow handles constant outputs at the IOB encoder level
        # without any LUT — emitting RBB MUX/LUT defaults for the dummy LUT
        # produces ~150 bitstream bits the vendor never sets.
        # The downstream IOB encoder already uses the net name (e.g.
        # '$PACKER_VCC') directly, so dropping the driver LUT does not break
        # constant-output emission.
        def _is_constant_driver_lut(lut):
            name = lut.get('name', '')
            if name in ('$PACKER_VCC_DRV', '$PACKER_GND_DRV'):
                return True
            # Defensive: also drop any zero-input LUT whose INIT is all-0 or
            # all-1 — same shape, slightly different name across nextpnr
            # versions.
            pins = lut.get('pins', [])
            nused = sum(1 for p in pins if p != -1)
            if nused == 0 and lut.get('init', 0) in (0, (1 << 64) - 1):
                return True
            return False

        for lut in placement['luts']:
            if _is_constant_driver_lut(lut):
                continue
            key = ensure_rbb(lut['sx'], lut['sy'], lut.get('site_type', 'L0'))
            used_rbbs[key]["luts"][lut['slice']] = lut

        for ff in placement.get('ffs', []):
            key = ensure_rbb(ff['sx'], ff['sy'], ff.get('site_type', 'L0'))
            used_rbbs[key]["ffs"][ff['slice']] = ff

        for ram in placement.get('rams', []):
            key = ensure_rbb(ram['sx'], ram['sy'], ram.get('site_type', 'L0'))
            used_rbbs[key]["rams"][ram['slice']] = ram

        # 8-FF model (Phase 1): per-slice OAQ_SEL / AMX_SEL state from emit.cc.
        # Keyed by (rbb_x, rbb_y) → { slice_idx → entry }. Each entry carries
        # an int `oaq_sel` (always present) and optionally `amx_sel` (set only
        # when an OXQ FF is bound on that slice).
        slice_muxes_by_rbb = {}
        for entry in placement.get('slice_muxes', []):
            sx = int(entry['sx']); sy = int(entry['sy'])
            site_type = entry.get('site_type', 'L0')
            rx, ry = site_to_rbb(sx, sy, site_type)
            slice_muxes_by_rbb.setdefault((rx, ry), {})[int(entry['slice'])] = entry

        native_ram32m_groups = {}
        for ram in placement.get('rams', []):
            if not is_native_ram32m_half(ram):
                continue
            group = ram.get('ram32m_group', '')
            native_ram32m_groups.setdefault(group, []).append(ram)

        for group, rams in native_ram32m_groups.items():
            if not group:
                raise UnsupportedBackendFeature("native RAM32M placement is missing ram32m_group metadata")
            if len(rams) != 8:
                raise UnsupportedBackendFeature(
                    f"RAM32M group {group!r} exports {len(rams)} slice cells, expected 8"
                )

            coords_by_bitplane = {}
            for bitplane in (0, 1):
                halves = {(int(r['sx']), int(r['sy'])) for r in rams
                          if int(r.get('ram32m_bitplane', -1)) == bitplane}
                if len(halves) != 1:
                    raise UnsupportedBackendFeature(
                        f"RAM32M group {group!r} does not collapse to one site per bitplane"
                    )
                coords_by_bitplane[bitplane] = next(iter(halves))

            sx0, sy0 = coords_by_bitplane[0]
            sx1, sy1 = coords_by_bitplane[1]
            if sx0 != sx1 or sy1 != sy0 + 1:
                raise UnsupportedBackendFeature(
                    f"RAM32M group {group!r} is not placed as adjacent horizontal L1 halves"
                )

        # --- CKMUX defaults / explicit dedicated clock sources ---
        mux_bits = {mux_idx: [0, 1, 1, 0] for mux_idx in range(10)}
        mux_bits[8] = _CLOCK_MUX_DEFAULTS[8][:]
        mux_bits[9] = _CLOCK_MUX_DEFAULTS[9][:]
        for clock in clock_sources.values():
            mux_idx, bits = _CLOCK_SOURCE_MUX_BITS[clock['token']]
            mux_bits[mux_idx] = bits[:]
        for mux_idx in range(10):
            bits = mux_bits[mux_idx]
            for sel_bit in range(4):
                f.write(f"set_case_analysis {bits[sel_bit]} TILE_00/CLK_{mux_idx}_{sel_bit}_SEL\n")
        # Vendor FF probes have never shown a toggled CLK_9_INV bit, so keep
        # the global inversion control at its observed default.
        f.write("set_case_analysis 0 TILE_00/CLK_9_INV\n")

        # Memory CLB rows (rbb_y values where L1 sites exist).
        MEMORY_CLB_ROWS = {6, 8, 22, 24}

        # Emit each used RBB.
        for (rbb_x, rbb_y) in sorted(used_rbbs.keys()):
            prefix = f"TILE_00/RBB_{rbb_x}_{rbb_y}"
            lut_slices = used_rbbs[(rbb_x, rbb_y)]["luts"]
            ff_slices = used_rbbs[(rbb_x, rbb_y)]["ffs"]
            ram_slices = used_rbbs[(rbb_x, rbb_y)]["rams"]
            is_memory_clb = rbb_y in MEMORY_CLB_ROWS
            sitewide_rams = [ram for ram in ram_slices.values() if is_sitewide_ram32x1d(ram)]
            sitewide_ram = sitewide_rams[0] if sitewide_rams else None
            native_ram32m_rams = [ram for ram in ram_slices.values() if is_native_ram32m_half(ram)]

            # Validate site-wide RAM32X1D constraints.
            if len(sitewide_rams) > 1:
                raise UnsupportedBackendFeature(
                    f"{prefix}: multiple site-wide RAM32X1D placements in one RBB are not legal"
                )
            if sitewide_ram is not None and len(ram_slices) != 1:
                raise UnsupportedBackendFeature(
                    f"{prefix}: mixed site-wide RAM32X1D and slice-local RAM/SRL placement is not supported"
                )
            if sitewide_ram is not None and any(sl not in (0, 1) for sl in lut_slices):
                raise UnsupportedBackendFeature(
                    f"{prefix}: site-wide RAM32X1D only supports colocated helper LUTs on slices A/B"
                )
            if sitewide_ram is not None and ff_slices:
                raise UnsupportedBackendFeature(
                    f"{prefix}: site-wide RAM32X1D unexpectedly shares an RBB with FF cells"
                )
            if sitewide_ram is not None:
                validate_sitewide_ram32x1d(sitewide_ram, prefix)
            if native_ram32m_rams:
                if len(native_ram32m_rams) != len(ram_slices):
                    raise UnsupportedBackendFeature(
                        f"{prefix}: native RAM32M half cannot mix with other RAM/SRL placements"
                    )
                validate_native_ram32m_site(ram_slices, lut_slices, ff_slices, prefix)

            for ff in ff_slices.values():
                validate_ff(ff, prefix)

            # Determine SR XOR: all FFs in CLB must agree on set vs reset type.
            # Vendor FDRE/FDCE probes with R tied low use MUX_SR XOR=1; SET-type
            # FFs flip that shared polarity so an asserted SR drives Q high.
            sr_xor = 1
            if ff_slices:
                sr_types = set()
                for ff in ff_slices.values():
                    ft = ff.get('ff_type') or 'FDRE'
                    sr_types.add(ff_is_set_type(ft))
                if len(sr_types) > 1:
                    raise UnsupportedBackendFeature(
                        f"{prefix}: cannot mix SET-type (FDSE/FDPE) and RESET-type (FDRE/FDCE) "
                        f"FFs in the same CLB (shared MUX_SR polarity)"
                    )
                if True in sr_types:
                    sr_xor = 0
                # Handle IS_R_INVERTED / IS_S_INVERTED: XOR with sr_xor
                for ff in ff_slices.values():
                    ft = ff.get('ff_type') or 'FDRE'
                    inv_key = 'is_s_inverted' if ft in ('FDSE', 'FDPE') else 'is_r_inverted'
                    if ff.get(inv_key, 0):
                        sr_xor ^= 1
                        break  # shared per CLB, one inversion applies to all

            # --- MUX selects per slice ---
            if sitewide_ram is not None:
                emit_sitewide_ram32x1d_muxes(f, prefix)
            else:
                for sl in range(4):
                    letter = SLICE_LETTER[sl]
                    lut = lut_slices.get(sl)
                    ram = ram_slices.get(sl)
                    is_carry = lut is not None and lut.get('carry_mode', False)

                    # A RAM uses the same physical LUT resource. If a RAM is placed
                    # on this slice, use its pin mapping for the mux selects and its
                    # INIT for the BC bits.
                    if ram is not None:
                        pins = ram['pins']
                        init = ram['init']
                        perm, mux5_pin, mux6_pin = assign_pins(sl, pins)
                        new_init = permute_init(init, perm)
                    elif lut is not None:
                        pins = lut['pins']
                        init = lut['init']
                        perm, mux5_pin, mux6_pin = assign_pins(sl, pins)
                        new_init = permute_init(init, perm)
                    else:
                        mux5_pin = -1
                        mux6_pin = -1
                        new_init = 0

                    # MUX_x5: selects A5 input source.
                    # In carry mode with absorbed LUT: 5th LUT input (if any).
                    # In carry mode without absorption: routes S[i] feed-through.
                    # Default (no input mapped, mux5_pin == -1): emit value=1
                    # i.e. ASEL=1. Vendor uses ASEL=1 as the "unconfigured /
                    # idle" position for MUX_x5 — observed in 631 / 640 RBB
                    # slice positions across 185 vendor designs (98.6%). The
                    # open flow previously emitted value=0 which lands a wrong
                    # bit in the physical mux storage. See
                    # docs/methodology/sym-bucket6-findings.md.
                    mux5_val = mux5_pin if mux5_pin >= 0 else 1
                    emit_mux_sel(f, prefix, f"MUX_{letter}5", mux5_val, 4)

                    # MUX_x6: selects A6 input source.
                    # In carry mode: routes DI[i] (generate signal).
                    # Default (mux6_pin == -1): ASEL=1, matching vendor.
                    mux6_val = mux6_pin if mux6_pin >= 0 else 1
                    emit_mux_sel(f, prefix, f"MUX_{letter}6", mux6_val, 4)

                    # MUX_xI: controls what feeds the FF D input / combinational output.
                    # Normal mode: ESEL=1 (pin 16), XOR=0 → LUT output.  For
                    # FF-only slices, nextpnr may route D directly from a site
                    # input; use the exported post-route D pin in that case.
                    # Carry mode: ESEL=1 (pin 16), XOR=1 → XORCY result (S^CI).
                    # Carry CO output: XOR=0 → MUXCY result (carry out).
                    # Default (empty slice, no LUT/RAM/FF): vendor emits
                    # ESEL=1 + XOR=1 in 616/640 of vendor positions (96.2%).
                    # When a LUT/RAM is present, keep XOR=0 (LUT output) unless
                    # explicitly carry.
                    # XOR=1 (the XORCY path, S ^ CI with CI = 0) for every slice
                    # but a carry CO output (MUXCY) or a RAM: the vendor's osc_probe
                    # writes MUX_DI/XOR = 1 for its LUT1 -> FF slice, and with XOR=0
                    # the flip-flop's D is the carry mux, not the LUT.
                    is_carry_co = lut is not None and lut.get('carry_co_output', False)
                    xor_val = 0 if (is_carry_co or ram is not None) else 1
                    ff = ff_slices.get(sl)
                    ff_d_pin = ff.get('d_pin', -1) if ff is not None else -1
                    mux_i_pin = 16
                    if ff is not None and lut is None and ram is None and ff_d_pin >= 0:
                        mux_i_pin = ff_d_pin
                    emit_mux_sel_with_xor(f, prefix, f"MUX_{letter}I", mux_i_pin, xor_val, 5)

            # --- MUX_CE and MUX_SR (shared) ---
            if sitewide_ram is None:
                ce_pin = resolve_shared_ff_control(
                    list(ff_slices.values()), prefix, 'ce_pin', 'ce_net', 8, "MUX_CE"
                )
                sr_pin = resolve_shared_ff_control(
                    list(ff_slices.values()), prefix, 'sr_pin', 'sr_net', 1, "MUX_SR"
                )
                ce_nets = {ff.get('ce_net', '') for ff in ff_slices.values() if ff.get('ce_net')}
                ce_xor = 1 if ce_nets == {'$PACKER_VCC'} else 0
                emit_mux_sel_with_xor(f, prefix, "MUX_CE", ce_pin, ce_xor, 5)
                emit_mux_sel_with_xor(f, prefix, "MUX_SR", sr_pin, sr_xor, 5)

            # --- BC bits (LUT INIT / RAM INIT) ---
            if sitewide_ram is not None:
                bc_image = derive_ram32x1d_bc(sitewide_ram, lut_slices)
                for bit in range(256):
                    val = (bc_image >> bit) & 1
                    f.write(f"set_case_analysis {val} {prefix}/BC[{bit}]\n")
            else:
                for sl in range(4):
                    ram = ram_slices.get(sl)
                    lut = lut_slices.get(sl)
                    if ram is not None:
                        pins = ram['pins']
                        init = ram['init']
                        perm, _, _ = assign_pins(sl, pins)
                        new_init = permute_init(init, perm)
                    elif lut is not None:
                        pins = lut['pins']
                        init = lut['init']
                        perm, _, _ = assign_pins(sl, pins)
                        new_init = permute_init(init, perm)
                    else:
                        new_init = 0

                    # IS_D_INVERTED absorption: invert the LUT truth table so
                    # the dedicated LUT→FF path delivers ~D to the FF, which
                    # is equivalent to the Verilog IS_D_INVERTED attribute.
                    ff = ff_slices.get(sl)
                    if ff is not None and ff.get('is_d_inverted', 0):
                        if lut is None and ram is None:
                            raise UnsupportedBackendFeature(
                                f"{prefix}: slice {SLICE_LETTER[sl]} FF has IS_D_INVERTED "
                                f"but no colocated LUT to absorb the inversion"
                            )
                        new_init ^= (1 << 64) - 1

                    # The truth tables sit in BC in reverse slice order: chunk 3
                    # (BC[192:255]) is slice A, chunk 0 is slice D -- the vendor's
                    # osc_probe keeps its slice-D inverter in BC[0:63] (its Q exits
                    # through DQ), and eflx-port's LUT_ON gating says the same.
                    for bit in range(64):
                        bc_idx = (3 - sl) * 64 + bit
                        val = (new_init >> bit) & 1
                        f.write(f"set_case_analysis {val} {prefix}/BC[{bc_idx}]\n")

            # --- WE_SEL (memory CLBs only, 3 bits) ---
            # The site-wide single-RBB RAM32X1D path now derives the locally
            # RE-backed cases from the exported logical port topology. Other
            # L1 memory modes still fall back to the older placeholder until
            # they have equally grounded semantics. A memory CLB used as plain
            # logic carries 0, as the vendor writes it.
            if is_memory_clb:
                if sitewide_ram is not None:
                    we_sel_val = derive_sitewide_ram32x1d_we_sel(sitewide_ram)
                elif ram_slices:
                    we_sel_val = 1
                else:
                    we_sel_val = 0
                for bit in range(3):
                    val = (we_sel_val >> bit) & 1
                    f.write(f"set_case_analysis {val} {prefix}/WE_SEL[{bit}]\n")

            # --- LUT_IPT_SEL (memory CLBs only, 5 bits) ---
            # The site-wide single-RBB RAM32X1D path uses a fixed vendor-backed
            # shape. Other L1 memory modes still reuse the same observed value
            # until their per-mode semantics are RE-backed.
            if is_memory_clb:
                if sitewide_ram is not None:
                    ipt_sel_val = derive_sitewide_ram32x1d_lut_ipt_sel(sitewide_ram)
                elif ram_slices:
                    ipt_sel_val = SITEWIDE_RAM32X1D_LUT_IPT_SEL
                else:
                    ipt_sel_val = 0
                for bit in range(5):
                    val = (ipt_sel_val >> bit) & 1
                    f.write(f"set_case_analysis {val} {prefix}/LUT_IPT_SEL[{bit}]\n")

            # --- CLK_SEL ---
            # Merge clock info from FFs and RAMs.
            clk_items = [{
                'clk_domain': ff.get('clk_domain', 0),
                'clk_net': ff.get('clk_net', ''),
                'clk_pin': ff.get('clk_pin', -1),
                'clk_token': ff.get('clk_token', ''),
            } for ff in ff_slices.values()]
            for ram in ram_slices.values():
                clk_items.append({
                    'clk_domain': ram.get('clk_domain', 0),
                    'clk_pin': ram.get('wclk_pin', -1),
                    'clk_net': ram.get('wclk_net', ''),
                    'clk_token': ram.get('clk_token', ''),
                })

            clk_domain = resolve_shared_clock_domain(clk_items, prefix)
            clk_pin = resolve_shared_ff_control(clk_items, prefix, 'clk_pin', 'clk_net', 0, "CLK")
            if sitewide_ram is not None:
                clk_sel_val = derive_sitewide_ram32x1d_clk_sel(sitewide_ram)
            else:
                clk_sel_val = clk_domain
                if clk_sel_val == 0 and clk_pin != 0 and not any(item.get('clk_token') for item in clk_items):
                    warn(f"{prefix}: shared CLK currently enters via site pin {clk_pin}, but CLK_SEL is still emitted as Clock_0")
            if clk_sel_val == 1 and 1 not in clock_sources:
                raise UnsupportedBackendFeature(
                    f"{prefix}: Clock_1 selected but no dedicated N1 clock source is constrained"
                )
            # 0 = Clock_0, 1 = Clock_1.
            f.write(f"set_case_analysis {clk_sel_val} {prefix}/CLK_SEL\n")

            # --- Slice-output mux state ---
            # Every slice of a used block gets its three selects, from its FFs
            # where it has them and the vendor's defaults otherwise: the vendor
            # writes all four slices of every block it uses (osc_probe's .vm:
            # OAQ_SEL 010, AMX_SEL 00010 / D 0001, AQ1_SEL 100000 / D 10000),
            # and a flip-flop in slice D stays static while A-C keep the
            # template's selects.
            sm_entries = slice_muxes_by_rbb.get((rbb_x, rbb_y), {})
            for sl in range(4):
                entry = sm_entries.get(sl, {})
                letter = SLICE_LETTER[sl]
                oaq_sel = int(entry.get('oaq_sel', 0b010))
                amx_sel = int(entry.get('amx_sel', 0b0001 if sl == 3 else 0b00010))
                aq1_sel = int(entry.get('aq1_sel', 0b10000 if sl == 3 else 0b100000))
                for name, value, nbits in (('OAQ_SEL', oaq_sel, 3),
                                           ('AMX_SEL', amx_sel, 4 if sl == 3 else 5),
                                           ('AQ1_SEL', aq1_sel, 5 if sl == 3 else 6)):
                    for fb in range(nbits):
                        f.write(f"set_case_analysis {(value >> fb) & 1} {prefix}/{name}_{letter}[{fb}]\n")
            # One more block field the SDC has no name for: bl10 wl31 of the
            # block's (col+1, row+1) tile, 0 in every block the vendor uses
            # (1 in the template); with it at 1 the flip-flop never toggles.
            emit_direct_nvm_bit(f, *_word_bit(rbb_y + 1, rbb_x + 1, 10, 31), 0)

        # --- Side-/north-edge IOB outputs ---
        # These use a different route-aware mux scheme than CLB rows. The
        # recovered mapping is keyed by the terminal D0 branch feeding the L6
        # PAD input, which nextpnr now exports in placement.json.
        for iob in placement.get('iobs', []):
            gx = int(iob.get('gx', -1))
            gy = int(iob.get('gy', -1))
            direction = iob.get('dir', 'inout')

            if direction == 'inout':
                raise UnsupportedBackendFeature(
                    f"IOB at ({gx},{gy}) uses bidirectional PAD mode, which is not yet RE-backed"
                )

            if direction != 'output':
                continue

            if gx not in (0, 31) and gy != 31:
                continue

            # West-edge IOBs with constant nets ($PACKER_GND / $PACKER_VCC) get
            # their constant-output configuration entirely at the IOB layer
            # (no fabric-routing mux), so emitting RBB_0_<gy>/MUX_O0..CE here
            # would land bits the vendor never writes — verified against
            # iob_const_00 / osc_probe vendor probes which both leave every
            # west RBB_0_<gy> empty except the dedicated clock-enable pad at
            # (0, 25).
            net = iob.get('net', '')
            if (gx == 0 and gy != 25 and
                    net in ('$PACKER_GND', '$PACKER_VCC')):
                # Still emit any IOB-level NVM bits (e.g. west IOB output
                # NVM bits for non-(0,25) positions) — only the per-RBB
                # MUX writes are vendor-empty for these.
                for reg, bit, value in derive_edge_iob_output_nvm_bits(iob):
                    emit_direct_nvm_bit(f, reg, bit, value)
                continue

            prefix = f"TILE_00/RBB_{gx}_{gy}"
            mux_o0, shared = derive_edge_iob_output_muxes(iob)
            # The output mux selects the D0 branch the data was routed through;
            # route_bits programs the fabric up to that D0 node, not this mux.
            emit_mux_sel(f, prefix, "MUX_O0", mux_o0, 4)
            emit_mux_sel(f, prefix, "MUX_O1", shared, 4)
            emit_mux_sel(f, prefix, "MUX_SR", shared, 4)
            emit_mux_sel(f, prefix, "MUX_CE", shared, 4)

            for reg, bit, value in derive_edge_iob_output_nvm_bits(iob):
                emit_direct_nvm_bit(f, reg, bit, value)


def land_on_lut_inputs(placement, routed):
    """Point each LUT/RAM pin at the LUT input its net actually lands on.

    The pool pins X[0..3] of a slice share one 4:1 fan-in, so the LUT input a
    net reaches is fixed by the node that delivers it (route_bits), not by the
    pool pin nextpnr names. `routed` = [[sx, sy, pin, lut_input]]; a pin is
    rewritten to the pool pin of that input, which assign_pins maps 1:1.
    """
    lands = {(sx, sy, pin): pin - pin % 6 + k for sx, sy, pin, k in routed}
    for cell in placement['luts'] + placement.get('rams', []):
        cell['pins'] = [lands.get((cell['sx'], cell['sy'], p), p) if p >= 0 else p
                        for p in cell['pins']]


def main():
    parser = argparse.ArgumentParser(description='Convert nextpnr placement to SDC')
    parser.add_argument('placement', help='Placement JSON from nextpnr')
    parser.add_argument('-o', '--output', required=True, help='Output SDC file')
    parser.add_argument('--lut-inputs', required=True,
                        help='route_bits output: the LUT input each routed pool pin lands on')
    args = parser.parse_args()

    with open(args.placement) as f:
        placement = json.load(f)
    with open(args.lut_inputs) as f:
        land_on_lut_inputs(placement, json.load(f))
    # nextpnr exports an IOB at its graph position (x, y); the IOB tables here
    # key it the way the vendor names it, RBB_<row>_<col> = (y, x) -- GPIO10 is
    # RBB_31_25, the north-edge IOB at graph (25, 31).
    for iob in placement.get('iobs', []):
        iob['gx'], iob['gy'] = iob['gy'], iob['gx']

    try:
        emit_sdc(placement, args.output)
    except UnsupportedBackendFeature as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(2)

    print(f"Wrote SDC to {args.output}")

    # Print stats.
    n_luts = len(placement['luts'])
    n_ffs = len(placement.get('ffs', []))
    n_rams = len(placement.get('rams', []))
    n_rbbs = len(set(
        [site_to_rbb(l['sx'], l['sy'], l.get('site_type', 'L0')) for l in placement['luts']] +
        [site_to_rbb(ff['sx'], ff['sy'], ff.get('site_type', 'L0')) for ff in placement.get('ffs', [])] +
        [site_to_rbb(r['sx'], r['sy'], r.get('site_type', 'L0')) for r in placement.get('rams', [])]
    ))
    print(f"  {n_luts} LUTs, {n_ffs} FFs, {n_rams} RAMs across {n_rbbs} RBBs")


if __name__ == '__main__':
    main()
