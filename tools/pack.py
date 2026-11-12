#!/usr/bin/env python3
"""
Bitstream packer for the Renesas SLG47910 (ForgeFPGA).

Takes PnR SDC output (set_case_analysis) and produces a bitstream .bin file
by applying known SDC to bitstream mappings.

Mapping sources (tried in order):
  1. Per-instance DB: fuzzing-derived mappings from fuzz/database/*_mappings.json
  2. Global CLK mux map: hardcoded verified CLK_8 bits plus defaults
  3. Edge IOB mux formula: perimeter MUX_O0/O1/CE/SR encoding
  4. Per-RBB CLK_SEL formulas: row 0/1/30, west-edge, east-edge clock selects
  5. Row-type formulas: generalized patterns from fuzz/database/row_formulas.json
  6. BC formulaic: generalized BC bit encoding for all CLB rows 2-28 (even)

Usage:
    python3 tools/pack.py <sdc_file> -o output.bin
    python3 tools/pack.py <sdc_file> -o output.bin --template empty.bin
    python3 tools/pack.py <sdc_file> -o output.bin --verify reference.bin
    python3 tools/pack.py <sdc_file> -o output.bin --mcu output_mcu.bin
"""

import argparse
import json
import re
import struct
import sys
from collections import defaultdict
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(PROJECT_ROOT))
from bitgen import coords  # noqa: E402

# Default template: empty bitstream from eda-placer
DEFAULT_TEMPLATE = PROJECT_ROOT / "device" / "template.bin"
DEFAULT_DB = PROJECT_ROOT / "device"

BITSTREAM_WORDS = 11264  # 45056 bytes
NVM_WORDS = 15           # first 15 words are NVM registers

# Supplemental per-instance mappings recovered from local vendor SDC/bitstream
# pairs. These fill a small gap where the generated row formulas do not yet
# cover the row-24 A5/A6/CI mux families at RBB_16_24.
_SUPPLEMENTAL_DB_MAPPINGS = {
    "RBB_16_24/MUX_A5/ASEL": {
        "word_idx": 6162, "bit": 9, "x": 17, "y": 16, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_A5/BSEL": {
        "word_idx": 6173, "bit": 9, "x": 17, "y": 17, "pos": 2, "inverted": True,
    },
    "RBB_16_24/MUX_A5/CSEL": {
        "word_idx": 6184, "bit": 9, "x": 17, "y": 18, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_A5/DSEL": {
        "word_idx": 6195, "bit": 9, "x": 17, "y": 19, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_A6/ASEL": {
        "word_idx": 6206, "bit": 12, "x": 17, "y": 20, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_A6/BSEL": {
        "word_idx": 6217, "bit": 12, "x": 17, "y": 21, "pos": 2, "inverted": True,
    },
    "RBB_16_24/MUX_A6/CSEL": {
        "word_idx": 6228, "bit": 12, "x": 17, "y": 22, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_A6/DSEL": {
        "word_idx": 6239, "bit": 12, "x": 17, "y": 23, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_CI/ASEL": {
        "word_idx": 6261, "bit": 11, "x": 17, "y": 25, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_CI/BSEL": {
        "word_idx": 6272, "bit": 11, "x": 17, "y": 26, "pos": 2, "inverted": True,
    },
    "RBB_16_24/MUX_CI/CSEL": {
        "word_idx": 6283, "bit": 11, "x": 17, "y": 27, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_CI/DSEL": {
        "word_idx": 6294, "bit": 11, "x": 17, "y": 28, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_CI/ESEL": {
        "word_idx": 6305, "bit": 11, "x": 17, "y": 29, "pos": 2, "inverted": False,
    },
    "RBB_16_24/MUX_CI/XOR": {
        "word_idx": 6316, "bit": 11, "x": 17, "y": 30, "pos": 2, "inverted": False,
    },
}


# ---------------------------------------------------------------------------
# Bitstream addressing
# ---------------------------------------------------------------------------

def xyp_to_word(x, y, pos):
    """Convert (X, Y, pos) to flat word index."""
    return x * 32 * 11 + y * 11 + pos


def word_to_xyp(word_idx):
    """Convert flat word index to (X, Y, pos)."""
    pos = word_idx % 11
    rem = word_idx // 11
    y = rem % 32
    x = rem // 32
    return x, y, pos


# ---------------------------------------------------------------------------
# Formulaic BC bit mapping — generalized for ALL CLB rows
# ---------------------------------------------------------------------------
# Verified 100% accurate against 8,768 BC mappings from fuzzing DBs for the
# generic CLB rows. Row 26 follows the same extrapolated pattern. Row 28 also
# follows it except for BRAM-adjacent A1/B0/C1/D0 chunk families, which use
# the same (X, Y, bit) lanes but land in pos 0 instead of pos 1.
#
# Key concepts:
#   ri = (rbb_y - 2) / 2       row index 0..13
#   pair_idx = ri // 2          pair index 0..6
#   is_Q = pair_idx % 4 == 1   "Q-type" rows (6,8,22,24) vs "P-type" (rest)
#   group = ri // 4             group index 0..3
#   sub = ri % 4                sub-index within group 0..3
#
# P-type: x_offset by slice (A/B=+2, C/D=0)
# Q-type: x_offset by chunk (ch0-1=0, ch2-3=+2)
# ---------------------------------------------------------------------------

# Y-row scrambling within 4-element blocks
Y_PERM = {0: 0, 1: 3, 2: 1, 3: 2}

# POS lookup table indexed by ri (row index)
# Pattern: decreases with diff sequence 0,-1,-1,0,-1,-1,...
_POS_TABLE = [9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 1]

# Vendor FIFO BRAM references show a row-28 special case at the BRAM-adjacent
# interior columns. The chunk families that normally land at pos 1 / bit 24
# move to pos 0 while keeping the same X/Y lane and bit index.
_BC_BRAM_ROW28_POS0_X = {4, 10, 13, 16}
_BC_BRAM_ROW28_POS0_FAMILIES = {
    (0, 1),  # A chunk 1
    (1, 0),  # B chunk 0
    (2, 1),  # C chunk 1
    (3, 0),  # D chunk 0
}


def _bc_base_bit(ri):
    """Compute the base bit (c0) for Slice A chunk 0 at a given row index."""
    group = ri // 4
    sub = ri % 4
    pair_idx = ri // 2
    is_q = pair_idx % 4 == 1
    return (7 + 6 * sub + 8 * group + 2 * int(is_q)) % 32


def _bc_chunk_bits(ri, is_q):
    """Compute (bit_A0, bit_A1, bit_A2, bit_A3) for Slice A."""
    c0 = _bc_base_bit(ri)
    sub = ri % 4
    if is_q:
        return c0, (c0 + 1) % 32, (c0 - 2) % 32, (c0 - 2) % 32
    elif sub == 0:
        return c0, (c0 + 19) % 32, (c0 + 17) % 32, (c0 + 18) % 32
    elif sub == 1:
        return c0, (c0 + 19) % 32, (c0 + 1) % 32, (c0 + 2) % 32
    else:  # sub == 2 or 3
        return c0, (c0 + 3) % 32, (c0 + 1) % 32, (c0 + 2) % 32


def bc_to_bitstream(rbb_x, rbb_row, bc_idx):
    """Convert a BC bit index to bitstream (word_idx, bit).

    Covers all CLB rows (rbb_row 2..28 even). Returns (word_idx, bit, False)
    or None if the row is not a CLB row.
    """
    if rbb_row < 2 or rbb_row > 28 or rbb_row % 2 != 0:
        return None

    ri = (rbb_row - 2) // 2  # 0..13
    pair_idx = ri // 2
    is_q = pair_idx % 4 == 1

    pos_main = _POS_TABLE[ri]

    # Decompose BC index
    slice_idx = bc_idx // 64        # 0=A, 1=B, 2=C, 3=D
    local_idx = bc_idx % 64
    chunk = local_idx // 16         # 0..3
    pos_in_chunk = local_idx % 16
    block = pos_in_chunk // 4       # 0..3
    offset = pos_in_chunk % 4       # 0..3

    # --- POS ---
    # Row 2 exception: chunk 0 of A/C (and chunk 1 of B/D) use pos_main+1
    if ri == 0:
        if (slice_idx in (0, 2) and chunk == 0) or \
           (slice_idx in (1, 3) and chunk == 1):
            pos = pos_main + 1  # = 10
        else:
            pos = pos_main      # = 9
    elif ri == 13:
        # Row 28 mirrors row 2: chunk 1 of A/C and chunk 0 of B/D sit one
        # position lower (816 vendor builds: 378 of 378 settings there; and on
        # silicon slices B and D of that row were dead with pos_main).
        if (slice_idx in (0, 2) and chunk == 1) or \
           (slice_idx in (1, 3) and chunk == 0):
            pos = pos_main - 1
        else:
            pos = pos_main
    else:
        pos = pos_main

    if (rbb_row == 28 and rbb_x in _BC_BRAM_ROW28_POS0_X and
            (slice_idx, chunk) in _BC_BRAM_ROW28_POS0_FAMILIES):
        pos = 0

    # --- BIT ---
    a_bits = _bc_chunk_bits(ri, is_q)  # (A0, A1, A2, A3)

    if is_q:
        # Q-type: each slice has its own bit offsets relative to c0
        c0 = a_bits[0]
        slice_bits = {
            0: a_bits,  # A
            1: (c0, (c0 + 1) % 32, (c0 - 1) % 32, (c0 - 1) % 32),  # B
            2: ((c0 - 1) % 32, (c0 - 2) % 32, c0, c0),              # C
            3: ((c0 - 1) % 32, (c0 - 2) % 32, (c0 + 1) % 32, (c0 + 1) % 32),  # D
        }
        bit = slice_bits[slice_idx][chunk]
    else:
        # P-type: B swaps chunks 0<->1 and 2<->3 relative to A; C=A, D=B
        if slice_idx in (0, 2):  # A or C
            bit = a_bits[chunk]
        else:  # B or D
            swapped = (1, 0, 3, 2)
            bit = a_bits[swapped[chunk]]

    # --- X_OFFSET ---
    if is_q:
        x_offset = 0 if chunk < 2 else 2
    else:
        x_offset = 2 if slice_idx < 2 else 0  # A/B=+2, C/D=+0

    # --- Y ---
    if is_q:
        # Q-type Y-half depends on both slice and chunk
        if chunk == 3:
            y_base = 16  # always upper
        elif chunk == 2:
            y_base = 0   # always lower
        elif slice_idx in (0, 2):
            y_base = 16  # A/C upper
        else:
            y_base = 0   # B/D lower

        # Q-type chunks 2,3 use identity permutation
        if chunk >= 2:
            y = y_base + block * 4 + offset
        else:
            y = y_base + block * 4 + Y_PERM[offset]
    else:
        # P-type: slice determines Y half
        if slice_idx in (0, 2):
            y_base = 16
        else:
            y_base = 0
        y = y_base + block * 4 + Y_PERM[offset]

    x = rbb_x + x_offset
    word_idx = xyp_to_word(x, y, pos)
    return word_idx, bit, False


# ---------------------------------------------------------------------------
# Load mapping database
# ---------------------------------------------------------------------------

def load_mapping_db(db_dir):
    """Load all mapping databases from fuzzing results.

    Returns dict: {sdc_path: {word_idx, bit, inverted}}
    """
    mappings = {}
    for json_path in sorted(db_dir.glob("*_mappings.json")):
        try:
            data = json.loads(json_path.read_text())
        except (json.JSONDecodeError, OSError) as e:
            print(f"Warning: could not load {json_path}: {e}", file=sys.stderr)
            continue

        for sdc_path, info in data.get("mappings", {}).items():
            mappings[sdc_path] = {
                "word_idx": info["word_idx"],
                "bit": info["bit"],
                "inverted": info["inverted"],
            }

    for sdc_path, info in _SUPPLEMENTAL_DB_MAPPINGS.items():
        mappings.setdefault(sdc_path, {
            "word_idx": info["word_idx"],
            "bit": info["bit"],
            "inverted": info["inverted"],
        })

    return mappings


# ---------------------------------------------------------------------------
# Row-type formula lookup (from tools/generalize.py output)
# ---------------------------------------------------------------------------

# The 12 RBB X columns in the fabric
RBB_X_LIST = [0, 1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31]
_X_TO_IDX = {x: i for i, x in enumerate(RBB_X_LIST)}


def load_row_formulas(db_dir):
    """Load generalized row-type formulas from row_formulas.json.

    Returns dict: {"row:signal": formula_dict} or empty dict.
    """
    path = db_dir / "row_formulas.json"
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text())
        return data.get("formulas", {})
    except (json.JSONDecodeError, OSError):
        return {}


def row_formula_to_bitstream(formula, rbb_x):
    """Apply a row-type formula to predict bitstream position.

    Returns (word_idx, bit, inverted) or None.
    """
    x = rbb_x + formula['x_off']
    pos = formula['pos']
    bit = formula['bit']
    inv = formula['inv']

    if formula['type'] == 'y_constant':
        y = formula['y']
    elif formula['type'] == 'y_parity':
        idx = _X_TO_IDX.get(rbb_x)
        if idx is None:
            return None
        if idx % 2 == 0:
            y_local = formula.get('y_upper')
            if y_local is None:
                return None
            y = y_local + 16
        else:
            y = formula.get('y_lower')
            if y is None:
                return None
    else:
        return None

    word_idx = xyp_to_word(x, y, pos)
    return word_idx, bit, inv


# ---------------------------------------------------------------------------
# Formulaic interior CLB mux encoding
# ---------------------------------------------------------------------------
# The interior CLB mux selectors (MUX_x5 / MUX_x6 / MUX_xI / MUX_CE / MUX_SR
# at the even columns 2-28) follow one regular encoding, which holds for every
# one of them in all 816 vendor builds of the corpus (row_formulas.json has no
# entries for them: see device/PROVENANCE.md).

_INTERIOR_MUX_SEL4 = {
    "ASEL": 0,
    "BSEL": 1,
    "CSEL": 2,
    "DSEL": 3,
}

_INTERIOR_MUX_SEL6 = {
    **_INTERIOR_MUX_SEL4,
    "ESEL": 4,
    "XOR": 5,
}

_INTERIOR_MUX_POS_5 = [10, 9, 8, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 1]
_INTERIOR_MUX_POS_6 = [9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 0]
_INTERIOR_MUX_POS_OTHER = [9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 1]

_INTERIOR_MUX_5_BASES = {
    "MUX_A5": (0, 0),
    "MUX_B5": (4, 4),
    "MUX_C5": (8, 8),
    "MUX_D5": (12, 12),
}

_INTERIOR_MUX_6_BASES = {
    "MUX_A6": (0, 4),
    "MUX_B6": (4, 8),
    "MUX_C6": (8, 12),
}

# MUX_D6 is the odd one out: it sits in the MUX_x5 bit lane and pos table, at
# Y 16 (odd column index) or 12 (even), not 16 above its lower base.
_INTERIOR_MUX_D6_Y = (12, 16)

_INTERIOR_MUX_CONST_Y = {
    "MUX_AI": 18,
    "MUX_BI": 18,
    "MUX_CI": 25,
    "MUX_DI": 25,
    "MUX_CE": 11,
    "MUX_SR": 11,
}

_INTERIOR_MUX_RE = re.compile(
    r'^(MUX_[ABCD][56]|MUX_[ABCD]I|MUX_CE|MUX_SR)/(ASEL|BSEL|CSEL|DSEL|ESEL|XOR)$'
)

def _interior_mux_row_bit(ri, base_by_sub):
    """Shared row-index to bit-position mapping for interior CLB muxes."""
    sub = ri % 4
    group = ri // 4
    return (base_by_sub[sub] + 8 * group) % 32


def interior_mux_to_bitstream(rbb_x, rbb_row, signal):
    """Convert an interior CLB mux selector to bitstream position."""
    if rbb_x in (0, 31):
        return None
    if rbb_row < 2 or rbb_row > 28 or (rbb_row % 2) != 0:
        return None

    ri = (rbb_row - 2) // 2
    x = rbb_x + 1

    sync = re.fullmatch(r"([ABCD])QsyncSRen", signal)
    if sync:
        # Whether a slice's flip-flop takes the block's set/reset (on the
        # clock). The vendor names it in its simulation netlist only (*.vm),
        # not among its settings; the bit follows the name in every block of
        # 200 builds. Left at 0, a routed reset did nothing on silicon.
        sl = "ABCD".index(sync.group(1))
        pos, lanes = ((_INTERIOR_MUX_POS_OTHER, [27, 1, 23, 29]) if sl % 2 == 0
                      else (_INTERIOR_MUX_POS_5, [6, 12, 18, 24]))
        return (xyp_to_word(rbb_x + (0 if sl < 2 else 2), 0, pos[ri]),
                _interior_mux_row_bit(ri, lanes), False)

    if signal == "MUX_CE/CIN":
        # What MUX_CE's fifth input (ESEL) is: 1, the clock enable of the
        # block below in the column (the vendor's CEin chain); 0, a constant.
        # The vendor names no setting for it. Its bit is 0 in every block
        # whose flip-flops have a constant clock enable (MUX_CE = ESEL, XOR:
        # 5,274 of 5,274 in the corpus) and 1 in every other (8,827 of
        # 8,827). Left at 1, a flip-flop above a block in use stood still on
        # silicon.
        return (xyp_to_word(x, 31, _INTERIOR_MUX_POS_OTHER[ri]),
                _interior_mux_row_bit(ri, [24, 14, 20, 26]), False)

    match = _INTERIOR_MUX_RE.match(signal)
    if not match:
        return None

    mux_name, sel = match.groups()
    idx = _X_TO_IDX.get(rbb_x)
    if idx is None:
        return None

    inverted = (sel == "BSEL")

    if mux_name in _INTERIOR_MUX_5_BASES:
        sel_offset = _INTERIOR_MUX_SEL4.get(sel)
        if sel_offset is None:
            return None
        pos = _INTERIOR_MUX_POS_5[ri]
        bit = _interior_mux_row_bit(ri, [7, 13, 19, 25])
        y_lower, y_upper = _INTERIOR_MUX_5_BASES[mux_name]
        y = (y_upper + 16 if (idx % 2) == 0 else y_lower) + sel_offset
        return xyp_to_word(x, y, pos), bit, inverted

    if mux_name in _INTERIOR_MUX_6_BASES:
        sel_offset = _INTERIOR_MUX_SEL4.get(sel)
        if sel_offset is None:
            return None
        pos = _INTERIOR_MUX_POS_6[ri]
        bit = _interior_mux_row_bit(ri, [26, 0, 22, 28])
        y_lower, y_upper = _INTERIOR_MUX_6_BASES[mux_name]
        y = (y_upper + 16 if (idx % 2) == 0 else y_lower) + sel_offset
        return xyp_to_word(x, y, pos), bit, inverted

    if mux_name == "MUX_D6":
        sel_offset = _INTERIOR_MUX_SEL4.get(sel)
        if sel_offset is None:
            return None
        pos = _INTERIOR_MUX_POS_5[ri]
        bit = _interior_mux_row_bit(ri, [7, 13, 19, 25])
        y = _INTERIOR_MUX_D6_Y[idx % 2] + sel_offset
        return xyp_to_word(x, y, pos), bit, inverted

    sel_offset = _INTERIOR_MUX_SEL6[sel]
    pos = _INTERIOR_MUX_POS_OTHER[ri]
    if mux_name in ("MUX_AI", "MUX_CI", "MUX_SR"):
        bit = _interior_mux_row_bit(ri, [25, 15, 21, 27])
    else:
        bit = _interior_mux_row_bit(ri, [24, 14, 20, 26])
    y = _INTERIOR_MUX_CONST_Y[mux_name] + sel_offset
    return xyp_to_word(x, y, pos), bit, inverted


# ---------------------------------------------------------------------------
# SDC parser
# ---------------------------------------------------------------------------

def parse_sdc(sdc_path):
    """Parse set_case_analysis entries from SDC file.

    Returns dict: {path_string: value} where value is 0 or 1.
    path_string is everything after TILE_00/.
    """
    result = {}
    pattern = re.compile(r'set_case_analysis\s+([01])\s+TILE_00/(.*)')
    with open(sdc_path) as f:
        for line in f:
            m = pattern.match(line.strip())
            if m:
                result[m.group(2)] = int(m.group(1))
    return result


_DIRECT_NVM_RE = re.compile(r'^NVM_R(\d+)_B(\d+)$')
_DIRECT_RAW_RE = re.compile(r'^RAW_W(\d+)_B(\d+)$')


def direct_nvm_to_bitstream(signal):
    """Map a synthetic direct-NVM signal onto the raw register words."""
    m = _DIRECT_NVM_RE.match(signal)
    if not m:
        m = _DIRECT_RAW_RE.match(signal)
        if not m:
            return None
        word = int(m.group(1))
        bit = int(m.group(2))
        if not (0 <= word < BITSTREAM_WORDS and 0 <= bit < 32):
            return None
        return word, bit, False

    reg = int(m.group(1))
    bit = int(m.group(2))
    if not (0 <= reg < NVM_WORDS and 0 <= bit < 32):
        return None
    return reg, bit, False


# ---------------------------------------------------------------------------
# Global CLK mux encoding
# ---------------------------------------------------------------------------
# 10 clock muxes (CLK_0..9), each with 4 SEL bits, plus CLK_9_INV.
# Empirically verified mapping from clock-diff fuzzing (make_clkmux):
#
# CLK_8 is the global clock enable mux. When a clock is used:
#   CLK_8 = {0,0,0,0}; when no clock: CLK_8 = {1,0,1,0}.
# Only bits 0 and 2 vary; bits 1 and 3 are always 0.
#
# CLK_0..7 and CLK_9 always have the same pattern {0,1,1,0} regardless
# of clock usage — their values match the empty template defaults.
# CLK_9_INV is always 0 (template default).
#
# Bitstream positions (verified across 4+ design pairs):
#   CLK_8_0_SEL → word 5912 (x=16, y=25, pos=5), bit 17, not inverted
#   CLK_8_2_SEL → word 5934 (x=16, y=27, pos=5), bit 19, not inverted

# Map from SDC signal name to (word_idx, bit, inverted)
_CLK_MUX_MAP = {
    "CLK_8_0_SEL": (5912, 17, False),
    "CLK_8_2_SEL": (5934, 19, False),
}

# Signals that are always at template defaults — mark as handled
_CLK_MUX_DEFAULT = set()
for _n in range(10):
    for _m in range(4):
        sig = f"CLK_{_n}_{_m}_SEL"
        if sig not in _CLK_MUX_MAP:
            _CLK_MUX_DEFAULT.add(sig)
_CLK_MUX_DEFAULT.add("CLK_9_INV")

_CLK_MUX_RE = re.compile(r'^CLK_\d+_\d+_SEL$|^CLK_\d+_INV$')


# ---------------------------------------------------------------------------
# Edge-column IOB mux encoding
# ---------------------------------------------------------------------------
# West/east edge RBBs use a compact row-local encoding for MUX_O0/O1/CE/SR
# selectors.  The selector letter chooses a fixed bitstream Y lane, while the
# RBB row chooses the shared (pos, bit) storage slot.  Rows 0/1/30/31 were
# recovered from targeted clean corner-output probes; the remaining rows came
# from the broader stored edge-mux fuzz corpus.

_EDGE_MUX_ROW_BITS = {
    0: (10, 17),
    1: (10, 12),
    2: (10, 7),
    3: (9, 26),
    4: (9, 13),
    5: (9, 0),
    6: (8, 19),
    7: (8, 20),
    8: (7, 25),
    9: (7, 26),
    10: (7, 15),
    11: (7, 2),
    12: (6, 21),
    13: (6, 8),
    14: (5, 27),
    15: (5, 30),
    16: (5, 1),
    17: (5, 4),
    18: (4, 23),
    19: (4, 10),
    20: (3, 29),
    21: (3, 16),
    22: (3, 3),
    23: (3, 4),
    24: (2, 9),
    25: (2, 10),
    26: (1, 31),
    27: (1, 18),
    28: (1, 5),
    29: (0, 24),
    30: (0, 11),
    31: (0, 6),
}

# Edge-aware Y_base: O0 and CE are identical on both edges, but O1 and SR
# shift by +1 on the east edge (X=31).  Verified 100% on 3,168 east-edge
# MUX_O/SR/CE SDC entries across 471 fuzz seeds.
_EDGE_MUX_Y_BASE_WEST = {
    "MUX_O0": 2,
    "MUX_O1": 9,
    "MUX_SR": 17,
    "MUX_CE": 26,
}
_EDGE_MUX_Y_BASE_EAST = {
    "MUX_O0": 2,
    "MUX_O1": 10,
    "MUX_SR": 18,
    "MUX_CE": 26,
}

_EDGE_MUX_SEL_OFFSET = {
    "ASEL": 0,
    "BSEL": 1,
    "CSEL": 2,
    "DSEL": 3,
}

_EDGE_MUX_INVERTED = {
    "ASEL": False,
    "BSEL": True,
    "CSEL": False,
    "DSEL": False,
}

_EDGE_MUX_RE = re.compile(r'^(MUX_O[01]|MUX_SR|MUX_CE)/(A|B|C|D)SEL(b?)$')
_EDGE_MUX_DEFAULT_ROWS = {0, 1, 30, 31}
_EDGE_MUX_DEFAULT_VALUE = {
    "ASEL": 1,
    "BSEL": 0,
    "CSEL": 0,
    "DSEL": 0,
}


def edge_mux_to_bitstream(rbb_x, rbb_row, signal):
    """Convert an edge-column IOB mux selector to bitstream position."""
    if rbb_x not in (0, 31):
        return None

    row_bits = _EDGE_MUX_ROW_BITS.get(rbb_row)
    if row_bits is None:
        return None

    match = _EDGE_MUX_RE.match(signal)
    if not match:
        return None

    mux_name = match.group(1)
    sel = f"{match.group(2)}SEL"
    is_complement = bool(match.group(3))

    y_base_table = _EDGE_MUX_Y_BASE_EAST if rbb_x == 31 else _EDGE_MUX_Y_BASE_WEST
    y = y_base_table[mux_name] + _EDGE_MUX_SEL_OFFSET[sel]
    pos, bit = row_bits
    inverted = _EDGE_MUX_INVERTED[sel] ^ is_complement
    return xyp_to_word(rbb_x, y, pos), bit, inverted


def edge_mux_is_template_default(rbb_x, rbb_row, signal, value):
    """Return True for observed edge mux default rows already in template."""
    if rbb_x not in (0, 31) or rbb_row not in _EDGE_MUX_DEFAULT_ROWS:
        return False

    match = _EDGE_MUX_RE.match(signal)
    if not match:
        return False

    sel = f"{match.group(2)}SEL"
    is_complement = bool(match.group(3))
    default_value = _EDGE_MUX_DEFAULT_VALUE[sel] ^ is_complement
    return value == default_value


# ---------------------------------------------------------------------------
# Per-RBB CLK_SEL encoding
# ---------------------------------------------------------------------------
# CLK_SEL is a single bit per RBB/IOB that selects clock domain 0 vs 1.
#
# Verified patterns:
#   - Top IOB row RBB_*_1:    (rbb_x, Y=26, pos=10, bit=23, noninv)
#   - Bottom IOB row RBB_*_0: (rbb_x, Y=26, pos=10, bit=28, noninv)
#   - Top penult row RBB_*_30:(rbb_x, Y=26, pos=0,  bit=22, noninv)
#   - West edge RBB_0_2..29:  (0, Y=16, pos/bit from _EDGE_MUX_ROW_BITS, inv)
#   - East edge RBB_31_*:     (31, Y=26, pos/bit from _EAST_EDGE_CLK_SEL, noninv)
#   - Interior RBB_{1,4,7,10,13,16,19,22,25}_{2..28}:
#       X_bs = rbb_x + 1, Y_bs = 25, noninv, bit_idx formula below.
#       Derived from make_multiclk fuzz (60 seeds, 100% verified on 60 obs).

# East-edge CLK_SEL bit positions per row.  Derived by brute-force correlation
# across all fuzz seeds.  Only rows with perfect (100%) match are included.
_EAST_EDGE_CLK_SEL = {
    # row: (pos, bit)  — all verified at X=31, Y=26, not inverted
    7:  (8, 11),   # 8/8 seeds
    8:  (8, 4),    # 7/7 seeds
    10: (7, 10),   # 8/8 seeds
}

# West-edge CLK_SEL bit positions per row.  Derived from make_multiclk fuzz
# (60 seeds).  All verified at X=0, Y=26, non-inverted (NOT the same as the
# MUX_O/CE/SR edge pattern which uses Y=16 inverted).
_WEST_EDGE_CLK_SEL = {
    # row: (pos, bit) — all verified at X=0, Y=26, not inverted
    8:  (8,  4),   # 7+ seeds
    9:  (7, 17),
    10: (7, 10),
    15: (5, 19),
    17: (4, 25),
    18: (4, 18),
    19: (4, 15),
}


def rbb_clk_sel_to_bitstream(rbb_x, rbb_row, signal):
    """Convert a covered per-RBB CLK_SEL signal to its bitstream positions.

    Returns a list of (word_idx, bit, inverted) or None.
    """
    if signal != "CLK_SEL":
        return None

    # Row 1 (top IOB): all columns
    if rbb_row == 1:
        return [(xyp_to_word(rbb_x, 26, 10), 23, False)]

    # Row 0 (bottom IOB): all columns except edges
    if rbb_row == 0 and rbb_x not in (0, 31):
        return [(xyp_to_word(rbb_x, 26, 10), 28, False)]

    # Row 30 (top penultimate): all columns except edges
    if rbb_row == 30 and rbb_x not in (0, 31):
        return [(xyp_to_word(rbb_x, 26, 0), 22, False)]

    # West edge (X=0), rows with verified data from make_multiclk fuzz.
    if rbb_x == 0:
        west_bits = _WEST_EDGE_CLK_SEL.get(rbb_row)
        if west_bits is not None:
            pos, bit = west_bits
            return [(xyp_to_word(0, 26, pos), bit, False)]

    # East edge (X=31), rows with verified data
    if rbb_x == 31:
        east_bits = _EAST_EDGE_CLK_SEL.get(rbb_row)
        if east_bits is not None:
            pos, bit = east_bits
            return [(xyp_to_word(31, 26, pos), bit, False)]

    # Interior CLBs (rbb_x in {1,4,7,10,13,16,19,22,25,28}), even rows 2..28.
    # Verified on make_multiclk plus vendor RAM-site probes at X=28.
    if rbb_x in (1, 4, 7, 10, 13, 16, 19, 22, 25, 28) and 2 <= rbb_row <= 28 and rbb_row % 2 == 0:
        ri = (rbb_row - 2) // 2
        bit_idx = 324 - 26 * ri + 16 * ((ri + 1) // 4)
        pos = bit_idx // 32
        bit = bit_idx % 32
        # CLK_SEL has a second copy in the RBB's middle tile, BL10 WL24 of its
        # first row in columns 1/7/../25 and WL31 of its second in 4/10/../28
        # (equal to CLK_SEL in every vendor build; 22 two-clock exceptions at
        # column 1). The template holds it at 1: left there, the flip-flops of
        # columns 19 and 25 never clocked.
        shadow = (coords.trace_to_word_bit(rbb_row, rbb_x + 1, 10, 24) if rbb_x % 6 == 1
                  else coords.trace_to_word_bit(rbb_row + 1, rbb_x + 1, 10, 31))
        return [(xyp_to_word(rbb_x + 1, 25, pos), bit, False), (*shadow, False)]

    return None


# ---------------------------------------------------------------------------
# Per-RBB WE_SEL encoding (memory CLB rows only)
# ---------------------------------------------------------------------------
# WE_SEL is a 3-bit field on memory CLBs that selects the write-enable source
# for distributed RAM (RAMSRL) cells.  The value depends on the full routing
# solution, not just placement.
#
# Bitstream position formula (verified 300 checks, 0 mismatches, 150 observations):
#   X_bs  = rbb_x + 1
#   Y_bs  = 26 for WE_SEL[0], 27 for WE_SEL[1]  (WE_SEL[2] not yet observed)
#   (pos, bit) = BC_position(ri) - 10  (absolute offset from BC slice-A chunk-0)
#
# Memory CLB rows: rbb_y ∈ {6, 8, 22, 24}  →  ri ∈ {2, 3, 10, 11}
#   rbb_y=6  (ri=2):  pos=8,  bit=11
#   rbb_y=8  (ri=3):  pos=7,  bit=17
#   rbb_y=22 (ri=10): pos=2,  bit=27
#   rbb_y=24 (ri=11): pos=2,  bit=1

_MEMORY_CLB_ROWS = {6, 8, 22, 24}

def _we_sel_pos_bit(rbb_row):
    """Compute (pos, bit) for WE_SEL at a given memory CLB row."""
    ri = (rbb_row - 2) // 2
    bc_pos = _POS_TABLE[ri]
    bc_c0 = _bc_base_bit(ri)
    bc_abs = bc_pos * 32 + bc_c0
    we_abs = bc_abs - 10
    return we_abs // 32, we_abs % 32


def we_sel_to_bitstream(rbb_x, rbb_row, signal):
    """Convert a WE_SEL[n] signal to its bitstream positions.

    Returns a list of (word_idx, bit, inverted) or None.
    """
    if rbb_row not in _MEMORY_CLB_ROWS:
        return None

    m = re.match(r'WE_SEL\[(\d+)\]', signal)
    if not m:
        return None

    sel_bit = int(m.group(1))
    if sel_bit > 2:
        return None

    pos, bit = _we_sel_pos_bit(rbb_row)
    # The local fuzz corpus only toggles WE_SEL[0:1], but the silicon notes say
    # the 3-bit field is fully solved.  Extend the observed contiguous Y-lane
    # pattern to bit 2 at Y=28.
    y_bs = 26 + sel_bit
    places = [(xyp_to_word(rbb_x + 1, y_bs, pos), bit, False)]
    if sel_bit == 0:
        # WE_SEL[0] has a second copy in the RBB's own column.  The template
        # holds it at 1 and the vendor clears it whenever a memory CLB is used
        # as plain logic; left set, the site's flip-flops stay dead on silicon
        # (rows 6/8/22/24 at column 28: static until this bit was cleared).
        places.append((xyp_to_word(rbb_x, y_bs, pos), bit, False))
    return places


def memory_clb_logic_bits(rbb_x, rbb_row):
    """The RAM defaults a memory CLB used as plain logic must drop.

    The template holds BL4 WL26-30 of a memory CLB's tiles at a column-dependent
    RAM default (11110 at columns 1/4/7/13, 11001 at 22/28, ...), where plain
    rows hold 0. The vendor clears WL26-29 in the four corner tiles and WL30 in
    the middle one whenever the CLB is plain logic (every such RBB of the
    vendor corpus; WL30 of the corners is content). Left set, the site's
    flip-flops stay dead: memory rows at columns 1/7/13 ran once cleared.
    Returns (word_idx, bit) pairs to clear.
    """
    out = []
    for dx in (0, 1):
        for dy in (0, 2):
            for wl in range(26, 30):
                out.append(coords.trace_to_word_bit(rbb_row + dx, rbb_x + dy, 4, wl))
    out.append(coords.trace_to_word_bit(rbb_row + 1, rbb_x + 1, 4, 30))
    return out


# ---------------------------------------------------------------------------
# Slice-output mux encoders (AQ1_SEL / AMX_SEL / OAQ_SEL)
# ---------------------------------------------------------------------------
# Per-slice MUX_XQ / MUX_X / MUX_OXQ source selects (vendor `defparam` names).
# See `docs/schematics/slice.v` for the structural reference and
# `docs/silicon/reference.md` §7.5 for the encoding write-up.
#
# Shared shape: every field bit lives at a single bitstream bit at
# `(X_bs = rbb_x + 1, Y_bs, pos, bit)` where `(pos, bit)` is a per-slice
# function of `ri = (rbb_y - 2) // 2` and `Y_bs` is per-(field, slice, fb,
# column-class). Bit 1 of *every* field is inverted in the bitstream (the
# "feedback / hold" mux input — XI, OXQ, XQ for AQ1/AMX/OAQ respectively).
# All other bits are non-inverted.
#
# Two column classes survive in vendor builds: `rbb_x % 6 ∈ {1, 4}` (10 CLB
# columns map to one of these). Slices A/B use a `column-invariant` Y_bs for
# AQ1_SEL and AMX_SEL; slices C/D and *all* OAQ_SEL slices change Y_bs by
# column class.
#
# Coverage: derived from `tools/mine_vm_corpus.py correlate --intersect
# --group y,x%6` (session 16d) + a per-(Y, inversion) bit-perfect scan over
# the full 1052-build vendor corpus; verified by the per-field scripts in
# `fuzz/pilot/xmux_xq/verify_{aq1,amx,oaq}_formula.py`.

# Per-slice (pos, bit) formula
# ----------------------------
# Slice A: pos = POS_TABLE[ri];                 bit = (c0 + _row_class_shift(ri)) % 32
# Slice B: pos = POS_TABLE[ri];                 bit = (slice-A bit - 1) % 32
# Slice C: pos = POS_TABLE[ri] (ri==13 → 0);    bit = (c0 + _slice_c_shift(ri)) % 32
# Slice D: pos = POS_TABLE[ri] (ri==0  → 10);   bit = (c0 + _slice_d_shift(ri)) % 32
#
# `_slice_c_shift` is the BC chunk-A1 offset (Q:+1, sub=0/1 non-Q:+19,
# sub=2/3 non-Q:+3). `_slice_d_shift` is 0 for non-Q, -2 for Q.
#
# OAQ_SEL routes through the OXQ FF. For slices A/B/C the OXQ-FF bits live at
# the *slice-C* `(pos, bit)` lane (verified row-by-row at the four exception
# rows). For slice D they live at the slice-D `(pos, bit)` lane.


def _row_class_shift(ri):
    """The per-row additive shift over `_bc_base_bit(ri)` for slice-A bits.

    Maps to the BC chunk-A3 position for non-Q rows, A0 for Q rows.
    Verified 100% via AQ1_SEL_A[5] / AMX_SEL_A[0,2,4] across the corpus.
    """
    pair_idx = ri // 2
    is_q = pair_idx % 4 == 1
    sub = ri % 4
    if is_q:
        return 0
    if sub == 0:
        return 18
    return 2


def _slice_c_shift(ri):
    """Per-row bit shift for slice-C bits (BC chunk-A1 offset)."""
    pair_idx = ri // 2
    is_q = pair_idx % 4 == 1
    sub = ri % 4
    if is_q:
        return 1
    if sub in (0, 1):
        return 19
    return 3  # sub=2,3 non-Q rows (ri=6,7)


def _slice_d_shift(ri):
    """Per-row bit shift for slice-D bits (BC chunk-A0 / A2 offset)."""
    pair_idx = ri // 2
    is_q = pair_idx % 4 == 1
    return -2 if is_q else 0


def _slice_pos_bit(slice_idx, ri):
    """Return (pos, bit) for a slice-output mux field at row index `ri`.

    `slice_idx` ∈ {0:A, 1:B, 2:C, 3:D}. Mirrors the conventions documented
    in `docs/silicon/reference.md` §7.5.
    """
    c0 = _bc_base_bit(ri)
    pos = _POS_TABLE[ri]
    if slice_idx == 0:       # A
        bit = (c0 + _row_class_shift(ri)) % 32
    elif slice_idx == 1:     # B
        bit = (c0 + _row_class_shift(ri) - 1) % 32
    elif slice_idx == 2:     # C
        bit = (c0 + _slice_c_shift(ri)) % 32
        if ri == 13:         # row 28: slice C pos decrements
            pos -= 1
    elif slice_idx == 3:     # D
        bit = (c0 + _slice_d_shift(ri)) % 32
        if ri == 0:          # row 2: slice D pos increments
            pos += 1
    else:
        return None
    return pos, bit


# Y_bs base per (slice_idx, x_mod_6) for each field; the field bit `fb` is
# stored at Y = base + fb. Both column classes survive in the corpus: {1, 4}.
# Empirically pinned to 100% match on every bit position with multi-value
# coverage; degenerate bits (always 0 in vendor synth) inherit the same
# linear-in-fb pattern.

_AQ1_SEL_Y_BASE = {  # (slice_idx, x_mod_6): Y_bs base
    (0, 1): 0,  (0, 4): 0,    # slice A: Y = fb (column-invariant)
    (1, 1): 0,  (1, 4): 0,    # slice B: same
    (2, 1): 18, (2, 4): 8,    # slice C: column-class dependent
    (3, 1): 24, (3, 4): 3,    # slice D
}

_AMX_SEL_Y_BASE = {
    (0, 1): 6,  (0, 4): 6,    # slice A: Y = 6 + fb (column-invariant)
    (1, 1): 6,  (1, 4): 6,    # slice B
    (2, 1): 24, (2, 4): 3,    # slice C
    (3, 1): 20, (3, 4): 8,    # slice D
}

# OAQ_SEL is column-class dependent on *all* slices.
_OAQ_SEL_Y_BASE = {
    (0, 1): 15, (0, 4): 17,   # slice A
    (1, 1): 12, (1, 4): 14,   # slice B
    (2, 1): 29, (2, 4): 0,    # slice C
    (3, 1): 29, (3, 4): 0,    # slice D
}

# For the OAQ_SEL family the bits live at the *slice-C* `(pos, bit)` lane for
# slices A/B/C, and the *slice-D* lane for slice D. AQ1_SEL/AMX_SEL use the
# corresponding slice's own lane.
_OAQ_SEL_LANE_SLICE = {0: 2, 1: 2, 2: 2, 3: 3}


def _xmux_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit,
                       y_base_table, lane_slice_table=None):
    """Shared per-slice MUX encoder used by AQ1_SEL / AMX_SEL / OAQ_SEL."""
    if rbb_row < 2 or rbb_row > 28 or rbb_row % 2 != 0:
        return None
    xc = rbb_x % 6
    if xc not in (1, 4):
        return None  # edge columns (x=0,31) not characterised
    key = (slice_idx, xc)
    if key not in y_base_table:
        return None
    ri = (rbb_row - 2) // 2
    lane = (lane_slice_table or {}).get(slice_idx, slice_idx)
    pb = _slice_pos_bit(lane, ri)
    if pb is None:
        return None
    pos, bit = pb
    y_bs = (y_base_table[key] + field_bit) % 32
    inverted = (field_bit == 1)  # the feedback/hold mux input is inverted
    return xyp_to_word(rbb_x + 1, y_bs, pos), bit, inverted


def aq1_sel_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit):
    """Convert AQ1_SEL_{A,B,C,D}[field_bit] to (word_idx, bit, inverted).

    Field width: 6 for slices A/B/C, 5 for slice D (no EXTRA_IN). Bit 1 (=
    XI source) is inverted in the bitstream; all other bits are not.
    Returns None for edge columns (x%6 ∉ {1,4}) and non-CLB rows.
    """
    return _xmux_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit,
                              _AQ1_SEL_Y_BASE)


def amx_sel_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit):
    """Convert AMX_SEL_{A,B,C,D}[field_bit] to (word_idx, bit, inverted).

    Field width: 5 for slices A/B/C, 4 for slice D. Bit 1 (= OXQ source) is
    inverted. Same `(pos, bit)` lane as AQ1_SEL per slice.
    """
    return _xmux_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit,
                              _AMX_SEL_Y_BASE)


def oaq_sel_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit):
    """Convert OAQ_SEL_{A,B,C,D}[field_bit] to (word_idx, bit, inverted).

    3-bit one-hot field for the OXQ FF's data input: {XI, XQ, OX0}. Bit 1
    (= XQ source) is inverted. Bits live at the slice-C `(pos, bit)` lane
    for slices A/B/C and the slice-D lane for slice D.
    """
    return _xmux_to_bitstream(rbb_x, rbb_row, slice_idx, field_bit,
                              _OAQ_SEL_Y_BASE, _OAQ_SEL_LANE_SLICE)


# ---------------------------------------------------------------------------
# Per-RBB LUT_IPT_SEL encoding (memory CLB rows; intentionally unresolved)
# ---------------------------------------------------------------------------
# The older row-6 formula was derived from generic RBB6M occupancy fuzzing.
# Vendor-backed RAM probes now show that sitewide memory modes use a different
# physical encoding, so applying that formula on the open RAM path makes the
# ground-truth comparison worse.  Until we have a truthful site-context model,
# do not write speculative bits. The current open sitewide RAM32X1D path only
# emits the vendor-observed fixed logical value 01000 (=8), which we treat as an
# accepted template-preserved shape.
def lut_ipt_sel_to_bitstream(rbb_x, rbb_row, signal):
    """Convert a covered LUT_IPT_SEL[n] signal to bitstream position."""
    del rbb_x, rbb_row, signal
    return None


def collect_lut_ipt_values(sdc_data):
    """Collect per-RBB LUT_IPT_SEL field values from SDC entries."""
    lut_ipt_re = re.compile(r'RBB_(\d+)_(\d+)/LUT_IPT_SEL\[(\d+)\]')
    values = defaultdict(int)
    for sdc_path, value in sdc_data.items():
        m = lut_ipt_re.search(sdc_path)
        if not m:
            continue
        rbb_x = int(m.group(1))
        rbb_row = int(m.group(2))
        bit = int(m.group(3))
        values[(rbb_x, rbb_row)] |= (value & 1) << bit
    return values


def lut_ipt_sel_is_known_fixed_shape(rbb_x, rbb_row, signal, value, lut_ipt_values):
    """Return True for the vendor-grounded LUT_IPT_SEL shapes of a memory CLB.

    A memory CLB used as plain logic carries 00000 and the sitewide
    single-RBB RAM32X1D mode the fixed logical value 01000 (=8), in every
    vendor-backed example. Preserve the template bitstream shape for both
    instead of reporting the field as unmapped.
    """
    if rbb_row not in _MEMORY_CLB_ROWS:
        return False

    m = re.match(r'LUT_IPT_SEL\[(\d+)\]', signal)
    if not m:
        return False

    if lut_ipt_values.get((rbb_x, rbb_row)) not in (0, 8):
        return False

    sel_bit = int(m.group(1))
    return ((8 >> sel_bit) & 1) == value


# ---------------------------------------------------------------------------
# BRAM-adjacent template-preserved aliases
# ---------------------------------------------------------------------------
# Two categories remain after the main BRAM-adjacent mux recovery:
#
# _BRAM_DEFAULT_SKIP_SIGNALS: the generic formula would write the wrong raw
# value, but the empty-bitstream template already holds the correct vendor
# value (template bit == vendor bit).  No route-template is needed — we just
# suppress the formula write entirely so the template bit is left untouched.
#
#   RBB_4_31/MUX_O1/CSEL: formula writes raw=0, template=1=vendor → skip.
#   RBB_16_28/MUX_D6/BSEL: formula writes raw=0, template=1=vendor → skip.
#
# _BRAM_TEMPLATE_PRESERVE_SIGNALS: the generic formula is wrong AND the
# template bit does not hold the correct vendor value.  The only source of
# truth is the vendor-routed bitstream, so we must copy it from the
# route-template.
#
#   RBB_1_30/MUX_O1/BSEL: the BRAM output routing clears a bit that the
#   non-BRAM convention leaves high.  The correct raw value (0) differs from
#   the template (1) and depends on the BRAM routing solution, not just the
#   SDC value.

_BRAM_DEFAULT_SKIP_SIGNALS = {
    (4, 31): {"MUX_O1/CSEL"},
    (16, 28): {"MUX_D6/BSEL"},
}

_BRAM_TEMPLATE_PRESERVE_SIGNALS = {
    (1, 30): {"MUX_O1/BSEL"},
}


def bram_default_skip_signal(rbb_x, rbb_row, signal):
    """Return True for BRAM signals whose template bit is already correct."""
    signals = _BRAM_DEFAULT_SKIP_SIGNALS.get((rbb_x, rbb_row))
    return signals is not None and signal in signals


def bram_template_preserve_signal(rbb_x, rbb_row, signal):
    """Return True for BRAM-adjacent aliases to preserve from route template."""
    signals = _BRAM_TEMPLATE_PRESERVE_SIGNALS.get((rbb_x, rbb_row))
    return signals is not None and signal in signals


# ---------------------------------------------------------------------------
# Packer core
# ---------------------------------------------------------------------------

def pack_bitstream(sdc_data, template_words, db_mappings, row_formulas=None,
                   route_template_words=None, route_claims=None):
    """Pack SDC data into a bitstream.

    Args:
        sdc_data: dict from parse_sdc()
        template_words: list of 32-bit words (empty bitstream)
        db_mappings: dict from load_mapping_db()
        row_formulas: dict from load_row_formulas() (optional)
        route_template_words: list of 32-bit words from placer bitstream to
            copy SM routing bits (pos 0-4) from (optional)

    Returns:
        (words, stats) where words is the packed bitstream and stats is a dict
        with packing statistics.
    """
    words = list(template_words)  # copy
    if row_formulas is None:
        row_formulas = {}

    stats = {
        "total_sdc": len(sdc_data),
        "packed_formulaic": 0,
        "packed_interior_mux": 0,
        "packed_row_formula": 0,
        "packed_db": 0,
        "packed_clk": 0,
        "packed_direct_nvm": 0,
        "packed_edge_mux": 0,
        "packed_rbb_clk": 0,
        "packed_we_sel": 0,
        "packed_lut_ipt": 0,
        "packed_slice_mux": 0,
        "skipped_slice_mux_uncovered": 0,
        "packed_bram_nvm": 0,
        "packed_bram_default_skip": 0,
        "packed_bram_template": 0,
        "skipped_complement": 0,
        "unmapped": 0,
        "unmapped_signals": [],
        "route_words_copied": 0,
    }

    # Apply SM routing bits from route template if provided.
    # The route template is expected to be derived from the same empty
    # bitstream template. We apply it as a delta: any bit that differs
    # between the route template and the base template is applied to the
    # output. This correctly handles SM bits that span all 11 pos values
    # without overwriting SDC-derived CLB configuration.
    if route_template_words is not None:
        for wi in range(min(len(words), len(route_template_words), len(template_words))):
            diff = route_template_words[wi] ^ template_words[wi]
            if diff:
                words[wi] ^= diff
                stats["route_words_copied"] += 1

    has_bram_hard_ip = any(
        re.match(r'RBB_\d+_(2[6-9]|3[01])/', path)
        for path in sdc_data
    )

    # Track which signals are complements. We only need to pack one side of
    # pairs like ASEL/ASELb and XOR/XORb.
    sel_complement_re = re.compile(r'(.+SEL)b$')
    xor_complement_re = re.compile(r'(.+XOR)b$')

    # Pre-compile the RBB path regex
    rbb_re = re.compile(r'RBB_(\d+)_(\d+)/(.*)')
    lut_ipt_values = collect_lut_ipt_values(sdc_data)

    def apply_bit(word_idx, bit, inverted, value):
        if word_idx < 0 or word_idx >= len(words) or bit < 0 or bit > 31:
            return False
        bit_val = value ^ inverted
        if bit_val:
            words[word_idx] |= (1 << bit)
        else:
            words[word_idx] &= ~(1 << bit)
        return True

    if has_bram_hard_ip:
        # Vendor BRAM designs require these NVM bits whenever rows 26-31 are
        # active. The open nextpnr flow does not emit them explicitly.
        apply_bit(193 // 32, 193 % 32, False, 1)  # reg_bram_keep=1
        apply_bit(320 // 32, 320 % 32, False, 0)  # bram_0_3_powerdown=0
        apply_bit(321 // 32, 321 % 32, False, 0)  # bram_4_7_powerdown=0
        stats["packed_bram_nvm"] = 3

    # Synthetic direct NVM/RAW overrides (RAW_W*_B* / NVM_R*_B*) are processed
    # in a final pass so they trump any conflicting bit landed earlier by the
    # row-formula / edge-mux tiers.  Some east-edge dynamic-output enable bits
    # share a physical word/bit with side-edge MUX_O1/BSEL etc., and the
    # inverted-mux encoding flips the value we wrote from the SDC raw override.
    direct_nvm_overrides = []
    for sdc_path, value in sorted(sdc_data.items()):
        d = direct_nvm_to_bitstream(sdc_path)
        if d is not None:
            direct_nvm_overrides.append((d, value))

    for sdc_path, value in sorted(sdc_data.items()):
        # Skip complement signals - they're redundant (SELb = ~SEL)
        # The non-inverted version carries the same info
        cm = sel_complement_re.match(sdc_path) or xor_complement_re.match(sdc_path)
        if cm and cm.group(1) in sdc_data:
            stats["skipped_complement"] += 1
            continue

        # Tier 1: per-instance DB (highest confidence — empirically verified)
        if sdc_path in db_mappings:
            m = db_mappings[sdc_path]
            apply_bit(m["word_idx"], m["bit"], m["inverted"], value)
            stats["packed_db"] += 1
            continue

        # Tier 2: global CLK mux signals
        if _CLK_MUX_RE.match(sdc_path):
            if sdc_path in _CLK_MUX_MAP:
                word_idx, bit, inverted = _CLK_MUX_MAP[sdc_path]
                apply_bit(word_idx, bit, inverted, value)
                stats["packed_clk"] += 1
                continue
            elif sdc_path in _CLK_MUX_DEFAULT:
                # Template already has correct default values
                stats["packed_clk"] += 1
                continue

        # Tier 2b: synthetic direct NVM/raw overrides — count here but apply
        # last to win over conflicting row-formula / edge-mux bits.
        direct_nvm = direct_nvm_to_bitstream(sdc_path)
        if direct_nvm:
            stats["packed_direct_nvm"] += 1
            continue

        # Tier 3: edge-column IOB mux formulas.  These are separate from the
        # row formulas because perimeter RBBs use different bit positions from
        # the interior CLB columns.
        rbb_match = rbb_re.match(sdc_path)
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)

            if has_bram_hard_ip and \
               bram_default_skip_signal(rbb_x, rbb_row, signal):
                stats["packed_bram_default_skip"] += 1
                continue

            if route_template_words is not None and has_bram_hard_ip and \
               bram_template_preserve_signal(rbb_x, rbb_row, signal):
                stats["packed_bram_template"] += 1
                continue

            result = edge_mux_to_bitstream(rbb_x, rbb_row, signal)
            if result:
                word_idx, bit, inverted = result
                apply_bit(word_idx, bit, inverted, value)
                stats["packed_edge_mux"] += 1
                continue
            if edge_mux_is_template_default(rbb_x, rbb_row, signal, value):
                stats["packed_edge_mux"] += 1
                continue

        # Tier 4: per-RBB CLK_SEL formulas.
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)
            result = rbb_clk_sel_to_bitstream(rbb_x, rbb_row, signal)
            if result:
                for word_idx, bit, inverted in result:
                    apply_bit(word_idx, bit, inverted, value)
                stats["packed_rbb_clk"] += 1
                continue

        # Tier 4b: per-RBB WE_SEL formulas (memory CLBs only).
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)
            result = we_sel_to_bitstream(rbb_x, rbb_row, signal)
            if result:
                for word_idx, bit, inverted in result:
                    apply_bit(word_idx, bit, inverted, value)
                prefix = sdc_path[:-len(signal)]
                if signal == "WE_SEL[0]" and not any(
                        sdc_data.get(f"{prefix}WE_SEL[{i}]") for i in range(3)):
                    for word_idx, bit in memory_clb_logic_bits(rbb_x, rbb_row):
                        apply_bit(word_idx, bit, False, 0)
                stats["packed_we_sel"] += 1
                continue

        # Tier 4b': per-RBB slice-output mux formulas (AQ1_SEL / AMX_SEL /
        # OAQ_SEL). Verified bit-perfect across the 1062-build vendor corpus
        # (see docs/silicon/reference.md §7.5 and fuzz/pilot/xmux_xq).
        # Edge columns (rbb_x % 6 ∉ {1, 4}) and non-CLB rows fall through to
        # template defaults — the encoders return None for those positions.
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)
            m = re.match(r'(AQ1|AMX|OAQ)_SEL_([ABCD])\[(\d+)\]', signal)
            if m:
                field = m.group(1)
                slice_idx = "ABCD".index(m.group(2))
                fb = int(m.group(3))
                fn = {
                    "AQ1": aq1_sel_to_bitstream,
                    "AMX": amx_sel_to_bitstream,
                    "OAQ": oaq_sel_to_bitstream,
                }[field]
                result = fn(rbb_x, rbb_row, slice_idx, fb)
                if result is not None:
                    word_idx, bit, inverted = result
                    apply_bit(word_idx, bit, inverted, value)
                    stats["packed_slice_mux"] += 1
                    continue
                else:
                    # Edge column or row outside CLB range — leave the
                    # template-default bit alone. Counted but not packed.
                    stats["skipped_slice_mux_uncovered"] += 1
                    continue

        # Tier 4c: per-RBB LUT_IPT_SEL formulas.
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)
            result = lut_ipt_sel_to_bitstream(rbb_x, rbb_row, signal)
            if result:
                word_idx, bit, inverted = result
                apply_bit(word_idx, bit, inverted, value)
                stats["packed_lut_ipt"] += 1
                continue
            if lut_ipt_sel_is_known_fixed_shape(rbb_x, rbb_row, signal, value,
                                                lut_ipt_values):
                stats["packed_lut_ipt"] += 1
                continue

        # Tier 5: row-type formulas (generalized from DB, 0 conflicts with DB
        # at all confidence levels).  Skip edge columns; different encoding.
        if rbb_match and row_formulas:
            if rbb_x not in (0, 31):
                formula = row_formulas.get(f"{rbb_row}:{signal}")
                if formula:
                    result = row_formula_to_bitstream(formula, rbb_x)
                    if result:
                        word_idx, bit, inverted = result
                        apply_bit(word_idx, bit, inverted, value)
                        stats["packed_row_formula"] += 1
                        continue

        # Tier 6: formulaic interior CLB mux encoding. This fills sparse holes
        # in row_formulas.json for the last CLB rows without overriding any
        # existing per-instance or row-formula mapping.
        if rbb_match:
            rbb_x = int(rbb_match.group(1))
            rbb_row = int(rbb_match.group(2))
            signal = rbb_match.group(3)
            result = interior_mux_to_bitstream(rbb_x, rbb_row, signal)
            if result:
                word_idx, bit, inverted = result
                apply_bit(word_idx, bit, inverted, value)
                stats["packed_interior_mux"] += 1
                continue

        # Tier 7: formulaic BC encoding (covers all CLB rows 2-28)
        bc_match = re.match(r'(RBB_(\d+)_(\d+))/BC\[(\d+)\]', sdc_path)
        if bc_match:
            rbb_x = int(bc_match.group(2))
            rbb_row = int(bc_match.group(3))
            bc_idx = int(bc_match.group(4))

            result = bc_to_bitstream(rbb_x, rbb_row, bc_idx)
            if result:
                word_idx, bit, inverted = result
                apply_bit(word_idx, bit, inverted, value)
                stats["packed_formulaic"] += 1
                continue

        # Unmapped
        stats["unmapped"] += 1
        stats["unmapped_signals"].append(sdc_path)

    # Apply direct NVM/RAW overrides last so they trump conflicting bits
    # written by tier-3 (edge mux) or other row-formula tiers.
    for (word_idx, bit, inverted), value in direct_nvm_overrides:
        apply_bit(word_idx, bit, inverted, value)

    # Final pass: re-force any route-template bit that differs from the
    # base template. The initial XOR at the top of this function set up
    # the route bits, but the SDC / row-formula / edge-mux tiers above
    # may have stomped some of them — most often the SM-region bits that
    # carry clock-fanout routing for a placed FF, where the SDC-derived
    # MUX_xI/SEL writes share physical positions with the routing-graph
    # SM mux selects.  Forcing route to win at the end is the correct
    # semantics: emit_route emits only those bits it has ground truth
    # for (via clock_fanout_synth + the raw clock_fanout deltas table),
    # and those values are derived from vendor probes, not synthesis.
    if route_template_words is not None:
        for wi in range(min(len(words), len(route_template_words),
                            len(template_words))):
            diff = route_template_words[wi] ^ template_words[wi]
            if diff:
                words[wi] = ((words[wi] & ~diff) |
                             (route_template_words[wi] & diff))

    # Force any explicitly claimed clock-fanout bits. These come from
    # emit_route's <route.bin>.claims.json sidecar and cover the bits
    # that the route happens to leave equal to the empty-bitstream
    # template default but that the clock_fanout table still claims as
    # active — e.g. word 10925 bit 13, which the empty template carries
    # at 1, the _CLOCK_FANOUT_DELTAS_28_26 table also wants at 1, but
    # the SDC tier clears via _EAST_IOB_CONST_MODE_BITS[27]. The simple
    # ``route ^ template`` diff above misses these.
    if route_claims:
        for w, b, v in route_claims:
            if v:
                words[w] |= (1 << b)
            else:
                words[w] &= ~(1 << b)

    # Chip-level baseline corrections. These bits are constant across all
    # 184 vendor designs in the corpus (see docs/methodology/sym-bucket6-findings.md),
    # owned by no encoder, and the empty-bitstream template carries them at
    # the wrong value. Apply unconditionally as a final pass — no encoder in
    # any tier writes to these positions, so this cannot conflict.
    for w, b, v in _CHIP_LEVEL_BASELINE_FIXUPS:
        if v:
            words[w] |= (1 << b)
        else:
            words[w] &= ~(1 << b)

    # Design-conditional baseline corrections. Closed-form predicates over
    # SDC-derived features identify bits the empty template gets wrong only
    # for some designs. Derived via sym_bucket6 SyGuS scan + HITL bisect
    # (session 14).
    #
    # w68/b10 = 1 iff NOT uses_west_iob (verified 184/184 across paired
    # vendor designs). Empty template carries the bit at 1, so we clear it
    # whenever a west IOB is in use. osc_probe uses a west IOB at gy=25
    # (the oscillator output pad) -> vendor clears -> we must clear.
    if _sdc_uses_west_iob(sdc_data):
        words[68] &= ~(1 << 10)

    return words, stats


# pack.parse_sdc strips the "TILE_00/" prefix, so the regex matches the
# path tail it actually stores.
_RBB_MUX_SEL_RE = re.compile(r"^RBB_(\d+)_(\d+)/MUX_(\w+)/(\w+)$")


def _sdc_uses_west_iob(sdc_data) -> bool:
    """True if any RBB at x in {0, 1} has a MUX_*/SEL=1 entry.

    Matches the predicate used by ``sym_bucket6.derive_summary_features``
    for the ``uses_west_iob`` derived feature.
    """
    for path, val in sdc_data.items():
        if val != 1:
            continue
        m = _RBB_MUX_SEL_RE.match(path)
        if not m:
            continue
        x = int(m.group(1))
        sel = m.group(4)
        if x in (0, 1) and sel in ("ASEL", "BSEL", "CSEL", "DSEL", "ESEL"):
            return True
    return False


# 13 chip-level constant bits at trace_x ∈ {0, 1}, trace_y = 31 (top corner)
# that the empty-bitstream baseline disagrees with vendor on. Constant across
# 184/184 vendor designs; no encoder writes these positions, so the safe fix
# is to overlay the vendor values onto the open bitstream at the end of
# packing. Discovered via sym_bucket6 SyGuS scan plus trace_pack_writes
# verification that no apply_bit hits these words.
_CHIP_LEVEL_BASELINE_FIXUPS = (
    (10933, 16, 1),
    (10988, 17, 1),
    (11032, 17, 1),
    (11043, 17, 1),
    (11076, 17, 1),
    (11098, 12, 1),
    (11120, 17, 1),
    (11131, 17, 1),
    (11164, 17, 0),
    (11208, 17, 1),
    (11208, 28, 1),
    (11219, 17, 1),
    (11252, 17, 1),
)


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------

def verify_bitstream(packed_words, reference_path):
    """Compare packed bitstream against a reference.

    Returns (n_match, n_mismatch, mismatches) where mismatches is a list
    of (word_idx, x, y, pos, packed_val, ref_val).
    """
    ref_data = Path(reference_path).read_bytes()
    n_ref = len(ref_data) // 4
    ref_words = list(struct.unpack(f"<{n_ref}I", ref_data))

    n_match = 0
    n_mismatch = 0
    mismatches = []

    for i in range(min(len(packed_words), len(ref_words))):
        if packed_words[i] == ref_words[i]:
            n_match += 1
        else:
            n_mismatch += 1
            x, y, pos = word_to_xyp(i)
            diff = packed_words[i] ^ ref_words[i]
            mismatches.append({
                "word_idx": i,
                "x": x, "y": y, "pos": pos,
                "packed": packed_words[i],
                "reference": ref_words[i],
                "diff_bits": [b for b in range(32) if (diff >> b) & 1],
            })

    return n_match, n_mismatch, mismatches


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Raw → MCU format conversion
# ---------------------------------------------------------------------------

_AXI_PERM_CACHE: list[int] | None = None
_AXI_PERM_PATH = PROJECT_ROOT / "device" / "axi_perm.bin"
_AXI_BASELINE_BITS_CACHE: list[int] | None = None
_AXI_BASELINE_PATH = PROJECT_ROOT / "device" / "axi_perm.baseline.bin"

_MCU_PREAMBLE_ZEROS = 1280
_MCU_SYNC_WORD = 0xAA22FF11
_MCU_FIXED_HEADER = bytes([
    0x00, 0x00, 0x00, 0x00,
    0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
])
_MCU_TOTAL_BYTES = 46408


def _load_axi_perm() -> list[int]:
    global _AXI_PERM_CACHE
    if _AXI_PERM_CACHE is not None:
        return _AXI_PERM_CACHE
    if not _AXI_PERM_PATH.exists():
        raise FileNotFoundError(
            f"AXI permutation table not found: {_AXI_PERM_PATH}\n"
            "Run tools/rebuild_perm.py to generate it."
        )
    data = _AXI_PERM_PATH.read_bytes()
    if data[:4] != b"AXIP":
        raise ValueError(f"{_AXI_PERM_PATH}: bad magic")
    n = struct.unpack_from("<I", data, 4)[0]
    _AXI_PERM_CACHE = list(struct.unpack_from(f"<{n}i", data, 8))
    return _AXI_PERM_CACHE


def _load_axi_baseline_bits() -> list[int]:
    global _AXI_BASELINE_BITS_CACHE
    if _AXI_BASELINE_BITS_CACHE is not None:
        return _AXI_BASELINE_BITS_CACHE
    if not _AXI_BASELINE_PATH.exists():
        raise FileNotFoundError(
            f"AXI baseline not found: {_AXI_BASELINE_PATH}\n"
            "Run tools/find_axi_perm.py or rebuild tools/axi_perm.baseline.bin."
        )
    data = _AXI_BASELINE_PATH.read_bytes()
    if len(data) != BITSTREAM_WORDS * 4:
        raise ValueError(
            f"{_AXI_BASELINE_PATH}: expected {BITSTREAM_WORDS * 4} bytes, got {len(data)}"
        )

    bits = []
    for i in range(BITSTREAM_WORDS):
        w = struct.unpack_from(">I", data, i * 4)[0]
        for b in range(31, -1, -1):
            bits.append((w >> b) & 1)
    _AXI_BASELINE_BITS_CACHE = bits
    return _AXI_BASELINE_BITS_CACHE


def raw_to_axi(raw_words: list[int]) -> list[int]:
    """Convert raw-format words (little-endian uint32) to AXI-order word list.

    Applies the AXI permutation table (tools/axi_perm.bin) and returns
    BITSTREAM_WORDS 32-bit integers in physical SPI address order. Unresolved
    permutation slots are filled from tools/axi_perm.baseline.bin.
    """
    perm = _load_axi_perm()
    baseline_bits = _load_axi_baseline_bits()

    # Unpack raw words to bits using the same convention as the permutation builder:
    # the raw .bin stores words little-endian, but the permutation was built by
    # reading them as big-endian (byte-swapping each word) then extracting MSB-first.
    raw_bits = []
    for w in raw_words:
        w_be = struct.unpack(">I", struct.pack("<I", w))[0]  # byte-swap to match loader
        for b in range(31, -1, -1):
            raw_bits.append((w_be >> b) & 1)

    # Apply permutation: axi_bits[i] = raw_bits[perm[i]], with unresolved
    # positions preserved from the recovered AXI baseline.
    axi_words = []
    for word_idx in range(BITSTREAM_WORDS):
        w = 0
        for bit_idx in range(32):
            axi_i = word_idx * 32 + bit_idx
            raw_j = perm[axi_i]
            if raw_j != -1:
                w = (w << 1) | raw_bits[raw_j]
            else:
                w = (w << 1) | baseline_bits[axi_i]
        axi_words.append(w)

    return axi_words


def raw_to_mcu(raw_words: list[int]) -> bytes:
    """Convert raw-format words (little-endian uint32) to 46408-byte MCU format.

    Calls raw_to_axi() for the permutation, then wraps the resulting AXI words
    with the MCU preamble, sync word, and fixed header. Each AXI word is stored
    little-endian in the payload per the MCU format.
    """
    axi_words = raw_to_axi(raw_words)

    # Build MCU payload: preamble + sync + header + AXI words (LE) + padding
    out = bytearray(_MCU_PREAMBLE_ZEROS)
    out += struct.pack(">I", _MCU_SYNC_WORD)
    out += _MCU_FIXED_HEADER
    for w in axi_words:
        out += struct.pack("<I", w)   # byte-reversed (little-endian) per MCU format
    if len(out) < _MCU_TOTAL_BYTES:
        out += bytes(_MCU_TOTAL_BYTES - len(out))
    return bytes(out[:_MCU_TOTAL_BYTES])


def main():
    parser = argparse.ArgumentParser(
        description="SLG47910 bitstream packer: SDC → .bin",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  # Pack from PnR SDC output
  python3 tools/pack.py pnr_output/EFLX_case_analysis_RBB.sdc -o bitstream.bin

  # Also produce MCU-format bitstream (ready to flash over serial)
  python3 tools/pack.py pnr_output/EFLX_case_analysis_RBB.sdc -o bitstream.bin \\
      --mcu bitstream_mcu.bin

  # Pack and verify against reference bitstream
  python3 tools/pack.py pnr_output/EFLX_case_analysis_RBB.sdc -o bitstream.bin \\
      --verify pnr_output/EFLX_bitstream.bin

  # Use custom template bitstream
  python3 tools/pack.py design.sdc -o out.bin --template custom_empty.bin
""")

    parser.add_argument("sdc", help="SDC file with set_case_analysis entries")
    parser.add_argument("-o", "--output", required=True, help="Output .bin file")
    parser.add_argument("--template", default=str(DEFAULT_TEMPLATE),
                        help=f"Template bitstream (default: {DEFAULT_TEMPLATE.name})")
    parser.add_argument("--db", default=str(DEFAULT_DB),
                        help="Mapping database directory")
    parser.add_argument("--route-template", metavar="PLACER.bin",
                        help="Copy SM routing bits (pos 0-4) from placer bitstream")
    parser.add_argument("--verify", metavar="REF.bin",
                        help="Verify against reference bitstream")
    parser.add_argument("--mcu", metavar="OUTPUT.bin",
                        help="Also write MCU-format bitstream (46408 bytes, ready to flash)")
    parser.add_argument("--axi-log", metavar="OUTPUT.log",
                        help="Write AXI-order words to log (one 8-digit hex per line, "
                             "matches vendor EFLX_bitstream_AXI.log format)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Show detailed output")

    args = parser.parse_args()

    # Load template
    template_data = Path(args.template).read_bytes()
    n_words = len(template_data) // 4
    template_words = list(struct.unpack(f"<{n_words}I", template_data))
    print(f"Template: {args.template} ({len(template_data)} bytes, {n_words} words)")

    # Load mapping database
    db_dir = Path(args.db)
    db_mappings = {}
    row_formulas = {}
    if db_dir.exists():
        db_mappings = load_mapping_db(db_dir)
        row_formulas = load_row_formulas(db_dir)
        print(f"Database: {len(db_mappings)} per-instance mappings, "
              f"{len(row_formulas)} row-type formulas from {db_dir}")

    # Load route template if provided
    route_template_words = None
    route_claims = None
    if args.route_template:
        rt_data = Path(args.route_template).read_bytes()
        rt_n = len(rt_data) // 4
        route_template_words = list(struct.unpack(f"<{rt_n}I", rt_data))
        print(f"Route template: {args.route_template} ({len(rt_data)} bytes)")
        # Auto-load sidecar claims file if present.  Emitted by
        # emit_route.py whenever it has applied clock-fanout rules, the
        # claims sidecar lists (word, bit, value) triples that must be
        # forced to win over SDC even when they happen to match the
        # empty-bitstream template default (where ``route ^ template``
        # would otherwise lose them).
        claims_path = Path(str(args.route_template) + '.claims.json')
        if claims_path.is_file():
            with open(claims_path) as cf:
                doc = json.load(cf)
            route_claims = [tuple(t) for t in doc.get("clock_fanout", [])]
            print(f"Route claims:  {claims_path.name} ({len(route_claims)} entries)")

    # Parse SDC
    sdc_data = parse_sdc(args.sdc)
    print(f"SDC: {args.sdc} ({len(sdc_data)} signals)")

    # Pack
    packed_words, stats = pack_bitstream(sdc_data, template_words, db_mappings,
                                         row_formulas, route_template_words,
                                         route_claims=route_claims)

    print(f"\nPacking results:")
    print(f"  Total SDC signals:    {stats['total_sdc']}")
    print(f"  Packed (per-inst DB): {stats['packed_db']}")
    print(f"  Packed (row formula): {stats['packed_row_formula']}")
    print(f"  Packed (CLB mux):     {stats['packed_interior_mux']}")
    print(f"  Packed (BC formula):  {stats['packed_formulaic']}")
    print(f"  Packed (CLK mux):     {stats['packed_clk']}")
    print(f"  Packed (direct raw):  {stats['packed_direct_nvm']}")
    print(f"  Packed (edge mux):    {stats['packed_edge_mux']}")
    print(f"  Packed (RBB CLK_SEL): {stats['packed_rbb_clk']}")
    print(f"  Packed (WE_SEL):      {stats['packed_we_sel']}")
    print(f"  Packed (slice mux):   {stats['packed_slice_mux']}")
    if stats['skipped_slice_mux_uncovered']:
        print(f"  Skipped (slice mux):  {stats['skipped_slice_mux_uncovered']}")
    print(f"  Packed (LUT_IPT):     {stats['packed_lut_ipt']}")
    print(f"  Packed (BRAM NVM):    {stats['packed_bram_nvm']}")
    print(f"  Skipped (BRAM dflt):  {stats['packed_bram_default_skip']}")
    print(f"  Preserved (BRAM tpl): {stats['packed_bram_template']}")
    print(f"  Skipped complements:  {stats['skipped_complement']}")
    print(f"  Unmapped:             {stats['unmapped']}")
    if stats['route_words_copied']:
        print(f"  Route words copied:   {stats['route_words_copied']}")

    total_packed = (stats['packed_formulaic'] + stats['packed_db'] +
                    stats['packed_row_formula'] + stats['packed_interior_mux'] +
                    stats['packed_clk'] + stats['packed_direct_nvm'] +
                    stats['packed_edge_mux'] +
                    stats['packed_rbb_clk'] + stats['packed_we_sel'] +
                    stats['packed_slice_mux'] + stats['skipped_slice_mux_uncovered'] +
                    stats['packed_lut_ipt'] + stats['packed_bram_default_skip'] +
                    stats['packed_bram_template'])
    effective = stats['total_sdc'] - stats['skipped_complement']
    if effective > 0:
        pct = 100.0 * total_packed / effective
        print(f"  Coverage:             {total_packed}/{effective} ({pct:.1f}%)")

    if args.verbose and stats['unmapped_signals']:
        print(f"\n  Unmapped signals ({stats['unmapped']}):")
        # Categorize unmapped
        cats = defaultdict(list)
        for sig in stats['unmapped_signals']:
            if '/BC[' in sig:
                cats['BC'].append(sig)
            elif 'MUX_' in sig:
                cats['MUX'].append(sig)
            elif 'CLK_' in sig:
                cats['CLK'].append(sig)
            elif 'inst_SM' in sig:
                cats['switch_matrix'].append(sig)
            else:
                cats['other'].append(sig)
        for cat, sigs in sorted(cats.items()):
            print(f"    {cat}: {len(sigs)}")
            for s in sigs[:5]:
                print(f"      {s}")
            if len(sigs) > 5:
                print(f"      ... and {len(sigs) - 5} more")

    # Write output
    out_data = struct.pack(f"<{len(packed_words)}I", *packed_words)
    Path(args.output).write_bytes(out_data)
    print(f"\nOutput: {args.output} ({len(out_data)} bytes)")

    # Optionally write MCU format and/or AXI log.  Compute AXI words once when
    # either output is requested so the permutation runs only once.
    if args.mcu or args.axi_log:
        axi_words = raw_to_axi(packed_words)
        if args.axi_log:
            lines = []
            for i, w in enumerate(axi_words):
                lines.append(f"{w:08x}\n")
                if i % 11 == 10:
                    lines.append("\n")
            Path(args.axi_log).write_text("".join(lines))
            print(f"AXI log:    {args.axi_log} ({len(axi_words)} words)")
        if args.mcu:
            out = bytearray(_MCU_PREAMBLE_ZEROS)
            out += struct.pack(">I", _MCU_SYNC_WORD)
            out += _MCU_FIXED_HEADER
            for w in axi_words:
                out += struct.pack("<I", w)
            if len(out) < _MCU_TOTAL_BYTES:
                out += bytes(_MCU_TOTAL_BYTES - len(out))
            mcu_data = bytes(out[:_MCU_TOTAL_BYTES])
            Path(args.mcu).write_bytes(mcu_data)
            print(f"MCU format: {args.mcu} ({len(mcu_data)} bytes)")

    # Verify if requested
    if args.verify:
        print(f"\nVerifying against: {args.verify}")
        n_match, n_mismatch, mismatches = verify_bitstream(
            packed_words, args.verify)

        print(f"  Words matching:     {n_match}/{n_match + n_mismatch}")
        print(f"  Words mismatching:  {n_mismatch}")

        if mismatches:
            total_diff_bits = sum(len(m["diff_bits"]) for m in mismatches)
            print(f"  Total differing bits: {total_diff_bits}")

            if args.verbose:
                # Show first 20 mismatches
                print(f"\n  First {min(20, len(mismatches))} mismatches:")
                for m in mismatches[:20]:
                    print(f"    word {m['word_idx']:5d} "
                          f"(X={m['x']:2d}, Y={m['y']:2d}, pos={m['pos']:2d}): "
                          f"packed=0x{m['packed']:08X} ref=0x{m['reference']:08X} "
                          f"diff_bits={m['diff_bits']}")

            if n_mismatch == 0:
                print("  PERFECT MATCH!")
                return 0
            else:
                print(f"  MISMATCH: {n_mismatch} words differ")
                return 1
        else:
            print("  PERFECT MATCH!")
            return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
