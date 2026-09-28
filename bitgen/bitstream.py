"""Bitstream generation: routed design -> config image -> .bin / .log / AXI.

Stage 6 of the pipeline (see docs/port-flow.dot). Reconstructs the writer path
from the decompile. The image builders + the file serializers are folded into
SerializeStage; the flat config Image and the low-level bit/CRC primitives stay
module-level (data structure + pure helpers shared with the encoders).

In-port pieces:

    coords.py                          (BL,WL)->word/bit map  (FUN_0055f840 layout)
    sm_routing.py                      SM routing-bit encoder (FUN_00573110/470)

Functions (decomp/eda-placer.real.c):
    FUN_00556ad0 :423498  core bit setter set_bit(buf, BL, WL, val)  -- read in full
    FUN_005601f0 :429368  build one final config image from routed tiles/SM (321 ln)
    FUN_0055f840 :428967  serialize image -> .bin (+ .log)           -- read in full
    FUN_0057ab60 :441316  serialize image -> AXI text (+ CRC16)
    FUN_00560450 :429689  CLB/IOB/NVM bit writer + verify (dual-mode, 7791 ln)
    FUN_00573110 :437480  SM L0 bit writer        FUN_00573470 :437646  SM L1+ bit writer

flow.bitstream() copies the output-enable flags into the generator, runs the
worker, then emits the enabled files.
"""

# Core bit setter                          <- FUN_00556ad0 @ ...c:423498
# Every one of the ~81 call sites in the writer ends here. The image keeps TWO
# parallel bitmaps per BL row (stride 0x28): a value bitmap (buf+0x2a) and a
# written/visited bitmap (buf+0x30). WL indexes the bit -- word = WL>>6, bit =
# WL&63. The setter writes val into the value bitmap, marks the bit visited, and
# warns "Overwriting set bits at x=%d, y=%d, BL[%d] WL[%d]" if a later write
# disagrees with an earlier one (the conflict check that makes double-drives
# visible). Only binary values are allowed ("Only binary value allowed").
#
# The (BL, WL) -> (x_bs, y_bs, pos, bit) layout is decoded in
# the decompile (FUN_00573110/470) + sm_routing.py::trace_to_bs (verified on 282,624 trace entries).

import struct
from pathlib import Path
from . import coords


def set_bit(image, bl: int, wl: int, val: int) -> None:
    """<- FUN_00556ad0. Set one config bit, recording it as written.

    Model of the dual-bitmap setter. `image` is the per-(BL) buffer; word/bit
    indexing follows WL>>6 / WL&63 (with the sign adjustment the decompile makes
    for the word-boundary parity).
    """
    if val not in (0, 1):
        raise ValueError(f"Only binary value allowed in bitstream, not {val}!")
    key = (bl, wl)
    if key in image.value and image.value[key] != val:
        _warn(f"Overwriting set bits at BL[{bl}] WL[{wl}], "
              f"current {image.value[key]}, new {val}.")
    image.value[key] = val
    image.written.add(key)


# Concrete config image: the 11264-word raster the serializers walk.
# Unlike the in-memory dual-bitmap above (which mirrors the per-BL buffer the
# vendor's writer fills tile by tile), this is the FINAL flat image: 11264
# little-endian 32-bit words == the 1024x352 raster == EFLX_bitstream.bin. The
# vendor initialises every config bit to its device default before the writers
# run; we model that "device default" with the empty-design template bitstream
# (see docs: the bitstream is ~98% device-constant template + a small per-design
# delta). The encoders then flip only the design-specific bits via set_word_bit
# / set_trace_bit.

class Image:
    """Flat config image (list of 11264 32-bit words)."""

    def __init__(self, words=None):
        self.words = list(words) if words is not None else [0] * coords.WORDS

    @classmethod
    def from_template(cls, path):
        """Load the device-default (empty-design) bitstream as the base image."""
        data = Path(path).read_bytes()
        n = len(data) // 4
        return cls(struct.unpack(f"<{n}I", data))

    def get_word_bit(self, word_idx, bit):
        return (self.words[word_idx] >> bit) & 1

    def set_word_bit(self, word_idx, bit, val):
        if word_idx < 0 or word_idx >= len(self.words):
            return
        if val:
            self.words[word_idx] |= (1 << bit)
        else:
            self.words[word_idx] &= ~(1 << bit)

    def set_trace_bit(self, x, y, bl, wl, val):
        """Set a bit addressed in the writer's (grid x,y / BL,WL) space."""
        word_idx, bit = coords.trace_to_word_bit(x, y, bl, wl)
        self.set_word_bit(word_idx, bit, val)

    def to_bytes(self):
        return struct.pack(f"<{len(self.words)}I", *self.words)

    def diff(self, other):
        """List of (word_idx, bit) where self and other differ."""
        out = []
        for i in range(min(len(self.words), len(other.words))):
            x = self.words[i] ^ other.words[i]
            while x:
                b = (x & -x).bit_length() - 1
                out.append((i, b))
                x &= x - 1
        return out


# AXI CRC16 Galois LFSR tap mask                  <- FUN_0057ab60 @ ...c:441532-441546
# Read off the per-bit XOR pattern of the gather-step (puVar27 accumulator): the
# feedback fb = state bit 15; new[0] = in ^ fb; new[i] = old[i-1] ^ fb for
# i in {1,3,5,6,7,9,13,15}. That tap set is the mask 0xA2EB (not one of the named
# CRC16 polynomials). Validated against the minted oracle: reproduces all 178,176
# cycle tails across the 174-design corpus (0 mismatches); negative controls with
# 0x8005 / 0x1021 fail. The decompile computes this 16-deep in a SIMD-parallel
# accumulator over the 11-word block; the equivalent scalar LFSR fed the cycle's
# body bits in push order is bit-identical.
_AXI_CRC_TAPMASK = 0xA2EB


def _axi_crc_step(state: int, inbit: int) -> int:
    """<- FUN_0057ab60 @ ...c:441532-441546. One 16-stage Galois LFSR step."""
    fb = (state >> 15) & 1
    state = ((state << 1) | inbit) & 0xFFFF
    if fb:
        state ^= _AXI_CRC_TAPMASK
    return state


def _axi_crc_word(bits) -> int:
    """<- FUN_0057ab60 @ ...c:441370(seed)/441588(flush)/441614(emit). Tail word
    for one cycle: feed the cycle's body bits in push order into the zero-seeded
    LFSR (441532), run 16 zero-input flush steps (441588-441612), then emit the
    16 state bits s15..s0 into tail bits 0..15 with bits 16..31 zero (441614-441669)
    -- i.e. tail = bitrev16(state)."""
    st = 0
    for b in bits:
        st = _axi_crc_step(st, b)
    for _ in range(16):                              # 16 zero-input flush steps
        st = _axi_crc_step(st, 0)
    return int(f"{st:016b}"[::-1], 2)                # emit s15..s0 -> bits 0..15


# Output position -> (raster word-in-row pos_bs, bit within word) for one cycle.
# Built once: the within-row permutation is device-constant (independent of the
# raster row), so the per-cycle word is just base_word(c) + pos_bs.
def _axi_perm_table():
    """<- FUN_0057ab60 @ ...c:441480-441586 (gather) + FUN_0055f840 @ ...c:429014-
    429052 (the .bin column order), specialized to the FPGA1K_1x1.fp the corpus is
    built on (cols=rows=1 -> one tile, FUN_00600ea0(1)=1 tile copy).

    Both serializers read the SAME internal buffer[tile_i][bl][WL_i] (built by
    FUN_005601f0; passed to both at decomp :453636/:453640/:453695). The .bin
    writer emits one raster row per WL_i, walking bl DESCENDING (lVar8 from
    (BLcount-1)*0x18 down to 0, :429021-429047) -- so the buffer cell bl maps to
    raster COLUMN (351 - bl) of that row (BLcount = 352). No mirroring: the buffer
    IS the raster, transposed by 351-bl.

    The AXI gather walks the same buffer with WL_i = the cycle (one raster row per
    cycle, 441419/441716), BL_i (iVar29) DESCENDING over 11 words
    (local_148 = BLcount/param_7 = 352/32 = 11; 441488/441579), innermost param_7
    =32 loop indexing BL_offset = 11*k, k = 0..31 (iVar21 = local_148 * (WL_i %
    param_7); 441505/441552). Push order: for j = BL_i = 10..0 (line L = 10-j):
    for k = 0..31: push buffer[0][11*k + j][cycle]. So the bit at output line L,
    word-bit k reads buffer cell bl = 11*k + (10-L) = raster column
    m = 351 - bl = 341 - 11*k + L. Output word_L = sum(bit_k << k) (LSB-first per
    FUN_009900b0 / the :441690 formatter)."""
    table = []                                       # table[L] = [(pos_bs, bit) for k in 0..31]
    for L in range(11):                              # line L -> BL_i j = 10 - L
        row = []
        for k in range(32):                          # k = output word bit position
            m = 341 - 11 * k + L                      # raster column of this bit
            pos_bs = m // 32                          # word index within the raster row
            serial = m % 32
            bit_bs = (serial // 8) * 8 + (7 - serial % 8)   # MSB-first byte packing
            row.append((pos_bs, bit_bs))
        table.append(row)
    return table


_AXI_PERM = _axi_perm_table()


# Image build (producers)
# FUN_005601f0 walks the routed tiles + switch matrices and calls the writers
# below to populate the image. The writers split by feature:
#
#   FUN_00573110 / FUN_00573470  SM routing-mux select bits (L0 / L1+). The
#       select-value model + per-bit decomposition is now in-port:
#       sm_select.compute_select / hop_writes (cross-checked vs the decompile and
#       the reference encoder). hop_writes returns the (BL, WL, bit) triples that
#       become set_bit() calls -- one routed hop's worth of config bits.
#   FUN_00560450  CLB / IOB / NVM / clock config + a verify mode (param_5 != 0
#       re-reads and only writes on mismatch). The bulk of non-SM bits. Per-
#       feature VALUES now in-port: clb_config.clb_config_writes / lut_init_writes
#       (BC bits); writer structure + branch offsets in config_writer.py.
#
# These are catalogued in the decompile; not duplicated here.

class SerializeStage:
    """Stage 6: assemble the final `template + delta` config Image and serialize it
    to the enabled output formats (.bin / .log / AXI text + CRC16).

    Stateless; the image it builds is the flat Image above. Construct once:

        ss = SerializeStage()
        image = ss.build_config_image(routed_design, template_path)
        ss.write_bin(image, "EFLX_bitstream.bin")
    """

    def build_config_image(self, rdb, template_path):
        """<- FUN_005601f0: populate the final config image from the routed design.

        The GENERATIVE (from-scratch) assembly: initialise the image to device
        defaults (the empty-design template), then flip the design-specific bits
        from the PORT'S OWN place + route -- no vendor oracle:

          * SM family    -- sm_emit.emit_design over rdb.routes (the routed trees
            the flow->router bridge stored), full reprogrammed-mux coverage
            (hops + co-writes + the clock-corner SM net + (0,'U') binds);
          * CLB config   -- config_writer.write_slice_fields valued from the
            generative slice_config fields (netlist+pack, not the .vm);
          * clock corner -- clock_route.apply_clock_corner (netlist-invariant).

        Falls back to the legacy encode.encode_design path when `rdb` carries no
        routed trees (the bare lazy-chain traversal)."""
        from . import encode
        image = Image.from_template(template_path)
        nets_hops = self._routes_to_oracle_hops(rdb)
        if not nets_hops:
            encode.EncodeStage().encode_design(image, rdb)
            return image
        from . import sm_emit, slice_config, config_writer, clock_route, endpoint
        input_tiles = frozenset(
            (s.chip_y, 0) for p, s in getattr(rdb.iospec, "ports", {}).items()
            if getattr(s, "kind", "") == "EFLX_IOB" and self._port_is_input(rdb, p))
        sm_emit.emit_design(None, nets_hops, {}, input_tiles,
                            image=image, driver_pin=endpoint.driver_pins(rdb))
        for col, fields in slice_config.generate(rdb).items():
            config_writer.write_slice_fields(image, col, fields)
        clock_route.apply_clock_corner(image)
        return image

    @staticmethod
    def _port_is_input(pd, port):
        return any(p.name == port and p.direction == "INPUT"
                   for p in pd.netlist.ports)

    @staticmethod
    def _routes_to_oracle_hops(pd):
        """{net: [hop dicts]} from the routed trees (pd.routes[net]['routed'] =
        {sink: [node path incl. site-pin endpoints]}). One hop per SM node b:
        programmed at b's tile, delivering to the NEXT path node's tile -- the
        PNR_ROUTE.log convention the SM emitter's neighborhood analysis uses."""
        from . import rgraph
        routes = getattr(pd, "routes", None) or {}
        if not any(info.get("routed") for info in routes.values()):
            return {}
        g = rgraph.load()
        out = {}
        for net, info in routes.items():
            hops = []
            for path in (info.get("routed") or {}).values():
                if not path:
                    continue
                for i, b in enumerate(path):
                    if not (isinstance(b, int) and b < len(g.nodes) and g.is_sm(b)):
                        continue
                    n = g.nodes[b]
                    nxt = path[i + 1] if i + 1 < len(path) else None
                    nt = g.nodes[nxt] if nxt is not None else n
                    hops.append(dict(sx=n.x, sy=n.y, dx=nt.x, dy=nt.y,
                                     level=n.level, branch=n.branch, index=n.index,
                                     j=1 if n.direction == "D" else 0))
            if hops:
                out[net] = hops
        return out

    def build_oracle_image(self, design, template_path=None):
        """<- FUN_005601f0, ORACLE-driven: assemble `template + delta` for a corpus
        `design` (a name) by wiring the three COMPLETE encoders (SM, CLB-config,
        clock) from the vendor oracle (route + placement .vm), plus the validated
        IOB (RBBIO) pad-config subset. Returns (image, stats) where stats counts the
        bits each family wrote. The OPEN residual (CLB LUT-init values + the IOB
        geometry remainder) stays at the template default -> residual delta (not
        fitted).

        This is the assembly the Task-3.1 validation harness checks bit-for-bit
        against the vendor reference on the complete families; it isolates encoder
        correctness from place/route convergence (Task 3.3) by using the oracle."""
        from . import encode, sm_validate
        tmpl = Image.from_template(template_path) if template_path else None
        # emit_image seeds the image from the device template (its own default when
        # tmpl is None), then writes the SM family; encode_design_oracle layers the
        # CLB config + clock families on top.
        image, n_sm, _unmatched = sm_validate.emit_image(design, tmpl=tmpl)
        es = encode.EncodeStage()
        n_clb = es.encode_clb(image, design)
        n_iob = es.encode_iob(image, design)
        n_clk = es.encode_clock(image, design)
        return image, {"sm": n_sm, "clb_config": n_clb, "iob": n_iob, "clock": n_clk}

    # Serializers (consumers)
    def write_bin(self, image, path) -> None:
        """<- FUN_0055f840 (.bin half). The flat word list IS the packed bitstream:
        11264 LE 32-bit words = the 1024x352 raster packed MSB-first. No header."""
        Path(path).write_bytes(image.to_bytes())

    def write_log(self, image, path) -> None:
        """<- FUN_0055f840 (.log half). Textual raster: 1024 rows x 352 bits.

        Verified: packing the .log row-major MSB-first reproduces the .bin exactly,
        so the .log is the image rendered one bit per character, 352 per line.
        """
        flat = []
        for w in image.words:
            for byte in (w & 0xff, (w >> 8) & 0xff, (w >> 16) & 0xff, (w >> 24) & 0xff):
                for k in range(7, -1, -1):
                    flat.append((byte >> k) & 1)
        with open(path, "w") as f:
            for r in range(coords.ROWS):
                row = flat[r * coords.COLS:(r + 1) * coords.COLS]
                f.write("".join(str(b) for b in row) + "\n")

    @staticmethod
    def write_axi_header(bus_width=32, bits_per_cycle_per_tile=32, cols=1, rows=1,
                         crc16=False) -> str:
        """<- FUN_0057ab60 @ ...c:441412. The exact AXI header line.

        Fields (decompile fprintf args): axi_bus_width=param_3 (-BITSTREAM_AXI_WIDTH,
        default 32); bits_per_cycle_per_tile=param_7 (-BITSTREAM_AXI_BITS_PER_TILE,
        32 for this corpus); array_size = *(ctx+0x120) x *(ctx+0x11c) (the floorplan
        tile array -- 1x1 for the FPGA1K_1x1.fp the corpus is built on); crc16=param_8
        (-ENABLE_BITSTREAM_OUTPUT_AXI_CRC). Reproduces the vendor header verbatim, e.g.
        single_ff: "axi_bus_width: 32, bits_per_cycle_per_tile: 32, array_size: 1x1,
        crc16: false"."""
        return (f"# axi_bus_width: {bus_width}, "
                f"bits_per_cycle_per_tile: {bits_per_cycle_per_tile}, "
                f"array_size: {cols}x{rows}, crc16: {str(bool(crc16)).lower()}\n")

    def write_axi(self, image, path, bus_width=32, bits_per_cycle_per_tile=32,
                  cols=1, rows=1, crc16=False) -> None:
        """<- FUN_0057ab60. AXI text: header line, then 1024 cycle blocks separated by
        a blank line (the per-cycle fputc(10) @ :441699). Each block is 11 body words
        (352 bits) plus, in crc16 mode, a 12th CRC word; words are 8-digit zero-padded
        lowercase hex (the %x formatter @ :441690 over 32-bit beats, param_3=32).

        The body is the within-row bit permutation gathered by the inner loops (see
        _axi_perm_table); the CRC16 is the LFSR over the same push-order bits
        (_axi_crc_word). This path is specialized to the corpus floorplan (1x1, one
        tile, 32-wide bus, 32 bits/cycle/tile); the general multi-tile gather is the
        decompile structure cited in _axi_perm_table."""
        words = image.words
        cols_per_row = coords.COLS // 32                 # 11 words per raster row
        with open(path, "w") as f:
            f.write(self.write_axi_header(bus_width, bits_per_cycle_per_tile, cols, rows, crc16))
            for c in range(coords.ROWS):                 # cycle = raster row (WL_i outermost)
                base = (c // 32) * (32 * cols_per_row) + (c % 32) * cols_per_row
                bits = []                                # 352 push-order body bits
                for L in range(cols_per_row):            # BL_i descending: line L -> j=10-L
                    for pos_bs, bit_bs in _AXI_PERM[L]:  # k = 0..31 (word bit position)
                        bits.append((words[base + pos_bs] >> bit_bs) & 1)
                for L in range(cols_per_row):            # 11 body words, word_L = sum(b_k << k)
                    word = 0
                    for k in range(32):
                        word |= bits[32 * L + k] << k
                    f.write(f"{word:08x}\n")
                if crc16:
                    f.write(f"{_axi_crc_word(bits):08x}\n")
                f.write("\n")                            # per-cycle blank separator

    # Stage glue (called from flow.bitstream)
    def generate(self, rdb, gen, outdir=".", template_path=None, design=None) -> "Image":
        """Build the image then emit every enabled output. The decompile runs this
        through one of the async workers (FUN_005c4b60); the work is the same.

        Two ways to build the `template + delta` image:
          * `design` given (a corpus name) -> ORACLE assembly (build_oracle_image):
            the three complete encoders driven from the vendor route + placement .vm.
            This is the Task-3.1 validated path.
          * otherwise -> the GENERATIVE assembly (build_config_image) from the routed
            design object `rdb` (the flow->encoder bridge, completed in Task 3.3).
        """
        if design is not None:
            image, _stats = self.build_oracle_image(design, template_path)
        else:
            image = self.build_config_image(rdb, template_path)
        base = Path(outdir)
        if gen.out_bin:
            self.write_bin(image, base / "EFLX_bitstream.bin")
        if gen.out_log:
            self.write_log(image, base / "EFLX_bitstream.log")
        if gen.out_axi:
            self.write_axi(image, base / "EFLX_bitstream_AXI.txt",
                           bus_width=gen.axi_bus_width, crc16=gen.out_axi_crc)
        return image


def _warn(msg: str): ...     # FUN_004d3120(0, ...)
