from os import path
import sys
import argparse
import xilinx_device
import filters
import struct
import itertools
import parse_sdf
sys.path.append(path.join(path.dirname(__file__), "../../.."))
from himbaechel_dbgen.chip import *

def lookup_port_type(t):
    if t == "INPUT": return PinType.INPUT
    elif t == "OUTPUT": return PinType.OUTPUT
    elif t == "BIDIR": return PinType.INOUT
    else: assert False

def gen_bel_name(site, bel_name):
    prim_st = site.primary.site_type()
    if prim_st in ("IOB33M", "IOB33S", "IOB33", "IOB18M", "IOB18S", "IOB18"):
        return f"{site.site_type()}.{bel_name}"
    else:
        return f"{bel_name}"

@dataclass
class PipClass(Enum):
    TILE_ROUTING = 0
    SITE_ENTRANCE = 1
    SITE_EXIT = 2
    SITE_INTERNAL = 3
    LUT_PERMUTATION = 4
    LUT_ROUTETHRU = 5
    CONST_DRIVER = 6

@dataclass
class PipExtraData(BBAStruct):
    site_key: int = -1
    bel_name: IdString = field(default_factory=IdString)
    pip_config: int = 0

    def serialise_lists(self, context: str, bba: BBAWriter):
        pass
    def serialise(self, context: str, bba: BBAWriter):
        bba.u32(self.site_key)
        bba.u32(self.bel_name.index)
        bba.u32(self.pip_config)

@dataclass
class BelExtraData(BBAStruct):
    name_in_site: IdString

    def serialise_lists(self, context: str, bba: BBAWriter):
        pass
    def serialise(self, context: str, bba: BBAWriter):
        bba.u32(self.name_in_site.index)

@dataclass
class SiteInst(BBAStruct):
    name_prefix: IdString
    site_x: int
    site_y: int
    rel_x: int
    rel_y: int
    int_x: int
    int_y: int
    variants: list[IdString] = field(default_factory=list)

    def serialise_lists(self, context: str, bba: BBAWriter):
        bba.label(f"{context}_variants")
        for i, variant in enumerate(self.variants):
            bba.u32(variant.index)
    def serialise(self, context: str, bba: BBAWriter):
        bba.u32(self.name_prefix.index)
        bba.u16(self.site_x)
        bba.u16(self.site_y)
        bba.u16(self.rel_x)
        bba.u16(self.rel_y)
        bba.u16(self.int_x)
        bba.u16(self.int_y)
        bba.slice(f"{context}_variants", len(self.variants))

@dataclass
class TileExtraData(BBAStruct):
    name_prefix: IdString
    tile_x: int
    tile_y: int
    sites: list[SiteInst] = field(default_factory=list)

    def serialise_lists(self, context: str, bba: BBAWriter):
        for i, site in enumerate(self.sites):
            site.serialise_lists(f"{context}_si{i}", bba)
        bba.label(f"{context}_sites")
        for i, site in enumerate(self.sites):
            site.serialise(f"{context}_si{i}", bba)
    def serialise(self, context: str, bba: BBAWriter):
        bba.u32(self.name_prefix.index)
        bba.u16(self.tile_x)
        bba.u16(self.tile_y)
        bba.slice(f"{context}_sites", len(self.sites))

def timing_pip_classs(pip: xilinx_device.PIP):
    return f"{'buf' if pip.is_buffered() else 'sw'}_d{pip.max_delay()}_r{pip.resistance()}_c{pip.capacitance()}"

seen_pip_timings = set()
seen_node_timings = set()

# --- prjxray bitstream-representation lookup -------------------------------
# A tile-routing pip is only usable if the bitstream can express it, i.e. its
# fasm feature "<TILETYPE>.<dst_wire>.<src_wire>" appears in either
# segbits_<tiletype>.db (has config bits) or ppips_<tiletype>.db (a pseudo-pip
# needing no bits).  A pip in neither is a trap: nextpnr will happily route
# through it, the fasm line is then dropped by fasm2frames, and the resulting
# bitstream silently lacks the connection.  Mark those so the router can avoid
# them.  See PIP_CFG_NO_BITS in extra_data.h.
PIP_CFG_ROUTETHRU = 0x1
PIP_CFG_NO_BITS = 0x80000000

xraydb_root_for_bits = None
_bits_cache = {}
bits_stats = {"known": 0, "nobits": 0, "notiledb": 0}
nobits_by_tiletype = {}

def dealias_tile_type(tile_type):
    """The tile type whose bit database actually expresses this tile.

    A top/bottom-of-column ``*_SING`` I/O tile carries no ``segbits_*_sing.db``
    of its own in any family: prjxray expresses it through the *base* tile type
    (``RIOI3_SING`` -> ``RIOI3``, ``LIOB33_SING`` -> ``LIOB33``, ...), resolving
    the tilegrid ``alias`` at assembly time (grid.py, TileSegbitsAlias).  The
    db revisions we build chipdbs against do not all carry that alias metadata
    in tilegrid.json, and the ``_SING`` suffix is the stable convention, so we
    strip it here.  Only a ``ppips_*_sing.db`` (the "0"-half pseudo-pips) ever
    exists for a SING tile, and it is merged in on top of the base type."""
    if tile_type.endswith("_SING"):
        return tile_type[:-len("_SING")]
    return tile_type

def dealias_wire_name(wire):
    """The name the base tile's bit database knows this SING wire by.

    The wires that exist only in a ``*_SING`` tile carry an extra ``_SING``
    infix over the base tile's name for the same wire:
    ``IOI_SING_LEAF_GCLK1`` is ``IOI_LEAF_GCLK1`` in ``segbits_rioi3.db``.
    The FASM writer strips exactly that infix before it emits the pip
    (fasm.cc, ``src_name.erase(spos, 5)`` for ``RIOI3_SING`` / ``LIOI3_SING``
    / ``RIOI_SING`` tiles), so the feature key the bitstream will actually
    carry is the stripped one and the classifier has to look that one up."""
    return wire.replace("_SING_", "_", 1)

def tile_type_features(tile_type):
    """Set of fasm feature keys prjxray can express for this tile type.
    Returns None if the tile type has no bit database at all."""
    if tile_type in _bits_cache:
        return _bits_cache[tile_type]
    feats = set()
    found_any = False
    # Read this tile's own db files, and -- for an aliased SING tile -- the
    # base type's as well.  Feature rows are prefixed with the file's own tile
    # type (``RIOI3.`` vs ``RIOI3_SING.``); pip_has_bits keys on the actual
    # tile type, so base-type keys are re-prefixed to match.  Without this,
    # every bitful pip of a SING tile (all in the base segbits db) is falsely
    # marked NO_BITS and the router cannot bring a clock into an upper-SING
    # OLOGIC -- the "Failed to route ... OLOGIC ... CLKINV_OUT" abort.
    base = dealias_tile_type(tile_type)
    lookup_types = [tile_type] if base == tile_type else [tile_type, base]
    for lt in lookup_types:
        for kind in ("segbits", "ppips"):
            fn = path.join(xraydb_root_for_bits, f"{kind}_{lt.lower()}.db")
            if not path.exists(fn):
                continue
            found_any = True
            with open(fn) as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln:
                        continue
                    key = ln.split()[0]
                    key_names_base_tile = lt != tile_type and key.startswith(lt + ".")
                    if key_names_base_tile:
                        key = tile_type + key[len(lt):]
                    feats.add(key)
    result = feats if found_any else None
    _bits_cache[tile_type] = result
    return result

def pip_has_bits(tile_type, dst_wire, src_wire):
    feats = tile_type_features(tile_type)
    if feats is None:
        # No database for this tile type at all -- we cannot tell, so do not
        # accuse the pip.  Counted separately and reported.
        bits_stats["notiledb"] += 1
        return True
    keys = [f"{tile_type}.{dst_wire}.{src_wire}"]
    tile_is_aliased = dealias_tile_type(tile_type) != tile_type
    if tile_is_aliased:
        # The base type's rows, merged in by tile_type_features(), name the
        # wires without the _SING infix -- and so does the FASM writer for
        # this tile.  Without this second key every clock pip of a SING tile
        # (IOI_OLOGIC0_CLKDIV <- IOI_SING_LEAF_GCLK*, the leaf clock into the
        # OLOGIC) is judged against a key nothing ever writes, marked
        # PIP_CFG_NO_BITS, and the router cannot bring a clock into a
        # top/bottom-of-column OLOGIC -- the "Failed to route ... OLOGIC ...
        # CLKDIVINV_OUT" abort on the LiteX Arty S7 demos.
        keys.append(f"{tile_type}.{dealias_wire_name(dst_wire)}.{dealias_wire_name(src_wire)}")
    pip_has_a_feature = any(k in feats for k in keys)
    if pip_has_a_feature:
        bits_stats["known"] += 1
        return True
    bits_stats["nobits"] += 1
    nobits_by_tiletype[tile_type] = nobits_by_tiletype.get(tile_type, 0) + 1
    return False

def import_tiletype(ch: Chip, tile: xilinx_device.Tile):
    tile_type = tile.tile_type()
    if tile.x == 0 and tile.y == 0:
        assert tile_type == "NULL"
        tile_type = "NULL_CORNER"
    tt = ch.create_tile_type(tile_type)
    # import tile wires
    for wire in tile.wires():
        nw = tt.create_wire(wire.name(), wire.intent())
        nw.flags = -1 # not a site wire
        if wire.is_gnd():
            nw.const_value = ch.strs.id("GND")
        if wire.is_vcc():
            nw.const_value = ch.strs.id("VCC")
    if tile_type == "NULL_CORNER":
        # pseudo ground bels
        tt.create_wire(f"GND", "GND", const_value="GND")
        tt.create_wire(f"VCC", "VCC", const_value="VCC")

        gnd = tt.create_bel(f"PSEUDO_GND", f"PSEUDO_GND", z=0)
        gnd.site = -1
        gnd.extra_data = BelExtraData(name_in_site=ch.strs.id("PSEUDO_GND"))
        gnd.flags |= BEL_FLAG_GLOBAL | BEL_FLAG_HIDDEN
        tt.add_bel_pin(gnd, "Y", "GND", PinType.OUTPUT)

        vcc = tt.create_bel(f"PSEUDO_VCC", f"PSEUDO_VCC", z=1)
        vcc.site = -1
        vcc.extra_data = BelExtraData(name_in_site=ch.strs.id("PSEUDO_VCC"))
        vcc.flags |= BEL_FLAG_GLOBAL | BEL_FLAG_HIDDEN
        tt.add_bel_pin(vcc, "Y", "VCC", PinType.OUTPUT)
    def add_pip(src_wire, dst_wire, pip_class=PipClass.TILE_ROUTING, timing="",
        site_key=-1, bel_name="", pip_config=0):
        np = tt.create_pip(src_wire, dst_wire, timing)
        np.flags = pip_class.value
        np.extra_data = PipExtraData(site_key=site_key, bel_name=ch.strs.id(bel_name),
            pip_config=pip_config)

    def lookup_site_wire(sw):
        canon_name = f"{sw.site.rel_name()}.{sw.name()}"
        if not tt.has_wire(canon_name):
            nw = tt.create_wire(name=canon_name,
                type="INTENT_SITE_GND" if sw.name() == "GND_WIRE" else "INTENT_SITE_WIRE")
            nw.flags = sw.site.primary.index
        return canon_name

    def add_site_io_pip(pin):
        pn = pin.name()
        s = pin.site
        site_key = (s.index << 8)
        if pin.tile_wire() is None:
            return None
        # TODO: timing class
        if pin.dir() in ("OUTPUT", "BIDIR"):
            if s.primary.site_type() == "IPAD" and pn == "O":
                return None
            # if s.rel_xy()[0] == 0 and s.primary.site_type() == "SLICEL" and pin.site_wire().name() == "A":
            #     # Add ground pip
            #     # self.add_pseudo_pip(self.row_gnd_wire_index, self.sitewire_to_tilewire(pin.site_wire()),
            #     #    pip_type=NextpnrPipType.CONST_DRIVER)
            #     pass
            add_pip(lookup_site_wire(pin.site_wire()), pin.tile_wire().name(),
                pip_class=PipClass.SITE_EXIT, timing="SITE_NULL")
        else:
            if s.site_type() in ("SLICEL", "SLICEM"):
                # Add permuation pseudo-pips for LUT inputs
                swn = pin.site_wire().name()
                if len(swn) == 2 and swn[0] in "ABCDEFGH" and swn[1] in "123456":
                    i = int(swn[1])
                    for j in range(1, 7):
                        if (i == 6) != (j == 6):
                            continue # don't allow permutation of input 6
                        pip_config = ("ABCDEFGH".index(swn[0]) << 8) | ((j - 1) << 4) | (i - 1)
                        if s.rel_xy()[0] == 1:
                            pip_config |= (4 << 8)
                        add_pip(s.pin(f"{swn[0]}{j}").tile_wire().name(), lookup_site_wire(pin.site_wire()),
                            pip_class=PipClass.LUT_PERMUTATION,
                            pip_config=pip_config,
                            site_key=(s.index << 8),
                            timing="SITE_NULL"
                        )
                    # Add LUT route-through pseudo-pip
                    rt_config = ("ABCDEFGH".index(swn[0]) << 8)
                    if s.rel_xy()[0] == 1:
                        rt_config |= (4 << 8)
                    rt_config |= ((i - 1) << 1)
                    add_pip(pin.tile_wire().name(), s.pin(f"{swn[0]}").tile_wire().name(),
                            pip_class=PipClass.LUT_ROUTETHRU,
                            pip_config=rt_config,
                            site_key=(s.index << 8),
                            timing="SITE_NULL"
                        )

                    return

            add_pip(pin.tile_wire().name(), lookup_site_wire(pin.site_wire()),
                pip_class=PipClass.SITE_ENTRANCE, timing="SITE_NULL")

    # TODO: ground/vcc
    tile_wire_count = len(tt.wires)
    # prjxray bel z is only unique within a site; the per-tile map below
    # re-allocates colliding z values so the archcheck location roundtrip
    # (bel <-> Loc) stays bijective within each tile type.
    seen_bel_z = {}
    # BRAM tiles carry semantic z slots (0..11, see extra_data.h BEL_*) that
    # the packer/fasm writer index directly; the inversion bels (RDCLKINV &
    # co) must keep clear of them, so they start at z 12 in these tiles.
    is_bram_tile = tile_type in ("BRAM_L", "BRAM_R")
    bram_nonsem_count = 0
    for site in tile.sites():
        seen_pins = set()
        seen_pips = set()
        for variant_idx, variant in enumerate(site.available_variants()):
            if variant in ("FIFO36E1", ): #unsupported atm
                continue
            sv = site.variant(variant)
            variant_key = (site.index << 8) | (variant_idx & 0xFF)
            # Import site bels
            for bel in sv.bels():
                default_z = (12 + bram_nonsem_count) if is_bram_tile else len(tt.bels)
                z = filters.get_bel_z_override(bel, default_z)
                # Overriden z of -1 means we skip this bel
                if z == -1:
                    continue
                is_non_semantic_bram_bel = is_bram_tile and z == default_z
                if is_non_semantic_bram_bel:
                    bram_nonsem_count += 1
                bel_name = gen_bel_name(sv, bel.name())
                # Site variants can import overlapping bels (e.g. the
                # ILOGICE3 site's ILOGICE3/ILOGICE2/ISERDESE2 variants all
                # carry a CE1USED bel); give non-primary variants a distinct
                # name suffix so bel names stay unique within a tile
                # (archcheck `bel != bel2` would otherwise fail).
                full_bel_name = f"{site.rel_name()}.{bel_name}"
                if variant_idx > 0:
                    full_bel_name += f"~{variant}"
                # Site variants can import overlapping bels sharing a z (e.g.
                # the ILOGICE3 site's ILOGICE3/ILOGICE2/ISERDESE2 variants all
                # carry a CE1USED bel): re-allocate a fresh tile-wide unique z
                # for the later one.
                if z in seen_bel_z:
                    z = len(tt.bels)
                    while z in seen_bel_z:
                        z += 1
                seen_bel_z[z] = full_bel_name
                nb = tt.create_bel(name=full_bel_name, type=filters.get_bel_type_override(bel.bel_type()), z=z)
                nb.site = variant_key
                nb.extra_data = BelExtraData(name_in_site=ch.strs.id(bel_name))
                if bel.bel_class() == "RBEL": nb.flags |= 2
                if filters.is_global_bel(bel):
                    nb.flags |= BEL_FLAG_GLOBAL
                # TODO: extra data (site etc)
                for pin in bel.pins():
                    tt.add_bel_pin(nb, pin.name, lookup_site_wire(pin.site_wire()), lookup_port_type(pin.dir()))
            # Import site pins
            for pin in sv.pins():
                pin_key = (pin.name(), pin.site_wire().name())
                if pin_key not in seen_pins:
                    add_site_io_pip(pin)
                    seen_pins.add(pin_key)
            # Import site pips
            for site_pip in sv.pips():
                if "LUT" in site_pip.bel().bel_type():
                    continue # ignore site LUT route-throughs
                bel_name = site_pip.bel().name()
                bel_pin = site_pip.bel_input()
                if (bel_name == "ADI1MUX" and bel_pin == "BDI1") or \
                    (bel_name == "BDI1MUX" and bel_pin == "DI") or \
                    (bel_name == "CDI1MUX" and bel_pin == "DI") or \
                    (bel_name.startswith("TFBUSED")) or \
                    (bel_name == "OMUX") or \
                    (bel_name == "IDELMUXE3"):

                    continue
                # Site variants can re-import the same site pip (e.g. every
                # ILOGICE3 variant carries IDATAININV_OUT -> IDATAIN); import
                # each (src, dst) once so pip names stay unique within a tile
                # (archcheck `pip != pip2` would otherwise fail).
                pip_key = (site_pip.src_wire().name(), site_pip.dst_wire().name())
                if pip_key in seen_pips:
                    continue
                seen_pips.add(pip_key)
                add_pip(lookup_site_wire(site_pip.src_wire()),
                    lookup_site_wire(site_pip.dst_wire()),
                    pip_class=PipClass.SITE_INTERNAL,
                    site_key=variant_key,
                    bel_name=bel_name,
                    pip_config=ch.strs.id(bel_pin).index,
                    timing="SITE_NULL"
                )
    # Import tile pips
    for pip in tile.pips():
        if not filters.include_pip(tile.tile_type(), pip):
            continue
        tcls = timing_pip_classs(pip)
        if tcls not in seen_pip_timings:
            ch.timing.set_pip_class(f"DEFAULT", tcls,
                delay=TimingValue(int(pip.min_delay()*1000), int(pip.max_delay()*1000)), # ps
                in_cap=TimingValue(int(pip.capacitance()*1000)), # fF
                out_res=TimingValue(int(pip.resistance())), # mohm
                is_buffered=pip.is_buffered())
            seen_pip_timings.add(tcls)
        rt = PIP_CFG_ROUTETHRU if pip.is_route_thru() else 0
        # Route-thru pips are not tile config bits at all (they borrow a site
        # bel), so the segbits/ppips test does not apply to them.
        fwd_nb = 0 if rt else (
            0 if pip_has_bits(tile_type, pip.dst_wire().name(), pip.src_wire().name()) else PIP_CFG_NO_BITS)
        add_pip(pip.src_wire().name(), pip.dst_wire().name(), pip_class=PipClass.TILE_ROUTING, timing=tcls,
            pip_config=rt | fwd_nb)
        if pip.is_bidi():
            rev_nb = 0 if rt else (
                0 if pip_has_bits(tile_type, pip.src_wire().name(), pip.dst_wire().name()) else PIP_CFG_NO_BITS)
            add_pip(pip.dst_wire().name(), pip.src_wire().name(), pip_class=PipClass.TILE_ROUTING, timing=tcls,
                pip_config=rt | rev_nb)

def import_sdf_timings(variant, sdfcell):
    # TODO: anything other than comb
    for entry in sdfcell.entries:
        if isinstance(entry, parse_sdf.IOPath):
            variant.add_comb_arc(entry.from_pin, entry.to_pin,
                TimingValue(int(min(entry.rising.minv, entry.falling.minv)*1000),
                    int(max(entry.rising.maxv, entry.falling.maxv)*1000)))

def import_bram_timings(timing, sdf, is_virtex7=False):
    # RAMB18 hold figures read off Vivado's own probed paths on the VC707
    # oracle (docs/extracted-timing.md), in place of the SDF's generic
    # (and much wider) device-level hold envelope. No equivalent
    # measurement exists for any other family.
    def vivado_hold_ps(port):
        if not is_virtex7: return None
        table = {
            "DIADI": 255, "DIPADIP": 255, "DIBDI": 296, "DIPBDIP": 296,
            "ADDRARDADDR": 183, "ADDRBWRADDR": 183,
            "ENARDEN": 96, "ENBWREN": 96,
            "WEA": 46, "WEBWE": 46,
        }
        # bram36's control-bus ports carry a trailing L/U (18K-half) suffix
        if port not in table and port[-1:] in ("L", "U") and port[:-1] in table:
            port = port[:-1]
        return table.get(port)

    def import_bus_sethold(cell, port, width, clock, entry):
        override = vivado_hold_ps(port)
        hold = TimingValue(override) if override is not None else TimingValue(int(entry.hold.minv*1000), int(entry.hold.maxv*1000))
        for i in range(width):
            cell.add_setup_hold(clock, f"{port}{i}", ClockEdge.RISING, TimingValue(int(entry.setup.minv*1000), int(entry.setup.maxv*1000)),
                hold)

    def import_bus_clkq(cell, port, width, clock, entry):
        for i in range(width):
            cell.add_clock_out(clock, f"{port}{i}", ClockEdge.RISING, TimingValue(int(min(entry.rising.minv, entry.falling.minv)*1000),
                int(max(entry.rising.maxv, entry.falling.maxv)*1000)))


    def import_pin_sethold(cell, port, clock, entry):
        override = vivado_hold_ps(port)
        hold = TimingValue(override) if override is not None else TimingValue(int(entry.hold.minv*1000), int(entry.hold.maxv*1000))
        cell.add_setup_hold(clock, f"{port}", ClockEdge.RISING, TimingValue(int(entry.setup.minv*1000), int(entry.setup.maxv*1000)),
                hold)

    # The data-in setup and hold depend on the port's WRITE_MODE: the SDF has
    # one cell per RAM_MODE and WRITE_MODE (NC = NO_CHANGE, RF = READ_FIRST,
    # WF = WRITE_FIRST), and READ_FIRST's setup and hold are shorter. DIADI and
    # DIPADIP take WRITE_MODE_A's entry, DIBDI and DIPBDIP WRITE_MODE_B's, so
    # each variant name ends in both modes; the uarch picks it per cell.
    def import_data_in_sethold(cell, di_width, dip_width, wsdp, mode_a, mode_b):
        # In SDP both halves of the data in are written by the port B clock.
        clock_a = "CLKBWRCLK" if wsdp else "CLKARDCLK"
        ram_mode = "RAMB18SDP" if wsdp else "RAMB18TDP"
        for port, mode in (("A", mode_a), ("B", mode_b)):
            sdfcell = sdf.cells[(f"RAMBFIFO36E1RAM_MODE_{ram_mode}_U_WRITE_MODE_U_{mode}_EN_ECC_READ_FALSE_EN_ECC_WRITE_FALSE", "RAMBFIFO36E1")]
            for entry in sdfcell.entries:
                if isinstance(entry, parse_sdf.SetupHoldCheck):
                    if port == "A" and entry.pin == "DIADIU": import_bus_sethold(cell, "DIADI", di_width, clock_a, entry)
                    if port == "A" and entry.pin == "DIPADIPU": import_bus_sethold(cell, "DIPADIP", dip_width, clock_a, entry)
                    if port == "B" and entry.pin == "DIBDIU": import_bus_sethold(cell, "DIBDI", di_width, "CLKBWRCLK", entry)
                    if port == "B" and entry.pin == "DIPBDIPU": import_bus_sethold(cell, "DIPBDIP", dip_width, "CLKBWRCLK", entry)

    write_modes = ("NC", "RF", "WF")
    for wsdp, rsdp, mode_a, mode_b in itertools.product((False, True), (False, True), write_modes, write_modes):
        variant = f"{'WSDP' if wsdp else 'WTDP'}_{'RSDP' if rsdp else 'RTDP'}_{mode_a}_{mode_b}"
        bram18 = timing.add_cell_variant("DEFAULT", f"RAMB18E1_RAMB18E1_{variant}")

        for entry in sdf.cells[("RAMBFIFO36E1", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.SetupHoldCheck):
                if entry.pin == "ADDRAU": import_bus_sethold(bram18, "ADDRARDADDR", 14, "CLKARDCLK", entry)
                if entry.pin == "ADDRBU": import_bus_sethold(bram18, "ADDRBWRADDR", 14, "CLKBWRCLK", entry)
                if entry.pin == "WEAU": import_bus_sethold(bram18, "WEA", 4, "CLKARDCLK", entry)
                if entry.pin == "WEBU": import_bus_sethold(bram18, "WEBWE", 8, "CLKBWRCLK", entry)
        for entry in sdf.cells[("RAMBFIFO36E1_ISFIFO_FALSE", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.SetupHoldCheck):
                if entry.pin == "ENARDENU": import_pin_sethold(bram18, "ENARDEN", "CLKARDCLK", entry)
                if entry.pin == "ENBWRENU": import_pin_sethold(bram18, "ENBWREN", "CLKBWRCLK", entry)
                if entry.pin == "RSTRAMAU": import_pin_sethold(bram18, "RSTRAMARSTRAM", "CLKARDCLK", entry)
                if entry.pin == "RSTRAMBU": import_pin_sethold(bram18, "RSTRAMB", "CLKBWRCLK", entry)
        import_data_in_sethold(bram18, 16, 2, wsdp, mode_a, mode_b)
        for entry in sdf.cells[("RAMBFIFO36E1RAM_MODE_U_RAMB18SDP_U_DOA_REG_U_0_EN_ECC_READ_FALSE", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.IOPath):
                if entry.to_pin == "DOADOU": import_bus_clkq(bram18, "DOADO", 16, "CLKARDCLK", entry)
                if entry.to_pin == "DOPADOPU": import_bus_clkq(bram18, "DOPADOP", 2, "CLKARDCLK", entry)
        for entry in sdf.cells[("RAMBFIFO36E1RAM_MODE_U_RAMB18TDP_U_DOB_REG_U_0_EN_ECC_READ_FALSE", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.IOPath):
                if entry.to_pin == "DOBDOU": import_bus_clkq(bram18, "DOBDO", 16, "CLKARDCLK" if rsdp else "CLKBWRCLK", entry)
                if entry.to_pin == "DOPBDOPU": import_bus_clkq(bram18, "DOPBDOP", 2, "CLKARDCLK" if rsdp else "CLKBWRCLK", entry)

        bram36 = timing.add_cell_variant("DEFAULT", f"RAMB36E1_RAMB36E1_{variant}")

        for u in ('L', 'U'):
            for entry in sdf.cells[("RAMBFIFO36E1", "RAMBFIFO36E1")].entries:
                if isinstance(entry, parse_sdf.SetupHoldCheck):
                    if entry.pin == "ADDRAU": import_bus_sethold(bram36, f"ADDRARDADDR{u}", 15 if u == 'L' else 14, "CLKARDCLK", entry)
                    if entry.pin == "ADDRBU": import_bus_sethold(bram36, f"ADDRBWRADDR{u}", 15 if u == 'L' else 14, "CLKBWRCLK", entry)
                    if entry.pin == "WEAU": import_bus_sethold(bram36, f"WEA{u}", 4, f"CLKARDCLK{u}", entry)
                    if entry.pin == "WEBU": import_bus_sethold(bram36, f"WEBWE{u}", 8, f"CLKBWRCLK{u}", entry)
            for entry in sdf.cells[("RAMBFIFO36E1_ISFIFO_FALSE", "RAMBFIFO36E1")].entries:
                if isinstance(entry, parse_sdf.SetupHoldCheck):
                    if entry.pin == "ENARDENU": import_pin_sethold(bram36, f"ENARDEN{u}", f"CLKARDCLK{u}", entry)
                    if entry.pin == "ENBWRENU": import_pin_sethold(bram36, f"ENBWREN{u}", f"CLKBWRCLK{u}", entry)
                    if entry.pin == "RSTRAMAU": import_pin_sethold(bram36, f"RSTRAMARSTRAM{u}", f"CLKARDCLK{u}", entry)
                    if entry.pin == "RSTRAMBU": import_pin_sethold(bram36, f"RSTRAMB{u}", f"CLKBWRCLK{u}", entry)
        import_data_in_sethold(bram36, 32, 4, wsdp, mode_a, mode_b)
        for entry in sdf.cells[("RAMBFIFO36E1RAM_MODE_U_RAMB18SDP_U_DOA_REG_U_0_EN_ECC_READ_FALSE", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.IOPath):
                if entry.to_pin == "DOADOU": import_bus_clkq(bram36, "DOADO", 32, "CLKARDCLK", entry)
                if entry.to_pin == "DOPADOPU": import_bus_clkq(bram36, "DOPADOP", 4, "CLKARDCLK", entry)
        for entry in sdf.cells[("RAMBFIFO36E1RAM_MODE_U_RAMB18TDP_U_DOB_REG_U_0_EN_ECC_READ_FALSE", "RAMBFIFO36E1")].entries:
            if isinstance(entry, parse_sdf.IOPath):
                if entry.to_pin == "DOBDOU": import_bus_clkq(bram36, "DOBDO", 32, "CLKARDCLK" if rsdp else "CLKBWRCLK", entry)
                if entry.to_pin == "DOPBDOPU": import_bus_clkq(bram36, "DOPBDOP", 4, "CLKARDCLK" if rsdp else "CLKBWRCLK", entry)

def main():
    xlbase = path.join(path.dirname(path.realpath(__file__)), "..")

    parser = argparse.ArgumentParser()
    parser.add_argument("--xray", help="Project X-Ray device database path for current family (e.g. ../prjxray-db/artix7)", type=str, required=True)
    parser.add_argument("--metadata", help="nextpnr-xilinx site metadata root", type=str, default=path.join(xlbase, "meta", "artix7"))
    parser.add_argument("--device", help="name of device to export", type=str, required=True)
    parser.add_argument("--constids", help="name of nextpnr constids file to read", type=str, default=path.join(xlbase, "constids.inc"))
    parser.add_argument("--bba", help="bba file to write", type=str, required=True)
    args = parser.parse_args()

    # Init database paths
    metadata_root = args.metadata
    xraydb_root = args.xray
    if "xc7z" in args.device:
        metadata_root = metadata_root.replace("artix7", "zynq7")
        xraydb_root = xraydb_root.replace("artix7", "zynq7")
    if "xc7k" in args.device:
        metadata_root = metadata_root.replace("artix7", "kintex7")
        xraydb_root = xraydb_root.replace("artix7", "kintex7")
    if "xc7s" in args.device:
        metadata_root = metadata_root.replace("artix7", "spartan7")
        xraydb_root = xraydb_root.replace("artix7", "spartan7")
    if "xc7v" in args.device:
        metadata_root = metadata_root.replace("artix7", "virtex7")
        xraydb_root = xraydb_root.replace("artix7", "virtex7")
    is_virtex7 = "xc7v" in args.device
    # segbits_*.db / ppips_*.db live alongside the tile_type_*.json we import
    global xraydb_root_for_bits
    xraydb_root_for_bits = xraydb_root
    # Load prjxray device data
    d = xilinx_device.import_device(args.device, xraydb_root, metadata_root)
    # Init constant ids
    ch = Chip("xilinx", args.device, d.width, d.height)
    ch.strs.read_constids(path.join(path.dirname(__file__), args.constids))
    ch.set_speed_grades(["DEFAULT", ]) # TODO: figure out how speed grades are supposed to work in prjxray
    # Import tile types
    for tile in d.tiles:
        tile_type = tile.tile_type()
        if tile.x == 0 and tile.y == 0:
            assert tile_type == "NULL"
            # location for pseudo gnd/vcc bels
            tile_type = "NULL_CORNER"
        if tile_type not in ch.tile_type_idx:
            import_tiletype(ch, tile)
        ti = ch.set_tile_type(tile.x, tile.y, tile_type)
        prefix, tx, ty = tile.split_name()
        ti.extra_data = TileExtraData(ch.strs.id(prefix), tx, ty)
        for site in tile.sites():
            ti.extra_data.sites.append(SiteInst(
                name_prefix=ch.strs.id(site.prefix),
                site_x=site.grid_xy[0], site_y=site.grid_xy[1],
                rel_x=site.rel_xy()[0], rel_y=site.rel_xy()[1],
                int_x=tile.interconn_xy[0], int_y=tile.interconn_xy[1],
                variants=[ch.strs.id(v) for v in site.available_variants()]
            ))
    # Import nodes
    seen_nodes = set()
    tt_used_wires = {}
    print("Processing nodes...")
    for row in range(d.height):
        # TODO: GND and VCC row connectivity...
        for col in range(d.width):
            t = d.tiles_by_xy[col, row]
            for w in t.wires():
                n = w.node()
                uid = n.unique_index()
                if uid in seen_nodes:
                    continue
                if len(n.wires) > 1:
                    node_wires = []
                    node_cap = 0
                    node_res = 0
                    # Add interconnect tiles first for better delay estimates in nextpnr
                    for j in range(2):
                        for w in n.wires:
                            if (w.tile.tile_type() in ("INT", "INT_L", "INT_R")) != (j == 0):
                                continue
                            if w.tile.tile_type() not in tt_used_wires:
                                tt_used_wires[w.tile.tile_type()] = w.tile.used_wire_indices()
                            node_cap += int(w.capacitance() * 1000)
                            node_res += int(w.resistance())
                            # prune wires from nodes that just pass through a tile without connecting to
                            # bels/pips, because nextpnr doesn't care about these
                            if w.index not in tt_used_wires[w.tile.tile_type()]:
                                continue
                            node_wires.append(NodeWire(w.tile.x, w.tile.y, w.index))
                    if len(node_wires) > 1:
                        timing_class = f"node_c{node_cap}_c{node_res}"
                        ch.add_node(node_wires, timing_class=timing_class)
                        if timing_class not in seen_node_timings:
                            ch.timing.set_node_class("DEFAULT", timing_class, delay=TimingValue(0), cap=TimingValue(node_cap), res=TimingValue(node_res))
                            seen_node_timings.add(timing_class)
                    del node_wires
                seen_nodes.add(uid)
    # Stub timing class
    ch.timing.set_pip_class("DEFAULT", "SITE_NULL", delay=TimingValue(20))
    # Stub bel timings. On virtex7 the LUT comb delay and the FF D/CK->Q
    # figures are replaced by the VC707 Vivado-oracle calibration
    # (docs/extracted-timing.md): nextpnr's generic guesses of 0.30 ns
    # clk->Q and 0.20 ns FF hold measured as low by 3x and 2x respectively
    # against Vivado's own probed paths.
    lut_delay = TimingValue(45, 124) if is_virtex7 else None
    ff_d_setup = TimingValue(34) if is_virtex7 else TimingValue(100, 100)
    ff_d_hold = TimingValue(87) if is_virtex7 else TimingValue(200, 200)
    ff_clk_q = TimingValue(100, 303) if is_virtex7 else TimingValue(300, 350)
    def add_lut_timing(lut):
        for i in range(1, 6+1):
            lut.add_comb_arc(f"A{i}", "O6", lut_delay or TimingValue(100, 125))
            if i <= 5: lut.add_comb_arc(f"A{i}", "O5", lut_delay or TimingValue(120, 150))
        for i in range(1, 8+1):
            lut.add_setup_hold("CLK", f"WA{i}", ClockEdge.RISING, TimingValue(100, 500), TimingValue(100, 200))
        lut.add_setup_hold("CLK", "WE", ClockEdge.RISING, TimingValue(100, 600), TimingValue(100, 100))
        lut.add_setup_hold("CLK", "DI1", ClockEdge.RISING, TimingValue(100, 100), TimingValue(100, 100))
        # DI2 is the data input of the O6 half of a 32-deep LUT-RAM (RAMD32 at
        # O6) and of an SRL16E: without a setup check those writes go untimed.
        lut.add_setup_hold("CLK", "DI2", ClockEdge.RISING, TimingValue(100, 100), TimingValue(100, 100))
    add_lut_timing(ch.timing.add_cell_variant("DEFAULT", "SLICE_LUTX"))
    ff = ch.timing.add_cell_variant("DEFAULT", "SLICE_FFX")
    ff.add_setup_hold("CK", "CE", ClockEdge.RISING, TimingValue(100, 100), TimingValue(0, 0))
    ff.add_setup_hold("CK", "SR", ClockEdge.RISING, TimingValue(100, 100), TimingValue(0, 0))
    ff.add_setup_hold("CK", "D", ClockEdge.RISING, ff_d_setup, ff_d_hold)
    ff.add_clock_out("CK", "Q", ClockEdge.RISING, ff_clk_q)

    # Load SDF for carry and mux
    timings_root = xraydb_root
    if "kintex7" in xraydb_root: # TODO: missing
        timings_root = xraydb_root.replace("kintex7", "artix7")
    slicem_sdf = parse_sdf.parse_sdf_file(path.join(timings_root, "timings", "slicem.sdf"))
    mux = ch.timing.add_cell_variant("DEFAULT", "SELMUX2_1")
    import_sdf_timings(mux, slicem_sdf.cells[("SELMUX2_1", "SLICEM/F7BMUX")])
    carry = ch.timing.add_cell_variant("DEFAULT", "CARRY4")
    import_sdf_timings(carry, slicem_sdf.cells[("CARRY4", "SLICEM")])
    # A LUT-RAM or SRL output also changes on the clock, after a write or a
    # shift. Those cells take one of these variants (assign_cell_tags in
    # xilinx.cc): the LUT arcs above plus clock-to-out, the slowest of the four
    # LUTs in the SDF. They are named after the attribute the packer gives
    # those cells, which is no cell type, so nothing else picks them up.
    def sdf_clock_out(celltype, port):
        arcs = [e for (ct, inst), cell in slicem_sdf.cells.items() if ct == celltype for e in cell.entries
                if isinstance(e, parse_sdf.IOPath) and e.from_pin == "CLK" and e.to_pin == port]
        if not arcs:
            return None
        return TimingValue(int(min(min(e.rising.minv, e.falling.minv) for e in arcs)*1000),
            int(max(max(e.rising.maxv, e.falling.maxv) for e in arcs)*1000))
    for variant, outputs in (("X_LUT_AS_DRAM", (("O6", "LUT_OR_MEM6LRAM"), ("O5", "LUT_OR_MEM5LRAM"))),
                             ("X_LUT_AS_SRL", (("O6", "LUT_OR_MEM6SHFREG"), ("MC31", "LUT_OR_MEM6SHFREG")))):
        clock_outs = [(port, sdf_clock_out(celltype, port)) for port, celltype in outputs]
        if any(delay is None for port, delay in clock_outs):
            continue
        mem = ch.timing.add_cell_variant("DEFAULT", variant)
        add_lut_timing(mem)
        for port, delay in clock_outs:
            mem.add_clock_out("CLK", port, ClockEdge.RISING, delay)

    import_bram_timings(ch.timing, parse_sdf.parse_sdf_file(path.join(timings_root, "timings", "BRAM_L.sdf")), is_virtex7)

    # Import package pins
    for package_name, package in sorted(d.packages.items(), key=lambda x:x[0]):
        pkg = ch.create_package(package_name)
        for pin, site in sorted(package.pin_map.items(), key=lambda x:x[0]):
            site_data = d.sites_by_name[site]
            bel_name = gen_bel_name(site_data, "PAD")
            tile_type = ch.tile_type_at(site_data.tile.x, site_data.tile.y)
            pkg.create_pad(pin, f"X{site_data.tile.x}Y{site_data.tile.y}",
            f"{site_data.rel_name()}.{bel_name}",
            "", 0) # TODO: bank
    # Report the bitstream-representation census.  A non-zero "no bits" count
    # is not an error -- it is the number of pips the router must now avoid
    # because prjxray cannot express them.  A large "no tile db" count means
    # whole tile types went unchecked and their pips are still traps.
    tot = bits_stats["known"] + bits_stats["nobits"]
    print("prjxray bitstream check: %d tile-routing pips, %d expressible, "
          "%d NOT expressible (%.2f%%), %d unchecked (tile type has no bit db)"
          % (tot, bits_stats["known"], bits_stats["nobits"],
             100.0 * bits_stats["nobits"] / max(tot, 1), bits_stats["notiledb"]),
          file=sys.stderr)
    for t, c in sorted(nobits_by_tiletype.items(), key=lambda kv: -kv[1])[:10]:
        print("    no bits: %-28s %6d pips" % (t, c), file=sys.stderr)
    ch.write_bba(args.bba)

if __name__ == '__main__':
    main()
