# 01 — Himbaechel framework + `uarch/xilinx`: architecture and current Xilinx support

> **Status: final draft for review.** Compiled from a dedicated analysis
> (see `drafts/01-himbaechel-architecture-draft.md`) cross-checked against
> coordinator first-hand verification (`drafts/00*.md`).
> Scope: upstream `nextpnr` at tag `nextpnr-0.11.1` (detached HEAD, commit `62e659e`).
> Source roots: `himbaechel/` (framework), `himbaechel/uarch/xilinx/` (xc7 uarch),
> `himbaechel/himbaechel_dbgen/` (dbgen), `himbaechel/uarch/ng-ultra` (UltraScale path).
> Shared placer/router in `common/place` / `common/route` is only described at its
> call sites here (doc 03 covers the algorithms).

---

## 1. Himbaechel framework overview

### Purpose

Himbaechel ("a series of bigger arches", `docs/himbaechel.md`) is the second-generation
nextpnr "uarch" framework, layered on top of the older *Viaduct* API for small arches.
Its reason to exist is scalability: a Viaduct arch builds a flat in-memory routing graph
at startup, which does not scale past ~20k logic elements. Himbaechel instead builds a
**deduplicated, compile-time chip database** ("chipdb") so that million-LUT devices stay
within ~100 MB instead of multi-GB flat databases
(`docs/himbaechel.md:1-8`, `himbaechel/himbaechel_api.h:29-52`).

The chipdb is **RapidWright-style in spirit but generated from prjxray**, not from
RapidWright directly: a Python generator (`himbaechel_dbgen/chip.py`) emits a `.bba`
(Blackbox/BBA assembly) file, compiled by `bbasm` into a position-independent binary
blob. That blob is memory-mapped at runtime and read through **relative pointers**
(`RelPtr` / `RelSlice`, defined in the kernel `relptr.h`, used throughout
`himbaechel/chipdb.h`).

### Key abstractions

- **`Arch` (himbaechel/arch.h:469)** — wraps the shared `BaseArch<ArchRanges>` and owns
  exactly one **`HimbaechelAPI` uarch** (`arch.h:489`). All database queries
  (bel/wire/pip/group iteration, names, delays) are implemented here generically against
  the chipdb PODs; the uarch supplies legality, timing, and flow hooks.
- **`HimbaechelAPI` (himbaechel_api.h:62)** — the per-uarch interface: `init_database()`,
  `getUArchOptions()`, `pack()`, `prePlace/postPlace/preRoute/postRoute`, `configurePlacerHeap/
  Static`, `getDefaultRouter`, validity callbacks (`checkBelAvail`, `checkPipAvail`,
  `isBelLocationValid`, `isPipInverting`), cell-delay lookups, cluster helpers, and drawing hooks.
- **`HimbaechelArch` (himbaechel_api.h:157)** — a global linked list of registered uarches;
  `Arch::Arch()` calls `HimbaechelArch::find_match(args.device)` and `create()` to instantiate
  the right uarch (`arch.cc:43-58`).
- **chipdb loading (`Arch::load_chipdb`, arch.cc:117-154)** — memory-maps the blob
  (`boost::iostreams::mapped_file_source`), casts it to `ChipInfoPOD*`, validates magic
  `0x00ca7ca7` and `database_version == 6` (`arch.cc:39`), then initialises extra constids
  from the blob's string pool.
- **Tile grid model** — `ChipInfoPOD` (chipdb.h:226-246) holds `tile_types` (deduplicated),
  `tile_insts` (the grid), `node_shapes`, `tile_shapes`, `packages`, `speed_grades`. Tiles
  reference a `TileTypePOD` (bels/wires/pips/groups) and a `TileRoutingShapePOD`
  (`wire_to_node` map). Tile names are synthesised as `<prefix>X<X>Y<Y>` (`arch.cc:188-200`).
- **Wire/node model** — wires within a tile are local; cross-tile connectivity is expressed by
  *nodes*. `TileWireDataPOD` carries `pips_uphill`/`pips_downhill`/`bel_pins` (chipdb.h:62-72).
  `RelNodeRefPOD` (chipdb.h:116-129) maps a tile wire to either itself
  (`MODE_TILE_WIRE`), a node root (`MODE_IS_ROOT`), or a row/global constant net
  (`MODE_ROW_CONST`/`MODE_GLB_CONST`). Iterators (e.g. `WireRange`, `UpdownhillPipRange`,
  `TileWireRange`, `BelPinRange`) in `arch.h:108-421` hide the node→tile-wire normalisation.
- **Pips** are `PipDataPOD` (chipdb.h:74-83) with `src_wire`/`dst_wire`, `type`/`flags`, a
  `timing_idx` into the speed grade, and an `extra_data` `RelPtr` (used by xilinx for
  `XlnxPipExtraDataPOD`).
- **Timing data** — `SpeedGradePOD` (chipdb.h:212-219) holds `pip_classes`, `node_classes`,
  and `cell_types` (comb/reg arcs); see §5.
- **Packages** — `PackageInfoPOD`/`PadInfoPOD` map package pins → pads (chipdb.h:136-155).

### Data flow

```
prjxray DB (tilegrid/tile_type/site_type JSON, timings/*.sdf, package_pins.csv, tileconn.json)
   + nextpnr-xilinx "meta" (site_type_*.json, wire_intents.json)
        │
        ▼  gen/xilinx_gen.py  +  himbaechel_dbgen/chip.py (bba.py writer)
   chipdb-<device>.bba  ──bbasm──▶  chipdb-<device>.bin  (memory-mapped at runtime)
        │
        ▼  Arch::load_chipdb → ChipInfoPOD*  (constids re-seeded from blob)
   Yosys JSON ─▶ Context / pack() [uarch/xilinx pack*.cc]  → SLICE_LUTX/FFX/CARRY4/RAMB*/DSP*/IOLOGIC/…
        │
        ▼  place() [common/place placer_heap / placer_static / placer1(sa)]
        │   ─ legalised by uarch isBelLocationValid + fixup_placement()
        ▼  route() [common/route router2 / router1]  (+ uarch route_clocks(), fixup_routing())
        │
        ▼  write_fasm() [fasm.cc] → .fasm
        │   ─ prjxray fasm2frames + xc7frames2bit (examples/bitgen_xray.sh) → .bit
```

The placer/router are shared: `Arch::place()` (arch.cc:266-289) dispatches to
`placer_heap` / `placer_static` / `placer1` depending on `--placer`; `Arch::route()`
(arch.cc:291-313) dispatches to `router1` / `router2`. Defaults are
`defaultPlacer = "heap"` and `defaultRouter = "default"` (which the xilinx uarch maps to
`"router2"`, `xilinx.cc:147`). The uarch configures the shared placers via
`configurePlacerHeap` / `configurePlacerStatic` (see §7/§5).

---

## 2. Per-file responsibilities — `himbaechel/uarch/xilinx`

Line counts are `wc -l` on the checkout.

| File | LoC | Role |
|------|----:|------|
| `xilinx.h` | 195 | `XilinxImpl : HimbaechelAPI` declaration; `XilinxCellTags` union (lut/ff/carry/mux net tags); `SiteIndex`; `LogicTileStatus`/`BRAMTileStatus`/`TileStatus` cache structs. |
| `xilinx.cc` | 820 | Uarch registration (`XilinxArch`, `match_device` on `xc7*`); device regex + die alias (`xc7a35t`→`xc7a50t`); `init_database`/`init`; `getUArchOptions` (`--fasm`, `--xdc`); bel/pip legality (`is_pip_unavail`, `update_logic_bel`, `update_bram_bel`); `assign_cell_tags`/`index_control_sets`; routing delay heuristics (`estimateDelay`/`predictDelay`/`getRouteBoundingBox`, `find_source_sink_locs`); placer config; `prePlace`/`postPlace`/`preRoute`/`postRoute`; `route_clocks`. |
| `cells.cc` | 197 | `XilinxPacker::create_cell` — physical cell templates (ports) for `SLICE_LUTX`, `SLICE_FFX`, `RAMD64E`, `RAMD32`, `MUXF7/8/9`, `CARRY8`, `MUXCY`, `XORCY`, `PAD`, IO buffers (`IBUF*`/`OBUF*`/`IOBUF*`/`DIFFINBUF`/`INBUF`/`IBUFCTRL`/`HPIO_VREF`), `INV`, `IDELAYCTRL`, `CARRY4`; `create_lut` helper. |
| `pack.h` | 221 | `XilinxPacker`/`XC7Packer` interfaces: `XFormRule` (generic cell→physical transform), `DRAMControlSet`/`DRAMType`, `CarryGroup`, all `pack_*`/`xform_*`/`insert_*` declarations. |
| `pack.cc` | 849 | `XilinxImpl::pack()` pipeline + generic transforms. `pack_luts` (LUT1-6/LUT6_2→`SLICE_LUTX`), `pack_ffs` (FD*→`SLICE_FFX`), `pack_lutffs`, `pack_inverters` (INV→LUT1), `pack_srls` (SRL16E/SRLC32E→LUT), `pack_muxfs`/`finalise_muxfs` (MUXF7/8→`SELMUX2_1`, MUXF9 rejected on xc7), `pack_constants`, `pack_bram`, `feed_through_lut/muxf`, `xform_cell`/`generic_xform`. Pack order at `pack.cc:821-848`. |
| `pack_carry.cc` | 403 | Carry-chain packing: `split_carry4s` (CARRY4→MUXCY/XORCY), `pack_carries` (re-group MUXCY/XORCY chains into `CARRY4`, constrain LUTs above/below, fold CI/CO, blast non-chain MUXCY→LUT3 / XORCY→LUT2). |
| `pack_dram.cc` | 527 | Distributed RAM packing: `dram_types` map (RAM32X1S/D … RAM512X1S/D); RAM64X1S/D, RAM32X1D, RAM128/256X1S/D (with MUXF7/8 decode trees), whole-slice RAM32M/RAM64M; DRAM→`SLICE_LUTX` (`X_LUT_AS_DRAM`). |
| `pack_dsp_xc7.cc` | 197 | `DSP48E1`→`DSP48E1_DSP48E1`; DSP cascade (`walk_dsp`), GND/VCC pin harvesting into `DSP_GND_PINS`/`DSP_VCC_PINS` attrs. |
| `pack_clocking.cc` | 536 | `prepare_clocking` (BUFG/BUFGCE→BUFGCTRL, MMCME2_BASE/PLLE2_BASE→_ADV); `pack_plls` (MMCM/PLL param defaults); `pack_gbs`; `preplace_clocking` (`find_bel_with_short_route`, `preplace_unique`); `route_clocks` (dedicated clock routing pass); `generate_constraints` (derived clock constraints through BUFG/PLL/MMCM). |
| `pack_io.cc` | 996 | Top-level IO: PAD insertion (`insert_pad_and_buf`), IO buffer decomposition (`decompose_iob`), HR/HP (IOB33/IOB18) transform rules, IOLOGIC packing (`pack_iologic`: IDDR/ODDR/ISERDESE2/OSERDESE2/IDELAYE2/ODELAYE2), `pack_idelayctrl` (duplicate IDELAYCTRL per bank), `check_valid_pad` (IOSTANDARD/DRIVE legality). |
| `pins.cc` | 480 | Static pin metadata: `get_invertible_pins` (IS_*_INVERTED optimisations), `get_tied_pins` (unused-pin tie values for BRAM/BUFG/PLL/DSP/IOLOGIC), `get_bram36_ul_pins` (36-bit BRAM L/U pin pairs), `get_top_level_pins`. |
| `pins.h` | 33 | Declarations for the above. |
| `xdc.cc` | 270 | Minimal XDC parser (see §4). |
| `xilinx_place.cc` | 640 | `xc7_logic_tile_valid` (full SLICEL/SLICEM legality: LUT/FF/MUX/CARRY/X-input conflicts), `isBelLocationValid`, `fixup_placement` (post-placement LUT5/6 input remap, PS7 tie-offs), `fixup_routing` (LUT permutation pip → `X_ORIG_PORT` remap, OSERDESE3 bypass). |
| `fasm.cc` | 1864 | FASM backend (`FasmBackend`): pseudo-pip config tables, LUT/FF/carry/logic, IO/IOLOGIC, BRAM, clocking (BUFGCTRL/PLL/MMCM/HCLK), DSP bit emission; MMCM/PLL divide/phase computation. |
| `mmcm_tables.cc` | 144 | `Xc7MMCM` lookup tables (FILTREG1 filter values per CLKFBOUT_MULT, LKTABLE). |
| `extra_data.h` | 107 | `XlnxBelExtraDataPOD`, `BelSiteKey`, `XlnxPipExtraDataPOD`, `PipClass` enum (TILE_ROUTING/SITE_ENTRY/SITE_EXIT/SITE_INTERNAL/LUT_PERMUTATION/LUT_ROUTETHRU/CONST_DRIVER), `SiteInstPOD`, `XlnxTileInstExtraDataPOD`, `LogicBelTypeZ`/`BRAMBelTypeZ`/`DSP48E1BelTypeZ` Z encodings. |
| `constids.inc` | 758 | IdString constant list (cell types, pins, params, wire intents). Note: it is a **superset** shared with the UltraScale vocabulary (URAM288, DSP48E2, IDELAYE3/ODELAYE3, ISERDESE3/OSERDESE3, BITSLICE, MMCME3/4, PLLE3/4, IBUFDS_GTE3/4, GTHE*, SYSMONE4) — many of these are **not** actually packed by this uarch. |
| `gen/xilinx_gen.py` | 489 | Chipdb generator entrypoint: imports prjxray + meta, builds tile types/wires/pips/site bels/site pips/nodes/timing/package pins, emits `.bba`. |
| `gen/xilinx_device.py` | 544 | prjxray→Python object model: `Device`/`Tile`/`Site`/`Wire`/`Node`/`PIP`/`Package`; loads tilegrid, tile_type JSONs, site_type JSONs, tileconn, package_pins.csv, timings. |
| `gen/filters.py` | 110 | prjxray→nextpnr mapping: bel Z overrides (SLICE/BRAM), bel-type overrides (`SLICE_LUTX`/`SLICE_FFX`/IOL_*/BUFGCTRL…), pip inclusion/exclusion rules. |
| `gen/parse_sdf.py` | 145 | Minimal SDF parser used to import prjxray cell timing. |
| `gen/tileconn.py` | 38 | Applies `tileconn.json` wire-pair merges to build cross-tile *nodes*. |
| `examples/` | — | `arty-a35/` (blinky.v + arty.xdc + blinky.sh) and `bitgen_xray.sh` (FASM→frames→bit). |

---

## 3. Supported primitives

### Logic (SLICEL / SLICEM)

| Primitive | Status | Where |
|---|---|---|
| LUT1–LUT6, LUT6_2 | ✅ packed to `SLICE_LUTX` (LUT6_2 = dual O5/O6) | `pack.cc:167-181` |
| FDRE/FDSE/FDCE/FDPE (+ `_1` neg-edge) | ✅ to `SLICE_FFX` | `pack.cc:183-223` |
| Latch (FD* as latch) | ✅ via `X_FF_AS_LATCH` attr | `xilinx.cc:590`, legality `xilinx_place.cc:297` |
| INV | ✅ folded to LUT1 | `pack.cc:808-819` |
| MUXF7 / MUXF8 | ✅ to `SELMUX2_1` (F7/F8 mux trees) | `pack.cc:348-479` |
| MUXF9 / F9MUX | ❌ **rejected on xc7** (`log_error`) | `pack.cc:442` |
| CARRY4 | ✅ (MUXCY/XORCY chain reassembly; leftovers soft-blasted to LUT2/LUT3) | `pack_carry.cc` |
| CARRY8 | declared in `cells.cc:80` but **not packed** (UltraScale path) | `cells.cc:80-89` |
| SRL16E / SRLC32E | ✅ to `SLICE_LUTX` (`X_LUT_AS_SRL`); **Q31 unsupported** | `pack.cc:481-525` (FIXME `:497`) |

### Distributed RAM

- `dram_types` declares: **RAM32X1S/D, RAM64X1S/D, RAM128X1S/D, RAM256X1S/D, RAM512X1S/D** (`pack_dram.cc:153-162`).
- Actually packed: RAM64X1S, RAM64X1D, RAM32X1D, RAM128X1D, RAM256X1D, RAM128X1S, RAM256X1S, plus whole-slice **RAM32M / RAM64M** (`pack_dram.cc:250-523`).
- **RAM512X1S/D are declared but have no packing branch** (they would survive packing unconverted — effectively unsupported). RAM32M16/RAM64M8/RAM32X2S/RAM64X8SW appear only in constids/pins.cc, not packed.

### Block RAM

| Primitive | Status |
|---|---|
| RAMB18E1 | ✅ TDP + SDP (WRITE_WIDTH_B=36) | `pack.cc:640-806` |
| RAMB36E1 | ✅ TDP + SDP (WRITE_WIDTH_B=72), L/U port splitting, 36-bit width modes | `pack.cc:640-806` |
| RAMBFIFO36E1 / RAMBFIFO18E1 / FIFO18E1 / FIFO36E1 | ❌ not packed (FIFO36E1 explicitly skipped in `gen/xilinx_gen.py:208`) |
| RAMB18E2 / RAMB36E2 / FIFO*E2 (UltraScale) | ❌ constids/invertible/tied tables only; no packer |

### DSP

- **DSP48E1** ✅ (`pack_dsp_xc7.cc:102-195`), including cascade chaining (`walk_dsp`).
- **DSP48E2** ❌ (constids only).

### IO / IOLOGIC

- Pads/buffers: PAD, IBUF, OBUF, OBUFT, IOBUF, IOBUF_DCIEN, IOBUF_INTERMDISABLE,
  IBUF_IBUFDISABLE, IBUF_INTERMDISABLE, IBUFDS(+_INTERMDISABLE_INT), OBUFDS, OBUFTDS,
  IOBUFDS(_DCIEN/_DIFF_OUT*), DIFFINBUF, HPIO_VREF, INBUF, IBUFCTRL, OBUFT_DCIEN
  (`cells.cc:99-168`, `pack_io.cc`), decomposed onto **IOB33 (HR)** / **IOB18 (HP)**.
- **IDDR** ✅ → `ILOGICE3_IFF`; **ODDR** ✅ → `OLOGICE3_OUTFF/TFF` or `OLOGICE2_OUTFF/TFF`.
- **ISERDESE2** ✅, **OSERDESE2** ✅ (master/slave cascade).
- **IDELAYE2** ✅, **ODELAYE2** ✅, **IDELAYCTRL** ✅ (duplicated per bank).
- Unconstrained IO is **not** supported (`pack_io.cc:361` — `log_error("FIXME: unconstrained IO not supported")`).

### Clocking

- **BUFG / BUFGCE** ✅ → `BUFGCTRL`; **BUFGCTRL** ✅.
- **MMCME2_BASE/ADV** ✅ (`MMCME2_ADV_MMCME2_ADV`), **PLLE2_BASE/ADV** ✅ (`PLLE2_ADV_PLLE2_ADV`).
- **BUFR / BUFIO** ✅ handled via pseudo-pip config (`fasm.cc:214-222`); **BUFHCE** ✅ (pseudo-pip `fasm.cc:189-199`); BUFGCE_DIV/BUFCE seen in route-clocks/constids.
- Global-buffer glbBufTypes for the static placer: `PSEUDO_GND/VCC/BUFGCTRL/BUFG_BUFG` (`xilinx.cc:356-359`).

### Hard IP / other

- **PS7 (Zynq)** ✅ (`PS7`→`PS7_PS7`, unused-input tie-off, `pack_io.cc:433`, `xilinx_place.cc:482-509`).
- **GT transceivers (GTPE2 / GTXE2 / GTHE2)** ❌ — site metadata exists (`meta/*/site_type_GTPE2_CHANNEL.json` etc.) and pins.cc lists invertible pins, but there is **no packing or FASM support**.
- **XADC / SYSMONE1 / BSCAN / ICAP / DNA_PORT / STARTUP / USR_ACCESS / EFUSE_USR / DCIRESET / PCIE** ❌ — site metadata only, no packer.

---

## 4. Constraints (XDC) support — `xdc.cc`

`XilinxImpl::parse_xdc` (invoked from `pack()` when `--xdc` is given, `pack.cc:824-826`)
is a line-based, regex/`boost::split` parser supporting a narrow subset:

- **`set_property`** on `[get_ports …]` and exact `[get_cells …]` targets
  (`xdc.cc:92-108`): properties are stored as cell attributes. Supports `-dict` and
  single property pairs; `[current_design]` and `INTERNAL_VREF` are warned-and-ignored.
  This is how `PACKAGE_PIN`/`LOC`, `IOSTANDARD`, `DRIVE`, `SLEW`, `PULLTYPE`, `IN_TERM`
  etc. reach the flow. `LOC` is aliased from `PACKAGE_PIN` (`pack_io.cc:341-343`).
- **`create_clock`** with `-period` (ns) on `[get_ports]`/`[get_nets]`
  (`xdc.cc:208-254`): sets `NetInfo::clkconstr` (period + 50% duty). `-name`, `-waveform`,
  `-add` are warned-and-ignored.
- **`set_multicycle_path`** supports setup multicycle tagging with `-to [get_cells ...]`
  NAME-glob selectors; `-hold` is parsed but does not set setup tags, and `-from` is
  rejected as unsupported (`xdc.cc`). `-setup N` moves the capture edge N−1 periods
  later; hold stays at the original edge (as Vivado `-setup N -hold N-1`). The Fmax of
  a tagged path is N / datapath delay (N − ½ between opposite edges). Only cells that
  keep their name through packing (e.g. `FDRE`) carry the tag; LUT-RAM macros such as
  `RAM32M` lose it.
- **Not supported**: `set_false_path`, `set_max_delay`/`set_min_delay`,
  `create_generated_clock`, `set_input_delay`/`set_output_delay`, path groups,
  `get_pins`/`get_iobanks`/`get_clocks` targets, and any constraint other than the
  three above (default: `log_warning("ignoring unsupported XDC command …")`, `xdc.cc:256`).

---

## 5. Timing

- **Timing data lives in the chipdb**, populated at generation time:
  - PIP timing: `PipTimingPOD` (`int_delay`/`in_cap`/`out_res`, buffered flag) — classed per
    pip by `gen/xilinx_gen.py:100-101,260-273` from prjxray `src_to_dst` data.
  - Node timing: `NodeTimingPOD` (`cap`/`res`/`delay`) for merged cross-tile nodes
    (`gen/xilinx_gen.py:409-447`).
  - Cell timing: `CellTimingPOD`/`CellPinTimingPOD` with comb arcs and reg arcs
    (setup/hold/clk_q) — LUT/FF stubbed in `xilinx_gen.py:450-462`; SELMUX2_1, CARRY4 and
    BRAM imported from prjxray SDF (`xilinx_gen.py:464-474`).
- **Only one speed grade exists**: `set_speed_grades(["DEFAULT"])`
  (`xilinx_gen.py:387`, TODO) and `init_database` calls `set_speed_grade("DEFAULT")`
  (`xilinx.cc:91`). Real -1/-2/-3 grades are not modelled.
- **Runtime lookups**: `Arch::get_cell_timing_idx`, `lookup_cell_delay`,
  `lookup_cell_seq_timings`, `lookup_port_tmg_type` (`arch.cc:470-520`) feed the shared
  timing engine through `getCellDelay`/`getPortTimingClass`/`getPortClockingInfo`
  (`arch.h:882-895`, defaulted in `himbaechel_api.cc:90-103`). BRAM variants select
  WTDP/WSDP × RTDP/RSDP timing by rewriting `timing_index` (`xilinx.cc:602-612`).
- **Timing-driven placement**: the *static* placer is configured with timing weights
  (`timing_c/mx/my`, `xilinx.cc:361-363`); the *heap* placer is **not** given explicit timing
  weights (only HPWL scales and control-set grouping). There is **no `timing_opt.cc`
  post-placement timing optimisation** invoked by himbaechel, and no arch-level `--report`
  timing gate in the uarch itself (generic `CommandHandler`/`timing.h` reporting applies).
- **Derived constraints**: `generate_constraints` (`pack_clocking.cc:384-535`) propagates
  `create_clock` periods through BUFGCTRL/IBUF/PLL/MMCM, computing VCO and output-clock
  frequencies for downstream timing analysis.

---

## 6. Chip families and chipdb generation inputs

### Families

`himbaechel/uarch/xilinx/meta/` (git submodule `491aefc`, "nextpnr-xilinx-meta") contains
**only per-site-type JSON + `wire_intents.json`** — there are **no per-chip database files**
here. The actual device databases come from **prjxray** at build time. Site-type counts:

| Family | site_type_*.json files | Notes |
|---|---|---|
| artix7 | 33 | includes GTPE2_CHANNEL/COMMON, PCIE_2_1, IOB33, SLICEL/M, DSP48E1, RAMB18E1/RAMBFIFO36E1… |
| kintex7 | 38 | GTXE2_CHANNEL/COMMON, IDELAYE2_FINEDELAY, ODELAYE2, ILOGICE2/3, OLOGICE2/3, IOB18/33… |
| spartan7 | 26 | no GT, no PCIE, IOB33, ILOGICE3/OLOGICE3… |
| zynq7 | 30 | adds PS7, IOB18/33, no GT |

### Devices built (CMakeLists.txt)

`ALL_HIMBAECHEL_XILINX_DEVICES = xc7a100t xc7a200t xc7a50t xc7s50 xc7z010 xc7z020`
(`himbaechel/uarch/xilinx/CMakeLists.txt`). The device regex accepts `xc7[azkv]\d+t?…`
(`xilinx.cc:81`), and `xc7a35t` is **aliased to the xc7a50t die** (`xilinx.cc:86-87`).
Notably **no `xc7k*` device is in the default list** even though `meta/kintex7` exists —
the kintex7 code path is present but not wired into a default device.

### Chipdb generation inputs

- prjxray per-family DB (`HIMBAECHEL_PRJXRAY_DB/<artix7|kintex7|spartan7|zynq7>`), consumed by
  `gen/xilinx_gen.py --xray … --device …`:
  `tilegrid.json`, `tile_type_*.json` (wires/pips/sites + `src_to_dst` timing),
  `*/package_pins.csv`, `tileconn.json`, `timings/*.sdf` (SLICEM/BRAM).
- nextpnr-xilinx meta: `site_type_*.json` (bels/pins/pips per site + variants) and
  `wire_intents.json`.
- `constids.inc` for the IdString pool.

### UltraScale / UltraScale+?

- **Not via `uarch/xilinx`.** This uarch is xc7-only (`match_device` = `xc7*`,
  `xilinx.cc:815`; MUXF9/CARRY8/E2-primitive constids exist but are not packed —
  the E2/E3/E4 vocabulary in `constids.inc` is a superset with only vestigial
  references, e.g. `xilinx.cc:235`, `pins.cc`).
- **`uarch/ng-ultra`** is the separate UltraScale-class path: it targets the single device
  **`NG-ULTRA`** (`ng_ultra.cc:1074`) using the **Project Beyond** database
  (`HIMBAECHEL_PRJBEYOND_DB`, `gen/arch_gen.py`), has its own packer (CARRY8/LUT6/DFF/BRAM/DSP
  etc., `ng-ultra/pack.cc` 2738 LoC, `constids.inc` 6026 LoC), location_map, CSV constraints,
  and a textual bitstream writer. It is *not* a drop-in UltraScale+ replacement for the xc7
  uarch — it is a distinct, actively-developed UltraScale effort for a non-Xilinx
  UltraScale-class part.
- **Consequence for the gap assessment**: the fork's `xcup` flow (Xilinx
  UltraScale+ via RapidWright, e.g. `zcu104`) has **no upstream counterpart**
  — upstream has neither Xilinx UltraScale(+) chipdbs nor E2/E3 packers in
  this uarch. Porting xcup is a separate, large work package (see docs 02/05).

---

## 7. Build / integration

- `himbaechel/CMakeLists.txt` builds the framework core (`HIMBAECHEL_SOURCES`) as
  `nextpnr-himbaechel-core` (or one binary per uarch with `HIMBAECHEL_SPLIT`), and registers
  uarches via `add_nextpnr_himbaechel_microarchitecture`. `HIMBAECHEL_UARCH` selects
  `example gowin xilinx ng-ultra gatemate` (or `all`).
- `himbaechel/uarch/xilinx/CMakeLists.txt` lists `SOURCES`, requires
  `HIMBAECHEL_PRJXRAY_DB`, and for each selected device runs
  `add_bba_produce_command(gen/xilinx_gen.py …)` + `add_bba_compile_command` to produce
  `himbaechel/xilinx/chipdb-<device>.bin` at build time.
- **Tests**: the xilinx uarch declares **no `TEST_SOURCES`** and the upstream `tests/`
  submodule (ce15412) contains only `ecp5/generic/gui/ice40` — **no himbaechel/xilinx tests**.
  (`ng-ultra` is the only himbaechel uarch with an in-tree gtest, `ng-ultra/tests/lut_dff.cc`.)
  `himbaechel/test_main.cc` is the gtest runner used when `BUILD_TESTS`.
- **Examples**: `examples/arty-a35/` (blinky for `xc7a35tcsg324-1`, synth via
  `synth_xilinx -arch xc7`) + `examples/bitgen_xray.sh` (prjxray fasm2frames/xc7frames2bit).
- **CI**: `.github/workflows/arch_ci.yml` builds `himbaechel` as one matrix entry via
  `.github/ci/build_himbaechel.sh` (`get_dependencies; build_nextpnr; run_tests;
  run_archcheck`) — this is a **build + archcheck smoke test only**; there is no prjxray DB in
  CI, so no device chipdb is generated and no P&R/bitstream regression run for xilinx.

---

## 8. Notable architectural limitations (observed)

The most meaningful `TODO`/`FIXME`/`NPNR_ASSERT_FALSE` markers in `uarch/xilinx`:

- **Unconstrained IO not supported** — `pack_io.cc:361` `log_error("FIXME: unconstrained IO not supported")`.
- **No true differential output** — `pack_io.cc:259` (`FIXME: true diff outputs`); diff outputs
  are emulated with an inverter into the N pin.
- **MUXF9/F9MUX rejected** on xc7 — `pack.cc:442` (`log_error("MUXF9 is not supported on xc7!")`).
- **SRL Q31 not implemented** — `pack.cc:497` (`FIXME: Q31 support`).
- **Routing delay model is deliberately coarse** — `xilinx.cc:767` and `:778`
  (`TODO: improve sophistication here based on old nextpnr-xilinx code`); pip/placer delay
  estimates are heuristics.
- **LUT permutation/routethru special-cases** — `xilinx.cc:291,293` (`FIXME: conflict with
  ground`, `FIXME: routethru to MUX`).
- **Carry feed-through inefficiency** — `pack_carry.cc:280` (`FIXME: in multiple fanout cases,
  cell duplication will probably be cheaper than feed-throughs`), `:40` (`FIXME: sometimes we
  can feed out of the chain`).
- **PLL/MMCM bitgen simplifications** — `fasm.cc:1485,1567` (`FIXME: variable duty cycle`),
  `fasm.cc:1557` (`FIXME: should these be calculated somehow?`), `fasm.cc:1534,1649`
  (`FIXME: should be INV not ZINV (XRay error?)`).
- **Missing pseudo-pips** — `fasm.cc:306` (`FIXME: PPIPs missing for DSPs`), `fasm.cc:312`
  (`FIXME: PPIPs missing for SING IOI3s`), `fasm.cc:224` (`FIXME: shouldn't these be in the
  X-RAY ppips database?`).
- **Timing database gaps** — single `DEFAULT` speed grade (`xilinx_gen.py:387`), stubbed LUT/FF
  timing (`xilinx_gen.py:450-462`), `xilinx_gen.py:466` (`kintex7 … TODO: missing` → reuses
  artix7 timing), `xilinx_gen.py:276` (`TODO: anything other than comb` in SDF import),
  `xilinx_gen.py:485` (`TODO: bank` for pads).
- **RapidWright-alignment hacks** — `fixup_routing` re-encodes LUT input permutations into
  `X_ORIG_PORT` attributes to keep Vivado/RapidWright happy (`xilinx_place.cc:512-625`);
  `fixup_placement` remaps LUT5/LUT6 physical inputs post-place (`xilinx_place.cc:389-481`).
- **Other** — `xilinx_device.py:425` (`FIXME: tied_value`), `:435` (`FIXME: pip/wire delays`);
  DSP constant-pin handling comments (`pack_dsp_xc7.cc:146`).

### Broader gaps vs the fork (to be expanded in docs 02–04)

- Missing primitives: GT transceivers, FIFO18/36, RAMBFIFO, XADC/SYSMON, BSCAN/ICAP/STARTUP,
  CFGLUT5, RAM512/wide RAM/M16/M8 variants, full MUXF9 (UltraScale) and CARRY8.
- Devices: only 5 distinct dies (xc7a50t/a100t/a200t/s50/z010/z020) + xc7a35t alias; no
  kintex7 in the default list; no UltraScale(+) in this uarch (separate ng-ultra effort).
- Constraints: only `set_property`/`create_clock` on ports/nets; no I/O timing, multicycle,
  false-path, generated clocks, or cell/pin targets.
- Timing: single speed grade, stub LUT/FF delays, no timing-driven post-placement optimisation.
- Tests/CI: no xilinx unit tests, no chipdb/P&R regression in upstream CI.
