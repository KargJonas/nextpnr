# 04 — Gap assessment: what `nextpnr-xilinx` has that upstream `himbaechel/uarch/xilinx` lacks

> **Status: final draft for review.** Diff of doc 02 (fork @ 0.9.3) against
> doc 01 (upstream @ nextpnr-0.11.1), plus coordinator first-hand checks
> (`drafts/00*.md`). Placement/legalisation verdicts (§6) now reference the
> completed doc 03. Items marked ⚠ are partial.
>
> **Philosophy.** Upstream is the successor architecture and is *preferred
> wherever it has a solution*; each gap below is therefore an upstream
> **blank to fill** with fork material, not a reason to replace upstream
> code. Rows explicitly say when the upstream equivalent already exists
> and the fork version must NOT be ported.
>
> Reading guide: each row = one capability gap. "Fork" column cites the fork
> implementation; "Upstream" column cites what exists (or the absence).

---

## 1. Families & devices

| Gap | Fork | Upstream |
|---|---|---|
| **Kintex-7 devices not in default build** | xc7k70t/160t/325t/420t/480t in prjxray-db + CI chipdb | ✅ **fixed in WP0**: `ALL_HIMBAECHEL_XILINX_DEVICES` now includes xc7k70t/160t/325t/420t/480t (die dirs exist in prjxray-db) |
| **Zynq-7 parts beyond z010/z020** | xc7z030/035/045/100 | ✅ **fixed in WP0**: xc7z030/045/100 added; xc7z035 deferred — no die dir in openXC7 prjxray-db (part dirs only) |
| **Spartan-7 coverage** | xc7s50 (in CI) | xc7s50 ✓ |
| **Artix-7** | xc7a35t/50t/100t/200t (a35t real part, not alias) | ⚠ xc7a50t/100t/200t + **xc7a35t aliased to the a50t die** (`xilinx.cc:86`); openXC7 prjxray-db has a35t *part* dirs (package pins) but **no a35t die dir**, so the alias stays until die data exists (WP0: verified, deferred) |
| **Virtex-7 (xc7vx485t)** | ✓ (VC707, GTX, `gtClockTemplateRoute` bodge) | ⚠ **fixed in WP0**: virtex7 meta (from openXC7 meta master, seeded from kintex7), CMake `xc7v` mapping, gen virtex7 selection + timings fallback, device regex `xc7vx\d+t?`; GTX packer still pending (WP7) |
| **UltraScale+ (xcup via RapidWright)** | xczu2cg, xczu7ev + E2/E3/E4 packers | ✗ uarch is xc7-only (`match_device`); `ng-ultra` is a different (non-Xilinx) part; E2/E3/E4 constids are vestigial |

**Gap class**: family *coverage* is mostly a device-list/database issue, not
code (kintex7/zynq7/virtex7 sites already modeled); the **xcup UltraScale+
flow is a large, separate work package** (RapidWright BBA export + E2/E3/E4
packers + FASM-via-Vivado), not part of a bugfix-grade port.

## 2. Primitives

| Primitive | Fork | Upstream | Gap |
|---|---|---|---|
| LUT1–6 / LUT6_2 / FD* / latches / INV | ✓ | ✓ | ≈ parity; fork has BEL-constrained LUT6_2 both-outputs fix (`bfdeaf7c`) and LUT6_2→MUX use-after-free fix (`31c8ea77`) — **check for same bugs upstream** |
| MUXF7/F8 | ✓ | ✓ | ≈ |
| MUXF9 | ✗ **also a hard error on xc7** (`pack.cc:648`: `if (ctx->xc7) log_error("MUXF9 is not supported on xc7!")`); F9MUX exists for xcup only | ✗ hard error (`pack.cc:441`) | **not a gap** — both sides reject MUXF9 on xc7; the fork's SELMUX2_1 mapping is F7/F8 only. Drop from the plan (earlier claim corrected) |
| CARRY4 | ✓ atomic packer + legacy split + carry-O relocation | ✓ (MUXCY/XORCY reassembly) | upstream packer is simpler; fork's carry-O relocation & legality (doc 03) are the big gap |
| CARRY8 | xcup only | constids only | in xcup package |
| SRL16E/SRLC16E/SRLC32E | ✓ SRL16E/SRLC32E + cascade placement | ✅ all three; SRLC32E Q31 cascades in D,C,B,A of one slice; an off-slice Q31 moves onto Q when Q is unused or already reads bit 31, otherwise it leaves the A LUT through `DOUTMUX` | ✅ cascade legalisation ported (WP3.4, `constrain_srl_cascades`, `pack.cc:789`; fork `697e293b`) |
| Distributed RAM | RAM32M/64M + 32/64/128/256×1S/D (same set as upstream) | RAM32M/64M + 64X1S/D, 32X1D, 128X1S/D, 256X1S/D | **not a gap** (WP3.3 verified): the fork's three fixes are already upstream (`c0194daf` scalar A0..A6 special-case at pack_dram.cc:240; `363c055d`/`b390e9c9` m256 zoffset 0 at pack_dram.cc:464); RAM512X1S/D, RAM32M16, RAM64M8, RAM64X2S, RAM64X8SW are declared-but-unpacked on BOTH sides |
| BRAM RAMB18/36E1 | ✓ TDP+SDP+registered+widths | ✓ TDP+SDP (L/U splitting, 36-bit modes) | fork fixes to port: SDP opposite-port width (`f1c77134`,`11f9b694`), 36-wide marker collision (`1b7d51b9`), ZINV_REGCLK* (`e71acda2`) |
| FIFO18/36, RAMBFIFO | constids only | ✗ (FIFO36E1 explicitly skipped in dbgen) | low priority (declare) |
| DSP48E1 | ✓ + cascade (`walk_dsp`) | ✓ + cascade | ≈ |
| IO: IBUF/OBUF/IOBUF/DS/DCIEN/DIFF | ✓ HR+HP | ✓ HR+HP | fork HP-bank fixes: IBUF_HP_BANK_GLUE Y0 skip, IN_ONLY skip, SLEW.SLOW skips, SSTL15/135 — **FASM-level, see §3** |
| IDDR/ODDR | ✓ SAME_EDGE_PIPELINED, TFF tristate, INV_OCLK | ✓ | fork IDDR fixes: SAME_EDGE_PIPELINED (`9a6a7e3b`), 4-IFF-flop init (`d455ae52`), routethru SRTYPE (`f77907ac`,`16accf3b`) |
| ISERDESE2/OSERDESE2 | ✓ master/slave, OFB, IS_CLKDIV_INVERTED, TRISTATE_WIDTH.W4 | ✓ master/slave | fork OFB placement + TRISTATE_WIDTH/CLKDIV-INV emission |
| IDELAYE2/ODELAYE2/IDELAYCTRL | ✓ (ODELAYE2 only kintex7 DBs) | ✓ | fork: IDELAYCTRL-with-no-delays = warning (`06769c05`) |
| Clocking BUFG/BUFGCTRL/BUFGCE/BUFH/BUFHCE/BUFMRCE | ✓ + phantom-BUFGCTRL guard + fabric-driven preplace + bottom-region pinning | ✅ BUFH/BUFHCE → `BUFHCE_BUFHCE` added (WP3.2); BUFG/BUFGCTRL/BUFHCE/BUFR/BUFIO/BUFMRCE | fork GBS placement rules (fabric-driven fallback, bottom-region pinning) still open (doc 03) |
| MMCME2/PLLE2 (BASE/ADV) | ✓ + LKTABLE/TABLE per-PLL, filter tables, CLKFBOUT_MULT_F range check, MMCM-reset fix | ✓ (mmcm_tables.cc + pack_clocking) | fork PLL filter-table programming (`e33b5f1a`,`74357a79`), CLKFBOUT_MULT_F range check (`0e85a878`), MMCM reset inversion fix (`09db508a` — was reverted upstream-side PR #129 in fork, check) |
| BUFR/BUFIO | ✓ + BUFR_DIVIDE on placed BUFR | ⚠ pseudo-pip config | `0b914578` BUFR_DIVIDE honouring |
| **GT transceivers (GTPE2/GTXE2)** | ✓ pack_gt_xc7.cc (channels, common, IBUFDS_GTE2 refclk, PLL remap) + GT-clock template route | ✗ metadata only | **large gap** — kintex7/virtex7 users need this |
| **Config/misc IP: BSCANE2, DNA_PORT, EFUSE_USR, ICAPE2, FRAME_ECCE2, STARTUPE2, USR_ACCESSE2, DCIRESET** | ✓ single-site preplacement (`d42d6c9b`) | ✗ no packer | **gap** — single-site preplacement pattern is easy to port |
| **XADC/SYSMONE1** | ✗ no packer either (only a guard error at `pack_io_xc7.cc:374`) | ✗ | **not a gap** — unsupported on both sides |
| PS7 | ✓ | ✓ (input tie-off present) | ≈ |
| PCIE_2_1 | ✓ (artix7 DB) | ✗ no packer | gap |
| PHASER_IN/OUT/REF | ✗ (only pin tables + pip blacklist comment `arch.cc:464-473`) | ✗ | not a gap |

## 3. FASM / bitstream correctness (fork's largest fix cluster)

Upstream `fasm.cc` (1864 L) vs fork (5292 L): the fork's extra ~3.4k lines
are mostly IO-bank correctness glue + GT/PCIE/cfg emission. High-value
items to port (each maps to a fork commit):

1. **Run-identity header** (`7037c948`) — GIT_DESCRIBE/chipdb/seed comments; trivial, valuable for reproducibility.
2. **Phantom-BUFGCTRL guard** (`fasm.cc:122`) — fixes double-programmed BUFG/dead clock.
3. **HP-bank IO glue fixes** (`d6b7f64d`, `e4a261ce`, `70a5952c`, `c2e50b99`, `6f33adf0` + DB bumps `f8e76430`, `a92eb3d1`): RIOB18 Y0 glue skip, IN_ONLY skip, SLEW.SLOW skips, SSTL15/135.
4. **SDP BRAM width fixes** (`f1c77134`, `11f9b694`, `1b7d51b9`, `e71acda2`).
5. **PLL/MMCM LKTABLE/TABLE + DIVCLK/phase** (`e33b5f1a`, `74357a79`); upstream already has mmcm_tables.cc — diff what's missing.
6. **BUFR_DIVIDE emission** (`0b914578`).
7. **OSERDES IS_CLKDIV_INVERTED / TRISTATE_WIDTH.W4 / IFF.INV_OCLK** (`b9ed05a2`, `c05f0d05`).
8. **GT/PCIE/cfg emission** (whole new writer sections — tied to §2 packers).
9. LUT6_2 O5/O6, carry S=VCC fill + COUT spine (`b03d0c98`) — verify upstream parity.

## 4. Constraints (XDC)

| Item | Fork | Upstream | Notes |
|---|---|---|---|
| set_property on get_ports | ✓ | ✓ | |
| `-dict` | ✓ | ✓ | |
| multiple targets per set_property | ✗ (exactly 4 args) | ✓ | upstream more general — keep |
| create_clock -period | ✓ | ✓ | |
| virtual clock (no target) | warning+skip | warning only | port the skip guard |
| no-match create_clock | loud "NOT applied" warning | warning | port (00c) |
| **set_multicycle_path -setup** | ✓ (`NEXTPNR_MCP_SETUP` + timing walk) | ✗ | check upstream SDC path (`common/kernel/sdc.cc`) before porting the attribute hack |
| **`name[0]` one-bit-vector de-busing** | ✓ (`b257be4d`) | ✗ | port (00c) |
| **silent non-design targets (verbose-only)** | ✓ (`3da43687`,`555d326c`) | ✗ (always warns) | port (00c) |
| **BEL attr on unknown tile = non-fatal** | ✓ (`8399469c`) | ? | check upstream behaviour |
| set_false_path / set_input_delay / set_output_delay / generated clocks | ignored (both) | ignored | future work, not a regression |

## 5. Timing

| Item | Fork | Upstream |
|---|---|---|
| Delay model | per-BEL SDF from prjxray timings/ (slicel/m, BRAM, carry4) | chipdb pip/node/cell timing; **LUT/FF stubbed** in gen; BRAM SDF-imported |
| Speed grades | real parts' grades | **single `DEFAULT` grade** (`xilinx_gen.py:387`) |
| DSP48E1 comb model | ✓ (`dsp48e1IsCombinational`) | ? (verify in doc 01 §5) |
| Multicycle | `NEXTPNR_MCP_SETUP` in timing walk | upstream SDC path (verify) |
| Final timing after router2 | ✓ (`7ea51730`) | ⚠ router2 hooks (doc 03 will confirm) |
| `--report` JSON | backported (`3266cf4d`) | ✓ native (`common/kernel/report.cc`) — nothing to port |
| Comb-loop policy | warn+ignore by default (`f6e06896`) | ? (verify) |
| Derived clocks through BUFG/PLL/MMCM | ✓ | ✓ (`generate_constraints`) |

## 6. Placement / legalisation / routing

*Full analysis in doc 03 (upstream algorithms kept; fork legality to port).*
Summary of the fork-side legality inventory (from doc 02 §6):

- Per-position site-exit (OUTMUX) budget (`f0f2c817`)
- Cross-position carry→FF rejection + 5FF-feed + FF1_uses_x guards (`b888dfd6`)
- Carry sum-FF co-location control-set guard (`70949581`)
- Carry-O relocation: split/sum-duplication/spine-link/row-spread (`c2c05095`, `8d4b732d`, `254a50ab`, `ecf5edd8`, `35c78e03`, …)
- SRL cascade placement like carry chains (`697e293b`)
- Frozen-tile (imported Vivado) trust + validity restore (`bf78fccf`)
- HeAP legaliser hardening: per-cell timeout (`369038ed`), eviction-last-resort (`80a44148`), deterministic sparse fallback (`347583c3`), radius budget (`4a3d7e12`), relocate pre-bound leaves (`31025814`), placer1_refine failure propagation (`0e24e3ff`)
- Preplacement: CFG single-site (`d42d6c9b`), fabric-driven BUFG + bottom-region pinning (`ce9e6a96`,`a70ae4a8`), SRCC BFS cap (`f7049aee`), IDELAYCTRL, GT
- Routing: `routeClock` backbone-first, `routeVcc` post-fill, pip blacklist (`setup_pip_blacklist`), router2 wire-reservation conflict-awareness (`df858b1e`), fixed-routes lock/unlock, GT-clock template

Which of these are (a) pure legality data to port, (b) workarounds for the
old upstream (obsolete), (c) real algorithmic improvements — **decided in
doc 03**: upstream's `StrictLegaliser` is the *same* greedy random-walk
family, refactored — so the fork's fail-fast/deterministic-fallback/
eviction-as-last-resort termination guarantees should be **ported, not
dropped** (doc 03 corrigendum); `budgetBased`/`slack_redist_iter` are
superseded by upstream `timing_opt.cc`;
the must-port set is the validity engine items (§4 of doc 03: OUTMUX budget,
cross-position carry→FF rejection, FF1-uses-X+5FF co-pack rejection, SRL
cascade placement, CFG preplacement, unconditional SLICEM/5LUT guards) plus
the netlist-level carry-O relocation decision, delay-model improvement, and
router2/routeVcc/routeClock porting risks.

## 7. Infrastructure / CI

| Item | Fork | Upstream |
|---|---|---|
| per-PR demos regression gate (5 demo projects, 3 chipdbs, bitstream hash goldens) | ✓ | ✗ (build + archcheck only) |
| Unit tests for xilinx | ✓? (primitive-tests repo external) | ✗ none in-tree |
| Examples | arty-a35, artyz7-20, attosoc, blinky, counter25 (VC707), zcu104 | arty-a35 only |
| Error-message quality (OBUFDS swapped-pin names, no-IOSTANDARD hint, CLKFBOUT_MULT_F range) | ✓ | ? |
| `--chipdb/--xdc/--fasm/--fixed-routes/--write-fixed-routes/--route-clock-only` CLI | ✓ | only `--fasm`, `--xdc` (verified `xilinx.cc:68-75`) | fixed-routes/route-clock-only = fork-only (niche) |

## 8. Not gaps (already upstream, nothing to port)

- `--report` mainline JSON (upstream native; fork backported it)
- chipdb RelPtr model, pip/node timing infrastructure, `generate_constraints`
- himbaechel framework itself (superset of fork's chipdb machinery)
- xdc `set_property` multi-target generality (upstream is better)
