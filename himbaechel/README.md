# nextpnr-himbaechel for the SLG47910

A flow from Verilog to a bitstream through nextpnr-himbaechel's `slg47910`
uarch: Yosys → nextpnr-himbaechel → this repo's bitstream tools (`bitgen/`,
`route_bits.py`, `emit_sdc.py`, `pack.py`).

The uarch is on the branch `himbaechel-slg47910` of
[KargJonas/nextpnr](https://github.com/KargJonas/nextpnr), on its way into
nextpnr; it builds its chipdb from the device database
[Project-Inkwell/forgefpga](https://github.com/Project-Inkwell/forgefpga),
whose files this repo makes: `graph.json` is `tools/gen_chipdb.py`'s
`build/chipdb.json`, `hops.json` is `tools/vendor_hops.py`'s, and
`timing.json` is `tools/kraken_timing.py`'s, from a Kraken export (below).

    make himbaechel-image FORGEFPGA=<forgefpga checkout>   # once
    make build/hb/blinky/bitstream_mcu.bin                 # designs/blinky through it
    make flash BIN=build/hb/blinky/bitstream_mcu.bin

`tools/hb_emit.py` turns nextpnr's names into the node ids and site
coordinates the bitstream tools take. `flow.sh <design.v> <top> <pins> <dir>` runs the
whole flow on any design; a pin file is the Viaduct's: `set_io <port> GPIO<n>`
or `<row> <col>`, `set_clk <port> W0|W1|N0|N1`. A design needs `clk_en` on
`0 25` (or `OSC_CTRL_EN`), or the on-chip oscillator does not run.

## The PLL

The PLL is the vendor's (app note AN-FG-006): the fabric drives its controls
through pads, and its clock comes in at W1. A design names them as the vendor's
does, ports set in the pin file by the controls' own names:

    set_clk pll_clk W1
    set_io  OSC_CTRL_EN OSC_CTRL_EN
    set_io  PLL_CTRL_EN PLL_CTRL_EN                 # and PLL_CTRL_BYPASS, PLL_REF_CLK_SELECTION
    set_io  PLL_CTRL_FBDIV[0] PLL_CTRL_FBDIV[0]     # ... [11]; REFDIV [0..5], POSTDIV1/2 [0..2]

and runs at 50 MHz x FBDIV / REFDIV / (POSTDIV1 x POSTDIV2) (on the Shrike to
0.1 %: FBDIV 16..544, REFDIV 1..32, POSTDIVs 2..7 measured, VCO 335 to 1405
MHz, the reference down to 1.6 MHz). BYPASS passes the reference through; REF_CLK_SELECTION 1 takes it from
GPIO2. A control held at a constant takes no LUT and no route: the pad holds it
in its own bits. One the design drives is routed to the pad's first or second
output (the second is the bel's OE pin) through a LUT of its own; BYPASS acts
at once, the dividers when EN rises. The vendor's builds the encoding was read
from are Kraken's (design set `pll`: a counter on the PLL, each control bit
flipped once, all driven from logic, and the oscillator and the PLL together).

Two clock domains feed the sites: clock mux 8 (W0, W1 or N0) and mux 9 (N1, or
W0). With the PLL or N0 on the first, the oscillator moves to the second, as in
the vendor's build of a design on both. A row-31 pad's output passes a
register on one of the two domains: the uarch puts it on the domain of the
flip-flops behind the pad (domain 1 when they are of both, as the vendor
does), and on the oscillator's when none is (where the vendor writes 1, a
domain no clock drives in a design on W0 alone). On the other domain it would
stop whenever that clock does: the PLL's while EN is low. N0 and N1 are
mapped from the vendor's builds only; no vendor design says what drives them
on the Shrike, so neither has run on it.

The chip states two things to the fabric, at pads of their own (app note
AN-FG-019): `PLL_LOCK`, by the second input of RBB_0_12, and `OSC_READY`, by
the first of RBB_0_25. A design reads them as inputs the pin file names as the
controls are named (`set_io PLL_LOCK PLL_LOCK`). Each leaves its pad by that
input's pin, past the pad's input register (one bit each, emit_sdc.py
`input_bits`: left set, the input passes the register on a domain no clock
drives and reads 0). On the Shrike PLL_LOCK rises 13 us after EN does and falls
with it, and OSC_READY reads 1; straight to a pad, through a LUT and through a
flip-flop. The encoding was read from Kraken's vendor builds of design set
`status`: each input read alone, both, through a LUT and through a flip-flop.

A GPIO the design reads is its pad's first input, by the same way (the IOB's
IN0, which leaves by the pad's O4) and the same bit at the pad's place (WL16,
at the tile and BL its enable has). Kraken's vendor builds of design set
`gpioall` (each of eight reading a random half of the pads) give it for every
GPIO pad but GPIO8. On the Shrike GPIO6 reads its pull-up, as the vendor's
image of the same design does.

A pad the design both drives and reads is an `inout` port driven by a
tristate buffer (`assign p = oe ? d : 1'bz`; the flow keeps one that drives a
port, `tribuf -logic`): one IOB, the data on PAD, the enable on OE and the
read on IN0. An enable the design drives passes the pad's register as the data
does (BL7 WL1 1, where 0 holds the enable at 1), and MUX_O1 takes the branch it
arrives by (Kraken's design set `inout`: GPIO11 driven, read, both, each with
its enable held and driven). On the Shrike GPIO11 drives while its enable is
high, floats to its pull-up while it is low, and reads back what the pad holds
either way. Row 31 only: the other edges' driven enable is not read yet.

Row 0's GPIOs (GPIO0-7) drive as row 31's do, the enable held by the shared
muxes' lane-4 bit (wl13), which row 0 needs with the shared muxes on B too.
The analyser sees none of them; the RP2040 reads GPIO6, its SPI MISO, and on
the Shrike GPIO6 now drives (Vicharak's SPI examples answer through it).

## BRAM

The 32 kb of block RAM (datasheet chapter 9: eight 4 kb slices, BRAM0-3 north
and BRAM4-7 south) sits outside the fabric: the fabric drives a slice's pins
through pads and reads its data from pads, as it does the PLL's controls. A
design names them as GoConfigure's io planner does, ports set in the pin file
by the pins' own names, and the slice's clocks by a side's clock outputs:

    set_clk clk W0
    set_io  clk BRAM_N_WCLK                  # north write clock (and BRAM_N_RCLK, BRAM_S_*)
    set_io  wen BRAM0_nWEN                   # nWCLKEN, nREN, nRCLKEN, RATIO[0..1]
    set_io  d[0] BRAM0_WDATA[0]              # WDATA[0..7], WADDR[0..8], RADDR[0..8]
    set_io  q[0] BRAM0_RDATA[0]              # an input: what the slice reads

A BRAM clock output takes a clock input (the port's own, or the clock input
whose net an output port carries): clock muxes 4-7 (N out0, N out1, S out0, S
out1) take its code. A write pin held at a constant is the pad's to hold; one
the design drives passes no register at the pad (as the PLL's controls), and
RDATA bits leave their pads past the input register. The image's header powers
the blocks a design uses (register bits 320 and 321). The pin map is Vicharak's
shrike_serv io planner (all eight slices; 4-7 are 0-3 with the column
mirrored); the pads' bits, all in each pad's own tile at BL9 (muxes from WL2,
WL9, WL17, WL26, the held value at WL6 or WL13, the input's bypass at WL16 or
WL23: O1, SR, the held value and input 1 a WL further on the rows 1 mod 3),
are from Kraken's vendor builds of design set `brampads` (every pin held at 0,
at 1 or driven at random, every RDATA read or not), and the clock muxes' from
the vendor builds that set them. Not done: the clock inversions and the power
down pin (left undriven, as GoConfigure's designs leave them), a Yosys memory
mapped onto a slice, and which of CLK_6 and CLK_7 is the south read clock.

On the Shrike our build of the vendor's own BRAM test (pol0: write once, read
slowly) reads its data back, and our build of Vicharak's stack_processor (a
stack in BRAM0 over SPI) pops all sixteen values and multiplies right, as
GoConfigure's image does. The loader matters: Vicharak's ShrikeFlash resets the
FPGA with EN and PWR low together, after which the BRAM reads 0 until the chip
next powers up; hw/firmware resets it by PWR alone.

## Timing

The fabric's delays are the vendor's own: the placer's install carries them
per condition (`sdf_dly/T40ULP/*.dly`), and Kraken reads them, checks them
against the vendor's routes (to the picosecond on 9 314 of 9 323 routed
resources) and exports them with the cells' arcs its builds state
(Kraken's timing.md §8). `tools/kraken_timing.py <export.ndjson>
<forgefpga>/slg47910/timing.json` makes the database's file: a delay class
per pip -- its own and the wire's it drives -- and the bels' arcs fitted by
hand to the logic units' (its docstring says how). The speed grades are the
conditions builds were timed under, the default first; `-o speed=<grade>`
picks another. Summed along the vendor's own routes the pips' delays are
1.04 times the vendor's arrivals (the greater of rise and fall at every hop,
where the vendor follows each edge).

The Shrike at room temperature lies between the conditions, at about
three-quarters of the slow one: of eleven placements of a 4x4 multiplier in
the self-test wrapper (bist.py) the one nextpnr times at 27.9 ns under the
slow condition fails at the oscillator's 20 ns and those at 26.9 ns and less
pass (14.4 ns and 13.2 ns under the typical one); across the corpus the
slowest passing design is timed at 25.4 ns, `mul5` fails at 28.4 and
`sign_part_assign` at 33 (both pass at 80 ns a step). The vendor's own timing
of its builds sits lower on the same silicon: of eleven vendor builds of
multipliers in the wrapper, the one it times at 19.6 ns slow passes at 20 ns
and those at 21.1 ns and more fail (2026-11-15), so its slow condition is
about the Shrike's own speed, and nextpnr's delays from the export are about a
third longer than the vendor's (where is not known: the pips agree along the
vendor's routes, below). So the slow condition, the
default, is a bound a design meets with a quarter to spare, and flow.sh asks
nextpnr for the oscillator's 50 MHz under it without stopping at a miss
(`--timing-allow-fail`): bist.py says SLOW for a design that then fails at
speed and passes on the PLL at 12.5 MHz. Its earlier way, inputs that step
every fourth cycle, held only the inputs: a register behind an enable kept what
a path too long for one cycle latched at the first edge, so `dynslice` and
designs like it failed however slowly they stepped (2026-11-17).

## What the chipdb and the uarch hold, and why not more

Each limit is a thing the bitstream tools do not write yet. None is a silent
wrong bit: the router is given no way to take what the tools would get wrong.

- **The wide muxes:** a site's F7A (slice A's LUT or B's, selected by A's
  XI), F7C (C or D, by C's XI) and F8 (F7A or F7C, by B's XI), as the
  vendor's packer uses them (its MUXF records in `PNR_PACK_PLACE.log`), for
  a netlist's MUXF7 and MUXF8: the uarch makes each MUXF8 or lone MUXF7 one
  cluster with its LUTs. A mux leaves by its slice's xMUX pin (MUX_X's
  EXTRA_IN, `AMX_SEL` 00001 at all 208 of the vendor's), true; its select
  takes MUX_xI's XOR as a D from XI does. flow.sh does not make them: with
  `abc -luts 2:2,3,6:5,10,20` (synth_xilinx's) a design's LUTs grew by half
  (a BIST wrapper from 33 to 56) -- ABC's 7- and 8-input LUTs cost more LUT6
  than its 6-input mapping saves.
- **A slice's two LUTs.** The LUT6 is two LUT5 on its first five inputs;
  with the sixth tied to 1 (`x6_TIE` 0, a bit the vendor names only in its
  `*.vm`: it follows the name in all 55,532 slices of 797 builds) the slice's
  output is the upper half and its xMUX pin the lower (MUX_X's OX0, as in
  the vendor's 369 such slices), true. The uarch pairs two LUTs that share an
  input and take five in all, rewritten onto the same five; the upper one may
  keep its flip-flop, the lower one has none, and a pad's LUT is not paired.
  Few pair in the regression designs (1 to 3 each): most of their LUTs take
  six inputs or feed a flip-flop.
- **Logic, but no memory, at the 40 memory sites.** A memory site has five
  bits a slice that invert its inputs I1-I5 (BL4 WL26-30 of the slice's
  corner tile). The vendor's LUT tables are logical, and its placer
  (`FUN_0057cd90`) sets each bit to the polarity of the signal reaching the
  input. `pack.py` holds them at 0, where a memory site takes its inputs as a
  logic site does, and `hb_emit.py` takes the polarity back in the LUT tables
  as it does there. Left at the template's idle values, I5 came complemented
  in some columns and not in others. On silicon (2026-11-16):
  `simple_multiplier` with 15 LUTs in memory sites passes at speed, and `mul5`
  with 34 passes with four cycles to a step. The memory sites' LUTs are
  0.4-1.1 ns slower than the logic sites' in the vendor's timing, which the
  uarch does not know yet.
- **Each slice's second flip-flop (OXQ)** takes D from XI, the slice's first
  flip-flop's Q or the slice's lower half (MUX_OXQ's OX0, the vendor's OAQ_SEL
  100, which it uses for 329 of its 645 second flip-flops, mostly a buffer)
  and leaves by the xMUX pin only; its sync, polarity and async bits
  (OxQsyncSRen, OxQpol, OQasyncSRen) follow the first one's. The packer does
  not yet put a flip-flop behind a lower half.
- **`mul6`-`mul8` do not route** (2026-11-16): the router stays at 20-30
  overused wires, the input pools' D1 and D0 nodes of a dense column (a D1
  node feeds the four D0 nodes of a tile, each in another slice's pool, so
  neighbouring sites compete for them). Not the synthesis: the vendor's own
  netlist of `mul8s` does not route under our placement either. Not the
  graph: the vendor's routes of it take only kinds of hop the chipdb has. Not
  the flip-flops' D on XI: taking it through a LUT of its own instead, as the
  vendor's packer does, changed nothing. The vendor packs about 3.3 LUTs a
  site and routes; what its placement does that ours does not is open.
- **The carry chain** runs from slice A to D of a site and on to the next site
  up its column (40 bits a column). Yosys maps adders and comparators to
  `CARRY4` (`+/xilinx/arith_map.v`); the uarch makes each a bit per slice:
  P from the slice's LUT, G from XI (a constant is MUX_xI's fifth input, a 1
  its XOR inverts), the carry in at a chain's foot a constant (`CIN_SEL`) or,
  from logic, the G of one more bit. The sum goes to the slice's flip-flop
  (MUX_XQ) or out by the xMUX pin (MUX_X), as can the carry.
- **Only the kinds of hop the vendor's router takes in its own builds**
  (the database's `hops.json`).
- **Inversions are taken back where a net ends.** A hop up a level-13 line
  inverts; a LUT's output arrives complemented at the pool and I5 pins, a
  flip-flop's true; the I4 pins complement what reaches them; a pad shows the
  complement of what reaches it, so every output pad gets a LUT of its own.
  `hb_emit.py` counts each sink's inversions and takes an odd count back in a
  LUT's table (an input, or a pad LUT's output), or in MUX_CE / MUX_SR / MUX_xI's
  XOR (an enable, a reset, XI).
- **Enables and resets come in by the I5 pins**, which share four feeders;
  MUX_CE / MUX_SR select the feeder one-hot, by its branch. A constant enable or
  reset is selected in the site, not routed. A site's flip-flops share one
  set/reset, acting on the clock (FDRE, FDSE: xQsyncSRen) or at once (FDCE,
  FDPE: the block's QasyncSRen). It always clears what a flip-flop holds; a
  setting one holds its value inverted (xQpol 0 inverts its D), so it starts
  at 1 and the sinks of its Q take the inversion back. A flip-flop with both a
  set and a reset has no cell in the chip, and neither has a latch. The
  vendor's placer says so ("not supported in EFLX hardware! Creating SW
  work-around to mimick the data-latch functionality in LUTs") and builds a
  latch as a LUT that feeds back into itself. So does the flow (`latch_map.v`,
  after abc, which takes no loop), and Yosys makes the flip-flop of an FDCE,
  an FDPE, such a latch and a mux. The uarch times no path through a LUT's
  feedback. Both run on silicon: a latch, and a flip-flop with a set and a
  reset that come from flip-flops. A set or a reset that a LUT makes of
  inputs changing together can glitch, which a cycle simulation does not see
  (`dff_different_styles` fails its BIST so).
- **A flip-flop's clock comes from a clock input** (W0, W1, N0, N1, E0), one
  GCLK pip from every site, and a site's flip-flops share one. A clock the
  fabric makes (a divider, a gated clock, a pad that is no clock input) goes
  the vendor's way (app notes AN-FG-009, AN-FG-019, "logic as clock"): out at
  pad (31,11)'s first output, the lane's enable at (31,12) held at 1, and
  back as clock input E0, which only domain 1 takes (mux 9 at its template,
  and eight more selects, `_E0_CLOCK_BITS`); the pads are written as the
  PLL's controls are. So one such clock a design, beside one on domain 0:
  with W0, E0 and E1 the vendor's placer refuses the design too. Read from
  Kraken's builds of design set `lac` (a counter on each lane, the enable
  driven, the clock from a LUT); on the Shrike a counter on n[3] of the
  oscillator's counter runs at exactly a sixteenth of it, and `muxtree`,
  `process` and `omsp_dbg_uart` pass (2026-11-17). Their clock inputs come
  from the BIST's LFSR, and an input that reaches a clock now changes on
  every other cycle, the data between: where both changed on one edge,
  Yosys's sim took the data from before it and the chip, its clock coming
  back the long way, the data after.
- **A site holds as many slices as the design needs**, but where a carry
  chain fills it: two while its logic takes at most four fifths of two a
  site, then three or four, as the vendor's packer packs densely only past
  half the chip. The input pool nodes are shared between neighbouring sites:
  four everywhere left a small design's fanned-out inputs no way through
  (`wandwor`). The earlier hard limits (two a site, no unrelated LUT beside
  a flip-flop that takes D from the fabric) were the graph's missing edges
  into D1B1-3 and D3B4 (2026-11-17).
- A row-31 pad's enable holds where the template rests it: on its pad's
  first idle line, in every build that ran. With a net on that line the tools
  move it to the next (as the vendor's do: 2 of 633 pads in its builds), and
  what an idle level-5 line follows is not known: its template select opens
  an input the graph does not have (82: lane 4 of `level5_down`), which the
  vendor leaves in every idle one. On D5B0N0 at (20, 29) the enable held
  (three pads); on D5B2N0 at (17, 28) two pads let go (`func_width_scope`),
  and held again with that select cleared (66), or with the enable's mux on
  another line -- not with the line's up-root or any level-13 line in use
  taken off its net, and not with lane 5 (98) or lanes 4 and 5 open: the two
  lanes stand where a level-13 branch 3 would be, which the graph lacks. Moved
  off its template line by one level less, onto another D1 line, an enable
  let go too (2026-11-17: the clock enable, done and pass pads of dense
  builds; it held once, in `wreduce`). So no net takes the lines an output
  pad's enable rests on: gen_chipdb.py lists them per pad (graph.json's
  `rest`: the D1 line of the first two (0,'D') muxes of each input pin and
  the level-5 line its mux selects, in the template), the chipdb types the
  pips into them `REST:x,y`, and the uarch keeps every net off those of the
  pads that are outputs. route_bits.py still refuses a routing that moves an
  enable.

## On silicon (Shrike, 2026-11-13)

- 2026-11-14, against simulation (bist.py): random 7- and 8-input functions
  on 20 MUXF7 and 8 MUXF8, registered and not; LUT pairs (lower half out by
  xMUX).

- `designs/blinky` and 25-bit counters with routed clock enables: GPIO10 every
  334 ms, bit 12 at 6.1 kHz; counters of 3 to 25 bits, every bit at its rate.
- A toggle flip-flop at each of 200 places across the array, driving GPIO10.
- Routed clock enables and synchronous resets, alone and together, driven by
  flip-flops or by LUTs; constants 0 and 1 on the row-31 pads.
- GPIO9 reads high on this rig whatever drives it, the vendor's image too.
- Twice in about twenty loads, a counter that runs right on every other load
  had extra edges at each carry into its upper bits for the whole capture.
  Cause unknown.
