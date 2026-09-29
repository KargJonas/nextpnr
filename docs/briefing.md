# An open nextpnr backend for the Renesas SLG47910 -- briefing

*State of 2026-09-29. Written for a reader who knows FPGA toolchains and has
followed the project's goal; it says what exists, how it was made, and what is
still missing.*

## Goal

The SLG47910 (ForgeFPGA, a 32x32 array with 1120 five-input LUTs and 1120
flip-flops) has no open toolchain;
the vendor's Go Configure hub wraps a closed placer (`eda-placer`, a 10 MB
EFLX compiler from Flex Logix) that emits the bitstream. The goal is the
icestorm/Project-X-Ray result for this part: Yosys -> nextpnr -> an open
bitstream generator, verified on real silicon (a Vicharak Shrike board, whose
RP2040 loads the FPGA over SPI).

## How the backend works

```
design.v --yosys synth_xilinx--> netlist.json
         --nextpnr-generic --uarch slg47910--> placement.json + routing.json
             chipdb: the vendor's own routing graph (48,544 nodes, 310,016
             edges), decrypted from a vendor run; sites, LUT/FF/carry bels
         --route_bits--> switch-matrix bits  (encoder ported from the vendor binary)
         --emit_sdc----> site config: LUT truth tables, FF/carry/IOB muxes
         --pack--------> 45,056-byte config image -> 46,408-byte SPI image
         --> RP2040 firmware programs the FPGA; a logic analyzer watches 8 pins
```

One repository, one container: `./flow.sh design.v top pins build/x`.

## How it was made -- three phases

1. **shrike (empirical).** Fuzzing the vendor tool with thousands of generated
   designs, diffing bitstreams, and hardware-in-the-loop checks on the board.
   It produced the nextpnr uarch, the chipdb, the site-config encoder and the
   SPI image format -- and one design (a toggling flip-flop) that ran on
   silicon, but only at a few specific placements.
2. **eflx-port (decompile-driven).** Instead of inferring encodings from
   diffs, port the vendor placer itself from its Ghidra decompilation
   (1.3 M lines), validating every function against the vendor's own
   artifacts and against live traces of the running binary (e9patch, rr).
   Fed the vendor's own placement and routing, its encoders reproduce the
   vendor bitstream bit-exactly across 175 designs for routing (68,978/68,978
   bits), site config (8,265/8,265) and clock (2,449/2,449), and 95% of LUT
   truth-table bits. It has no IOB model.
3. **Kraken (general).** Both projects were throw-away tooling tied to one
   chip. Kraken is the general version: it runs the vendor tool in a harness
   under DynamoRIO capture, decompiles it, and keeps every observation with
   its provenance so that claims about the chip are tested against each other
   automatically rather than accumulating in notes.

This repository fuses the first two: shrike's nextpnr uarch and site-config
encoder, eflx-port's routing encoder, one device model (both projects' graph
files are byte-identical; the generated chipdb matches shrike's exactly).

## Methods

- **Differential fuzzing** of the vendor tool: vary one thing, diff bitstreams.
- **Porting from the decompile**, each function cited to its address and
  checked against vendor output ("oracle mode": vendor place-and-route in,
  our bits out, compared bit for bit).
- **Dynamic tracing** of the vendor binary: static binary rewriting to log its
  bit writer, record-and-replay for value provenance (now DynamoRIO in Kraken).
- **Silicon bisection**: flash variants, capture 2 s on the logic analyzer,
  count transitions per pin.
- **Ring-oscillator probes**: a five-LUT inverter ring on the PMOD pins. No
  clock involved; if it holds still, the stable state names the net branch
  that inverts. This located three routing defects in one evening.

## What the fusion found (with silicon evidence)

- The placements where shrike's flow "worked" were carried by **vendor bits
  replayed from per-placement tables** mislabelled as clock data: on silicon,
  removing the replayed routing bits stops the output. The tables grew
  from misreading the vendor's route log (nets 3 and 4, read as the clock
  spine, are the design's data nets -- the vendor's own log names net 2 the
  clock), and the entry for one placement even writes into a LUT's truth
  table.
- The 50 MHz oscillator enters at the **W0** clock input (datasheet Table 10)
  and rides the dedicated clock tree; a fixed set of entry bits is needed
  (removing them stops the flip-flop), not per-placement routing.
- Routing facts, now built in: a slice-exit mux selects by the position of
  the driving pin in its fan-in; **level-13 long-line hops invert** (seen on
  silicon; route_bits reports every such path); nextpnr could route *through*
  an unused IOB and could take a LUT output out through another slice's pins
  (both removed from the uarch); a LUT input lands on the physical input fixed
  by the delivering node, not the pin nextpnr names (the vendor's rule from
  eflx-port, now applied).

## Status

On silicon (Shrike, 50 MHz oscillator on W0): a toggle flip-flop and its LUT
run at **every one of the 140 L0 sites**, one site per bitstream, with nextpnr
placing and routing the rest; a freely placed toggle flip-flop runs; a LUT4
passes its input through on each of the four inputs of each of the four
slices; a 4-bit counter built from LUTs counts on its low bits. Output pads
GPIO8-10 and 12-15 work from the fabric.

How the dead sites were found, each by comparing with a vendor build Kraken
made of the same kind of design:

- Two switch-matrix dispatch bugs carried over from eflx-port (level-3
  down-turns from any source but a level-5 line; level-1 down-turns fed by
  level-3 branch >= 4): 497/512 and 296/4170 routed muxes wrong against the
  vendor's own routes, now 1 and 73 (the rest timing choices).
- Memory CLB rows (6/8/22/24) keep a column-dependent RAM default in BL4
  WL26-30 that the vendor clears for plain logic.
- CLK_SEL has a second copy in each RBB's middle tile (BL10 WL24 or WL31 by
  column class); the template's 1 there left columns 19 and 25 unclocked.
  Found by a vendor 256-flip-flop Johnson counter spanning the array: our
  encoding of its placement ran only once the vendor's unwritten bits were
  grafted onto a dead RBB, and this one bit sufficed.

Not yet working: anything using the carry chain (the blinky counter) and any
LUT input or flip-flop D that enters a site through its eight shared pins.
The vendor's 25-bit counter runs on the board; ours does not, because (a) the
uarch chains carries from row to row where the silicon chains them column to
column (RBB_10_16 -> RBB_13_16 -> ...), (b) CIN_SEL (BL10 WL17 of the middle
tile, both rows) is never written, and (c) the shared-pin muxes (MUX_x5,
MUX_x6, MUX_xI) are one-hot over four inputs but emit_sdc writes the site
pin number into them in binary.

## Next steps

1. Carry: chain RBB column to column in the uarch, write CIN_SEL, and compute
   S inside the LUT from its dedicated inputs with MUX_x5/x6 idle, as the
   vendor does; then the 25-bit counter and the board LED (GPIO16).
2. Shared pins: recover which of the eight shared site pins each one-hot
   select of MUX_x5/x6/xI picks (vendor packed instances give the net on
   each port; the routes give the delivering node), and restrict the uarch to
   the pins each mux can reach.
3. The east-edge IOB and GPIO11: the vendor drives GPIO11 on this board; our
   pad configuration for it is still wrong.
