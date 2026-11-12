# nextpnr-himbaechel for the SLG47910

A flow from Verilog to a bitstream through nextpnr-himbaechel's `slg47910`
uarch: Yosys → nextpnr-himbaechel → this repo's bitstream tools (`bitgen/`,
`route_bits.py`, `emit_sdc.py`, `pack.py`).

The uarch is on the branch `himbaechel-slg47910` of
[KargJonas/nextpnr](https://github.com/KargJonas/nextpnr), on its way into
nextpnr; it builds its chipdb from the device database
[Project-Inkwell/forgefpga](https://github.com/Project-Inkwell/forgefpga),
whose files this repo makes: `graph.json` is `tools/gen_chipdb.py`'s
`build/chipdb.json`, `hops.json` is `tools/vendor_hops.py`'s.

    make himbaechel-image FORGEFPGA=<forgefpga checkout>   # once
    make build/hb/blinky/bitstream_mcu.bin                 # designs/blinky through it
    make flash BIN=build/hb/blinky/bitstream_mcu.bin

`tools/hb_emit.py` turns nextpnr's names into the node ids and site
coordinates the bitstream tools take. `flow.sh <design.v> <top> <pins> <dir>` runs the
whole flow on any design; a pin file is the Viaduct's: `set_io <port> GPIO<n>`
or `<row> <col>`, `set_clk <port> W0`. A design needs `clk_en` on `0 25`, or
the on-chip oscillator does not run.

## What the chipdb and the uarch hold, and why not more

Each limit is a thing the bitstream tools do not write yet. None is a silent
wrong bit: the router is given no way to take what the tools would get wrong.

- **LUT inputs I0-I3 only, a flip-flop's D from its slice's LUT only.** I4, I5
  and a flip-flop's direct D come through the site's shared pins, whose
  one-hot MUX_x5/x6/xI `emit_sdc.py` writes as a binary pin number. Synthesis
  maps to four-input LUTs (`synth -lut 4`), and a flip-flop whose D is not
  its own LUT's gets a buffer LUT.
- **No carry chain, no memory, no MUXF, no second flip-flop of a slice, no
  logic at the 40 memory sites.**
- **Only the kinds of hop the vendor's router takes in its own builds**
  (the database's `hops.json`).
- **Inversions are taken back in LUT tables.** A hop up a level-13 line
  inverts, and so does a LUT's output wherever it leaves its site; a pad shows
  the complement of what reaches it. `hb_emit.py` takes back an odd count at a
  LUT's input in that LUT's table, and every output pad gets a LUT of its own,
  whose table takes back the level-13 hops on the way to the pad. So only nets
  that end at LUT inputs or at a pad may take a level-13 line; an enable or a
  reset a LUT drives is taken back by MUX_CE / MUX_SR's XOR.
- **Enables and resets come in by the I5 pins**, which share four feeders;
  MUX_CE / MUX_SR select the feeder one-hot, by its branch. A constant enable or
  reset is selected in the site, not routed. Only a synchronous reset is known,
  so Yosys makes every flip-flop one (`flow.sh`); an async clear or preset is
  refused.
- **At most two slices are used in one site** (`-o slices=N`): the input pool
  nodes are shared between neighbouring sites, and without it no router finishes.
- Pip delays are the graph's costs in ps, at least 100: congestion is priced by
  delay. The cells have no timing and nextpnr runs `--no-tmdriv`: no Fmax.

## On silicon (Shrike, 2026-11-13)

- `designs/blinky` and 25-bit counters with routed clock enables: GPIO10 every
  334 ms, bit 12 at 6.1 kHz; counters of 3 to 25 bits, every bit at its rate.
- A toggle flip-flop at each of 200 places across the array, driving GPIO10.
- Routed clock enables and synchronous resets, alone and together, driven by
  flip-flops or by LUTs; constants 0 and 1 on the row-31 pads.
- GPIO9 reads high on this rig whatever drives it, the vendor's image too.
- Twice in about twenty loads, a counter that runs right on every other load
  had extra edges at each carry into its upper bits for the whole capture.
  Cause unknown.
