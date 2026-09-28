# nextpnr-slg47910

An open toolchain for the Renesas SLG47910 (ForgeFPGA): Yosys, nextpnr with an
`slg47910` Viaduct uarch, and a bitstream generator, checked on a Vicharak
Shrike board. It merges the two earlier projects -- shrike (nextpnr uarch,
site-config encoder, SPI image format, hardware loop) and eflx-port (the
switch-matrix encoder ported from the vendor placer) -- into one flow over one
device model. The briefing in [`docs/briefing.md`](docs/briefing.md) explains
how it came about and where it stands.

**Status: builds bitstreams for any design in the supported subset; none of
them runs on silicon yet.** See the briefing's "Status" and "Next steps".

## Use

```
make image                                   # once: the container (yosys, nextpnr + uarch)
./flow.sh designs/blinky/blinky.v blinky designs/blinky/blinky.pins build/blinky
python3 hw/hw.py ab build/blinky/bitstream_mcu.bin   # flash + 2 s logic-analyzer capture
```

`flow.sh` runs everything in the container; `hw/hw.py` runs on the host (it
needs pyserial and sigrok-cli) against the Shrike on `/dev/ttyACM0`, whose
RP2040 runs `hw/firmware` (PlatformIO: `pio run -e pico -t upload`), and an
fx2lafw logic analyzer whose D0..D7 are FPGA GPIO8..15.

A pins file takes `set_io <port> GPIO<n>` and `set_clk <port> W0` (the 50 MHz
oscillator). Output pads are supported on GPIO8-15.

## Layout

| path | what |
|------|------|
| `nextpnr/slg47910/` | the Viaduct uarch: chipdb loading, bels, packing, placement checks, JSON export |
| `nextpnr/upstream.patch` | the two lines nextpnr itself needs (uarch registration, a Viaduct fix) |
| `Containerfile` | nextpnr at `7de80fb` + the uarch, yosys, python |
| `device/` | the device model: decrypted vendor routing graph, template image, format tables ([provenance](device/PROVENANCE.md)) |
| `tools/gen_chipdb.py`, `tools/routing_graph.py` | vendor graph -> `build/chipdb.json` |
| `tools/route_bits.py` | nextpnr routing -> switch-matrix bits, via `bitgen/` |
| `bitgen/` | the switch-matrix encoder transcribed from the vendor placer (eflx-port) |
| `tools/emit_sdc.py` | nextpnr placement -> site configuration (LUT truth tables, FF/carry/IOB muxes) |
| `tools/pack.py` | site configuration + routing bits -> config image and SPI image |
| `tools/decrypt_eflx.py` | decrypts the vendor's `_def` files (to regenerate `device/graph`) |
| `hw/` | flashing + capture tool, RP2040 firmware |
| `designs/` | `blinky/` (the target), `probes/` (ring oscillators, clock witness, osc probe) |
