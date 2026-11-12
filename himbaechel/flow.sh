#!/bin/bash
# Verilog -> bitstream for the SLG47910 through nextpnr-himbaechel:
#
#   yosys (synth -lut 4, every flip-flop a synchronous-reset one: the only kind
#   whose reset is known) -> nextpnr-himbaechel --uarch slg47910 (chipdb from the
#   vendor graph) -> hb_emit (names -> node ids) -> route_bits + emit_sdc + pack
#
# Usage: himbaechel/flow.sh <design.v> <top> <pins> <build_dir> [nextpnr args ...]
# Needs `make himbaechel-image`, and build/chipdb.json for the node ids (tools/gen_chipdb.py).
set -euo pipefail

HB="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HB")"
[[ $# -ge 4 ]] || { echo "usage: $0 <design.v> <top> <pins> <build_dir> [nextpnr args ...]" >&2; exit 1; }
SRC="$(realpath "$1")"; TOP="$2"; PINS="$(realpath "$3")"; OUT="$(realpath -m "$4")"; shift 4
mkdir -p "$OUT"
[[ -f "$REPO/build/chipdb.json" ]] || python3 "$REPO/tools/gen_chipdb.py" -o "$REPO/build/chipdb.json"

podman run --rm --userns=keep-id -v "$HOME:$HOME" -w "$OUT" localhost/nextpnr-slg47910-hb bash -euo pipefail -c "
yosys -q -l yosys.log -p 'read_verilog -lib -specify +/xilinx/cells_sim.v +/xilinx/cells_xtra.v; read_verilog -sv $SRC; synth -top $TOP -flatten -lut 4; dfflegalize -cell \$_SDFFE_PP0P_ 0; abc -lut 4; techmap -D LUT_WIDTH=4 -map +/xilinx/lut_map.v -map +/xilinx/ff_map.v; clean; write_json netlist.json'
nextpnr-himbaechel --device slg47910 -o pins=$PINS -o placement=placement_raw.json \
    -o routing=routing_raw.json --json netlist.json --no-tmdriv $* > nextpnr.log 2>&1 || { tail -30 nextpnr.log; exit 1; }
python3 $HB/tools/hb_emit.py $REPO/build/chipdb.json placement_raw.json routing_raw.json placement.json routing.json
python3 $REPO/tools/route_bits.py routing.json -o route.bin --lut-inputs lut_inputs.json
python3 $REPO/tools/emit_sdc.py placement.json -o placed.sdc --lut-inputs lut_inputs.json
python3 $REPO/tools/pack.py placed.sdc -o bitstream.bin --route-template route.bin --mcu bitstream_mcu.bin > pack.log
grep -E 'Unmapped|Coverage' pack.log
"
echo "$OUT/bitstream_mcu.bin"
