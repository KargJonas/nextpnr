#!/bin/bash
# Verilog -> bitstream for the SLG47910, all inside the repo's image
# (`make image` builds it):
#
#   yosys synth_xilinx -> nextpnr-generic --uarch slg47910
#     -> emit_sdc (site config) + route_bits (switch-matrix bits + clock entry)
#     -> pack (config image, raw .bin and the MCU/SPI .bin the Shrike loads)
#
# Usage: ./flow.sh <design.v> <top> <pins> <build_dir> [nextpnr --vopt ...]
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
[[ $# -ge 4 ]] || { echo "usage: $0 <design.v> <top> <pins> <build_dir> [--vopt k=v ...]" >&2; exit 1; }
SRC="$(realpath "$1")"; TOP="$2"; PINS="$(realpath "$3")"; OUT="$(realpath -m "$4")"; shift 4
mkdir -p "$OUT"
[[ -f "$REPO/build/chipdb.json" ]] || python3 "$REPO/tools/gen_chipdb.py" -o "$REPO/build/chipdb.json"

podman run --rm --userns=keep-id -v "$HOME:$HOME" -w "$OUT" localhost/nextpnr-slg47910 bash -euo pipefail -c "
yosys -q -l yosys.log -p 'read_verilog $SRC; synth_xilinx -family xc7 -noiopad -noclkbuf -nobram -nodsp -nolutram -top $TOP; write_json netlist.json'
nextpnr-generic --uarch slg47910 --vopt chipdb=$REPO/build/chipdb.json \
    --vopt pin_constraints=$PINS --vopt sdc_out=placement.json --vopt route_out=routing.json \
    --json netlist.json $* > nextpnr.log 2>&1 || { tail -30 nextpnr.log; exit 1; }
python3 $REPO/tools/route_bits.py routing.json -o route.bin --lut-inputs lut_inputs.json
python3 $REPO/tools/emit_sdc.py placement.json -o placed.sdc --lut-inputs lut_inputs.json
python3 $REPO/tools/pack.py placed.sdc -o bitstream.bin --route-template route.bin --mcu bitstream_mcu.bin > pack.log
grep -E 'Unmapped|Coverage' pack.log
"
echo "$OUT/bitstream_mcu.bin"
