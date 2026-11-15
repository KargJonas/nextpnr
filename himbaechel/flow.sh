#!/bin/bash
# Verilog -> bitstream for the SLG47910 through nextpnr-himbaechel:
#
#   yosys (synth -lut 6, adders and comparators on CARRY4, flip-flops with a
#   synchronous or an asynchronous clear or set) -> nextpnr-himbaechel --uarch slg47910 (chipdb from the
#   vendor graph) -> hb_emit (names -> node ids) -> route_bits + emit_sdc + pack
#
# Usage: himbaechel/flow.sh <design.v> <top> <pins> <build_dir> [nextpnr args ...]
# The router does not always converge: a run that fails or takes longer than
# NEXTPNR_TIMEOUT (s, default 200) is tried again with the next of three seeds,
# and so is one whose routes the bitstream tools refuse (a pad's enable that
# would not hold: route_bits.py).
# nextpnr places and routes for FREQ MHz (50, the Shrike's oscillator) and
# goes on when it misses it: its estimate is pessimistic (nextpnr.log has it).
# The container runs under an init (--init), so a SIGTERM to this script's
# process group stops what runs inside it: as PID 1 nextpnr would ignore it.
# Needs `make himbaechel-image`, and build/chipdb.json for the node ids (tools/gen_chipdb.py).
set -euo pipefail

HB="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HB")"
[[ $# -ge 4 ]] || { echo "usage: $0 <design.v> <top> <pins> <build_dir> [nextpnr args ...]" >&2; exit 1; }
SRC="$(realpath "$1")"; TOP="$2"; PINS="$(realpath "$3")"; OUT="$(realpath -m "$4")"; shift 4
mkdir -p "$OUT"
[[ -f "$REPO/build/chipdb.json" ]] || python3 "$REPO/tools/gen_chipdb.py" -o "$REPO/build/chipdb.json"

podman run --rm --init --userns=keep-id -e NEXTPNR_TIMEOUT -e FREQ -e FORGEFPGA -v "$HOME:$HOME" -w "$OUT" localhost/nextpnr-slg47910-hb bash -euo pipefail -c "
yosys -q -l yosys.log -p 'read_verilog -lib -specify +/xilinx/cells_sim.v +/xilinx/cells_xtra.v; read_verilog -sv $SRC; synth -top $TOP -flatten -run :fine; techmap -map +/techmap.v -map +/xilinx/arith_map.v -D LUT_SIZE=6; synth -lut 6 -run fine:; dfflegalize -cell \$_SDFFE_PP0P_ 0 -cell \$_SDFFE_PP1P_ 1 -cell \$_DFFE_PP0P_ 0 -cell \$_DFFE_PP1P_ 1; opt_merge; abc -lut 6; opt_merge; techmap -D LUT_WIDTH=6 -map +/xilinx/lut_map.v -map +/xilinx/ff_map.v; clean; write_json netlist.json'
for seed in 1 2 3; do
  : > tools.log
  timeout \${NEXTPNR_TIMEOUT:-200} nextpnr-himbaechel --device slg47910 -o pins=$PINS -o placement=placement_raw.json \
    -o routing=routing_raw.json --json netlist.json --freq \${FREQ:-50} --timing-allow-fail --seed \$seed $* > nextpnr.log 2>&1 ||
    { [[ \$? == 124 ]] && echo \"ERROR: nextpnr did not finish in \${NEXTPNR_TIMEOUT:-200} s\" > tools.log; false; } &&
    python3 $HB/tools/hb_emit.py $REPO/build/chipdb.json placement_raw.json routing_raw.json placement.json routing.json 2> tools.log &&
    python3 $REPO/tools/route_bits.py routing.json -o route.bin --lut-inputs lut_inputs.json 2>> tools.log && break
  [[ \$seed == 3 ]] && { tail -30 nextpnr.log; cat tools.log; exit 1; }
done
python3 $REPO/tools/emit_sdc.py placement.json -o placed.sdc --lut-inputs lut_inputs.json
python3 $REPO/tools/pack.py placed.sdc -o bitstream.bin --route-template route.bin --mcu bitstream_mcu.bin > pack.log
grep -E 'Unmapped|Coverage' pack.log
"
echo "$OUT/bitstream_mcu.bin"
