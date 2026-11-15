#!/usr/bin/env python3
"""Run a design on the Shrike and check it against simulation, with no I/O.

    bist.py <design.v> <top> <build dir> [--cycles N] [--from N] [--bits HI:LO] [--no-hw] [--flow flow.sh]

The design is wrapped: an LFSR drives every input (a reset input is held
active for the first 16 cycles, and comes from a flip-flop), every output is folded into a 32-bit
signature (a MISR) each cycle (from cycle --from on, 0 by default: past a
reset, what a flip-flop starts at no longer counts), and after N cycles the
wrapper compares the
signature with the one Yosys's simulation of the same wrapper gives (`sim
-zinit`: every register starts at 0, as on the chip). The wrapper runs on
the 50 MHz oscillator and shows on the logic analyser:

    GPIO8  (D0)  a heartbeat, bit 20 of the cycle count
    GPIO10 (D2)  done: N cycles have run -- bit 18 of the count once they have
    GPIO13 (D5)  pass: the signature was the simulation's -- bit 19 of the count if so

Done and pass toggle rather than hold: a pad that is not driven reads its
pull-up, a constant 1.

Prints one line: PASS, FAIL (wrong signature), SLOW (wrong signature at 50
MHz, the simulation's with four cycles to a step: the design's paths are too
long for the oscillator, as built), DEAD (no heartbeat, never done, or a pad
that holds 1), UNDEFINED (the simulation leaves every output bit x at some time), or
BUILD-FAIL with the flow's last error. Output bits the simulation leaves x
at some time (Verilog lets the chip hold anything there: an index out of
range, a division by zero, a constant x) are not folded: `left=` says how
many. `fmax` is nextpnr's estimate after routing, in
MHz: a pessimistic one (designs it puts at 37 run at 50).
"""
import argparse, json, os, re, signal, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
IMAGE = "localhost/nextpnr-slg47910-hb"
CLOCKS = re.compile(r"^(i_)?(clk|clock|clk_i|clki)$", re.I)
RESETS = re.compile(r"^(i_)?(rst|reset|arst|srst|rst_i|reset_i)$", re.I)
RESETS_N = re.compile(r"^(i_)?(rst_?n|reset_?n|nrst|nreset|resetb|rstb)$", re.I)


def podman(cmd, cwd):
    return subprocess.run(["podman", "run", "--rm", "--userns=keep-id", "-v", f"{Path.home()}:{Path.home()}",
                           "-w", str(cwd), IMAGE, "bash", "-c", cmd], capture_output=True, text=True)


def ports(src, top, out):
    """The design's top (`-`: the one Yosys finds) and its ports."""
    pick = "-auto-top" if top == "-" else f"-top {top}"
    r = podman(f"yosys -q -p 'read_verilog -sv {src}; hierarchy {pick}; proc; write_json ports.json'", out)
    if r.returncode:
        raise RuntimeError("yosys: " + (r.stderr or r.stdout).strip().splitlines()[-1])
    mods = json.load(open(out / "ports.json"))["modules"]
    if top == "-":
        # Yosys 0.33's -auto-top marks no module that has no cells (a design of assigns only).
        tops = [n for n, m in mods.items() if m["attributes"].get("top")] or (list(mods) if len(mods) == 1 else [])
        if not tops:
            raise RuntimeError("no top: Yosys marks none of the modules")
        top = tops[0]
    plist = [(name, p["direction"], len(p["bits"])) for name, p in mods[top]["ports"].items()]
    if any(d == "inout" for _, d, _ in plist):
        raise RuntimeError("an inout port")
    if not any(d == "output" for _, d, _ in plist):
        raise RuntimeError("no output port: nothing to check")
    if any(not re.fullmatch(r"\w+", n) for n, _, _ in plist):
        raise RuntimeError("a port name the wrapper cannot write")
    return top, plist


def wrapper(top, plist, cycles, golden, start=0, only=None, slow=0, left=(), record=False):
    ins = [(n, w) for n, d, w in plist if d == "input" and not CLOCKS.match(n)]
    outs = [(n, w) for n, d, w in plist if d == "output"]
    clocks = [n for n, d, w in plist if d == "input" and CLOCKS.match(n)]
    # n counts cycles, as wide as the heartbeat and done need; early (the
    # first 15 cycles) and on (cycles --from to N) are flags that equality
    # compares set and clear: a comparison of n against a constant is a carry
    # chain, and a carry chain fills its site.
    width = max(21, (cycles + 1).bit_length())
    lines = [f"module bist_top(input wire clk, output wire [2:0] la, output wire clk_en);",
             "    reg [31:0] lfsr = 32'h1;", "    reg [31:0] misr = 0;", f"    reg [{width - 1}:0] n = 0;",
             f"    reg done = 0, pass = 0, early = 1, on = {int(start == 0)};"]
    conns, bit = [], 0
    for name, w in ins:
        bits = []
        for _ in range(w):
            bits.append(f"lfsr[{bit % 32}]")
            bit += 7
        value = "{" + ", ".join(reversed(bits)) + "}" if w > 1 else bits[0]
        if RESETS.match(name) or RESETS_N.match(name):
            # Held for the first 16 cycles, and from a flip-flop: an
            # asynchronous reset sees every glitch of a LUT, which no
            # simulation of cycles does.
            held = "1'b1" if RESETS.match(name) else "1'b0"
            lines.append(f"    reg [{w - 1}:0] in_{name} = {{{w}{{{held}}}}};")
            lines.append(f"    always @(posedge clk) in_{name} <= early ? {{{w}{{{held}}}}} : {value};")
        else:
            lines.append(f"    wire [{w - 1}:0] in_{name} = {value};")
        conns.append(f".{name}(in_{name})")
    for name, w in outs:
        lines.append(f"    wire [{w - 1}:0] out_{name};")
        conns.append(f".{name}(out_{name})")
    conns += [f".{c}(clk)" for c in clocks]
    lines.append(f"    {top} dut({', '.join(conns)});")
    total = sum(w for _, w in outs)
    flat = "{" + ", ".join(f"out_{n}" for n, _ in outs) + "}" if outs else "1'b0"
    if only:  # only these of the outputs, to find which a failure is in
        hi, lo = (int(b) for b in only.split(":"))
        flat, total = f"{flat} >> {lo}", hi - lo + 1
    # left: the bits the simulation leaves x (Verilog lets the chip hold
    # anything there), not folded. record (the simulation's wrapper): undef
    # holds x in every bit that ever was x (x | 0 is x).
    keep = "".join("0" if total - 1 - k in left else "1" for k in range(max(total, 1)))
    lines.append(f"    wire [{max(total, 1) - 1}:0] flat = ({flat}) & {max(total, 1)}'b{keep};")
    if record:
        lines.append(f"    (* keep *) reg [{max(total, 1) - 1}:0] undef = 0;")
    fold = " ^ ".join(f"{{{{{32 - min(32, total - k)}{{1'b0}}}}, flat[{min(total, k + 32) - 1}:{k}]}}"
                      if min(32, total - k) < 32 else f"flat[{k + 31}:{k}]" for k in range(0, max(total, 1), 32))
    # --slow K: the inputs step, and the signature takes the outputs, every 2^K
    # cycles only: a design with no flip-flops of its own then has that long.
    tick = f"(&n[{slow - 1}:0])" if slow else "1'b1"
    lines += [f"    wire [31:0] fold = {fold};",
              "    always @(posedge clk) begin",
              f"        if ({tick}) lfsr <= {{lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]}};",
              "        n <= n + 1'b1;",
              "        if (n == 14) early <= 1'b0;",
              f"        if (n == {start - 1} && !done) on <= 1'b1;" if start else "",
              f"        if (n == {cycles - 1}) on <= 1'b0;",
              f"        if ({tick} && on) misr <= {{misr[30:0], misr[31] ^ misr[21] ^ misr[1] ^ misr[0]}} ^ fold;",
              f"        if (n == {cycles}) begin done <= 1'b1; pass <= (misr == 32'h{golden:08x}); end",
              "    end"]
    if record:
        lines.append(f"    always @(posedge clk) if ({tick} && on) undef <= undef | (flat ^ flat);")
    lines += ["    assign la = {pass & n[19], done & n[18], n[20]};",
              "    assign clk_en = 1'b1;",
              "endmodule"]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("top"); ap.add_argument("out")
    ap.add_argument("--cycles", type=int, default=65536)
    ap.add_argument("--from", dest="start", type=int, default=0)
    ap.add_argument("--bits", help="fold only outputs HI:LO (of all outputs side by side, the last port lowest)")
    ap.add_argument("--slow", type=int, default=0, help="step the inputs and the signature every 2^K cycles")
    ap.add_argument("--no-hw", action="store_true")
    ap.add_argument("--flow", default=str(REPO / "himbaechel/flow.sh"))
    a = ap.parse_args()
    src, out = Path(a.src).resolve(), Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    try:
        top, plist = ports(src, a.top, out)
    except RuntimeError as e:
        print(f"BUILD-FAIL {e}"); return
    def simulate(left):
        """The signature and the output bits that were ever x, simulating the
        wrapper as the chip starts: every register 0, memories too (`memory`
        maps them onto flip-flops, as the flow does). async2sync: an
        asynchronous reset as a mux on the flip-flop's output, so that its
        release is seen at the edge after, as on the chip (sim lets a
        flip-flop see a reset that falls at the same edge)."""
        (out / "bist_sim.v").write_text(wrapper(top, plist, a.cycles, 0, a.start, a.bits, a.slow, left, True))
        # The vendor-neutral cells a design may instantiate (LUT5, FDRE, ...) as Yosys models them.
        r = podman(f"yosys -q -p 'read_verilog +/xilinx/cells_sim.v; read_verilog -sv {src} bist_sim.v; hierarchy -top bist_top; proc; flatten; memory; opt_clean; async2sync; "
                   f"sim -clock clk -zinit -n {a.cycles + 2} -w; write_json sim.json'", out)
        if r.returncode:
            raise RuntimeError("sim: " + (r.stderr or r.stdout).strip().splitlines()[-1])
        nets = json.load(open(out / "sim.json"))["modules"]["bist_top"]["netnames"]
        misr, undef = (nets[k]["attributes"].get("init") for k in ("misr", "undef"))
        if misr is None or undef is None:
            raise RuntimeError("sim: no final state for misr")
        return misr, {len(undef) - 1 - k for k, c in enumerate(undef) if c not in "01"}
    try:
        misr, left = simulate(set())
        if left:
            misr, _ = simulate(left)
    except RuntimeError as e:
        print(f"BUILD-FAIL {e}"); return
    bits = (int(a.bits.split(":")[0]) - int(a.bits.split(":")[1]) + 1 if a.bits
            else sum(w for _, d, w in plist if d == "output"))
    if len(left) == bits or not set(misr) <= set("01"):
        print("UNDEFINED the simulation leaves every output bit x at some time: nothing to compare with"); return
    golden = int(misr, 2)
    note = f" left={len(left)}/{bits}" if left else ""  # bits left out: x in simulation
    (out / "bist_top.v").write_text(open(src).read() + "\n" + wrapper(top, plist, a.cycles, golden, a.start, a.bits, a.slow, left))

    (out / "pins").write_text("set_clk clk W0\nset_io la[0] GPIO8\nset_io la[1] GPIO10\nset_io la[2] GPIO13\n"
                              "set_io clk_en 0 25\n")
    # The flow in a process group of its own, stopped by SIGTERM: that reaches
    # the container through podman (flow.sh runs it under an init), where a
    # SIGKILL would leave the container running.
    flow = subprocess.Popen([a.flow, str(out / "bist_top.v"), "bist_top", str(out / "pins"), str(out / "hb")],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        r = subprocess.CompletedProcess(flow.args, 0, *flow.communicate(timeout=900))
        r.returncode = flow.returncode
    except subprocess.TimeoutExpired:
        os.killpg(flow.pid, signal.SIGTERM)
        flow.communicate()
        print("BUILD-FAIL the flow took longer than 900 s"); return
    if r.returncode or not (out / "hb/bitstream_mcu.bin").exists():
        err = [l for l in (r.stdout + r.stderr).splitlines() if "ERROR" in l or "Error" in l] or (r.stdout + r.stderr).splitlines()[-1:]
        print(f"BUILD-FAIL {err[0].strip()[:200] if err else '?'}"); return
    luts = len(json.load(open(out / "hb/placement.json"))["luts"])
    fmax = re.findall(r"Max frequency for clock 'clk': ([\d.]+) MHz", open(out / "hb/nextpnr.log").read())[-1]
    if a.no_hw:
        print(f"BUILT luts={luts} golden={golden:08x} fmax={fmax}{note}"); return
    r = subprocess.run([sys.executable, str(REPO / "hw/hw.py"), "ab", str(out / "hb/bitstream_mcu.bin"),
                        "--rate", "1000000", "--ms", "200"], capture_output=True, text=True, cwd=REPO)
    final, edges = {"!": None, "#": None, "&": None}, {"!": 0, "#": 0, "&": 0}
    for line in open(out / "hb/bitstream_mcu.vcd"):
        if line.startswith("#"):
            for p in line[1:].split()[1:]:
                if p[1:] in final:
                    final[p[1:]] = int(p[0]); edges[p[1:]] += 1
    if edges["!"] < 4 or edges["#"] < 4 or (edges["&"] < 4 and final["&"] != 0):
        print(f"DEAD luts={luts} edges: heartbeat={edges['!']} done={edges['#']} pass={edges['&']} (pass pad at {final['&']})")
    elif edges["&"] >= 4:
        print(f"PASS luts={luts} golden={golden:08x} fmax={fmax}{note}")
    else:
        # A wrong signature is a wrong design or one too slow for the clock:
        # with four cycles to a step (--slow 2) only the second still fails.
        again = ""
        if not a.slow:
            r = subprocess.run([sys.executable, __file__, str(src), a.top, str(out / "slow"), "--cycles", str(a.cycles),
                                "--from", str(a.start), "--slow", "2", "--flow", a.flow]
                               + (["--bits", a.bits] if a.bits else []), capture_output=True, text=True)
            again = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "?"
        print(f"{'SLOW' if again.startswith('PASS') else 'FAIL'} luts={luts} golden={golden:08x} fmax={fmax}{note}"
              + (f" (four cycles to a step: {again})" if again else ""))


if __name__ == "__main__":
    main()
