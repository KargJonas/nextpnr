#!/usr/bin/env python3
"""Kraken's himbaechel export -> the SLG47910 database's timing.json.

    kraken_timing.py <export.ndjson> <timing.json>

The export (POST /v1/exports, format himbaechel, of a project whose builds
the vendor placer made) carries, where rule #33's claim stands, a
`speed_grade` per condition -- each one builds were timed under `timed`
-- and a `tile_timing` per grade and position: each routing wire's delay and
each pip's, [min, max] of rise and fall (timing.md §8); and rule #27's arcs
of each kind of unit as a `cell_timing` per grade.

timing.json is what slg47910_gen.py takes:

- `grades`: the conditions builds were timed under -- the routing is stated
  for every condition, the cells' arcs only for those -- the slowest first,
  the default: a bound -- the Shrike at room temperature runs a design in
  about three-quarters of what the slow condition times it at.
- `pips`: "x,y,from,to" -> a class, `classes`: class -> [min, max] per grade.
  A pip's delay is its own plus the wire's it drives: the vendor's model
  (a hop arrives at what it was entered from, plus the connection's delay
  and the resource's own), which himbaechel's charges once per pip. A class
  is the default grade's delay to the nearest 10 ps; its value under each
  grade the median of its pips'.
- `cells`: per bel type the arcs the uarch's bels take, made from the units'
  arcs as below. A unit's ports: inputs I(6s+k), k = 0..5, of slice s, I24
  the carry in; outputs O(3s) slice s's LUT, O(3s+1) its xMUX, O(3s+2) its
  flip-flop, O12 the carry out; F0 the clock; T(9s+3) the data pin of slice
  s's flip-flop as its LUT drives it -- the placer's arc to it is the LUT's
  delay and the flip-flop's setup together.
    - LUT: input k -> O, the least and greatest of I(6s+k) -> O(3s) over
      the slices (the delay to the unit's output pin: the internal paths
      are taken as long, which errs late).
    - CARRY: CI -> CO a quarter of I24 -> O12 (the chain through the four
      slices); P -> CO from I(0..3) -> O12 (slice A to the carry out),
      less slice A's LUT and three slices of chain; CI -> O from I24 ->
      O10 (the carry in to slice D's sum), less three slices of chain.
    - FF: CLK -> Q from F0 -> O(3s+2); setup of D from T(9s+3) less the LUT.
    - OFF (the second flip-flop, out by the xMUX): CLK -> Q from F0 ->
      O(3s+1).
  These are a first fit by hand of a bel model to the unit's arcs, not a
  rule's: they err late where the unit's arcs disagree.
- `outputs`: the way out of a logic unit -- from its output port to the
  first wire a net takes, the pin's delay and the wire's together -- per
  class `out:<layer>/<type>/o<port>><wire>`, [min, max] per grade (null
  where the grade has none): rule #34's, a `pip_class` of the export. The
  vendor charges it to the net, a flip-flop's about 1.4 ns past its clock to
  out, a LUT's about 0.4 ns; the units' arcs hold none of it.
"""
import collections
import json
import statistics
import sys


def main():
    export, out = sys.argv[1:]
    timed, tiles, cells, ways_out = set(), collections.defaultdict(dict), {}, collections.defaultdict(dict)
    for line in open(export):
        record = json.loads(line)
        kind = record["kind"]
        if kind == "speed_grade" and record.get("timed"):
            timed.add(record["name"])
        elif kind == "tile_timing":
            tiles[record["grade"]][(record["x"], record["y"])] = record
        elif kind == "cell_timing":
            cells.setdefault(record["grade"], []).append(record)
        elif kind == "pip_class" and record["name"].startswith("out:"):
            ways_out[record["name"]][record["grade"]] = record["delay"][2:4]  # the slow corner: a named grade's both
    grades = [grade for grade in timed if grade in cells]
    if not grades:
        sys.exit("the export says no condition builds were timed under with cells' arcs: rule #33's claim does not stand")

    def slowness(grade):
        return statistics.median(d[1] for record in list(tiles[grade].values())[::37] for _, _, d, _ in record["pips"])

    grades.sort(key=slowness, reverse=True)
    default = grades[0]

    # Each pip's delay under each grade, its wire's added.
    delay = {}
    for grade in grades:
        for (x, y), record in tiles[grade].items():
            wire = {name: d for name, d in record["wires"]}
            for src, dst, d, _inverts in record["pips"]:
                own = wire.get(dst) or [0, 0]
                delay.setdefault(f"{x},{y},{src},{dst}", {})[grade] = [d[0] + own[0], d[1] + own[1]]
    pips, members = {}, collections.defaultdict(list)
    for key, by in delay.items():
        name = f"T{round(by[default][1] / 10) * 10}"
        pips[key] = name
        members[name].append(by)
    classes = {
        name: [[round(statistics.median(m[g][0] for m in of)), round(statistics.median(m[g][1] for m in of))] for g in grades]
        for name, of in members.items()
    }
    outputs = {name: [by.get(grade) for grade in grades] for name, by in ways_out.items()}
    json.dump({"grades": grades, "pips": pips, "classes": classes, "outputs": outputs,
               "cells": {grade: bel_arcs(cells[grade]) for grade in grades}}, open(out, "w"))
    print(f"{len(grades)} grade(s), {len(pips)} pips in {len(classes)} classes, {len(outputs)} ways out of a unit -> {out}")


def bel_arcs(records):
    """The bels' arcs, from the logic units' (`cell_timing` of `<layer>/RBB_6L`)."""
    comb, outs = collections.defaultdict(list), collections.defaultdict(list)
    for record in records:
        if not record["name"].split(":")[0].endswith("/RBB_6L"):
            continue
        for arc in record["comb"]:
            comb[(arc["from"], arc["to"])].append(arc["delay"])
        for arc in record["clock_out"]:
            outs[arc["pin"]].append(arc["delay"])

    def span(delays):
        """[least, greatest] of the slow corner's [min, max] of each."""
        return [min(d[2] for d in delays), max(d[3] for d in delays)] if delays else None

    def arcs(pairs):
        return span([d for pair in pairs for d in comb.get(pair, [])])

    lut = {k: arcs([(f"I{6 * s + k}", f"O{3 * s}") for s in range(4)]) for k in range(6)}
    chain = span(comb[("I24", "O12")])
    step = [v / 4 for v in chain]
    a_to_co = arcs([(f"I{k}", "O12") for k in range(4)])
    p_to_co = [max(0, a_to_co[i] - lut[0][i] - 3 * step[i]) for i in range(2)]
    ci_to_o = [max(0, v - 3 * s) for v, s in zip(span(comb[("I24", "O10")]), step)]
    setup = arcs([(f"I{6 * s + k}", f"T{9 * s + 3}") for s in range(4) for k in range(4)])
    lut_most = max(lut[k][1] for k in range(4))
    return {
        "LUT": {f"I{k}": lut[k] for k in range(6) if lut[k]},
        "CARRY": {"CI_CO": [round(v) for v in step], "P_CO": sorted(round(v) for v in p_to_co), "CI_O": [round(v) for v in ci_to_o]},
        "FF": {"CLK_Q": span([d for s in range(4) for d in outs.get(f"O{3 * s + 2}", [])]),
               "SETUP": [max(0, setup[0] - lut_most), max(0, setup[1] - lut_most)]},
        "OFF": {"CLK_Q": span([d for s in range(4) for d in outs.get(f"O{3 * s + 1}", [])])},
    }


if __name__ == "__main__":
    main()
