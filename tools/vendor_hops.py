#!/usr/bin/env python3
"""The kinds of switch-matrix hop the vendor's router takes, from its builds.

    vendor_hops.py <vendor build work dir>... > hops.json

Each work dir holds the vendor's PNR_ROUTE.log, its routes hop by hop. A kind
of hop is the mux it drives, the mux it comes from (each as level, direction,
branch, index) and how far apart their tiles are. Writes, sorted, every kind
seen with how often: [mux, source, dx, dy, count]. The himbaechel chipdb
offers the router only these (himbaechel/uarch/slg47910/gen/slg47910_gen.py).
"""
import collections, json, re, sys

LINE = re.compile(r"net (\d+): \((\d+) (\d+)\)->\((\d+) (\d+)\) (\d+) (\d+) (\d+) (\d+)")


def main():
    seen = collections.Counter()
    for work in sys.argv[1:]:
        try:
            log = open(f"{work}/PNR_ROUTE.log").read()
        except OSError:
            continue
        prev, net = {}, None
        for m in LINE.finditer(log):
            n, sx, sy, dx, dy, level, branch, index, down = map(int, m.groups())
            if n != net:
                prev, net = {}, n
            key = (level, "D" if down else "U", branch, index)
            src = prev.get((sx, sy))
            if src:
                seen[(key, src[0], sx - src[1][0], sy - src[1][1])] += 1
            prev[(dx, dy)] = (key, (sx, sy))
    json.dump(sorted([list(k), list(s), ox, oy, n] for (k, s, ox, oy), n in seen.items()), sys.stdout)


if __name__ == "__main__":
    main()
