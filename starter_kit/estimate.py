#!/usr/bin/env python3
"""A rough estimate of a submission's rank key and crash rate from the released flights, without flying.

    python3 estimate.py out/submission.json
"""
import argparse
import json
import math
import os
import random
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "..", "data", "train", "manifest.jsonl")
BAND_CUT = 4.5
SAT_RATE = 0.10
REPEATS = 3
OBJECTS, PARTS, PHASES = ("pole", "tree", "terrain"), ("body", "rotor"), ("leg", "turn")
CELLS = [(o, p, ph) for o in OBJECTS for p in PARTS for ph in PHASES]
REFERENCE = "0.29 to 0.36"
ERROR_RANK = (0.07, 0.16)
ERROR_CRASH = (0.11, 0.19)


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def features(p, patrol, obstacle):
    poles = (obstacle or {}).get("xy") or []
    near = 3.0
    if poles and len(patrol) >= 2:
        near = min(3.0, min(min(seg_dist(q, patrol[i], patrol[i + 1]) for i in range(len(patrol) - 1))
                            for q in poles))
    turns = []
    for i in range(1, len(patrol) - 1):
        a = (patrol[i][0] - patrol[i - 1][0], patrol[i][1] - patrol[i - 1][1])
        b = (patrol[i + 1][0] - patrol[i][0], patrol[i + 1][1] - patrol[i][1])
        na, nb = math.hypot(*a), math.hypot(*b)
        if na > 0 and nb > 0:
            turns.append(math.degrees(math.acos(max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (na * nb))))))
    length = sum(math.dist(patrol[i], patrol[i + 1]) for i in range(len(patrol) - 1))
    return [float(p["corridor"]), float(p["density"]), float(p["wind"]), float(p["ego_max_vel"]),
            float(p["alt"]), float(len(poles)), near,
            float((obstacle or {}).get("radius") or 0.0) if poles else 0.0,
            float(len(patrol)), length, (sum(turns) / len(turns)) if turns else 0.0]


def load_flights():
    X, Y = [], []
    for line in open(MANIFEST):
        r = json.loads(line)
        if r.get("collection_status") != "valid" or not r.get("admissible"):
            continue
        cells = {tuple(c) for c in (r.get("mechanism") or [])}
        X.append(features(r["params"], r["patrol"], r["poles"]))
        Y.append((r["outcome"] == "collision", cells))
    cols = list(zip(*X))
    mu = [st.mean(c) for c in cols]
    sd = [st.pstdev(c) or 1.0 for c in cols]
    Z = [[(v - m) / s for v, m, s in zip(x, mu, sd)] for x in X]
    return Z, Y, mu, sd


def band_score(counts, flown):
    if not flown:
        return 0.0
    return sum(min(1.0, math.sqrt((counts.get(c, 0) / flown) / SAT_RATE)) for c in CELLS) / len(CELLS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("submission")
    ap.add_argument("--k", type=int, default=25, help="neighbours per scenario")
    ap.add_argument("--draws", type=int, default=400)
    a = ap.parse_args()
    scen = json.load(open(a.submission))["scenarios"]
    Z, Y, mu, sd = load_flights()
    prep = []
    for s in scen:
        z = [(v - m) / d for v, m, d in zip(features(s, s["patrol"], s.get("obstacle")), mu, sd)]
        order = sorted(range(len(Z)), key=lambda j: sum((zi - zj) ** 2 for zi, zj in zip(z, Z[j])))
        prep.append((order[:a.k], 0 if float(s["corridor"]) <= BAND_CUT else 1))
    rng = random.Random(0)
    ranks, crashes, reached = [], [], {}
    for _ in range(a.draws):
        counts, flown, crash = [{}, {}], [0, 0], 0
        for nb, band in prep:
            crashed, cells = Y[nb[rng.randrange(len(nb))]]
            for _rep in range(REPEATS):
                flown[band] += 1
                crash += crashed
                for c in cells:
                    counts[band][c] = counts[band].get(c, 0) + 1
        for c in set(counts[0]) | set(counts[1]):
            reached[c] = reached.get(c, 0) + 1
        ranks.append((band_score(counts[0], flown[0]) + band_score(counts[1], flown[1])) / 2)
        crashes.append(crash / max(1, sum(flown)))
    print(f"submission: {len(scen)} scenarios, estimated from {len(Y)} released flights\n")
    print(f"  estimated rank key    {st.mean(ranks):.3f}")
    print(f"  estimated crash rate  {st.mean(crashes):.2f}")
    print(f"  reference generators, flown: rank key {REFERENCE}\n")
    print("  mechanism cells reached in at least half of the draws:")
    for c in CELLS:
        if reached.get(c, 0) >= a.draws / 2:
            print(f"    {'/'.join(c)}")
    print("\nThis is a rough estimate. It borrows outcomes from similar released flights, so it cannot")
    print("see anything those flights do not contain. Checked against seven generators flown at the")
    print(f"full budget, the rank key estimate was off by {ERROR_RANK[0]:.2f} on average and by up to"
          f" {ERROR_RANK[1]:.2f},")
    print(f"and the crash rate by {ERROR_CRASH[0]:.2f} on average and by up to {ERROR_CRASH[1]:.2f}."
          " It separates large differences only.")
    print("The score that counts comes from the evaluation flights in Isaac Sim.")

if __name__ == "__main__":
    main()
