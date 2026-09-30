#!/usr/bin/env python3
"""F11 random reference generator. Emits a competition SUBMISSION: a list of scenario specifications
sampled from a SUBSET of the registered scenario space (the space is the f11_validate BOUNDS; this
generator fixes area, corridor, voxel, alt, and pole_height, samples pole_radius and n_poles from ranges
narrower than the validator allows, and puts half its mass on zero wind). Random search is the baseline
every generation entrant must beat, so this is both the reference submission and the null the leaderboard
scores against. Deterministic given --seed. Pure JSON, no Isaac, laptop-runnable. Emits the patrol
schema the validator and harness consume; the legacy centreline `ys` form it used to emit no longer
passes the validator and was rejected by every current consumer."""
import json
import math
import random
import argparse

# this generator's sampling space, a subset of the registered space (the validator's BOUNDS); some fields
# are fixed and some ranges are narrower than the validator allows
SPACE = {
    "seed": (1, 100000),                # forest layout seed
    "density": (0.012, 0.030),          # trees per m^2
    "pole_radius": (0.03, 0.06),        # bare-pole trunk radius (m), around the F6 recall boundary
    "n_poles": (3, 7),                  # number of centreline pole hazards
    "ego_max_vel": (1.5, 3.5),          # cruise speed (m/s)
    "wind": (0.0, 8.0),                 # crosswind base speed (m/s); 0 means no wind
    "area": 20.0, "corridor": 6.0, "pole_height": 3.0, "voxel": 0.10, "alt": 1.6,
}
HALF = 12.0          # route runs y in [-HALF, HALF]: a 24 m leg, inside the validator's [8, 34]
WP_EXCL = 3.0        # poles must clear the waypoints by this much (validator POLE_WP_EXCL)
MIN_SEP = 1.5        # minimum pole-to-pole separation (validator POLE_MIN_SEP)


def sample_scenario(rng, i):
    n = rng.randint(*SPACE["n_poles"])
    lo, hi = -HALF + WP_EXCL, HALF - WP_EXCL
    lane = SPACE["corridor"] / 2.0
    xy = []
    for _ in range(400):                            # rejection-sample until the spacing rule holds
        if len(xy) >= n:
            break
        c = (round(rng.uniform(-lane, lane), 2), round(rng.uniform(lo, hi), 2))
        if all(math.dist(c, q) >= MIN_SEP for q in xy):
            xy.append(c)
    return {
        "id": f"s{i:03d}",
        "seed": rng.randint(*SPACE["seed"]),
        "area": SPACE["area"], "corridor": SPACE["corridor"],
        "density": round(rng.uniform(*SPACE["density"]), 4),
        "patrol": [[0.0, -HALF], [0.0, HALF]],
        "obstacle": {"type": "poles", "xy": [list(c) for c in xy],
                     "radius": round(rng.uniform(*SPACE["pole_radius"]), 3),
                     "height": SPACE["pole_height"]},
        "wind": round(rng.uniform(*SPACE["wind"]), 1) if rng.random() < 0.5 else 0.0,
        "voxel": SPACE["voxel"], "ego_max_vel": round(rng.uniform(*SPACE["ego_max_vel"]), 2),
        "alt": SPACE["alt"],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20, help="scenarios to emit (competition cap is 20)")
    ap.add_argument("--seed", type=int, default=0, help="RNG seed, for a reproducible submission")
    ap.add_argument("--name", default="random", help="submission name (leaderboard label)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    n = max(1, min(a.n, 20))                      # the competition cap is 20 scenarios
    rng = random.Random(a.seed)
    sub = {"submission": a.name, "generator": "random", "gen_seed": a.seed,
           "scenarios": [sample_scenario(rng, i) for i in range(n)]}
    json.dump(sub, open(a.out, "w"), indent=1)
    print(f"[f11] {a.name}: {n} scenarios -> {a.out}")


if __name__ == "__main__":
    main()
