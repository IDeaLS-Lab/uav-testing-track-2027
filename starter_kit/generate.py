#!/usr/bin/env python3
"""A working submission generator, deliberately simple, meant to be replaced.

This one draws uniformly from the registered space and does no selection at all, which is the
competition's own baseline. It exists so an entrant can build, run and submit something in five
minutes, then change one function.

The rule most often got wrong: the SAME SEED MUST PRODUCE THE SAME BYTES. Evaluation runs the
container twice on each seed and compares them. Every source of randomness must be seeded from
F11_SEED, and the clock, the filesystem and the network must not be read.
"""
import json
import math
import os
import random

# The registered space. These are the validator's bounds; a scenario outside them is rejected.
BOUNDS = {
    "density": (0.012, 0.120), "ego_max_vel": (1.2, 3.5), "wind": (0.0, 8.0),
    "alt": (0.5, 4.0), "pole_radius": (0.02, 0.10),
}
CORRIDORS = [2.0, 3.0, 4.5, 6.0, 9.0, 12.0]
AREA, VOXEL, POLE_HEIGHT = 20.0, 0.10, 3.0
MIN_LEG, MAX_LEG = 8.0, 34.0
MIN_TURN, MAX_TURN = 25.0, 160.0   # the turn at each intermediate waypoint, in degrees
MARGIN = 3.0                 # keep waypoints this far inside the field edge
POLE_WP_EXCL, POLE_MIN_SEP = 3.0, 1.5


def turns(wps):
    """The heading change at each intermediate waypoint, in degrees."""
    out = []
    for i in range(1, len(wps) - 1):
        ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
        bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-9 or nb < 1e-9:
            return None
        c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
        out.append(math.degrees(math.acos(c)))
    return out


def route(rng, n_wp):
    """A patrol whose legs AND turns are within bounds and which stays inside the field.

    The turn check is the one most likely to be missed. Legs are easy to think about; a route whose
    legs all fit can still double back at a waypoint, and the validator rejects the whole submission
    for it. Check turns on the ROUNDED coordinates, because the scenario is written rounded and a
    turn of 160.02 degrees becomes 160.0 or does not, depending on where the rounding lands.
    """
    lim = AREA - MARGIN
    for _ in range(2000):
        wps = [[round(rng.uniform(-lim, lim), 2), round(rng.uniform(-lim, lim), 2)]
               for _ in range(n_wp)]
        legs = [math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
        if not legs or not all(MIN_LEG <= L <= MAX_LEG for L in legs):
            continue
        t = turns(wps)
        if t is None:
            continue
        if all(MIN_TURN <= x <= MAX_TURN for x in t):
            return wps
    return None


def poles(rng, wps, n, lane):
    """n poles inside the cleared lane, clear of waypoints and of each other."""
    out = []
    for _ in range(n):
        for _ in range(500):
            i = rng.randrange(len(wps) - 1)
            a, b = wps[i], wps[i + 1]
            t = rng.uniform(0.15, 0.85)
            px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy) or 1.0
            off = rng.uniform(-lane, lane)
            p = [round(px - dy / L * off, 2), round(py + dx / L * off, 2)]
            if min(math.dist(p, w) for w in wps) < POLE_WP_EXCL:
                continue
            if out and min(math.dist(p, q) for q in out) < POLE_MIN_SEP:
                continue
            out.append(p)
            break
    return out


def scenario(rng, i):
    cw = rng.choice(CORRIDORS)
    wps = route(rng, rng.randint(2, 6))
    if wps is None:
        return None
    return {
        "id": f"s{i:03d}",
        "seed": rng.randint(1, 100000),
        "area": AREA, "voxel": VOXEL, "corridor": cw,
        "density": round(rng.uniform(*BOUNDS["density"]), 4),
        "wind": round(rng.uniform(*BOUNDS["wind"]), 2),
        "ego_max_vel": round(rng.uniform(*BOUNDS["ego_max_vel"]), 2),
        "alt": round(rng.uniform(*BOUNDS["alt"]), 3),
        "patrol": wps,
        "obstacle": {"type": "poles", "xy": poles(rng, wps, rng.randint(0, 6),
                                                  max(0.3, cw / 2 - 0.6)),
                     "radius": round(rng.uniform(*BOUNDS["pole_radius"]), 3),
                     "height": POLE_HEIGHT},
    }


def main():
    seed = int(os.environ.get("F11_SEED", "0"))
    n = int(os.environ.get("F11_N", "20"))
    rng = random.Random(seed)          # the ONLY source of randomness, seeded from the environment

    scenarios = []
    while len(scenarios) < n:
        s = scenario(rng, len(scenarios))
        if s is not None:
            scenarios.append(s)

    os.makedirs("/out", exist_ok=True)
    with open("/out/submission.json", "w") as f:
        json.dump({"submission": "starter_kit", "scenarios": scenarios}, f,
                  indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
