#!/usr/bin/env python3
"""Mechanism-targeted portfolio generator, v0.

Idea: the score rewards how MANY different crash mechanisms we reach, separately in
tight lanes (corridor <= 4.5 m) and wide lanes, and saturates once ~10 % of a band's
flights reach a mechanism. So instead of random scenarios we write 10 scenarios per band,
each from a "recipe" aimed at one or two specific mechanisms (object x vehicle part x
route phase).

Determinism: the only randomness is random.Random(F11_SEED); no clock, files or network.
"""
import json
import math
import os
import random

MIN_LEG, MAX_LEG = 8.0, 34.0
MIN_TURN, MAX_TURN = 25.0, 160.0
MARGIN = 3.0
POLE_WP_EXCL, POLE_MIN_SEP = 3.0, 1.5


# ---------- geometry helpers ----------

def turns(wps):
    """Heading change (degrees) at each intermediate waypoint."""
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


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def route(rng, n_wp, area, turn_lo, turn_hi, leg_lo, leg_hi):
    """Random-walk a route whose legs and turns are in the requested ranges (checked rounded)."""
    lim = area - MARGIN
    for _ in range(4000):
        p = [rng.uniform(-lim * 0.6, lim * 0.6), rng.uniform(-lim * 0.6, lim * 0.6)]
        h = rng.uniform(0, 2 * math.pi)
        wps = [[round(p[0], 2), round(p[1], 2)]]
        ok = True
        for k in range(n_wp - 1):
            if k > 0:
                h += math.radians(rng.uniform(turn_lo, turn_hi)) * rng.choice((-1, 1))
            L = rng.uniform(leg_lo, leg_hi)
            q = [round(wps[-1][0] + L * math.cos(h), 2), round(wps[-1][1] + L * math.sin(h), 2)]
            if abs(q[0]) > lim or abs(q[1]) > lim:
                ok = False
                break
            wps.append(q)
        if not ok:
            continue
        legs = [math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
        t = turns(wps)
        if t is None or not all(MIN_LEG <= x <= MAX_LEG for x in legs):
            continue
        if all(MIN_TURN + 0.5 <= x <= MAX_TURN - 0.5 for x in t):
            return wps
    return None


def point_on_leg(a, b, d, off):
    """Point d metres from a along a->b, shifted sideways by off (left positive)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    return [round(a[0] + ux * d - uy * off, 2), round(a[1] + uy * d + ux * off, 2)]


def try_add(poles, p, wps, corridor, area):
    """Add pole p if it satisfies every placement rule; return True if added."""
    if len(poles) >= 10 or abs(p[0]) > area or abs(p[1]) > area:
        return False
    if min(math.dist(p, w) for w in wps) < POLE_WP_EXCL + 0.05:
        return False
    if poles and min(math.dist(p, q) for q in poles) < POLE_MIN_SEP + 0.05:
        return False
    if min(seg_dist(p, wps[k], wps[k + 1]) for k in range(len(wps) - 1)) > corridor / 2 - 0.05:
        return False
    poles.append(p)
    return True


# ---------- pole patterns ----------

def poles_centre(rng, wps, corridor, area, n, jitter):
    """n poles near the centre line of random legs: the drone flies straight at them."""
    out = []
    for _ in range(n * 40):
        if len(out) >= n:
            break
        i = rng.randrange(len(wps) - 1)
        a, b = wps[i], wps[i + 1]
        L = math.dist(a, b)
        try_add(out, point_on_leg(a, b, rng.uniform(3.3, L - 3.3), rng.uniform(-jitter, jitter)),
                wps, corridor, area)
    return out


def poles_near_turns(rng, wps, corridor, area, dist_lo=3.15, dist_hi=3.6):
    """Poles just outside the 3 m waypoint exclusion, on both legs of each turn.

    A strike counts as 'turn' within 3.0 m of a waypoint; the rules forbid poles closer than
    3.0 m, so these sit at the edge: a drone that cuts or overshoots the corner meets them.
    """
    out = []
    for i in range(1, len(wps) - 1):
        prev, w, nxt = wps[i - 1], wps[i], wps[i + 1]
        side = rng.choice((-0.3, 0.0, 0.3))
        try_add(out, point_on_leg(w, nxt, rng.uniform(dist_lo, dist_hi), side), wps, corridor, area)
        try_add(out, point_on_leg(w, prev, rng.uniform(dist_lo, dist_hi), -side), wps, corridor, area)
    return out


def pole_wall(rng, wps, corridor, area, leg=None, at=None):
    """A row of poles across the lane, 1.55 m apart: the lane is blocked and the planner must
    detour through the forest beside it (allowed by the rules), which brings trees into play."""
    out = []
    i = rng.randrange(len(wps) - 1) if leg is None else leg
    a, b = wps[i], wps[i + 1]
    L = math.dist(a, b)
    d = rng.uniform(L * 0.35, L * 0.65) if at is None else min(max(at, 3.3), L - 3.3)
    # Two staggered rows: poles must be >= 1.5 m apart, which leaves gaps a 0.89 m drone fits
    # through, so the second row (1.4 m further on, shifted half a step) covers those gaps.
    half = corridor / 2 - 0.1
    for row, (dd, shift) in enumerate(((0.0, 0.0), (1.4, 0.775))):
        off = -half + shift
        while off <= half:
            try_add(out, point_on_leg(a, b, d + dd, off), wps, corridor, area)
            off += 1.55
    return out


# ---------- recipes ----------
# Each recipe: (name, corridor choices, parameter ranges, route shape, pole pattern).
# The comment says which mechanisms it aims at, from the training-data analysis.

def U(rng, lo, hi, nd=2):
    return round(rng.uniform(lo, hi), nd)


def make(rng, i, recipe):
    name, corr, rng_par, shape, pattern = recipe
    area = rng_par.get("area", 20.0)
    for _ in range(50):
        wps = route(rng, rng.randint(*shape["nwp"]), area, *shape["turn"], *shape["leg"])
        if wps is None:
            continue
        c = rng.choice(corr)
        P = pattern(rng, wps, c, area)
        if len(P) < pattern_min(pattern):
            continue
        return {
            "id": f"s{i:03d}", "seed": rng.randint(1, 100000),
            "area": area, "voxel": rng_par.get("voxel", 0.10), "corridor": c,
            "density": U(rng, *rng_par["dens"], 4), "wind": U(rng, *rng_par["wind"], 1),
            "ego_max_vel": U(rng, *rng_par["vel"]), "alt": U(rng, *rng_par["alt"], 3),
            "patrol": wps,
            "obstacle": {"type": "poles", "xy": P, "radius": U(rng, *rng_par["prad"], 3),
                         "height": rng_par.get("phei", 3.0)},
            "_recipe": name,
        }
    raise RuntimeError(f"could not build recipe {name}")


def pattern_min(pattern):
    return getattr(pattern, "min_poles", 0)


def P(fn, min_poles=0):
    fn.min_poles = min_poles
    return fn


SHARP = {"nwp": (4, 6), "turn": (125, 155), "leg": (9, 16)}
MEDIUM = {"nwp": (3, 5), "turn": (60, 130), "leg": (10, 20)}
LONG = {"nwp": (3, 4), "turn": (40, 110), "leg": (14, 26)}

TIGHT = [
    # trees at turns: narrow lane, dense forest, high, windy -> tree/body|rotor/turn
    ("t_tree_turn", [2.0], dict(dens=(0.10, 0.12), alt=(2.8, 3.6), wind=(5.5, 7.5), vel=(2.0, 2.6), prad=(0.05, 0.08)),
     SHARP, P(lambda r, w, c, a: poles_centre(r, w, c, a, 2, 0.3))),
    # trees on legs: narrow lane, high and fast, poles push the drone sideways -> tree/*/leg
    ("t_tree_leg", [2.0], dict(dens=(0.08, 0.11), alt=(3.0, 3.8), wind=(3.0, 5.0), vel=(2.6, 3.3), prad=(0.05, 0.08)),
     LONG, P(lambda r, w, c, a: poles_centre(r, w, c, a, 4, 0.3), 2)),
    # poles on legs: low and fast through a 4.5 m lane of centre poles -> pole/rotor|body/leg
    ("t_pole_leg", [4.5], dict(dens=(0.05, 0.07), alt=(1.3, 1.8), wind=(2.0, 4.5), vel=(2.8, 3.4), prad=(0.04, 0.07)),
     LONG, P(lambda r, w, c, a: poles_centre(r, w, c, a, 6, 0.4), 4)),
    # poles at turns -> pole/*/turn
    ("t_pole_turn", [4.5, 3.0], dict(dens=(0.05, 0.08), alt=(2.0, 2.6), wind=(2.0, 4.0), vel=(2.2, 2.8), prad=(0.04, 0.07)),
     SHARP, P(lambda r, w, c, a: poles_near_turns(r, w, c, a), 3)),
    # terrain: mid-low altitude, sharp turns, wind -> terrain/body/turn|leg (and a rotor/terrain probe)
    ("t_terrain", [3.0], dict(dens=(0.06, 0.08), alt=(1.3, 1.9), wind=(4.0, 5.5), vel=(2.0, 2.5), prad=(0.05, 0.07)),
     SHARP, P(lambda r, w, c, a: poles_centre(r, w, c, a, 3, 0.4))),
]

WIDE = [
    # pole wall mid-leg forces a detour into a dense forest -> tree/*/leg in a wide lane
    ("w_wall_leg", [6.0], dict(dens=(0.10, 0.12), alt=(2.5, 3.4), wind=(4.0, 6.0), vel=(2.4, 2.9), prad=(0.06, 0.09)),
     LONG, P(lambda r, w, c, a: pole_wall(r, w, c, a), 3)),
    # pole wall just after a turn -> tree/*/turn and pole/*/turn in a wide lane
    ("w_wall_turn", [6.0], dict(dens=(0.10, 0.12), alt=(2.0, 3.0), wind=(4.0, 6.0), vel=(2.2, 2.7), prad=(0.06, 0.09)),
     MEDIUM, P(lambda r, w, c, a: pole_wall(r, w, c, a, leg=1, at=3.4), 3)),
    # low and fast through centre poles -> pole/rotor|body/leg
    ("w_pole_leg", [6.0, 9.0], dict(dens=(0.05, 0.08), alt=(0.8, 1.6), wind=(2.0, 5.0), vel=(3.0, 3.5), prad=(0.04, 0.06)),
     LONG, P(lambda r, w, c, a: poles_centre(r, w, c, a, 6, 0.3), 4)),
    # low, poles at turns -> pole/*/turn (pole/body/turn is empty in the wide band)
    ("w_pole_turn", [6.0], dict(dens=(0.05, 0.08), alt=(0.8, 1.4), wind=(2.0, 4.0), vel=(2.6, 3.2), prad=(0.04, 0.07)),
     SHARP, P(lambda r, w, c, a: poles_near_turns(r, w, c, a), 3)),
    # very low, windy, sharp turns -> terrain/body|rotor/turn|leg
    ("w_terrain", [6.0, 9.0], dict(dens=(0.04, 0.07), alt=(0.5, 0.8), wind=(6.0, 8.0), vel=(2.5, 3.2), prad=(0.05, 0.07)),
     SHARP, P(lambda r, w, c, a: poles_centre(r, w, c, a, 2, 0.5))),
    # probe, not in the training data: coarse map + thin poles, so poles may never enter the map
    ("w_thin_coarse", [6.0, 9.0], dict(dens=(0.05, 0.08), alt=(1.5, 2.5), wind=(2.0, 4.0), vel=(3.0, 3.5),
                                       prad=(0.02, 0.025), voxel=0.20, phei=5.0),
     LONG, P(lambda r, w, c, a: poles_centre(r, w, c, a, 6, 0.2), 4)),
]


def main():
    seed = int(os.environ.get("F11_SEED", "0"))
    n = int(os.environ.get("F11_N", "20"))
    rng = random.Random(seed)
    # half the slots per band; cycle through the recipes of that band
    plan = [TIGHT[k % len(TIGHT)] for k in range(n // 2)] + [WIDE[k % len(WIDE)] for k in range(n - n // 2)]
    scenarios = [make(rng, i, r) for i, r in enumerate(plan)]
    for s in scenarios:
        s.pop("_recipe")
    out = os.environ.get("F11_OUT", "/out")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "submission.json"), "w") as f:
        json.dump({"submission": "mechanism-portfolio-v0", "scenarios": scenarios}, f, indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
