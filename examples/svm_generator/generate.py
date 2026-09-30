#!/usr/bin/env python3
"""A generator that proposes scenarios the trained SVM expects to end in a collision.

This is what runs inside the container at evaluation time. It does not train, does not read
the training data, and does not touch the network. It loads model.json, which was embedded in
the image at build time, samples candidate scenarios from the registered space, and retains the
ones the model scores highest.

The interface, unchanged from the starter kit:
    in   F11_SEED, F11_N from the environment
    out  /out/submission.json
    exit 0

Reproducibility: every random draw comes from random.Random(F11_SEED). The model is a fixed
file, so the same seed gives the same bytes.
"""
import json
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = json.load(open(os.path.join(HERE, "model.json")))

BOUNDS = {
    "density": (0.012, 0.120), "ego_max_vel": (1.2, 3.5), "wind": (0.0, 8.0),
    "alt": (0.5, 4.0), "pole_radius": (0.02, 0.10),
}
CORRIDORS = [2.0, 3.0, 4.5, 6.0, 9.0, 12.0]
AREA, VOXEL, POLE_HEIGHT = 20.0, 0.10, 3.0
MIN_LEG, MAX_LEG = 8.0, 34.0
MIN_TURN, MAX_TURN = 25.0, 160.0
MARGIN = 3.0
POLE_WP_EXCL, POLE_MIN_SEP = 3.0, 1.5
OVERSAMPLE = 40      # candidates drawn per scenario kept


# --- the model, scored without sklearn ------------------------------------------------
def score(feats):
    """The SVC decision function: sum_i a_i exp(-gamma |sv_i - z|^2) + b. Higher means the
    model expects a collision."""
    z = [(f - m) / s for f, m, s in zip(feats, MODEL["mu"], MODEL["sd"])]
    g, total = MODEL["gamma"], 0.0
    for a, sv in zip(MODEL["dual_coef"], MODEL["support_vectors"]):
        d2 = 0.0
        for zi, vi in zip(z, sv):
            t = zi - vi
            d2 += t * t
        total += a * math.exp(-g * d2)
    return total + MODEL["intercept"]


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def featurise(s):
    """Same features as train.py, in the same order, read off a scenario we are about to
    emit rather than off a flown manifest row."""
    wps, poles = s["patrol"], s["obstacle"]["xy"]
    path_len = sum(math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1))
    near = (min(min(seg_dist(q, wps[i], wps[i + 1]) for i in range(len(wps) - 1))
                for q in poles) if poles else 20.0)
    return [s["density"], s["wind"], s["ego_max_vel"], s["alt"], s["corridor"],
            float(len(poles)), s["obstacle"]["radius"], float(len(wps)), path_len, near]


# --- sampling the registered space ----------------------------------------------------
def turns(wps):
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
    lim = AREA - MARGIN
    for _ in range(2000):
        wps = [[round(rng.uniform(-lim, lim), 2), round(rng.uniform(-lim, lim), 2)]
               for _ in range(n_wp)]
        legs = [math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
        if not legs or not all(MIN_LEG <= L <= MAX_LEG for L in legs):
            continue
        t = turns(wps)
        if t is not None and all(MIN_TURN <= x <= MAX_TURN for x in t):
            return wps
    return None


def poles(rng, wps, n, lane):
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


def candidate(rng, i):
    cw = rng.choice(CORRIDORS)
    wps = route(rng, rng.randint(2, 6))
    if wps is None:
        return None
    return {
        "id": f"s{i:03d}", "seed": rng.randint(1, 100000),
        "area": AREA, "voxel": VOXEL, "corridor": cw,
        "density": round(rng.uniform(*BOUNDS["density"]), 4),
        "wind": round(rng.uniform(*BOUNDS["wind"]), 2),
        "ego_max_vel": round(rng.uniform(*BOUNDS["ego_max_vel"]), 2),
        "alt": round(rng.uniform(*BOUNDS["alt"]), 3),
        "patrol": wps,
        "obstacle": {"type": "poles",
                     "xy": poles(rng, wps, rng.randint(0, 6), max(0.3, cw / 2 - 0.6)),
                     "radius": round(rng.uniform(*BOUNDS["pole_radius"]), 3),
                     "height": POLE_HEIGHT},
    }


def main():
    seed = int(os.environ.get("F11_SEED", "0"))
    n = int(os.environ.get("F11_N", "20"))
    rng = random.Random(seed)

    # Draw a pool, score it, keep the top n. Scoring the whole pool before selecting keeps
    # the choice deterministic in the seed: the pool is fixed, so the ranking is too.
    pool = []
    while len(pool) < n * OVERSAMPLE:
        c = candidate(rng, len(pool))
        if c is not None:
            pool.append(c)

    ranked = sorted(pool, key=lambda s: -score(featurise(s)))
    chosen = ranked[:n]
    for i, s in enumerate(chosen):     # renumber so ids are contiguous after selection
        s["id"] = f"s{i:03d}"

    os.makedirs("/out", exist_ok=True)
    with open("/out/submission.json", "w") as f:
        json.dump({"submission": "svm_generator", "scenarios": chosen}, f,
                  indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
