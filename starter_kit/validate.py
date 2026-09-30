#!/usr/bin/env python3
"""F11 submission validator, patrol schema. A competition submission is UNTRUSTED third-party JSON, so
it is validated against the registered scenario space before anything runs: every field must
be present, of the right type, finite, within bounds, the submission name and scenario ids must be safe
path tokens and unique, the scenario count must be within the cap, and each scenario's route and poles
must satisfy the same guards the dataset generator enforces by construction (leg lengths, interior turn
range, forest-edge margin, in-lane poles, waypoint exclusion, pole separation). Route-guard constants
mirror scripts/f11_dataset_gen.py. Exits non-zero and prints the first violation on any failure, so the
harness can fail closed. numpy-free."""
import re, hashlib
import sys
import json
import math
import argparse

CAP = 20
SAFE = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")   # \Z, not $: $ matches before a trailing newline, \Z does not
# density is trees per square metre over the (2*area)^2 field. The upper bound was 0.030, which is
# 48 trees over 1600 m2, one per 33 m2: four to ten times sparser than a managed forest and sparse
# enough that a cleared lane through it is barely a constraint. The ceiling is set by the scene
# generator's minimum tree spacing, measured directly: at the old 3.4 m spacing a request for 160
# trees delivered only 99, so a higher bound would have silently disagreed with the realised scene.
# At 2.0 m spacing 160 place and at 1.8 m 240 place, so the ceiling moves with spacing. Only the
# UPPER bound changed: the lower bound stays at 0.012 because nothing justified excluding sparse
# scenes, and raising it would have pushed three generators in this repo out of the registered space
# and silently renormalised input_div for every historical submission.
#
# Deliverability at the top of the range is NOT guaranteed by the spacing alone. Trees are packed
# into the field MINUS the cleared route corridor, so a wider or longer route leaves less planting
# area while the requested count is unchanged. Corridor width and realised density are therefore
# coupled. Generators must attest delivery per scene rather than assume it.
BOUNDS = {
    "density": (0.012, 0.120), "ego_max_vel": (1.2, 3.5), "wind": (0.0, 8.0),
    "seed": (1, 100000), "area": (10.0, 40.0), "corridor": (2.0, 12.0), "alt": (0.5, 4.0),
    "voxel": (0.05, 0.20), "pole_radius": (0.02, 0.10), "pole_height": (1.0, 5.0),
}
MIN_WP, MAX_WP = 2, 6
MIN_LEG, MAX_LEG = 8.0, 34.0
MIN_DISTINCT_RECIPES = 5    # of twenty slots, ignoring the seed
MIN_TURN_DEG, MAX_TURN_DEG = 25.0, 160.0
MARGIN = 3.0
POLE_WP_EXCL = 3.0
POLE_MIN_SEP = 1.5
MAX_POLES = 10
FLIGHT_KEYS = {"id", "seed", "density", "area", "corridor", "ego_max_vel", "wind", "alt",
               "voxel", "patrol", "obstacle"}
# Recorded on the scenario but never read by the flight, so never hashed into its fingerprint.
METADATA_KEYS = {"stratum", "turn_total_deg", "note", "generator"}
SWEPT = 0.444          # X500 swept radius, measured from the airframe colliders
GAP_EPS = 0.05         # a configuration-space gap thinner than this is not a gap
TIGHT_GAP = 0.40       # legal but reported: less than this much free width for the point-robot


def isnum(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and x == x and abs(x) != float("inf")


def die(msg):
    print(f"[f11_validate] REJECT: {msg}")
    sys.exit(1)


def is_xy(p):
    return isinstance(p, list) and len(p) == 2 and all(isnum(c) for c in p)


def seg_dist(p, a, b):
    """Distance from point p to segment a-b."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def turn_deg(a, b, c):
    """Turn angle at b on the path a-b-c: 0 = straight through, 180 = full reversal."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = c[0] - b[0], c[1] - b[1]
    nu, nv = math.hypot(ux, uy), math.hypot(vx, vy)
    if nu == 0 or nv == 0:
        return 180.0
    cosv = max(-1.0, min(1.0, (ux * vx + uy * vy) / (nu * nv)))
    return math.degrees(math.acos(cosv))


def check_route(sid, wps, area):
    lim = area - MARGIN
    for j, w in enumerate(wps):
        if not is_xy(w):
            die(f"scenario {sid}: patrol waypoint {j} must be [x, y] with finite numbers")
        if abs(w[0]) > lim or abs(w[1]) > lim:
            die(f"scenario {sid}: waypoint {j} outside +-{lim} m (forest extent {area} minus the {MARGIN} m margin)")
    for j in range(len(wps) - 1):
        leg = math.hypot(wps[j + 1][0] - wps[j][0], wps[j + 1][1] - wps[j][1])
        if not (MIN_LEG <= leg <= MAX_LEG):
            die(f"scenario {sid}: leg {j} length {leg:.2f} m out of [{MIN_LEG}, {MAX_LEG}]")
    for j in range(1, len(wps) - 1):
        t = turn_deg(wps[j - 1], wps[j], wps[j + 1])
        if not (MIN_TURN_DEG <= t <= MAX_TURN_DEG):
            die(f"scenario {sid}: turn at waypoint {j} is {t:.1f} deg, out of [{MIN_TURN_DEG}, {MAX_TURN_DEG}]")


def check_poles(sid, xy, wps, area, corridor):
    for j, p in enumerate(xy):
        if not is_xy(p):
            die(f"scenario {sid}: pole {j} must be [x, y] with finite numbers")
        if abs(p[0]) > area or abs(p[1]) > area:
            die(f"scenario {sid}: pole {j} outside the +-{area} m forest extent")
        lane = min(seg_dist(p, wps[k], wps[k + 1]) for k in range(len(wps) - 1))
        if lane > corridor / 2:
            die(f"scenario {sid}: pole {j} is {lane:.2f} m from the route, outside the {corridor / 2:.1f} m lane")
        wp_d = min(math.hypot(p[0] - w[0], p[1] - w[1]) for w in wps)
        if wp_d < POLE_WP_EXCL:
            die(f"scenario {sid}: pole {j} is {wp_d:.2f} m from a waypoint, inside the {POLE_WP_EXCL} m exclusion")
        for k in range(j):
            sep = math.hypot(p[0] - xy[k][0], p[1] - xy[k][1])
            if sep < POLE_MIN_SEP:
                die(f"scenario {sid}: poles {k} and {j} are {sep:.2f} m apart, under the {POLE_MIN_SEP} m separation")


def route_stations(wps):
    """Arc-length parameterisation of the patrol polyline."""
    out, acc = [], 0.0
    for j in range(len(wps) - 1):
        a, b = wps[j], wps[j + 1]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        out.append((acc, a, b, L))
        acc += L
    return out, acc


def pole_frame(p, segs):
    """(along-track arc length, signed lateral offset) of a pole against the polyline."""
    best = None
    for s0, a, b, L in segs:
        if L <= 0:
            continue
        ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
        t = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy
        tc = max(0.0, min(L, t))
        lat = (p[0] - a[0]) * (-uy) + (p[1] - a[1]) * ux
        d = math.hypot(p[0] - (a[0] + ux * tc), p[1] - (a[1] + uy * tc))
        if best is None or d < best[0]:
            best = (d, s0 + tc, lat)
    return (best[1], best[2]) if best else (0.0, 0.0)


def check_navigable(sid, xy, wps, corridor, radius):
    """Report the tightest free lateral width the cleared lane offers, in configuration space.

    Obstacles are inflated by the swept radius and the lane deflated by it, so the vehicle is a
    point. This is REPORTED, not enforced. An earlier version rejected a fully blocked lane as
    impassable; the flown data refutes that premise. Of the two blocked scenarios in the
    180-flight space comparison one was completed successfully, because nothing constrains the
    trajectory to the lane and the planner can detour through the sparse forest beside it. Only
    a lane narrower than the vehicle is rejected, which is impossible by construction rather
    than by judgement."""
    if not xy:
        return None
    segs, total = route_stations(wps)
    half = corridor / 2.0 - SWEPT
    if half <= 0:
        die(f"scenario {sid}: corridor {corridor} m is narrower than the {2 * SWEPT:.2f} m vehicle")
    framed = [pole_frame(p, segs) for p in xy]
    reach = radius + SWEPT
    worst, worst_s = half * 2.0, 0.0
    step = 0.10
    n = int(total / step) + 1
    for i in range(n + 1):
        sta = min(total, i * step)
        blocked = []
        for (ps, lat) in framed:
            if abs(sta - ps) <= reach:
                # lateral half-width the pole occupies at this station
                dx = max(0.0, reach ** 2 - (sta - ps) ** 2) ** 0.5
                blocked.append((lat - dx, lat + dx))
        free = half * 2.0
        if blocked:
            blocked.sort()
            merged = [list(blocked[0])]
            for lo, hi in blocked[1:]:
                if lo <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], hi)
                else:
                    merged.append([lo, hi])
            edges, cur = [], -half
            for lo, hi in merged:
                if lo > cur:
                    edges.append(lo - cur)
                cur = max(cur, hi)
            if cur < half:
                edges.append(half - cur)
            free = max(edges) if edges else 0.0
        if free < worst:
            worst, worst_s = free, sta
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("submission")
    a = ap.parse_args()
    try:
        sub = json.load(open(a.submission))
    except Exception as e:
        die(f"submission is not valid json: {e}")
    if not isinstance(sub, dict) or not isinstance(sub.get("scenarios"), list):
        die("submission must be an object with a 'scenarios' list")
    name = sub.get("submission")
    if not isinstance(name, str) or not SAFE.match(name):
        die(f"submission name {name!r} must be a string token [A-Za-z0-9_-]{{1,64}}")
    scen = sub["scenarios"]
    if len(scen) != CAP:
        die(f"{len(scen)} scenarios; must be exactly {CAP}. The rules fix the budget at {CAP} "
            f"scenarios flown three times, and a short submission would be scored on fewer flights "
            f"than everyone else's while its per-cell rates were computed over a smaller denominator")
    ids = set()
    fingerprints = {}
    seedless = {}
    tight = []
    for k, s in enumerate(scen):
        if not isinstance(s, dict):
            die(f"scenario {k} is not an object")
        sid = s.get("id")
        if not isinstance(sid, str) or not SAFE.match(sid):
            die(f"scenario {k} id {sid!r} must be a string token [A-Za-z0-9_-]{{1,64}}")
        if sid in ids:
            die(f"duplicate scenario id {sid!r}")
        ids.add(sid)
        if not isinstance(s.get("seed"), int) or isinstance(s.get("seed"), bool):
            die(f"scenario {sid}: seed must be an integer")
        for f in ("seed", "density", "area", "corridor", "ego_max_vel", "wind", "alt", "voxel"):
            if not isnum(s.get(f)):
                die(f"scenario {sid}: field {f} missing or not a finite number")
            lo, hi = BOUNDS[f]
            if not (lo <= s[f] <= hi):
                die(f"scenario {sid}: {f}={s[f]} out of [{lo}, {hi}]")
        wps = s.get("patrol")
        if not isinstance(wps, list) or not (MIN_WP <= len(wps) <= MAX_WP):
            die(f"scenario {sid}: patrol must be a list of {MIN_WP}..{MAX_WP} waypoints")
        check_route(sid, wps, s["area"])
        o = s.get("obstacle")
        if not isinstance(o, dict) or o.get("type") != "poles":
            die(f"scenario {sid}: obstacle must be {{type: poles, ...}}")
        for f in ("pole_radius", "pole_height"):
            key = "radius" if f == "pole_radius" else "height"
            if not isnum(o.get(key)):
                die(f"scenario {sid}: obstacle {key} missing or not finite")
            lo, hi = BOUNDS[f]
            if not (lo <= o[key] <= hi):
                die(f"scenario {sid}: obstacle {key}={o[key]} out of [{lo}, {hi}]")
        xy = o.get("xy")
        if not isinstance(xy, list) or len(xy) > MAX_POLES:
            die(f"scenario {sid}: obstacle xy must be a list of 0..{MAX_POLES} pole positions")
        check_poles(sid, xy, wps, s["area"], s["corridor"])
        gap = check_navigable(sid, xy, wps, s["corridor"], o["radius"])
        if gap is not None and gap < TIGHT_GAP:
            tight.append((sid, gap))
        # Unknown keys are rejected, and declared METADATA keys are allowed but excluded from the
        # fingerprint below. Hashing them would let a scenario padded with an ignored field hash
        # differently while flying identically; rejecting them outright would reject the provenance
        # fields the project's own generators legitimately carry.
        extra = sorted(set(s) - FLIGHT_KEYS - METADATA_KEYS)
        if extra:
            die(f"scenario {sid}: unknown field(s) {extra}; only {sorted(FLIGHT_KEYS)} affect the "
                f"flight and {sorted(METADATA_KEYS)} are permitted as provenance")
        oextra = sorted(set(o) - {"type", "radius", "height", "xy"})
        if oextra:
            die(f"scenario {sid}: unknown obstacle field(s) {oextra}")
        # Hash only what changes the flight, with pole order canonicalised since it does not.
        canon = {k_: v for k_, v in s.items() if k_ in FLIGHT_KEYS and k_ != "id"}
        canon["obstacle"] = dict(o, xy=sorted([list(map(float, q)) for q in xy]))
        # A distinct id over identical content is the same flight scored twice.
        fp = hashlib.sha256(json.dumps(canon, sort_keys=True,
                                       separators=(",", ":")).encode()).hexdigest()
        if fp in fingerprints:
            die(f"scenario {sid} is identical to {fingerprints[fp]!r} apart from its id; "
                f"duplicate scenarios inflate a submission without exploring anything")
        fingerprints[fp] = sid
        # The fingerprint above includes the seed, so twenty copies of one recipe under twenty seeds
        # pass it. A red-team pass found exactly that: the same route, corridor, density, wind, speed,
        # altitude and pole placement, twenty times, accepted. The forests differ because the seed
        # differs, so these are not identical scenes, but a generator that emits one recipe and
        # varies only the seed has not generated anything. The scoring already punishes it severely,
        # 0.042 against a reference 0.408, and a submission should still be told why rather than
        # discovering it from a low score.
        cs = {k_: v for k_, v in canon.items() if k_ != "seed"}
        fps = hashlib.sha256(json.dumps(cs, sort_keys=True,
                                        separators=(",", ":")).encode()).hexdigest()
        seedless.setdefault(fps, []).append(sid)
    # A submission must carry more than a handful of distinct recipes. The fingerprint above includes
    # the seed, so twenty copies of one recipe under twenty seeds pass it: a red-team pass found
    # exactly that. The forests differ because the seed differs, so these are not identical scenes,
    # but a generator that emits one recipe and varies only the seed has not generated anything. The
    # threshold is deliberately low, to reject that and not a generator exploring a narrow region.
    if len(seedless) < MIN_DISTINCT_RECIPES:
        big = max(seedless.values(), key=len)
        die(f"only {len(seedless)} distinct scenario recipe(s) across {len(scen)} slots once the "
            f"seed is ignored; {len(big)} of them are the same recipe ({big[0]} and "
            f"{len(big) - 1} others). At least {MIN_DISTINCT_RECIPES} distinct recipes are required")

    for sid, g in tight:
        w = "lane fully blocked, passable only by leaving it" if g < GAP_EPS else "tight"
        print(f"[f11_validate] note: scenario {sid} leaves {g:.2f} m of free lateral width ({w})")
    print(f"[f11_validate] OK: {name} with {len(scen)} scenarios within the registered space")


if __name__ == "__main__":
    main()
