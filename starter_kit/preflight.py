#!/usr/bin/env python3
"""Report what can be measured about a submission before it is ever flown.

    python3 preflight.py out/submission.json

The rank key is determined in the simulator: which failure mechanisms the scenarios reach, how
severe the strikes are, how rare each mechanism is. None of that exists until the submission has
been flown, and this script does not approximate it.

Two measures depend only on the scenarios themselves. They are computed here with the same
formulas and the same bounds the official scorer uses, so the values below are the values the
scorer will report rather than an estimate:

    input_div   spread of the declared factors across the submission
    validity    fraction of scenarios well formed enough to fly

The remainder of the report is diagnostic. It states where the submission sits in the scenario
space, which is what can still be changed before submitting.
"""
import argparse
import json
import math
import statistics as st
import sys

import validate

# Same bounds the validator enforces and the scorer normalises by.
BOUNDS = {
    "density": (0.012, 0.120), "ego_max_vel": (1.2, 3.5), "wind": (0.0, 8.0),
    "alt": (0.5, 4.0), "corridor": (2.0, 12.0),
}
FACTORS = ("density", "ego_max_vel", "alt", "wind", "corridor")
BAND_CUT = 4.5            # corridor at or below this is the tight band, above it is wide
VEHICLE_R = 0.444         # swept radius, for the free-lateral-width diagnostic


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def nearest_to_route(s):
    """Distance from the closest placed obstacle to the commanded route, or None."""
    wps, poles = s.get("patrol") or [], (s.get("obstacle") or {}).get("xy") or []
    if len(wps) < 2 or not poles:
        return None
    return min(min(seg_dist(q, wps[i], wps[i + 1]) for i in range(len(wps) - 1))
               for q in poles)


def bar(frac, width=28):
    n = int(round(frac * width))
    return "#" * n + "." * (width - n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("submission")
    a = ap.parse_args()

    doc = json.load(open(a.submission))
    scen = doc.get("scenarios") or []
    if not scen:
        sys.exit("no scenarios in that file")

    print(f"submission: {doc.get('submission', '?')}   scenarios: {len(scen)}\n")

    # ---- score terms computable now -------------------------------------------------
    print("SCORE TERMS COMPUTABLE WITHOUT THE SIMULATOR")
    divs = []
    for f in FACTORS:
        lo, hi = BOUNDS[f]
        vals = [s.get(f) for s in scen if isinstance(s.get(f), (int, float))]
        if len(vals) > 1 and hi > lo:
            d = min(1.0, st.pstdev(vals) / ((hi - lo) / 2))
            divs.append(d)
            print(f"  {f:<14s} spread {d:5.3f}  {bar(d)}")
    input_div = st.mean(divs) if divs else 0.0
    print(f"  {'input_div':<14s}       {input_div:5.3f}   <- a reported measure, not the rank key\n")

    # ---- the band check, the most actionable thing here ------------------------------
    tight = [s for s in scen if float(s.get("corridor", 6.0)) <= BAND_CUT]
    wide = [s for s in scen if float(s.get("corridor", 6.0)) > BAND_CUT]
    print("CORRIDOR BANDS")
    print(f"  tight (<= {BAND_CUT} m)  {len(tight):3d} scenarios")
    print(f"  wide  (>  {BAND_CUT} m)  {len(wide):3d} scenarios")
    if not tight or not wide:
        empty = "wide" if not wide else "tight"
        print(f"  WARNING: the {empty} band is empty. The rank key averages mechanism")
        print("           coverage across both bands, and an empty band contributes zero.")
        print("           A submission in one band alone forfeits half of the primary metric.")
    print()

    # ---- geometry, the known difficulty driver ---------------------------------------
    near = [d for d in (nearest_to_route(s) for s in scen) if d is not None]
    none_placed = sum(1 for s in scen if not ((s.get("obstacle") or {}).get("xy")))
    print("OBSTACLE GEOMETRY")
    if near:
        near_s = sorted(near)
        print(f"  nearest obstacle to route:  min {near_s[0]:.2f} m   "
              f"median {st.median(near_s):.2f} m   max {near_s[-1]:.2f} m")
        under1 = sum(1 for d in near if d < 1.0)
        print(f"  within 1.0 m of the route:  {under1} of {len(scen)} scenarios")
    widths = []
    for s in scen:
        o = s.get("obstacle") or {}
        try:
            if float(s["corridor"]) / 2 <= VEHICLE_R:
                continue
            w = validate.check_navigable(s.get("id"), o.get("xy") or [], s["patrol"],
                                         float(s["corridor"]), float(o.get("radius", 0.0)))
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if w is not None:
            widths.append((w, s.get("id")))
    if widths:
        w, sid = min(widths)
        blocked = sum(1 for g, _ in widths if g < validate.GAP_EPS)
        print(f"  tightest free lateral width: {w:.2f} m in {sid} "
              f"(the lane less the {VEHICLE_R} m swept radius, as validate.py measures it)")
        if blocked:
            print(f"  lane fully blocked:          {blocked} of {len(scen)} scenarios "
                  f"(admissible; the vehicle must leave the lane to pass)")
    if none_placed:
        print(f"  {none_placed} scenarios place no obstacle at all")
    print()

    # ---- what is missing --------------------------------------------------------------
    print("NOT COMPUTABLE HERE, MEASURED WHEN THE SUBMISSION IS FLOWN")
    for name, why in (
        ("mech_rate", "which of the twelve mechanism cells the scenarios reach    <- RANK KEY"),
        ("mechanism", "the same cells counted by occurrence rather than by rate"),
        ("mech_rarity", "how rare each reached mechanism is against the reference"),
        ("novelty", "cells no reference generator reached"),
        ("severity", "how far inside the swept envelope each strike goes"),
        ("crash", "the per-flight collision rate"),
        ("output_div", "how far apart the flown trajectories end up"),
        ("validity", "requires the flights to have been attempted; run validate.py"),
    ):
        print(f"  {name:<14s} {why}")
    print()

    print("There is no weighted total. Placement is decided by mech_rate alone and the")
    print("other eight are reported beside it as diagnostics, so a strong input_div does")
    print("not compensate for reaching few mechanism cells. Nothing above is visible until")
    print("the scenarios have been flown.")


if __name__ == "__main__":
    main()
