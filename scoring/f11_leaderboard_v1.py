#!/usr/bin/env python3
"""Competition leaderboard v1: rank submissions on the metrics the programme actually validated.

The previous leaderboard (scripts/f11_leaderboard.py) is SUPERSEDED and must not be used. It ranked
submissions by peak forward map optimism, which RQ2 established does not predict violations, so it
would have scored participants on a signal the programme disqualified. It also read the legacy
`obstacle.ys` field and raised KeyError on every scenario the current generators emit.

This one consumes the output of scripts/f11_score_v1.py (one metrics json per submission) and ranks
by the measured metrics:

  mech_rate    rate-weighted failure-mechanism coverage. PRIMARY. Credit scales with how reliably a
               submission produces each phenotype, so one lucky occurrence does not score like a
               reproducible capability. Presence-based counting was measured to saturate: three
               generators tied at 0.500 over 100 flights each.
  mech_rarity  same, doubled for phenotypes no other submission found. Only meaningful once there
               are several submissions to be rare against.
  severity     continuous body-envelope clearance. Reported; measured to saturate at 44 percent
               ceiling and to track crash count closely.
  crash        collision fraction. Reported, never decisive: separating two submissions on it needs
               about 377 flights each.
  input_div    spread of the submission's own scenario parameters. Cheap anti-collapse term.
  validity     fraction of submitted scenarios that passed the validator and flew. Reported, not a
               gate by default. The validator is the gate: it rejects a submission with any invalid
               scenario, and a rejected submission is never flown. Validity below 1 therefore comes
               only from flights that ended without a usable record; those flights are already
               excluded from coverage, and refusing the entry for them would penalise lost
               evaluations. --min-validity still sets an explicit gate when one is wanted.

RANKING, DECIDED: submissions are ranked on mech_rate alone. Crash count and coverage rank
generators in OPPOSITE order (measured over 180 flights), so the choice picks the winner rather
than breaking a tie, and the organisers chose to reward finding DIFFERENT kinds of failure over
producing the most crashes. A weighted blend was considered and rejected: blending two measures
that disagree hides which one selected the winner behind a set of weights nobody can defend.

Every other metric is REPORTED beside the rank, never folded into it.

PLACES: submissions are ordered by mech_rate, highest first. Submissions with equal mech_rate share
a place and the next place is skipped (1, 1, 3). Rows inside a shared place are ordered by name.

Usage: f11_leaderboard_v1.py --score <metrics.json> [--score ...] --out leaderboard.json
"""
import os
import re
import math, json, argparse

ACCEPTED_SCORER = "f11_score_v1/8"
SAFE = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")
RANK_KEY = "mech_rate"
# The validator is the gate; validity is reported. A lost flight is excluded from coverage already.
MIN_VALIDITY = 0.0
TIE_EPS = 1e-9
# Reported alongside the rank. Membership here is what the score file must contain; none of these
# contributes to the ranking.
REPORTED = ("mech_rate", "mech_rarity", "severity", "crash", "input_div", "output_div",
            "mechanism", "novelty", "validity")
# The metrics a score file must carry to be admitted. Named for what it is: nothing here is
# weighted, and nothing is summed. The rank key alone decides a placement.
REQUIRED_METRICS = sorted(set(REPORTED) | {RANK_KEY})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--score", action="append", required=True, help="f11_score_v1 output json")
    ap.add_argument("--min-validity", type=float, default=MIN_VALIDITY,
                    help=f"minimum validity to be ranked, default {MIN_VALIDITY}. Validity is the "
                         f"fraction of submitted scenarios that passed the validator and flew, and "
                         f"the rules call it a gate. Recorded in the board.")
    ap.add_argument("--out", default="leaderboard.json")
    a = ap.parse_args()
    if not math.isfinite(a.min_validity) or not (0.0 <= a.min_validity <= 1.0):
        raise SystemExit(f"--min-validity {a.min_validity} must lie in [0, 1]")
    entries = []
    seen = set()
    for p in a.score:
        try:
            d = json.load(open(p))
        except Exception as e:
            raise SystemExit(f"{p}: unreadable score file: {e}")
        if not isinstance(d, dict):
            raise SystemExit(f"{p}: score file is not an object")
        ver = d.get("scorer")
        if ver != ACCEPTED_SCORER:
            raise SystemExit(f"{p}: scorer version {ver!r}, expected {ACCEPTED_SCORER!r}; "
                             f"a board must not mix scorer versions")
        m = d.get("metrics") or {}
        missing = [k for k in REQUIRED_METRICS if k not in m]
        if missing:
            raise SystemExit(f"{p}: missing metrics {missing}; produced by an older scorer?")
        # Every metric is defined on [0, 1]. Anything else is a corrupt or crafted score file,
        # and silently ranking on it would put an arbitrary value at the top of the board.
        for k in sorted(m):
            v = m[k]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise SystemExit(f"{p}: metric {k}={v!r} is not a finite number")
            if not (0.0 <= v <= 1.0):
                raise SystemExit(f"{p}: metric {k}={v} outside [0, 1]")
        fl, sb = d.get("flights"), d.get("submitted")
        for nm, v in (("flights", fl), ("submitted", sb)):
            if isinstance(v, bool) or not isinstance(v, int) or v < 0:
                raise SystemExit(f"{p}: {nm}={v!r} must be a non-negative integer")
        if fl == 0 or sb == 0:
            raise SystemExit(f"{p}: flights={fl} submitted={sb}; an empty run is not a board entry")
        if fl > sb:
            raise SystemExit(f"{p}: {fl} flights against {sb} submitted scenarios; "
                             f"a submission cannot fly more than it submitted")
        # The rules call validity a gate, so enforce it here rather than assume the scorer did. Without
        # this an entry could carry a high coverage score off a handful of scenarios that flew while
        # the rest failed the validator, and be ranked against submissions that flew everything.
        # validity is the fraction of SUBMITTED scenarios that passed the validator and flew, so it
        # cannot exceed the fraction that flew at all. Checking that costs nothing and catches a
        # score file whose validity was not computed from the run it claims to describe.
        if m["validity"] > fl / sb + 1e-9:
            raise SystemExit(f"{p}: validity {m['validity']} exceeds the flown fraction {fl}/{sb} = "
                             f"{fl / sb:.4f}; validity counts scenarios that passed the validator "
                             f"AND flew, so it cannot be larger.")
        if m["validity"] < a.min_validity:
            raise SystemExit(f"{p}: validity {m['validity']} is below the gate of {a.min_validity} "
                             f"({fl} flights against {sb} submitted). Fix the submission, or lower "
                             f"--min-validity deliberately, which is recorded in the board.")
        # Identity comes from the score file the harness wrote, NOT from the filename, which an
        # entrant-controlled path could otherwise set.
        name = d.get("submission")
        if not isinstance(name, str) or not SAFE.match(name):
            raise SystemExit(f"{p}: submission name {name!r} must be a token [A-Za-z0-9_-]{{1,64}}")
        if name in seen:
            raise SystemExit(f"{p}: submission {name!r} appears twice; one entry per submission")
        seen.add(name)
        entries.append({"submission": name,
                        # `exact` decides order and place; `total` is the display value.
                        "exact": m[RANK_KEY],
                        "total": round(m[RANK_KEY], 4),
                        "metrics": {k: round(m[k], 4) for k in sorted(m)
                                    if isinstance(m[k], (int, float)) and not isinstance(m[k], bool)},
                        "phenotypes": d.get("phenotypes", []),
                        "flights": d.get("flights"), "submitted": d.get("submitted")})
    entries.sort(key=lambda e: (-e["exact"], e["submission"]))
    for i, e in enumerate(entries):
        prev = entries[i - 1] if i else None
        e["rank"] = prev["rank"] if prev and abs(prev["exact"] - e["exact"]) <= TIE_EPS else i + 1
    groups = {}
    for e in entries:
        groups.setdefault(e["rank"], []).append(e["submission"])
    for e in entries:
        e["shares_placement_with"] = sorted(s for s in groups[e["rank"]] if s != e["submission"])

    board = {"scorer": ACCEPTED_SCORER, "rank_key": RANK_KEY,
             "min_validity": a.min_validity,
             "tiebreak": None,
             "note": "ranked on rate-weighted failure-mechanism coverage alone, highest first. Crash "
                     "count and coverage rank submissions in opposite order, so this choice selects "
                     "the winner; the track rewards finding different KINDS of failure. Every other "
                     "metric is reported, none is ranked on. Equal coverage shares a place.",
             "entries": entries}
    # Write through a temporary file and rename: a failure partway through must not truncate the
    # board that is already published.
    tmp = f"{a.out}.tmp"
    with open(tmp, "w") as f:
        json.dump(board, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, a.out)

    print(f"{'place':>6s}{'submission':>12s}{'COVERAGE':>9s}{'rarity':>11s}{'crash':>8s}"
          f"{'severity':>10s}{'flights':>9s}")
    for e in entries:
        m = e["metrics"]
        mark = "=" if e["shares_placement_with"] else " "
        print(f"{e['rank']:5d}{mark}{e['submission']:>12s}{e['total']:9.4f}{m['mech_rarity']:11.3f}"
              f"{m['crash']:8.3f}{m['severity']:10.3f}{str(e['flights']):>9s}")
    shared = sum(1 for e in entries if e["shares_placement_with"])
    print(f"\n{shared} of {len(entries)} submissions share a place, marked =")
    print(f"-> {a.out}   (ranked on coverage alone; crash and severity are reported, not ranked on)")


if __name__ == "__main__":
    main()
