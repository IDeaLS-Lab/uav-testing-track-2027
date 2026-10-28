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

TIES ARE NOT BROKEN. An earlier version ordered ties by mech_rarity, then crash, then name. That was
withdrawn once the reference arms were rebuilt against this rank key: no pair of four separated, so
ties are the common case, and a tie-broken board would have ranked at one remove on quantities this
file declares unrankable. Rarity fails on its own terms as well, spreading nearly three times wider
than the rank key and ordering the same four arms differently.

Submissions inside the band therefore SHARE a placement. Order inside a shared placement is by
submission name and carries no claim: it exists so the output is reproducible.

Usage: f11_leaderboard_v1.py --score <metrics.json> [--score ...] --band <width> \
                             --out leaderboard.json
"""
import os
import re
import math, json, argparse

ACCEPTED_SCORER = "f11_score_v1/8"
SAFE = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")
RANK_KEY = "mech_rate"
# The narrowest per-pair band measured on the rebuilt reference arms. Publishing inside this width
# claims a separation no measurement on this bench supports.
# Per-pair 95% Welch t bands between the four sample tools, three evaluations each, on the scores
# of the single-scorer rescoring (team_eval/scoring_all, 26 Sep 2026).
BAND_FLOOR = 0.094
BAND_MEASURED_RANGE = (0.094, 0.146)
BAND_FLOOR_EVALUATIONS = 3   # independent evaluations the floor rests on
# The validator is the gate; validity is reported. A lost flight is excluded from coverage already.
MIN_VALIDITY = 0.0
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
    ap.add_argument("--band", type=float, required=True,
                    help="width inside which submissions share a placement, measured from repeated "
                         "evaluations of the reference arms. Required rather than defaulted: a "
                         "board with no band silently claims every difference is real. Measured "
                         "per-pair bands on this rank key run 0.094 to 0.146; passing the widest is "
                         "the conservative choice, since a narrower one asserts separations the "
                         "noisiest pair does not support.")
    ap.add_argument("--band-source", default=None,
                    help="where this band came from, recorded verbatim in the board. Required "
                         "whenever --band is below the measured floor, so a narrow band cannot be "
                         "passed without saying what measured it.")
    ap.add_argument("--override-band-floor", action="store_true",
                    help=f"publish with a band below the measured floor of {BAND_FLOOR}. Requires "
                         f"--band-source and --band-evaluations, and is recorded in the board.")
    ap.add_argument("--band-evaluations", type=int, default=None,
                    help=f"number of INDEPENDENT evaluations the band was measured from. Required "
                         f"when overriding the floor, and must exceed the "
                         f"{BAND_FLOOR_EVALUATIONS} the floor rests on: a narrower band is a claim "
                         f"of more precision, which only more replication can support.")
    ap.add_argument("--band-detail", default=None, metavar="PAIRS.json",
                    help="json file of per-pair band measurements, embedded verbatim in the board so "
                         "a placement can be audited against the pair that produced its width. The "
                         "rules publish uncertainty per pair; a single scalar cannot carry that.")
    ap.add_argument("--min-validity", type=float, default=MIN_VALIDITY,
                    help=f"minimum validity to be ranked, default {MIN_VALIDITY}. Validity is the "
                         f"fraction of submitted scenarios that passed the validator and flew, and "
                         f"the rules call it a gate. Recorded in the board.")
    ap.add_argument("--out", default="leaderboard.json")
    a = ap.parse_args()
    if not math.isfinite(a.band) or a.band < 0.0:
        raise SystemExit(f"--band {a.band} must be a finite non-negative width")
    # A zero band asserts that every difference, however small, is real. No number of evaluations
    # supports that, so it is refused unconditionally rather than left to the override.
    if a.band == 0.0:
        raise SystemExit("--band 0 declares infinite precision on a measured score. Every pair would "
                         "be ordered, including differences far inside the noise. There is no "
                         "override for this.")
    if not math.isfinite(a.min_validity) or not (0.0 <= a.min_validity <= 1.0):
        raise SystemExit(f"--min-validity {a.min_validity} must lie in [0, 1]")
    # A band narrower than anything measured turns noise into rank order, which is the failure this
    # whole section exists to prevent. Refuse it unless the operator both says what measured it AND
    # shows more replication than produced the floor. Recording the override is not a substitute for
    # constraining it.
    if a.band < BAND_FLOOR:
        if not a.override_band_floor or not (a.band_source or "").strip():
            raise SystemExit(
                f"--band {a.band} is below the measured floor of {BAND_FLOOR}, the narrowest per-pair "
                f"band on the rebuilt reference arms. A board published at this width would order "
                f"differences no measurement supports. To proceed, pass --override-band-floor and "
                f"--band-source describing the measurement that justifies it; both are recorded in "
                f"the board.")
        if a.band_evaluations is None or a.band_evaluations <= BAND_FLOOR_EVALUATIONS:
            raise SystemExit(
                f"--band {a.band} is below the floor, so --band-evaluations must be given and must "
                f"exceed the {BAND_FLOOR_EVALUATIONS} independent evaluations the floor rests on. "
                f"Got {a.band_evaluations!r}. A narrower band is a claim of greater precision, and "
                f"only more replication can support it.")
        # How much narrower is not free either. A standard error shrinks as 1/sqrt(N), so N
        # evaluations support a band no narrower than the floor scaled by sqrt(floor_N / N). Without
        # this, "count 4" would license any width at all, and the override would be a formality.
        allowed = BAND_FLOOR * math.sqrt(BAND_FLOOR_EVALUATIONS / a.band_evaluations)
        if a.band < allowed:
            raise SystemExit(
                f"--band {a.band} is narrower than {a.band_evaluations} evaluations support. A "
                f"standard error falls as 1/sqrt(N), so the floor of {BAND_FLOOR} measured on "
                f"{BAND_FLOOR_EVALUATIONS} evaluations scales to {allowed:.4f} at {a.band_evaluations}. "
                f"Publish at {allowed:.4f} or wider, or measure more evaluations.")
    band_detail = None
    if a.band_detail:
        try:
            band_detail = json.load(open(a.band_detail))
        except Exception as e:
            raise SystemExit(f"--band-detail {a.band_detail}: unreadable: {e}")
        # Embedding whatever json happened to be in the file would make the audit field decorative.
        # It must be pair -> width, and the published scalar must belong to the range it describes.
        if not isinstance(band_detail, dict) or not band_detail:
            raise SystemExit(f"--band-detail {a.band_detail}: must be a non-empty object mapping a "
                             f"pair name to its measured band width")
        for k, v in band_detail.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                    or v <= 0.0:
                raise SystemExit(f"--band-detail {a.band_detail}: pair {k!r} width {v!r} must be a "
                                 f"finite positive number")
        lo, hi = min(band_detail.values()), max(band_detail.values())
        if not (lo <= a.band <= hi):
            raise SystemExit(f"--band {a.band} lies outside the per-pair widths it claims to come "
                             f"from ({lo} to {hi}). The published band must be one the measurement "
                             f"supports.")

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
                        # `exact` decides order and placement; `total` is the display value. Grouping
                        # on the rounded number would let display precision move a submission across
                        # a band boundary.
                        "exact": m[RANK_KEY],
                        "total": round(m[RANK_KEY], 4),
                        "metrics": {k: round(m[k], 4) for k in sorted(m)
                                    if isinstance(m[k], (int, float)) and not isinstance(m[k], bool)},
                        "phenotypes": d.get("phenotypes", []),
                        "flights": d.get("flights"), "submitted": d.get("submitted")})
    # Sort on the rank key, then by name so a shared placement prints reproducibly. Name is NOT a
    # tiebreak in the ranking sense: it orders rows inside a placement and never splits one.
    entries.sort(key=lambda e: (-e["exact"], e["submission"]))
    # Grouping reads the band against the group leader and so is only correct on a descending list.
    # Assert it rather than assume it: a future edit to the sort would otherwise degrade the
    # placements silently instead of failing.
    for x, y in zip(entries, entries[1:]):
        if y["exact"] > x["exact"]:
            raise SystemExit(f"internal: board not sorted descending on {RANK_KEY} "
                             f"({x['submission']}={x['exact']} before {y['submission']}={y['exact']})")
    # Grow a placement group while the next submission is within the band OF THE GROUP LEADER.
    # "Within the band" is not transitive, so chaining off each neighbour would let a long ladder of
    # small steps collapse into one placement; measuring from the leader bounds every group's width
    # by the band itself.
    placement, lead = 0, None
    for i, e in enumerate(entries):
        if lead is None or (lead - e["exact"]) > a.band:
            placement, lead = i + 1, e["exact"]
        e["rank"] = placement
    groups = {}
    for e in entries:
        groups.setdefault(e["rank"], []).append(e["submission"])
    for e in entries:
        e["shares_placement_with"] = sorted(s for s in groups[e["rank"]] if s != e["submission"])
    # Now that placements are fixed, order rows INSIDE a shared placement by name. Leaving them in
    # coverage order would print an order the board explicitly declines to claim, which is what the
    # published contract says it does not do.
    entries.sort(key=lambda e: (e["rank"], e["submission"]))

    board = {"scorer": ACCEPTED_SCORER, "rank_key": RANK_KEY, "band": a.band,
             "band_source": a.band_source,
             "band_measured_range": list(BAND_MEASURED_RANGE),
             "band_floor": BAND_FLOOR,
             "band_floor_evaluations": BAND_FLOOR_EVALUATIONS,
             # On the ordinary path the band rests on the evaluations that produced the floor; record
             # that rather than null, so the field never reads as "unknown provenance".
             "band_evaluations": (a.band_evaluations if a.band_evaluations is not None
                                  else BAND_FLOOR_EVALUATIONS),
             "band_floor_overridden": bool(a.override_band_floor and a.band < BAND_FLOOR),
             "band_per_pair": band_detail,
             "min_validity": a.min_validity,
             "tiebreak": None,
             "note": "ranked on rate-weighted failure-mechanism coverage alone. Crash count and "
                     "coverage rank submissions in opposite order, so this choice selects the "
                     "winner; the track rewards finding different KINDS of failure. Every other "
                     "metric is reported, none is ranked on. Submissions within the band SHARE a "
                     "placement and are not ordered against each other: ties are not broken, "
                     "because the quantities available to break them are ones this board declines "
                     "to rank. Row order inside a shared placement is by name and is arbitrary. The "
                     "band is one conservative scalar; the measured per-pair bands span "
                     "band_measured_range and are published with the board.",
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
        # '=' marks a placement this submission shares with another, so a reader cannot mistake the
        # printed row order for a ranking.
        mark = "=" if e["shares_placement_with"] else " "
        print(f"{e['rank']:5d}{mark}{e['submission']:>12s}{e['total']:9.4f}{m['mech_rarity']:11.3f}"
              f"{m['crash']:8.3f}{m['severity']:10.3f}{str(e['flights']):>9s}")
    shared = sum(1 for e in entries if e["shares_placement_with"])
    print(f"\nband {a.band:.3f}; {shared} of {len(entries)} submissions share a placement, marked =")
    print(f"-> {a.out}   (ranked on coverage alone; crash and severity are reported, not ranked on)")


if __name__ == "__main__":
    main()
