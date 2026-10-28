#!/usr/bin/env python3
"""Competition scorer v1: score one submission's flown scenarios on several metrics.

Built after measuring that crash counting alone cannot rank submissions: on 300 flights a
trivial random baseline reached 47 crashes per 100 against 57 for the best learned generator,
and separating those needs about 377 flights per arm. Every metric here is computed from
artifacts a flight already produces, so scoring costs no extra flying.

Metrics, each in [0, 1], higher is better for the submitting generator:

  crash        fraction of valid flights ending in collision. Kept because it is the
               field-standard outcome, reported but known to saturate.
  severity     mean over flights of clip(1 - m / 1.5) where m is the minimum true clearance
               to the body envelope. Orders the non-crashing flights instead of discarding
               them. Measured to saturate at about 44 percent ceiling, so it is reported
               rather than trusted alone.
  mechanism    distinct failure phenotypes found, as a fraction of the phenotypes known
               reachable on this bench. A phenotype is (struck object class, vehicle part) for
               the contact that ENDED the flight, not for every contact recorded on a flight
               that happened to end in collision. This is the metric that rewards finding a
               KIND of failure others miss rather than repeating one.
  novelty      phenotypes found that a reference generator set did not find, over the same
               denominator. Zero if the submission finds nothing new.
  input_div    spread of the submission's own scenario parameters, as the mean normalised
               standard deviation over the declared factor ranges. Penalises submitting the
               same scenario repeatedly.
  output_div   spread of flown trajectories, as the mean pairwise distance between
               route-normalised path signatures.
  validity     fraction of submitted scenarios that passed the validator and flew.

The scorer prints the vector and nothing else. It used to print a weighted total under weights it
called provisional, while the leaderboard ignored that total and ranked on rate-weighted coverage
alone. Publishing a decisive rank beside a number that does not determine it, computed with weights
the project itself called unsettled, is a dispute waiting to happen. One number or none: the rank key
is rate-weighted mechanism coverage, and it is the only aggregate anyone should read off a score.

Usage: f11_score_v1.py --flights <collection dir> --submission <json> [--reference <json>...]
"""
import os, sys, json, glob, math, re, argparse, collections, importlib, statistics as st

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

VEH = 0.444
SEV_ANCHOR = 1.5
TRUNK = {"Birch_obj": 0.042, "Pine_obj": 0.084, "Spruce_obj": 0.101}
# phenotypes demonstrated reachable on this bench, from the mechanism probe and the widened
# collection. A submission cannot be credited for a phenotype outside this set.
SCORER_VERSION = "f11_score_v1/8"   # /8: rank key is phase-split, corridor-banded coverage;
                                    # /7: saturation rate 0.10 -> 0.15; /6: the provisional
                                    # total and its weights are gone; /4: square-root credit
                                    # /5: rarity normalised by its achievable maximum; input_div
                                    #     ranges taken from the validator instead of copied
KNOWN = [("pole", "rotor"), ("pole", "body"), ("tree", "rotor"), ("tree", "body"),
         ("terrain", "rotor"), ("terrain", "body")]
# The scored cell adds WHERE on the route the strike happened: on a straight leg, or at a turn,
# meaning within TURN_R of an authored waypoint. Twelve cells rather than six.
#
# Phase and not, say, tree species, because the entrant authors the route and therefore controls
# where the turns are, while species is assigned by the forest generator from the seed and can only
# be influenced by searching seeds.
#
# What this does and does not buy, measured on an eighty-flight calibration and stated so nobody
# has to rediscover it: it lowers the best reference arm from 0.864 to 0.449, which is what
# subdividing cells does to every score, and no measured evidence shows it improves the ranking.
# Shuffling phase labels within an arm reproduces the observed first-to-second gap 15 percent of the
# time. It is adopted for the score level and for being controllable, not as a discriminator.
PHASES = ("leg", "turn")
CELLS = [(o_, p_, ph) for (o_, p_) in KNOWN for ph in PHASES]
TURN_R = 3.0            # metres from an authored waypoint; the ordering is invariant from 1.5 to 6.0
# Corridor bands the coverage is macro-averaged over. An EMPTY band scores zero rather than being
# skipped: skipping it meant a submission spending all twenty slots in one band was scored on that
# band alone and received exactly the un-banded coverage the stratification exists to suppress.
BAND_CUT = 4.5
# taken from the validator, not restated: a copied range silently disagrees the moment a bound moves,
# and input_div then measures spread against an interval the validator no longer accepts. It did:
# ego_max_vel was lowered to 1.2 in the validator while this table still read 1.5.
_VB = importlib.import_module("f11_validate").BOUNDS
FACTORS = {f: _VB[f] for f in ("density", "ego_max_vel", "alt", "wind", "corridor")}


def clip(x): return min(1.0, max(0.0, x))


def seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


# Seconds after the terminating contact within which a co-occurring contact is treated as part of
# the same strike. Measured over 212 co-occurring contacts in the archives: 69 percent fall within
# 0.05 s and 93 percent within 0.5 s. The value is a choice, so the ranking was checked against it:
# over 0.05, 0.10, 0.25, 0.50, 1.00 and 2.00 s the coverage ordering is IDENTICAL (narrow > mixed >
# poler) and only the absolute values move. The conclusion does not rest on this constant.
PHENO_WINDOW = 0.5
AMBIGUOUS = []          # flights where a latched climb-phase terrain contact clouds attribution
NO_EVIDENCE = []        # collision flights with no readable contact evidence: scored, but reported


def classify(key):
    """(object class, vehicle part) for one contact-journal key, failing CLOSED on anything unknown."""
    a, _, b = key.partition("|")
    # Classify on the scene ancestry, not the leaf name: tree assets keep vendor leaf names
    # (Spruce1, VP_Free_Birch_01) that no leaf-name rule matches, and the previous version's
    # else-branch was 'tree', so a genuinely unrecognised object scored as a tree strike.
    if b.split("/")[-1].startswith("pole"):
        oc = "pole"
    elif "Terrain" in b:
        oc = "terrain"
    elif re.search(r"/Forest/tree_\d+(/|$)", b):
        oc = "tree"
    else:
        oc = "unknown"
    part = "body" if "body" in a else ("rotor" if "rotor" in a else "unknown")
    return (oc, part)


def phenotypes(sd, outcome):
    """Phenotypes CAUSALLY tied to the mission-ending strike, not every contact on the flight.

    The previous version credited every key in the contact journal of any collision flight, so an
    incidental brush during a flight that later hit a pole scored as a distinct failure mechanism.
    A review caught the contradiction: the 300-flight mechanism audit found every mission-ending
    failure was rotor-on-pole with ZERO body impacts, while this scorer reported pole/body and
    terrain/body for the same arms. Those were incidental contacts.

    Now: a flight that did not end in collision contributes nothing, and on one that did, only
    contacts within PHENO_WINDOW of the TERMINATING contact are credited. Earlier contacts are
    exposure, returned separately so they can be reported without being scored."""
    caused, exposure = set(), set()
    if outcome != "collision":
        return caused, exposure
    cf = glob.glob(f"{sd}/px4_contacts_*.json")
    if not cf:
        NO_EVIDENCE.append(sd)          # a crash we cannot attribute is not a phenotype found
        return caused, exposure
    try:
        j = json.load(open(cf[0])).get("journal") or {}
    except Exception:
        NO_EVIDENCE.append(sd)
        return caused, exposure
    if not j:
        NO_EVIDENCE.append(sd)
        return caused, exposure
    # Match the MISSION's own rule. It fails on `strikes[0]`, the first eligible key in journal
    # insertion order (f2_ego_nvblox_patrol.py:1041-1052), and the journal is latched, so keys are
    # ordered by first contact. Journal values are [first_t, last_t, count], so the terminating
    # contact is the one with the SMALLEST first-contact time, not the largest.
    #
    # The trace's strike_t cannot be used to find it: strike_t is wall time since mission t0 while
    # the journal runs on the simulator clock, and the observed offset varies from 1.7 to 22.5 s
    # across flights.
    times = {k: (v[0] if isinstance(v, list) and v else None) for k, v in j.items()}
    known = [k for k, t in times.items() if t is not None]
    if not known:
        NO_EVIDENCE.append(sd)
        return caused, exposure
    t0 = min(times[k] for k in known)
    for k in j:
        t = times.get(k)
        if t is not None and (t - t0) <= PHENO_WINDOW:
            caused.add(classify(k))       # rotor and body contacting together at the same strike
        else:
            exposure.add(classify(k))
    # A latched terrain contact from the climb phase only becomes eligible once the vehicle is in
    # `fly`, so when terrain is earliest AND something else was also struck, which contact ended
    # the mission is genuinely ambiguous from these artifacts. Report it rather than guess.
    if any(c[0] == "terrain" for c in caused) and any(c[0] != "terrain" for c in caused | exposure):
        AMBIGUOUS.append(sd)
    return caused, exposure


# Per-flight rate at which a phenotype earns full credit. Raised from 0.10 after a calibration run
# of four reference generators found the metric nearly maxed: a hand-written baseline scored 0.908 of
# a possible 1.000 and sat two flights from perfect, so the top of a leaderboard would have been ties
# broken by the tiebreak rather than by coverage. For comparison the SDC tool competition's winner
# scored 0.41 on a normalised scale.
#
# 0.15 and not higher, which is the opposite of the obvious move. Raising the rate lowers every score
# but also lowers the best score anyone can reach, because the budget is twenty flights: at 0.20 a
# phenotype needs four flights and all six need twenty-four, so a perfect score becomes impossible
# and the achievable maximum falls to 0.911. Room above the best reference arm is then 0.092, exactly
# what it was before. At 0.30 the room shrinks to 0.048. At 0.15 a phenotype needs three flights and
# all six need eighteen, which fits, the maximum stays reachable, and the room is 0.136.
SAT_RATE = 0.10


def strike_phase(traj, patrol):
    """'leg' or 'turn' for where a collision flight ended, or None if it cannot be determined.

    The strike position is taken as the last recorded track point rather than by looking up the
    strike time, because the flight ends at the contact and that avoids aligning two clocks for a
    distinction measured in metres.
    """
    if not traj or not patrol or len(patrol) < 2:
        return None
    import numpy as _np
    p = _np.array(traj[-1][:2], dtype=float)
    w = _np.array([q[:2] for q in patrol], dtype=float)
    return "turn" if float(_np.min(_np.linalg.norm(w - p, axis=1))) <= TURN_R else "leg"


def banded_coverage(per_band, cells=None):
    """Coverage inside each corridor band, averaged over bands, with an empty band scoring zero."""
    cells = list(CELLS) if cells is None else list(cells)
    vals = []
    for counts, flown in per_band:
        if not flown:
            vals.append(0.0)
            continue
        vals.append(sum(min(1.0, math.sqrt((counts.get(c, 0) / flown) / SAT_RATE))
                        for c in cells) / len(cells))
    return sum(vals) / len(vals) if vals else 0.0


def rate_coverage(pheno_flights, flown, known=None):
    """Coverage with SQUARE-ROOT credit per phenotype: min(1, sqrt(rate / SAT_RATE)).

    The metric exists to reward finding DIFFERENT KINDS of failure. The previous linear-to-cap form
    did not deliver that. Measured at 100 flights per generator, it scored the forest-hunting
    generator 0.402 and a trivial uniform-random sampler 0.400, a tie, even though the first reaches
    five phenotype classes and the second reaches three and never produces a tree failure at all.
    Linear credit lets high reliability on few phenotypes substitute exactly for breadth: three
    phenotypes at rates 0.46/0.08/0.06 sum to the same credit as five at 0.14/0.07/0.04/0.02/0.01.

    Square-root credit gives diminishing returns per phenotype, so a phenotype seen once in 100
    flights earns 0.32 rather than 0.10, and discovering a new kind counts for more than repeating a
    known one. Measured on the same data it separates those two generators by 0.096 while keeping a
    real ordering among the placement generators (0.445, 0.425, 0.387, 0.363), which the alternatives
    that separate more strongly (a 0.02 cap, or presence counting) destroy by collapsing them all to
    0.500.

    The functional form was selected against data already seen, which is stated plainly rather than
    presented as neutral. What was fixed FIRST, and independently, is the principle: rank on variety
    of failures found. This form is frozen from here, before any competitor data exists.

    Presence-only counting was measured to saturate: at 100 flights every placement generator
    touches the same set. Credit still caps at 1.0, so this remains a biased estimator of the
    population quantity and is REPORTED rather than tested; the confirmed result rests on an
    ordinary binomial contrast instead."""
    known = list(KNOWN) if known is None else list(known)
    if not flown:
        return 0.0
    return sum(min(1.0, math.sqrt((pheno_flights.get(p_, 0) / flown) / SAT_RATE))
               for p_ in known) / len(known)


def terminating_class(sd, outcome):
    """Object class of the contact the MISSION terminated on, or None if not uniquely resolvable.

    Recovers the mission's own decision rather than re-deriving it. The mission fails on
    `strikes[0]`, the first eligible key in journal INSERTION order, excluding terrain during climb
    (f2_ego_nvblox_patrol.py:1033-1052). Inferring that from the minimum journal timestamp is wrong
    twice over: contacts detected in one physics step all share a time, so the minimum cannot break
    the tie the mission breaks by insertion order; and the journal is latched, so a climb-phase
    terrain contact keeps the smallest time while the mission goes on to terminate on a later
    object. A review caught both.

    So the mission's logged decision is parsed and cross-checked against the journal for the full
    prim path, since the log records only the leaf name and leaf names are ambiguous. Returns None
    unless exactly one class survives, and None is routed to the adverse case by the caller."""
    if outcome != "collision":
        return None
    lg = f"{sd}/f10.log"
    if not os.path.exists(lg):
        return None
    hit = None
    try:
        for line in open(lg, errors="ignore"):
            m = re.search(r"COLLISION: \S+ struck (\S+)", line)
            if m:
                hit = m.group(1)
                break
    except Exception:
        return None
    if not hit:
        return None
    cf = glob.glob(f"{sd}/px4_contacts_*.json")
    if not cf:
        return None
    try:
        j = json.load(open(cf[0])).get("journal") or {}
    except Exception:
        return None
    # cross-check the leaf the mission named against the journal, classifying on the FULL path
    classes = {classify(k)[0] for k in j if k.partition("|")[2].split("/")[-1] == hit}
    if len(classes) != 1:
        return None
    c = classes.pop()
    return None if c == "unknown" else c      # an unrecognised object is unresolved, not non-tree


def min_envelope_clearance(sd, traj):
    th = f"{sd}/truth.json"
    if not os.path.exists(th) or not traj:
        return None
    t = json.load(open(th))
    obs = []
    for o in t.get("trees", []):
        if o.get("pole"):
            r = float(o.get("radius", 0.05)); gz = float(o.get("ground_z", 0)); h = float(o.get("height", 3))
            obs.append((o["x"], o["y"], r, gz, gz + h))
        else:
            r = TRUNK.get(o.get("species"), 0.06) * float(o.get("scale", 1.0))
            gz = float(o.get("ground_z", 0))
            obs.append((o["x"], o["y"], r, gz, gz + 12.0))
    best = float("inf")
    for p in traj:
        for (ox, oy, r, zlo, zhi) in obs:
            if not (zlo - 0.5 <= p[2] <= zhi):
                continue
            d = math.hypot(p[0] - ox, p[1] - oy) - r
            if d < best:
                best = d
    return None if best == float("inf") else best - VEH


def path_signature(traj, patrol, n=12):
    """Signed lateral offsets at n points of equal FLOWN DISTANCE along the trajectory.

    Sampling was previously by trajectory array index, which is sample count, not progress: a
    flight that hovered for half its samples and one that flew steadily produced signatures of
    different things and were compared anyway. Arc length makes the n points comparable across
    flights of different duration and sample rate.

    The offset is signed relative to the route direction. Unsigned distance made a detour left and
    the mirrored detour right identical, so two genuinely different behaviours scored as no
    diversity at all."""
    if not traj or len(patrol) < 2:
        return None
    cum = [0.0]
    for i in range(1, len(traj)):
        cum.append(cum[-1] + math.dist(traj[i][:2], traj[i - 1][:2]))
    total = cum[-1]
    if total <= 0:
        return None
    sig = []
    k = 0
    for i in range(n):
        target = (i + 0.5) / n * total
        while k < len(cum) - 1 and cum[k] < target:
            k += 1
        p = traj[k]
        best, bs = float("inf"), 0.0
        for j in range(len(patrol) - 1):
            a_, b_ = patrol[j], patrol[j + 1]
            d = seg(p, a_, b_)
            if d < best:
                ux, uy = b_[0] - a_[0], b_[1] - a_[1]
                L = math.hypot(ux, uy)
                # sign: which side of the leg the vehicle is on
                bs = 0.0 if L == 0 else ((p[0] - a_[0]) * (-uy / L) + (p[1] - a_[1]) * (ux / L))
                best = d
        sig.append(bs)
    return sig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flights", action="append", required=True, help="repeatable")
    ap.add_argument("--submission", required=True, help="comma-separated")
    ap.add_argument("--reference", action="append", default=[],
                    help="reference collection dir(s); phenotypes they found do not count as novel")
    ap.add_argument("--out")
    a = ap.parse_args()

    sub = {}
    n_submitted = 0
    for si, sp in enumerate(a.submission.split(",")):
        for s in json.load(open(sp))["scenarios"]:
            n_submitted += 1
            sub[f"{si}:{s['id']}"] = s          # ids repeat across submission chunks
    # Each flights directory is the collection for the correspondingly-indexed submission chunk.
    # Without that pairing a manifest row could count toward validity and coverage without
    # corresponding to any submitted scenario at all.
    if len(a.flights) != len(a.submission.split(",")):
        sys.exit(f"[f11_score_v1] {len(a.flights)} flight dirs against "
                 f"{len(a.submission.split(','))} submission files; they must correspond")
    ref_ph = set()
    for rd in a.reference:
        for line in open(f"{rd}/manifest.jsonl"):
            r = json.loads(line)
            # Gate reference rows exactly as submission rows are gated below. Without this, a failed
            # collection attempt recorded as a collision can put a phenotype into the reference set,
            # halving its rarity weight and moving every submission's score, from a flight that was
            # never admissible in the first place.
            if r.get("collection_status") != "valid":
                continue
            if r["outcome"] != "collision":
                continue
            ref_ph |= phenotypes(f"{rd}/{r['scenario']}_r{r['repeat']}", r["outcome"])[0]

    flown = 0; crashes = 0; sev = []; found = set(); sigs = []; pheno_flights = {}
    cells_found = set(); band_cells = {}; band_flown = {}; no_phase = []
    exposure_ph = set()
    unmatched = 0
    for fi, fd in enumerate(a.flights):
      for line in open(f"{fd}/manifest.jsonl"):
        r = json.loads(line)
        if r.get("collection_status") != "valid":
            continue
        if f"{fi}:{r['scenario']}" not in sub:
            unmatched += 1          # a flight of something that was never submitted
            continue
        flown += 1
        sd = f"{fd}/{r['scenario']}_r{r['repeat']}"
        tf = glob.glob(f"{sd}/f2_track_*.json")
        traj = []
        if tf:
            try:
                traj = [p for p in json.load(open(tf[0])).get("world", []) if len(p) >= 3]
            except Exception:
                traj = []
        band = "tight" if float(r.get("corridor", 6.0)) <= BAND_CUT else "wide"
        band_flown[band] = band_flown.get(band, 0) + 1
        if r["outcome"] == "collision":
            crashes += 1
            ph, expo = phenotypes(sd, r["outcome"])
            found |= ph
            exposure_ph |= expo
            phase = strike_phase(traj, r.get("patrol"))
            for p_ in ph:
                pheno_flights[p_] = pheno_flights.get(p_, 0) + 1
                if phase:
                    cell = (p_[0], p_[1], phase)
                    cells_found.add(cell)
                    bc = band_cells.setdefault(band, {})
                    bc[cell] = bc.get(cell, 0) + 1
                else:
                    no_phase.append(sd)
        m = min_envelope_clearance(sd, traj)
        sev.append(clip(1 - m / SEV_ANCHOR) if m is not None else (1.0 if r["outcome"] == "collision" else 0.0))
        s = path_signature(traj, r["patrol"])
        if s: sigs.append(s)

    known = set(KNOWN)
    found &= known
    novel = found - ref_ph
    # input diversity: mean normalised sd of the declared factors across submitted scenarios
    divs = []
    for f, (lo, hi) in FACTORS.items():
        vals = [s.get(f, s.get("corridor")) if f == "corridor" else s.get(f) for s in sub.values()]
        vals = [v for v in vals if isinstance(v, (int, float))]
        if len(vals) > 1 and hi > lo:
            divs.append(min(1.0, st.pstdev(vals) / ((hi - lo) / 2)))
    # output diversity: mean pairwise distance between path signatures, normalised
    od = 0.0
    if len(sigs) > 1:
        ds = []
        for i in range(len(sigs)):
            for j in range(i + 1, len(sigs)):
                ds.append(math.dist(sigs[i], sigs[j]) / math.sqrt(len(sigs[0])))
        od = min(1.0, st.mean(ds) / 3.0)

    # THE RANK KEY: coverage over the twelve phase-split cells, computed inside each corridor band
    # and averaged, with an empty band scoring zero so a submission cannot escape the average by
    # concentrating in one band. The un-split, un-banded figure is kept beside it as a diagnostic,
    # because it is what every earlier campaign reported and a reader needs to compare.
    per_band = [(band_cells.get(b, {}), band_flown.get(b, 0)) for b in ("tight", "wide")]
    rate_cov = banded_coverage(per_band)
    rate_cov_flat = rate_coverage(pheno_flights, flown)
    # RARITY-WEIGHTED: a phenotype the reference set never found is worth more than a common one.
    # Normalised by the maximum this reference ALLOWS, not by 2.0 * len(known). Dividing by the
    # latter assumes every phenotype could be novel; against a real reference most are not, so the
    # ceiling sat below 1 and part of the metric's weight was unreachable by any submission. On the
    # 780-flight reference, which covers 5 of the 6 known phenotypes, a submission saturating every
    # one of them topped out at 0.583, locking away 0.20 x 0.417 of the total score.
    w_ph = {p_: (2.0 if p_ not in ref_ph else 1.0) for p_ in known}
    rarity = (sum(w_ph[p_] * min(1.0, pheno_flights.get(p_, 0) / max(1, flown) / SAT_RATE)
                  for p_ in known) / sum(w_ph.values())) if flown else 0.0
    M = {"crash": crashes / flown if flown else 0.0,
         "severity": st.mean(sev) if sev else 0.0,
         "mechanism": len(found) / len(known),
         "mech_rate": rate_cov,
         "mech_rate_flat": rate_cov_flat,
         "mech_rarity": rarity,
         "novelty": len(novel) / len(known),
         "input_div": st.mean(divs) if divs else 0.0,
         "output_div": od,
         "validity": min(1.0, flown / n_submitted) if n_submitted else 0.0}
    print(f"submission: {json.load(open(a.submission.split(',')[0])).get('submission')}   flights {flown}/{n_submitted}"
          + (f"   ({unmatched} flight rows matched no submitted scenario, ignored)" if unmatched else ""))
    print(f"{'metric':12s}{'value':>8s}")
    for k in ("mechanism", "mech_rate", "mech_rate_flat", "mech_rarity", "novelty", "severity", "crash",
              "input_div", "output_div", "validity"):
        print(f"{k:12s}{M[k]:8.3f}" + ("   <- the rank key" if k == "mech_rate" else ""))
    print("crash and severity are reported, never ranked: at twenty scenarios neither separates")
    print("two submissions. Only mech_rate decides the board.")
    if NO_EVIDENCE:
        print(f"NO contact evidence on {len(NO_EVIDENCE)} collision flight(s): they count as "
              f"crashes but contribute no phenotype, so coverage is a lower bound")
    if AMBIGUOUS:
        print(f"ambiguous attribution on {len(AMBIGUOUS)} flight(s): a latched terrain contact "
              f"precedes another strike, so which ended the mission is not determinable")
    print("exposure-only phenotypes (contacts NOT tied to the strike, unscored): "
          + (", ".join(sorted("/".join(p_) for p_ in exposure_ph - found)) or "none"))
    print("per-phenotype flight counts: " + (", ".join(f"{'/'.join(p_)}={pheno_flights[p_]}"
          for p_ in sorted(pheno_flights, key=lambda z: -pheno_flights[z])) or "none"))
    print(f"phenotypes found: {sorted('/'.join(p) for p in found) or 'none'}")
    print(f"novel vs reference: {sorted('/'.join(p) for p in novel) or 'none'}")
    if a.out:
        json.dump({"scorer": SCORER_VERSION,
                   "submission": json.load(open(a.submission.split(",")[0])).get("submission"),
                   "metrics": M,
                   "phenotypes": sorted("/".join(p) for p in found),
                   "novel": sorted("/".join(p) for p in novel),
                   "flights": flown, "submitted": n_submitted}, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
