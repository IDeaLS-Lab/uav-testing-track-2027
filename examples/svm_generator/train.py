#!/usr/bin/env python3
"""Train a support vector classifier that predicts whether a scenario ends in a collision,
then save it. This runs ONCE, locally, before the image is built.

    python3 train.py ../../data/train/manifest.jsonl -o model.json

The container never runs this. It loads model.json, which is baked into the image at build
time, because evaluation gives the container no network and no data mount.

The model is written as plain JSON rather than a pickle so the image does not have to match
the scikit-learn version used for training, and so a reviewer can read what was submitted.
"""
import argparse
import json
import math

import numpy as np
from sklearn.svm import SVC
from sklearn.model_selection import cross_val_score, StratifiedKFold

# The scenario parameters a generator controls. Every field here is one a generator writes into
# the scenario, so the model maps decisions to outcomes rather than observations to outcomes.
FEATS = ["density", "wind", "ego_max_vel", "alt", "corridor",
         "n_poles", "pole_radius", "n_waypoints", "path_len", "min_pole_to_route"]


def seg_dist(p, a, b):
    """Distance from point p to segment ab, the quantity that separates these scenarios best."""
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def featurise(row):
    """One manifest row to one feature vector. Returns None if the row is unusable."""
    p = row.get("params") or {}
    wps = row.get("patrol") or []
    poles = ((row.get("poles") or {}).get("xy")) or []
    if len(wps) < 2:
        return None

    path_len = sum(math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1))
    if poles:
        near = min(min(seg_dist(q, wps[i], wps[i + 1]) for i in range(len(wps) - 1))
                   for q in poles)
    else:
        near = 20.0        # sentinel: no obstacle was placed near the route at all

    return [
        p.get("density", 0.0), p.get("wind", 0.0), p.get("ego_max_vel", 0.0),
        p.get("alt", 0.0), p.get("corridor", 0.0),
        float(len(poles)), (row.get("poles") or {}).get("radius", 0.0),
        float(len(wps)), path_len, near,
    ]


def load(path, split="train"):
    X, y = [], []
    for line in open(path):
        if not line.strip():
            continue
        r = json.loads(line)
        # Fit on the train split alone. The val split ships alongside it so a model can be
        # checked against scenarios it was not fitted on; pooling the two removes that check.
        if split != "all" and r.get("split") != split:
            continue
        # Only flights that actually produced evidence. A run that failed to collect tells
        # nothing about whether the scenario was hard.
        if not r.get("admissible") or r.get("outcome") not in ("goal", "collision"):
            continue
        f = featurise(r)
        if f is None:
            continue
        X.append(f)
        y.append(1 if r["outcome"] == "collision" else 0)
    return np.array(X, float), np.array(y, int)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest")
    ap.add_argument("-o", "--out", default="model.json")
    ap.add_argument("--split", default="train", choices=("train", "val", "all"),
                    help="which split to fit on; default train")
    a = ap.parse_args()

    X, y = load(a.manifest, a.split)
    print(f"{len(y)} usable flights, {y.sum()} collisions, {len(y) - y.sum()} goals")
    if len(y) < 40:
        raise SystemExit("not enough labelled flights to fit anything meaningful")

    # Standardise by hand so the container needs no sklearn to score: the scaler is three
    # lines of arithmetic and travels in the model file.
    mu, sd = X.mean(0), X.std(0)
    sd[sd < 1e-9] = 1.0
    Z = (X - mu) / sd

    svc = SVC(kernel="rbf", C=2.0, gamma="scale")
    cv = cross_val_score(svc, Z, y, cv=StratifiedKFold(5, shuffle=True, random_state=0),
                         scoring="roc_auc")
    print(f"5-fold ROC AUC {cv.mean():.3f} (sd {cv.std():.3f})")
    if cv.mean() < 0.55:
        print("warning: barely above chance. The scenario space may not separate this way.")

    svc.fit(Z, y)

    # Export the fitted decision function: sign(sum_i a_i K(sv_i, z) + b) with an RBF kernel.
    json.dump({
        "kind": "svc-rbf",
        "feats": FEATS,
        "mu": mu.tolist(), "sd": sd.tolist(),
        "gamma": float(1.0 / (X.shape[1] * Z.var())),
        "dual_coef": svc.dual_coef_[0].tolist(),
        "support_vectors": svc.support_vectors_.tolist(),
        "intercept": float(svc.intercept_[0]),
        "cv_auc": float(cv.mean()),
        "n_train": int(len(y)),
        "split": a.split,
    }, open(a.out, "w"))
    print(f"wrote {a.out} with {len(svc.support_vectors_)} support vectors")


if __name__ == "__main__":
    main()
