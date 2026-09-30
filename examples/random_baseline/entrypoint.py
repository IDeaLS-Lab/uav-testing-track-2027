#!/usr/bin/env python3
"""Container entrypoint for the uniform-sampling baseline: the whole interface in one file.

Reads F11_SEED and F11_N from the environment, writes /out/submission.json, exits 0.
"""
import os
import sys
import random
import subprocess

seed = int(os.environ.get("F11_SEED", "0"))
n = int(os.environ.get("F11_N", "20"))
rc = subprocess.run([sys.executable, "/gen/random_gen.py", "--n", str(n),
                     "--seed", str(seed), "--name", "random-baseline",
                     "--out", "/out/submission.json"]).returncode
sys.exit(rc)
