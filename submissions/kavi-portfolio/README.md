# Mechanism-targeted portfolio generator (v0)

The rank key rewards breadth: 12 mechanism cells (object x vehicle part x route phase), scored
separately in tight (<= 4.5 m) and wide lanes, and saturating at about 10 % of a band's flights.
This generator therefore does not sample at random. It writes 10 scenarios per band, each built
from a recipe aimed at one or two specific cells.

**Recipes, tight lanes:** trees at turns (2 m lane, dense, high, windy) · trees on legs (2 m lane,
high, fast) · poles on legs (4.5 m lane, low, fast, centre-line poles) · poles at turns (poles just
outside the 3 m waypoint exclusion) · terrain (mid-low altitude, sharp turns, wind).

**Recipes, wide lanes:** pole wall mid-leg (two staggered rows across the lane, dense forest, so the
planner detours into the trees) · pole wall just after a turn · poles on legs (low, fast) · poles at
turns (low) · terrain (0.5 to 0.8 m altitude, strong wind, sharp turns) · thin poles with a coarse
map (voxel 0.20 m, pole radius 0.02 m), a probe of a region the released flights do not cover.

Recipe parameters come from an analysis of the 357 released flights: for each mechanism, the
parameter ranges of the flights that reached it, and the cells the wide band never reached.

Everything is drawn from `random.Random(F11_SEED)`; the same seed gives the same bytes.
