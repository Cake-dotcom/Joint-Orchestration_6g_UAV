# Data generation

No external dataset is required. The simulator generates synthetic user positions,
traffic arrivals, fading realizations and NDT prediction errors in memory using
NumPy's seeded random generator. The four algorithms receive the same exogenous
trace for each seed (common random numbers), enabling a fair paired comparison.

The seed index, complete simulation configuration, per-seed metrics and slot-level
metrics are saved under `results/` after each run. These are synthetic simulation
traces, not measurements from a real UAV deployment.
