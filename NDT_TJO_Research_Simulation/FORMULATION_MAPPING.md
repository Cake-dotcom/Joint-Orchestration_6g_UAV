# Mapping from the supplied formulation to the runnable prototype

This note explains what the code implements and where it deliberately simplifies
the mathematical formulation in `joint_orchestration.pdf`.

| Formulation component | Code location | Implementation note |
|---|---|---|
| UAV-user geometry and path gain | `algorithms.py::_path_gain` | 3D distance, free-space path loss at 3.5 GHz and Rayleigh fading |
| SINR and achievable rate | `algorithms.py::run_algorithm` (radio scheduling loop) | Per-RB SINR includes reuse interference from other UAVs |
| User association | `algorithms.py::run_algorithm` (association block) | Greedy channel-aware, load-penalized association |
| Admission | `algorithms.py::run_algorithm` (admission block) | CRA deterministic check; TTJO/NDT-O nominal NDT check; NDT-TJO adds a residual margin |
| Queue evolution, Eq. (6) | `algorithms.py::run_algorithm` (queue update) | `Q[t+1] = max(0, Q[t] + admitted_arrivals - service_bits)` |
| Computing constraint, Eq. (11) | `algorithms.py::run_algorithm` (CPU allocation) | Per-UAV CPU is divided among active associated users and cannot exceed the capacity |
| AI model residency, Eqs. (16)-(19) | `_select_models`, `run_algorithm` | Model cache capacity and minimum quality are checked; real KV-cache growth is abstracted |
| UAV mobility, Eqs. (29)-(34) | `update_slow_configuration` | Waypoint movement is bounded per management frame; battery is tracked |
| NDT fidelity and robust screening, Eqs. (37)-(39) | `_fidelity_choice`, `_calibration_margins`, `run_algorithm` | Three synthetic fidelity levels and empirical one-sided Gaussian-residual quantiles |
| Original long-term objective, Eq. (44) | `run_algorithm` objective calculation | Uses dimensionless implementation scaling; not directly comparable to physical joules without a declared normalization |
| Lyapunov drift-plus-penalty, Eqs. (59)-(65) | RB priority and objective terms | Queue/utility weighting is DPP-inspired; it is not an exact numerical minimization of P2 |
| Slow-timescale objective, Eq. (78) | `update_slow_configuration` | Weighted user clustering plus coordinate-wise waypoint candidate refinement |
| CRA, TTJO, NDT-O and NDT-TJO | `ALGORITHMS`, `run_algorithm` | Shared simulator with algorithm-specific slow decisions and NDT screening |

## Please do not overstate the implementation

The paper formulation includes further requirements that this prototype does not
fully instantiate: Fisher-information-based ISAC sensing accuracy, detailed
propulsion power, packet-level UL/DL protocol delays, real accelerator memory and
KV-cache accounting, hardware-specific TTFT/TPOT, and a trained NDT. See
`ASSUMPTIONS_AND_SCOPE.md` for the full list. The prototype is suitable for a
clearly scoped synthetic algorithm comparison, not a claim that every equation in
the full formulation has been implemented at deployment fidelity.
