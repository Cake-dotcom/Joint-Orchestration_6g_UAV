# Implementation assumptions, scope and research limitations

## Why this file is included

The supplied mathematical formulation defines the joint-orchestration problem and
algorithm structure, but it explicitly leaves several numerical system parameters
and model functions to be instantiated before numerical experiments can be run.
This project therefore supplies a small, reproducible **synthetic simulation
instance**. Every added numeric value is an implementation assumption, not a
measurement from a real UAV testbed and not a value stated in the formulation PDF.

## What is represented

- Three UAVs, twenty-four ground users and twelve orthogonal resource blocks per UAV.
- Six service classes (IC, HRLLC, MC, UC, AIAC and ISAC).
- Per-slot synthetic traffic arrivals and slowly moving user locations.
- A 3.5-GHz free-space path-loss approximation with Rayleigh power fading.
- A simplified shared-resource SINR calculation with inter-UAV co-channel interference.
- Queue backlog, request admission, user association, RB assignment and transmit power.
- Per-UAV compute capacity and three AI model sizes/quality levels.
- UAV waypoint movement at management-frame boundaries.
- Three NDT fidelity levels, synthetic prediction residuals and a calibration-quantile margin.
- A simplified energy accounting model and battery-reserve tracking.

## Algorithm distinctions implemented

1. **CRA:** fixed waypoint positions, fixed model residency and fixed lowest NDT fidelity;
   it uses deterministic local latency feasibility but no NDT candidate screening.
2. **TTJO:** updates waypoints and AI model residency at management-frame boundaries;
   NDT fidelity is fixed at level 1 and its NDT screening is nominal.
3. **NDT-O:** updates waypoints, model residency and NDT fidelity; it screens using
   nominal NDT predictions without an uncertainty margin.
4. **NDT-TJO:** uses the same two-timescale decisions but adds a one-sided calibrated
   prediction-residual margin to the nominal NDT feasibility test.

The fast-timescale implementation is a greedy finite-candidate heuristic inspired by
the admission/association, radio, computing and feasibility stages in the formulation.
Its radio priority includes a DPP-inspired queue/utility weighting controlled by `V`.
The slow-timescale waypoint step uses weighted clustering plus a small coordinate
search. This is not an exact solver for the full mixed-integer nonconvex stochastic
optimization problem, and the drift-plus-penalty derivation does not amount to a
formal stability or global-optimality proof for this implementation.

## Important simplifications

The following details are not fully modeled and must not be claimed as fully validated:

- The simulator uses a single simplified end-to-end latency estimate rather than a
  complete packet-level uplink/processing/downlink protocol stack.
- ISAC is represented as a service class with a latency/rate target. The full Fisher
  information matrix and sensing covariance constraint are not implemented.
- The mobility/energy model uses a constant hover-power approximation plus a simple
  movement-energy term; it is not a validated aerodynamic propulsion model.
- AI model execution is abstracted through model residency, model quality and cycles
  per bit; real accelerator memory, KV-cache growth, TTFT and TPOT are not modeled
  at hardware measurement fidelity.
- The NDT is represented by synthetic latency prediction residuals at three fidelity
  levels. No trained digital twin or real residual dataset is supplied.
- The uncertainty margin is calibrated from a synthetic Gaussian residual sample.
  The NDT false-accept metric compares nominally accepted requests against a
  separately generated latent-latency trace. Its probabilistic interpretation holds
  only under the synthetic calibration and latent-state model.
- The battery is tracked and reserve violations are recorded; this simplified setup
  does not include a full flight-control or battery-discharge model.
- The candidate searches are heuristics and do not prove global optimality.

## How to report this honestly

Describe the results as **simulation results for a synthetic UAV-assisted network
instance under the stated assumptions**. Include the parameter table from
`config/default.json` in the paper. Do not describe the generated data as real-world
measurements, and do not claim full implementation of sensing/Fisher-information,
real accelerator KV-cache, physical propulsion, or a trained NDT.

Before submission, compare the assumptions here with what your instructor expects
and clearly state the scope in the paper's Experimental Setup and Limitations.
