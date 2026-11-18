# Drafting notes for Results and Performance Analysis

The default experiment uses 10 independent seeds, 18 management frames per seed, 10 slots per frame, 3 UAVs and 24 users. All four algorithms use the same generated traffic, mobility, fading and prediction-residual traces for each seed.

## NDT-O versus NDT-TJO (descriptive aggregate means)
- Objective: 0.2879 (NDT-O) versus 0.2907 (NDT-TJO), a +0.99% change.
- Admission rate: 97.22% versus 95.02%, a -2.27% relative change.
- End-to-end latency violation rate: 0.981% versus 0.696%, a -28.99% relative change.
- Average queue backlog: 2698.2 versus 2368.3 bits, a -12.23% relative change.
- NDT nominal-screen false-accept rate: 0.079% versus 0.000%, a -100.00% relative change.
- Average objective trade-off: +0.99% and admission-rate change: -2.27% should be discussed alongside the safety-related metrics.

## Cautious result paragraph (adapt after supervisor review)
Under the documented synthetic simulation assumptions, NDT-TJO reduced the measured latency-violation rate and average queue backlog relative to NDT-O, while also lowering the NDT nominal-screen false-accept rate. This improvement was accompanied by a lower admission rate and a higher aggregate objective value, indicating a trade-off between conservative feasibility screening and service utility/cost. The result supports the role of uncertainty-aware screening in this particular simulated instance, but it does not establish universal superiority, field performance, or global optimality.

## Reporting cautions
- These are descriptive averages and standard deviations, not a hypothesis test; do not claim statistical significance.
- The NDT false-accept metric is calculated using the synthetic latent-latency and residual model; it is not a measured real-network failure probability.
- The overall SLA metric uses the simulator's simplified latency proxy; see ASSUMPTIONS_AND_SCOPE.md.
- Include the full parameter table from config/default.json and disclose all model simplifications.
- If code/parameters are changed before final submission, rerun and use the newly generated summary, table and graphs.