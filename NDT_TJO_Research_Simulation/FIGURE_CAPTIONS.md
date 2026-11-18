# Suggested figure captions

1. **Average objective value:** Mean normalized objective for CRA, TTJO, NDT-O and NDT-TJO across independent synthetic simulation seeds. Error bars indicate standard deviation.
2. **Energy consumption:** Average per-slot energy under the simplified hover, radio, compute, movement and NDT energy model. Error bars indicate standard deviation.
3. **Queue backlog:** Mean queued workload over scheduling time, averaged across seeds; shaded regions show one standard deviation.
4. **Admitted-service utility:** Mean weighted admitted-service utility per scheduling slot.
5. **Latency SLA violations:** Percentage of admitted requests whose simulator latency proxy exceeds the class-specific deadline.
6. **NDT prediction error:** Mean absolute synthetic NDT latency-prediction residual for requests admitted under each NDT-enabled configuration.
7. **Mean latency:** Mean latency proxy for admitted requests; a request receiving no usable radio/compute service in that slot is assigned a finite one-slot-over-deadline penalty.
8. **NDT uncertainty-screen false acceptance:** Percentage of nominally accepted requests for which the synthetic latent-latency trace exceeds the relevant service deadline. This metric is based on the generated latent-latency and prediction-residual model, not a real network measurement.
9. **Cumulative energy:** Cumulative energy under the same simplified energy accounting model.

All figures are generated from `results/summary.csv` and `results/timeseries.csv` by `src/joint_orchestration/plotting.py`.
