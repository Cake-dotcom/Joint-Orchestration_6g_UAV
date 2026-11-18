"""The four control configurations used in the comparative experiment."""
from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np
from .config import SimulationConfig
from .scenario import CYCLES_PER_BIT, DEADLINES_S, UTILITY_WEIGHTS, MIN_RATE_BPS

ALGORITHMS = ("CRA", "TTJO", "NDT-O", "NDT-TJO")


@dataclass
class SlowConfiguration:
    positions: np.ndarray
    model_residency: np.ndarray
    fidelity: int
    switching_count: int
    movement_distance_m: float


def _path_gain(user_xy: np.ndarray, uav_xy: np.ndarray, cfg: SimulationConfig,
               fading_power: np.ndarray) -> np.ndarray:
    """Simplified 3.5-GHz air-to-ground free-space gain with Rayleigh fading."""
    delta = user_xy[:, None, :] - uav_xy[None, :, :]
    horizontal = np.linalg.norm(delta, axis=2)
    distance_m = np.sqrt(horizontal ** 2 + cfg.uav_altitude_m ** 2)
    distance_km = np.maximum(distance_m / 1000.0, 1e-3)
    fspl_db = 32.4 + 20.0 * np.log10(cfg.carrier_frequency_ghz) + 20.0 * np.log10(distance_km)
    large_scale_gain = 10.0 ** (-fspl_db / 10.0)
    return large_scale_gain * np.maximum(fading_power, 1e-4)


def _kmeans_targets(user_xy: np.ndarray, weights: np.ndarray, current: np.ndarray,
                    iterations: int = 6) -> np.ndarray:
    """Small deterministic weighted k-means used to generate waypoint candidates."""
    centers = current.copy()
    for _ in range(iterations):
        dist = np.linalg.norm(user_xy[:, None, :] - centers[None, :, :], axis=2)
        labels = np.argmin(dist, axis=1)
        for i in range(centers.shape[0]):
            mask = labels == i
            if np.any(mask):
                w = np.maximum(weights[mask], 1e-6)
                centers[i] = np.average(user_xy[mask], axis=0, weights=w)
    return centers


def _select_models(user_xy: np.ndarray, requested_model: np.ndarray,
                   arrivals: np.ndarray, positions: np.ndarray,
                   cfg: SimulationConfig) -> np.ndarray:
    """Cache the two most demanded models per UAV subject to memory capacity."""
    residency = np.zeros((cfg.n_uavs, cfg.n_models), dtype=bool)
    ai_mask = requested_model >= 0
    if not np.any(ai_mask):
        residency[:, 0] = True
        return residency
    for i in range(cfg.n_uavs):
        distances = np.linalg.norm(user_xy[ai_mask] - positions[i], axis=1)
        local_weight = arrivals[ai_mask] + 20_000.0
        # Inverse-distance weighting approximates the expected local AI demand.
        proximity = 1.0 / (1.0 + distances / 250.0)
        demand = np.zeros(cfg.n_models, dtype=float)
        for model in range(cfg.n_models):
            demand[model] = np.sum(local_weight * proximity * (requested_model[ai_mask] == model))
        order = np.argsort(-demand)
        used = 0.0
        for model in order:
            mem = cfg.model_memory_gb[model]
            if demand[model] > 0 and used + mem <= cfg.memory_capacity_gb + 1e-9:
                residency[i, model] = True
                used += mem
        if not np.any(residency[i]):
            residency[i, int(order[0])] = True
    return residency


def _fidelity_choice(service_class: np.ndarray, arrivals: np.ndarray) -> int:
    load = float(np.sum(arrivals))
    if load <= 1e-9:
        return 1
    critical = np.isin(service_class, [1, 5])
    critical_fraction = float(np.sum(arrivals[critical]) / load)
    if critical_fraction >= 0.30:
        return 3
    if critical_fraction >= 0.12:
        return 2
    return 1


def initial_configuration(cfg: SimulationConfig, first_user_xy: np.ndarray) -> SlowConfiguration:
    # A fixed triangular deployment, deliberately not demand-adaptive.
    positions = np.array([
        [0.20 * cfg.area_m, 0.28 * cfg.area_m],
        [0.50 * cfg.area_m, 0.72 * cfg.area_m],
        [0.80 * cfg.area_m, 0.28 * cfg.area_m],
    ], dtype=float)[:cfg.n_uavs]
    if cfg.n_uavs > 3:
        extra = np.linspace(0.15, 0.85, cfg.n_uavs - 3)
        positions = np.vstack([positions, np.column_stack([extra * cfg.area_m, np.full_like(extra, 0.5 * cfg.area_m)])])
    residency = np.zeros((cfg.n_uavs, cfg.n_models), dtype=bool)
    residency[:, 0] = True
    if cfg.n_models > 1:
        residency[:, 1] = True
    return SlowConfiguration(positions, residency, 1, 0, 0.0)


def update_slow_configuration(algorithm: str, previous: SlowConfiguration,
                              user_xy: np.ndarray, service_class: np.ndarray,
                              arrivals: np.ndarray, requested_model: np.ndarray,
                              cfg: SimulationConfig) -> SlowConfiguration:
    """Frame-level waypoint/model/fidelity control for each method.

    CRA holds all slow-timescale variables fixed. TTJO optimizes waypoints and
    model residency but fixes NDT fidelity. NDT-O and NDT-TJO additionally
    adapt fidelity; only NDT-TJO uses uncertainty margins in the fast loop.
    """
    if algorithm == "CRA":
        return SlowConfiguration(previous.positions.copy(), previous.model_residency.copy(),
                                  previous.fidelity, 0, 0.0)

    weights = np.maximum(arrivals, 10_000.0)
    targets = _kmeans_targets(user_xy, weights, previous.positions)
    proposed = previous.positions.copy()
    for i in range(cfg.n_uavs):
        delta = targets[i] - previous.positions[i]
        distance = np.linalg.norm(delta)
        if distance > cfg.max_move_per_frame_m:
            delta *= cfg.max_move_per_frame_m / max(distance, 1e-9)
        proposed[i] = np.clip(previous.positions[i] + delta, 0.0, cfg.area_m)

    # A short coordinate-search refinement: accept a target waypoint only if
    # it improves weighted nearest-UAV distance after movement cost.
    old_positions = previous.positions.copy()
    current = old_positions.copy()
    for i in range(cfg.n_uavs):
        best = current[i].copy()
        best_score = _placement_score(user_xy, weights, current, cfg) + 0.35 * float(np.sum(np.linalg.norm(current - old_positions, axis=1)))
        candidates = [current[i], proposed[i], 0.5 * (current[i] + proposed[i])]
        for candidate in candidates:
            trial = current.copy()
            trial[i] = candidate
            score = _placement_score(user_xy, weights, trial, cfg) + 0.35 * float(np.sum(np.linalg.norm(trial - old_positions, axis=1)))
            if score < best_score:
                best_score, best = score, candidate.copy()
        current[i] = best

    if algorithm == "TTJO":
        fidelity = 1
    else:
        fidelity = _fidelity_choice(service_class, arrivals)

    if algorithm in ("NDT-O", "NDT-TJO"):
        residency = _select_models(user_xy, requested_model, arrivals, current, cfg)
    else:
        residency = _select_models(user_xy, requested_model, arrivals, current, cfg)

    switching = int(np.count_nonzero(residency != previous.model_residency))
    movement = float(np.sum(np.linalg.norm(current - old_positions, axis=1)))
    return SlowConfiguration(current, residency, fidelity, switching, movement)


def _placement_score(user_xy: np.ndarray, weights: np.ndarray, positions: np.ndarray,
                     cfg: SimulationConfig) -> float:
    distances = np.linalg.norm(user_xy[:, None, :] - positions[None, :, :], axis=2)
    nearest = np.min(distances, axis=1)
    movement_penalty = 0.0
    overlap_penalty = 0.0
    if cfg.n_uavs > 1:
        pair = []
        for i in range(cfg.n_uavs):
            for j in range(i + 1, cfg.n_uavs):
                pair.append(np.linalg.norm(positions[i] - positions[j]))
        overlap_penalty = 0.5 * float(np.sum(1.0 / (np.asarray(pair) + 30.0))) * 1e5
    return float(np.average(nearest, weights=weights)) + movement_penalty + overlap_penalty


def _expected_rate_bps(gain: float, cfg: SimulationConfig, share: float) -> float:
    noise_w = 10.0 ** ((cfg.noise_power_dbm - 30.0) / 10.0)
    p_rb = cfg.total_tx_power_w / cfg.n_rbs
    # Approximate inter-UAV interference; detailed RB-level interference is
    # computed later after scheduling.
    interference = noise_w + 0.25 * p_rb * gain
    sinr = p_rb * gain / max(interference, 1e-15)
    per_rb = cfg.bandwidth_per_rb_hz * np.log2(1.0 + sinr)
    return max(1.0, per_rb * max(1.0, share))


def _calibration_margins(cfg: SimulationConfig) -> np.ndarray:
    rng = np.random.default_rng(78123)
    margins = []
    for sigma in cfg.ndt_error_std_s:
        residual = rng.normal(0.0, sigma, size=cfg.calibration_samples)
        margins.append(max(0.0, float(np.quantile(residual, 1.0 - cfg.risk_delta))))
    return np.asarray(margins, dtype=float)


def run_algorithm(algorithm: str, scenario, cfg: SimulationConfig) -> Tuple[Dict[str, float], list]:
    if algorithm not in ALGORITHMS:
        raise ValueError(f"Unknown algorithm: {algorithm}")
    T, U, I = cfg.slots, cfg.n_users, cfg.n_uavs
    queue = np.zeros(U, dtype=float)
    battery = np.full(I, cfg.battery_initial_j, dtype=float)
    conf = initial_configuration(cfg, scenario.user_xy[0])
    margins = _calibration_margins(cfg)

    metrics = {
        "objective": [], "energy_j": [], "utility": [], "avg_queue_bits": [],
        "admitted": 0, "offered": 0, "rejected": 0, "latency_samples": [],
        "latency_violations": 0, "nominal_screen_passed": 0,
        "nominal_screen_violations": 0, "battery_reserve_violations": 0,
        "coverage_samples": [], "ndt_abs_error": [], "ndt_cost": [],
        "switch_count": 0, "movement_m": 0.0, "robust_filtered": 0,
    }
    timeseries = []
    total_energy = 0.0
    total_utility = 0.0
    total_arrivals = 0
    total_violations = 0
    total_nominal_passed = 0
    total_nominal_violations = 0
    total_robust_filtered = 0
    total_latency_count = 0
    total_coverage = []
    total_ndt_error = []
    total_objective = []

    for t in range(T):
        frame = t // cfg.slots_per_frame
        slot_in_frame = t % cfg.slots_per_frame
        if slot_in_frame == 0 and t > 0:
            start = max(0, t - cfg.slots_per_frame)
            forecast_arrivals = np.sum(scenario.arrivals_bits[start:t], axis=0)
            conf = update_slow_configuration(
                algorithm, conf, scenario.user_xy[t], scenario.service_class,
                forecast_arrivals, scenario.requested_model, cfg,
            )
            metrics["switch_count"] += conf.switching_count
            metrics["movement_m"] += conf.movement_distance_m

        user_xy = scenario.user_xy[t]
        gains = _path_gain(user_xy, conf.positions, cfg, scenario.fading_power[t])
        arrivals = scenario.arrivals_bits[t].copy()
        classes = scenario.service_class
        metrics["offered"] += int(np.count_nonzero(arrivals > 0))
        total_arrivals += int(np.count_nonzero(arrivals > 0))

        # Association candidate based on channel quality and current UAV load.
        # First assign all active users, then estimate per-user RB/CPU shares
        # from the completed association map (avoids optimistic early-user shares).
        association = np.full(U, -1, dtype=int)
        candidate_base_latency = np.full(U, np.inf, dtype=float)
        associated_load = np.zeros(I, dtype=int)
        active = (queue > 0) | (arrivals > 0)
        for u in range(U):
            if not active[u]:
                continue
            candidate_scores = []
            for i in range(I):
                model = scenario.requested_model[u]
                if classes[u] == 4 and model >= 0:
                    if not conf.model_residency[i, model] or cfg.model_quality[model] < cfg.min_ai_quality:
                        continue
                # Channel-aware load balancing; higher gain is preferred but
                # heavily loaded UAVs are progressively penalized.
                score = gains[u, i] / (1.0 + 0.22 * associated_load[i])
                candidate_scores.append((score, i))
            if not candidate_scores:
                continue
            _, chosen = max(candidate_scores, key=lambda item: item[0])
            association[u] = chosen
            associated_load[chosen] += 1

        for u in range(U):
            i = association[u]
            if i < 0:
                continue
            share = max(1.0, cfg.n_rbs / max(1, associated_load[i]))
            estimate_rate = _expected_rate_bps(gains[u, i], cfg, share)
            expected_cpu = cfg.cpu_capacity_cycles_s / max(1, associated_load[i])
            cycles = CYCLES_PER_BIT[classes[u]]
            candidate_base_latency[u] = (
                (queue[u] + arrivals[u]) / estimate_rate
                + arrivals[u] * cycles / max(expected_cpu, 1.0)
                + 0.010
            )

        # Admission: all methods share the same deterministic hard checks.
        # CRA has no NDT screen. TTJO/NDT-O use nominal NDT prediction; NDT-TJO
        # adds a calibrated one-sided residual margin.
        admitted_arrivals = np.zeros(U, dtype=float)
        nominal_passed = np.zeros(U, dtype=bool)
        latent_ndt_latency = np.full(U, np.nan, dtype=float)
        robust_filtered_now = 0
        for u in range(U):
            if arrivals[u] <= 0 or association[u] < 0:
                continue
            i = association[u]
            estimated = candidate_base_latency[u]
            deadline = DEADLINES_S[classes[u]]
            deterministic_ok = battery[i] > cfg.battery_reserve_j + 10.0
            if classes[u] == 4:
                m = scenario.requested_model[u]
                deterministic_ok = deterministic_ok and m >= 0 and conf.model_residency[i, m]
            if not deterministic_ok:
                continue

            fidelity = max(1, min(3, conf.fidelity))
            if algorithm == "CRA":
                accepted = estimated <= deadline * 1.10
            else:
                prediction_error = scenario.ndt_noise_s[t, u, fidelity - 1]
                latent_true_latency = max(0.0, estimated + scenario.network_jitter_s[t, u])
                predicted_latency = latent_true_latency + prediction_error
                latent_ndt_latency[u] = latent_true_latency
                nominal_ok = predicted_latency <= deadline
                nominal_passed[u] = nominal_ok
                if algorithm == "NDT-TJO":
                    robust_ok = predicted_latency + margins[fidelity - 1] <= deadline
                    accepted = robust_ok
                    if nominal_ok and not robust_ok:
                        robust_filtered_now += 1
                else:
                    accepted = nominal_ok
            if accepted:
                admitted_arrivals[u] = arrivals[u]

        # Count nominally passing requests that are actually admitted; robustly
        # rejected requests do not have a post-action ground-truth outcome.

        # Re-associate active queues, including work admitted in earlier slots.
        active_after_admission = (queue + admitted_arrivals) > 0
        for u in range(U):
            if active_after_admission[u] and association[u] < 0:
                best_i = int(np.argmax(gains[u]))
                if classes[u] != 4 or conf.model_residency[best_i, scenario.requested_model[u]]:
                    association[u] = best_i

        # Greedy radio-block scheduling. Each UAV can use each RB once; the
        # same RB may be reused by another UAV, creating inter-UAV interference.
        rb_owner = np.full((I, cfg.n_rbs), -1, dtype=int)
        rb_counts = np.zeros(U, dtype=int)
        p_rb = cfg.total_tx_power_w / cfg.n_rbs
        noise_w = 10.0 ** ((cfg.noise_power_dbm - 30.0) / 10.0)
        for i in range(I):
            users_i = np.where((association == i) & active_after_admission)[0]
            if users_i.size == 0:
                continue
            # Guarantee one RB per active associated user when feasible.
            # Remaining RBs are assigned greedily using queue and class urgency.
            # DPP-inspired urgency: backlog controls queue pressure, while V
            # weights the service-utility term relative to backlog. This is a
            # finite-candidate heuristic, not an exact P2 optimizer.
            urgency = {
                int(u): ((queue[u] + admitted_arrivals[u]) / 100_000.0
                + cfg.lyapunov_v * 0.35) * UTILITY_WEIGHTS[classes[u]]
                for u in users_i
            }
            order = sorted(users_i.tolist(), key=lambda u: urgency[u], reverse=True)
            r = 0
            if len(order) <= cfg.n_rbs:
                for u in order:
                    rb_owner[i, r] = u
                    rb_counts[u] += 1
                    r += 1
            while r < cfg.n_rbs:
                scores = []
                for u in users_i:
                    score = urgency[int(u)] / (1.0 + 1.25 * rb_counts[u])
                    scores.append(score)
                owner = int(users_i[int(np.argmax(scores))])
                rb_owner[i, r] = owner
                rb_counts[owner] += 1
                r += 1

        radio_rate = np.zeros(U, dtype=float)
        for i in range(I):
            for r in range(cfg.n_rbs):
                u = rb_owner[i, r]
                if u < 0:
                    continue
                interference = noise_w
                for j in range(I):
                    if j != i and rb_owner[j, r] >= 0:
                        interference += p_rb * gains[u, j]
                sinr = p_rb * gains[u, i] / max(interference, 1e-15)
                radio_rate[u] += cfg.bandwidth_per_rb_hz * np.log2(1.0 + sinr)

        # Computing allocations follow backlog urgency and enforce per-UAV CPU.
        cpu_alloc = np.zeros(U, dtype=float)
        for i in range(I):
            users_i = np.where((association == i) & active_after_admission)[0]
            if users_i.size == 0:
                continue
            weights = np.array([
                (queue[u] + admitted_arrivals[u] + 10_000.0) ** 0.45
                * UTILITY_WEIGHTS[classes[u]] for u in users_i
            ])
            weights = weights / max(float(np.sum(weights)), 1e-12)
            cpu_alloc[users_i] = cfg.cpu_capacity_cycles_s * weights

        queue_before = queue.copy()
        service_bits = np.zeros(U, dtype=float)
        actual_latency = np.full(U, np.nan, dtype=float)
        for u in range(U):
            work = queue[u] + admitted_arrivals[u]
            if work <= 0:
                continue
            i = association[u]
            if i < 0:
                continue
            cycles = CYCLES_PER_BIT[classes[u]]
            radio_capacity = radio_rate[u] * cfg.slot_duration_s
            compute_capacity = cpu_alloc[u] * cfg.slot_duration_s / cycles
            service_bits[u] = min(work, radio_capacity, compute_capacity)
            if radio_rate[u] <= 1.0 or cpu_alloc[u] <= 1.0:
                # No usable service this slot: count as a deadline miss using a
                # finite one-slot-over-deadline latency proxy. The queue remains.
                actual_latency[u] = DEADLINES_S[classes[u]] + cfg.slot_duration_s
            else:
                actual_latency[u] = (
                    (queue_before[u] + admitted_arrivals[u]) / radio_rate[u]
                    + admitted_arrivals[u] * cycles / cpu_alloc[u]
                    + 0.010
                )

        queue = np.maximum(0.0, queue + admitted_arrivals - service_bits)
        admitted_mask = admitted_arrivals > 0
        admitted_now = int(np.count_nonzero(admitted_mask))
        rejected_now = int(np.count_nonzero((arrivals > 0) & ~admitted_mask))
        nominal_admitted_mask = nominal_passed & admitted_mask
        nominal_admitted = int(np.count_nonzero(nominal_admitted_mask))
        metrics["nominal_screen_passed"] += nominal_admitted
        total_nominal_passed += nominal_admitted
        metrics["robust_filtered"] += robust_filtered_now
        total_robust_filtered += robust_filtered_now
        if nominal_admitted:
            hidden_violations = nominal_admitted_mask & (latent_ndt_latency > DEADLINES_S[classes])
            total_nominal_violations += int(np.count_nonzero(hidden_violations))
        metrics["admitted"] += admitted_now
        metrics["rejected"] += rejected_now
        total_utility += float(np.sum(UTILITY_WEIGHTS[classes[admitted_mask]]))

        latency_values = actual_latency[admitted_mask]
        if latency_values.size:
            deadlines = DEADLINES_S[classes[admitted_mask]]
            violated = latency_values > deadlines
            count_v = int(np.count_nonzero(violated))
            total_violations += count_v
            total_latency_count += int(latency_values.size)
            metrics["latency_samples"].extend(latency_values.tolist())
            # The NDT-specific hidden-violation counter is evaluated separately
            # against the synthetic latent network-state latency above.

        # Metrics from actual allocated service.
        avg_queue = float(np.mean(queue))
        served_users = np.where(radio_rate > 0)[0]
        if served_users.size:
            coverage = float(np.mean(radio_rate[served_users] >= MIN_RATE_BPS[classes[served_users]]))
        else:
            coverage = 0.0
        total_coverage.append(coverage)

        # Prediction error measured against the estimated pre-decision latency.
        if algorithm != "CRA":
            selected = np.where(admitted_mask)[0]
            if selected.size:
                fidelity_idx = conf.fidelity - 1
                total_ndt_error.extend(np.abs(scenario.ndt_noise_s[t, selected, fidelity_idx]).tolist())

        # Simplified energy accounting: hover, radio, compute, movement and NDT.
        tx_energy = float(np.count_nonzero(rb_owner >= 0) * p_rb * cfg.slot_duration_s)
        compute_energy = float(I * cfg.compute_power_w * cfg.slot_duration_s)
        hover_energy = float(I * cfg.hover_power_w * cfg.slot_duration_s)
        movement_energy = 0.0
        if slot_in_frame == 0 and t > 0:
            movement_energy = 0.018 * conf.movement_distance_m
        ndt_energy = cfg.ndt_energy_per_frame_j[conf.fidelity - 1] / cfg.slots_per_frame
        switch_energy = conf.switching_count * cfg.model_switch_energy_j / cfg.slots_per_frame
        energy = hover_energy + tx_energy + compute_energy + movement_energy + ndt_energy + switch_energy
        total_energy += energy

        ndt_cost = cfg.ndt_cost_per_frame[conf.fidelity - 1] / cfg.slots_per_frame if algorithm != "CRA" else 0.0
        utility_slot = float(np.sum(UTILITY_WEIGHTS[classes[admitted_mask]]) / max(1, U))
        energy_norm = energy / max(1.0, I * cfg.hover_power_w * cfg.slot_duration_s)
        switch_norm = (conf.switching_count / cfg.slots_per_frame) if slot_in_frame == 0 else 0.0
        objective = (
            cfg.eta_energy * energy_norm
            + cfg.eta_ndt * ndt_cost
            + cfg.eta_switch * switch_norm
            - cfg.eta_utility * utility_slot
        )
        total_objective.append(float(objective))
        metrics["energy_j"].append(energy)
        metrics["utility"].append(utility_slot)
        metrics["avg_queue_bits"].append(avg_queue)
        metrics["objective"].append(float(objective))
        metrics["ndt_cost"].append(float(ndt_cost))

        # Battery reserve is tracked as an explicit hard constraint. The
        # prototype stops allocating admissions if the remaining reserve is hit.
        for i in range(I):
            battery[i] -= energy / max(1, I)
            if battery[i] < cfg.battery_reserve_j:
                metrics["battery_reserve_violations"] += 1
        timeseries.append({
            "slot": t,
            "time_s": (t + 1) * cfg.slot_duration_s,
            "avg_queue_bits": avg_queue,
            "energy_j": energy,
            "cumulative_energy_j": total_energy,
            "admitted_this_slot": admitted_now,
            "rejected_this_slot": rejected_now,
            "latency_violations_cumulative": total_violations,
            "objective": float(objective),
            "fidelity": conf.fidelity,
        })

    offered = max(1, metrics["offered"])
    admitted = metrics["admitted"]
    latency_array = np.asarray(metrics["latency_samples"], dtype=float)
    result = {
        "algorithm": algorithm,
        "seed": int(scenario.seed),
        "objective_mean": float(np.mean(total_objective)),
        "energy_total_j": float(total_energy),
        "energy_mean_j_per_slot": float(total_energy / T),
        "admitted_requests": int(admitted),
        "offered_requests": int(metrics["offered"]),
        "rejection_rate_pct": float(100.0 * metrics["rejected"] / offered),
        "admission_rate_pct": float(100.0 * admitted / offered),
        "service_utility_mean": float(total_utility / T),
        "avg_queue_bits": float(np.mean(metrics["avg_queue_bits"])),
        "final_queue_bits": float(np.mean(queue)),
        "mean_latency_s": float(np.mean(latency_array)) if latency_array.size else float("nan"),
        "p95_latency_s": float(np.quantile(latency_array, 0.95)) if latency_array.size else float("nan"),
        "latency_violation_rate_pct": float(100.0 * total_violations / max(1, total_latency_count)),
        "nominal_pass_then_violate_pct": float(100.0 * total_nominal_violations / max(1, total_nominal_passed)),
        "robust_filter_count": int(total_robust_filtered),
        "coverage_mean_pct": float(100.0 * np.mean(total_coverage)),
        "ndt_abs_prediction_error_s": float(np.mean(total_ndt_error)) if total_ndt_error else 0.0,
        "mean_ndt_cost": float(np.mean(metrics["ndt_cost"])),
        "model_switch_count": int(metrics["switch_count"]),
        "uav_movement_total_m": float(metrics["movement_m"]),
        "battery_reserve_violations": int(metrics["battery_reserve_violations"]),
    }
    return result, timeseries
