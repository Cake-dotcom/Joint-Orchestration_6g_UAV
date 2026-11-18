"""Deterministic synthetic traffic, mobility, fading and NDT-error traces."""
from dataclasses import dataclass
import numpy as np
from .config import SimulationConfig

SERVICE_NAMES = ("IC", "HRLLC", "MC", "UC", "AIAC", "ISAC")
# Illustrative service requirements; see ASSUMPTIONS_AND_SCOPE.md.
DEADLINES_S = np.array([0.36, 0.20, 0.40, 0.50, 0.50, 0.28], dtype=float)
UTILITY_WEIGHTS = np.array([0.90, 1.35, 0.65, 0.75, 1.15, 1.00], dtype=float)
CYCLES_PER_BIT = np.array([150.0, 250.0, 80.0, 100.0, 2000.0, 300.0], dtype=float)
MIN_RATE_BPS = np.array([0.35e6, 0.60e6, 0.20e6, 0.18e6, 0.25e6, 0.40e6], dtype=float)


@dataclass
class Scenario:
    seed: int
    user_xy: np.ndarray          # [slot, user, xy]
    service_class: np.ndarray    # [user]
    arrivals_bits: np.ndarray   # [slot, user]
    requested_model: np.ndarray # [user], -1 for non-AIAC
    fading_power: np.ndarray    # [slot, user, UAV]
    ndt_noise_s: np.ndarray     # [slot, user, fidelity-1]
    network_jitter_s: np.ndarray # [slot, user], unmodeled state variation


def generate_scenario(cfg: SimulationConfig, seed: int) -> Scenario:
    """Generate one shared exogenous trace, reused by all four algorithms."""
    rng = np.random.default_rng(seed)
    T, U, I = cfg.slots, cfg.n_users, cfg.n_uavs

    initial_xy = rng.uniform(80.0, cfg.area_m - 80.0, size=(U, 2))
    user_xy = np.empty((T, U, 2), dtype=float)
    current = initial_xy.copy()
    for t in range(T):
        if t:
            current = current + rng.normal(0.0, cfg.user_walk_std_m, size=(U, 2))
            current = np.clip(current, 20.0, cfg.area_m - 20.0)
        user_xy[t] = current

    service_class = rng.integers(0, len(SERVICE_NAMES), size=U)
    arrivals_bits = np.zeros((T, U), dtype=float)
    for u in range(U):
        p = cfg.arrival_probability_by_class[service_class[u]]
        active = rng.random(T) < p
        packet_sizes = rng.uniform(cfg.arrival_bits_min, cfg.arrival_bits_max, size=T)
        arrivals_bits[:, u] = active * packet_sizes

    requested_model = np.full(U, -1, dtype=int)
    ai_users = np.where(service_class == 4)[0]
    if ai_users.size:
        requested_model[ai_users] = rng.integers(0, cfg.n_models, size=ai_users.size)

    # Unit-mean exponential power gain is the power-domain Rayleigh fading model.
    fading_power = rng.exponential(1.0, size=(T, U, I))

    ndt_noise_s = np.empty((T, U, 3), dtype=float)
    for level, sigma in enumerate(cfg.ndt_error_std_s):
        ndt_noise_s[:, :, level] = rng.normal(0.0, sigma, size=(T, U))
    # Residual network-state variation around the analytic latency proxy. The
    # NDT predicts this latent quantity with the fidelity-dependent error above.
    network_jitter_s = rng.normal(0.0, 0.018, size=(T, U))

    return Scenario(
        seed=seed,
        user_xy=user_xy,
        service_class=service_class,
        arrivals_bits=arrivals_bits,
        requested_model=requested_model,
        fading_power=fading_power,
        ndt_noise_s=ndt_noise_s,
        network_jitter_s=network_jitter_s,
    )
