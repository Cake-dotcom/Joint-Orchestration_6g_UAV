"""Simulation configuration for the NDT-TJO research prototype.

All numeric values are explicitly labelled as illustrative assumptions in
ASSUMPTIONS_AND_SCOPE.md. They are not claimed to be values from a field trial.
"""
from dataclasses import dataclass, asdict
from typing import Any, Dict


@dataclass
class SimulationConfig:
    # Network scale
    n_uavs: int = 3
    n_users: int = 18
    n_rbs: int = 12
    n_models: int = 3

    # Time horizon
    frames: int = 18
    slots_per_frame: int = 10
    slot_duration_s: float = 0.10
    seeds: int = 10

    # Radio and geometry
    area_m: float = 1000.0
    uav_altitude_m: float = 120.0
    max_move_per_frame_m: float = 220.0
    bandwidth_per_rb_hz: float = 180_000.0
    carrier_frequency_ghz: float = 3.5
    noise_power_dbm: float = -104.0
    total_tx_power_w: float = 0.25

    # Compute and storage
    cpu_capacity_cycles_s: float = 36e9
    memory_capacity_gb: float = 4.2
    model_memory_gb: tuple = (1.0, 1.8, 2.4)
    model_quality: tuple = (0.82, 0.90, 0.97)
    min_ai_quality: float = 0.80

    # Battery and simplified energy model
    battery_initial_j: float = 24_000.0
    battery_reserve_j: float = 2_400.0
    hover_power_w: float = 65.0
    compute_power_w: float = 0.55
    model_switch_energy_j: float = 8.0

    # NDT levels: 1=low, 2=medium, 3=high
    ndt_error_std_s: tuple = (0.075, 0.045, 0.025)
    ndt_cost_per_frame: tuple = (0.025, 0.060, 0.115)
    ndt_energy_per_frame_j: tuple = (0.35, 0.70, 1.25)
    calibration_samples: int = 2000
    risk_delta: float = 0.05

    # Objective weights (dimensionless implementation scaling)
    eta_energy: float = 0.35
    eta_ndt: float = 0.10
    eta_switch: float = 0.08
    eta_utility: float = 0.47
    lyapunov_v: float = 1.0

    # Synthetic workload and user mobility
    user_walk_std_m: float = 7.0
    arrival_probability_by_class: tuple = (0.20, 0.22, 0.19, 0.22, 0.24, 0.20)
    arrival_bits_min: float = 55_000.0
    arrival_bits_max: float = 165_000.0

    @property
    def slots(self) -> int:
        return self.frames * self.slots_per_frame

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["slots"] = self.slots
        return d

    @classmethod
    def from_json(cls, path: str) -> "SimulationConfig":
        import json
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        raw.pop("slots", None)
        for key in (
            "model_memory_gb", "model_quality", "ndt_error_std_s",
            "ndt_cost_per_frame", "ndt_energy_per_frame_j",
            "arrival_probability_by_class",
        ):
            if key in raw:
                raw[key] = tuple(raw[key])
        return cls(**raw)
