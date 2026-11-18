"""Fast tests that verify the bundled project can execute end-to-end."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from joint_orchestration.config import SimulationConfig
from joint_orchestration.scenario import generate_scenario
from joint_orchestration.algorithms import ALGORITHMS, run_algorithm


def test_all_four_algorithms_return_finite_metrics():
    cfg = SimulationConfig(frames=2, slots_per_frame=4, seeds=1, n_users=9)
    scenario = generate_scenario(cfg, seed=123)
    for algorithm in ALGORITHMS:
        result, series = run_algorithm(algorithm, scenario, cfg)
        assert result["algorithm"] == algorithm
        assert len(series) == cfg.slots
        assert result["offered_requests"] >= result["admitted_requests"]
        assert result["energy_total_j"] > 0
        assert result["admission_rate_pct"] >= 0
        assert result["latency_violation_rate_pct"] >= 0
