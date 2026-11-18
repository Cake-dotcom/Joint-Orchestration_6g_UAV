"""Command-line entry point.

Examples:
    python run_experiments.py
    python run_experiments.py --quick
    python run_experiments.py --seeds 20 --frames 30
    python run_experiments.py --config config/default.json --output results
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from joint_orchestration.config import SimulationConfig
from joint_orchestration.runner import run_experiments


def main():
    parser = argparse.ArgumentParser(description="Run the NDT-TJO four-algorithm simulation.")
    parser.add_argument("--config", default=str(ROOT / "config" / "default.json"), help="JSON configuration file")
    parser.add_argument("--output", default=str(ROOT / "results"), help="Output directory")
    parser.add_argument("--seeds", type=int, default=None, help="Override number of random seeds")
    parser.add_argument("--frames", type=int, default=None, help="Override management frames per run")
    parser.add_argument("--users", type=int, default=None, help="Override number of users")
    parser.add_argument("--quick", action="store_true", help="Small smoke run (2 seeds, 5 frames)")
    args = parser.parse_args()

    cfg = SimulationConfig.from_json(args.config)
    if args.quick:
        cfg.seeds = 2
        cfg.frames = 5
    if args.seeds is not None:
        cfg.seeds = args.seeds
    if args.frames is not None:
        cfg.frames = args.frames
    if args.users is not None:
        cfg.n_users = args.users
    if cfg.seeds < 1 or cfg.frames < 1 or cfg.n_users < 1:
        parser.error("seeds, frames and users must be positive integers")

    output = Path(args.output)
    if not output.is_absolute():
        output = ROOT / output
    run_experiments(cfg, output, verbose=True)


if __name__ == "__main__":
    main()
