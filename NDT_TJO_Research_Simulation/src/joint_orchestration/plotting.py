"""Publication-style figures made from the simulation CSV outputs."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ALGORITHM_ORDER = ["CRA", "TTJO", "NDT-O", "NDT-TJO"]


def _read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _bar_metric(summary, metric, title, ylabel, filename, percent=False):
    means, stds = [], []
    for alg in ALGORITHM_ORDER:
        row = next(r for r in summary if r["algorithm"] == alg)
        means.append(float(row[f"{metric}_mean"]))
        stds.append(float(row[f"{metric}_std"]))
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.bar(ALGORITHM_ORDER, means, yerr=stds, capsize=4, alpha=0.86)
    ax.set_title(title, fontsize=12, pad=12)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.set_axisbelow(True)
    if percent:
        ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _line_timeseries(rows, metric, title, ylabel, filename):
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    for alg in ALGORITHM_ORDER:
        subsets = [r for r in rows if r["algorithm"] == alg]
        by_slot = {}
        for r in subsets:
            by_slot.setdefault(int(r["slot"]), []).append(float(r[metric]))
        slots = sorted(by_slot)
        means = [np.mean(by_slot[s]) for s in slots]
        stds = [np.std(by_slot[s]) for s in slots]
        ax.plot(slots, means, label=alg, linewidth=1.8)
        ax.fill_between(slots, np.asarray(means) - np.asarray(stds),
                        np.asarray(means) + np.asarray(stds), alpha=0.10)
    ax.set_title(title, fontsize=12, pad=12)
    ax.set_xlabel("Scheduling slot")
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.30)
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)


def create_all_figures(output_dir: str | Path):
    out = Path(output_dir)
    figdir = out / "figures"
    figdir.mkdir(exist_ok=True)
    summary = _read_csv(out / "summary.csv")
    timeseries = _read_csv(out / "timeseries.csv")

    _bar_metric(summary, "objective_mean", "Average objective value", "Objective (lower is better)", figdir / "01_objective_comparison.png")
    _bar_metric(summary, "energy_mean_j_per_slot", "Average energy consumption", "Energy (J/slot)", figdir / "02_energy_comparison.png")
    _line_timeseries(timeseries, "avg_queue_bits", "Queue backlog over time", "Mean queued workload (bits)", figdir / "03_queue_backlog.png")
    _bar_metric(summary, "service_utility_mean", "Admitted-service utility", "Weighted utility per slot", figdir / "04_service_utility.png")
    _bar_metric(summary, "latency_violation_rate_pct", "End-to-end latency SLA violations", "Violation rate (%)", figdir / "05_sla_violations.png", percent=True)
    _bar_metric(summary, "ndt_abs_prediction_error_s", "NDT absolute prediction error", "Absolute error (s)", figdir / "06_ndt_prediction_error.png")
    _bar_metric(summary, "mean_latency_s", "Mean latency of admitted requests", "Latency (s)", figdir / "07_mean_latency.png")
    _bar_metric(summary, "nominal_pass_then_violate_pct", "NDT uncertainty-screen false acceptance", "False-accept rate (%)", figdir / "08_nominal_screen_failure.png", percent=True)

    # Optional cumulative energy trace, averaged over independent seeds.
    _line_timeseries(timeseries, "cumulative_energy_j", "Cumulative network energy", "Cumulative energy (J)", figdir / "09_cumulative_energy.png")
