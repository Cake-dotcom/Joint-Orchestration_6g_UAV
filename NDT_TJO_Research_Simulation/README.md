# NDT-TJO: Reproducible Four-Algorithm Simulation

This ZIP contains a runnable Python simulation prototype for the four configurations
in the supplied mathematical formulation:

- CRA — Conventional Resource Allocation
- TTJO — Two-Timescale Joint Orchestration
- NDT-O — NDT-assisted orchestration without uncertainty screening
- NDT-TJO — proposed NDT-assisted two-timescale orchestration with uncertainty-aware screening

The simulator generates synthetic traffic/channel traces, runs all four algorithms
on the same traces, writes CSV metrics and creates comparison graphs.

> **Research integrity note:** The formulation PDF does not supply every numeric
> parameter or every physical model function needed for numerical experiments.
> This package fills those gaps with documented assumptions and a simplified
> synthetic simulation. Read `ASSUMPTIONS_AND_SCOPE.md` before using the results.
> The outputs are not real-world UAV measurements or proof of global optimality.

## 1. Requirements

- Windows 10/11, Linux or macOS
- Python 3.10 or later (64-bit recommended)
- Internet only for the first installation of NumPy and Matplotlib
- No external dataset is required

## 2. Install on Windows (PowerShell in VS Code)

Open the extracted project folder in VS Code, then run these commands in the
PowerShell terminal:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

If PowerShell blocks activation, run this once in the same terminal:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

## 3. Run the complete experiment

From the project root:

```powershell
python run_experiments.py
```

Default run settings: 3 UAVs, 24 users, 12 resource blocks, 18 management frames,
10 slots per frame and 10 independent random seeds. This produces 180 slots per
seed and runs all four algorithms on every seed.

For a quick installation/smoke check:

```powershell
python run_experiments.py --quick
```

For a longer run:

```powershell
python run_experiments.py --seeds 20 --frames 30
```

For a smaller user population:

```powershell
python run_experiments.py --users 12
```

## 4. Output files

After execution, see `results/`:

| File | Contents |
|---|---|
| `summary.csv` | Mean and standard deviation across seeds for every metric |
| `per_seed_metrics.csv` | One row per algorithm and random seed |
| `timeseries.csv` | Slot-level queue, energy, admissions, objective and fidelity |
| `simulation_config.json` | Exact parameter values used for the run |
| `RESULTS_OVERVIEW.txt` | Human-readable metric summary |
| `figures/01_objective_comparison.png` | Average objective by algorithm |
| `figures/02_energy_comparison.png` | Energy consumption comparison |
| `figures/03_queue_backlog.png` | Queue backlog over time |
| `figures/04_service_utility.png` | Admitted-service utility |
| `figures/05_sla_violations.png` | Latency SLA violation rate |
| `figures/06_ndt_prediction_error.png` | NDT absolute prediction error |
| `figures/07_mean_latency.png` | Mean admitted-request latency |
| `figures/08_nominal_screen_failure.png` | Nominal pass followed by actual SLA violation |
| `figures/09_cumulative_energy.png` | Cumulative energy over time |

The figures use mean traces across the random seeds; the CSV files preserve the
underlying per-seed values for further analysis.

## 5. Run the smoke test

```powershell
python -m pip install pytest
python -m pytest -q
```

The test checks that all four algorithms execute, return the expected number of
slot records and produce valid basic metrics.

## 6. Change simulation parameters

Edit `config/default.json`. The run saves a copy of the exact configuration to
`results/simulation_config.json`. Keep this file with every paper draft and result
set so the experiment can be reproduced.

Key parameters include `n_uavs`, `n_users`, `n_rbs`, `frames`, `slots_per_frame`,
`seeds`, `cpu_capacity_cycles_s`, `memory_capacity_gb`, `ndt_error_std_s`,
`risk_delta` and the objective weights.

## 7. Suggested experimental workflow

1. Run `python run_experiments.py --quick` to verify the environment.
2. Run `python run_experiments.py` for the default 10-seed comparison.
3. Preserve `results/simulation_config.json`, all CSV files and all PNG figures.
4. Inspect the NDT-O versus NDT-TJO comparison, especially SLA violation rate and
   nominal-screen failure rate.
5. Only report conclusions supported by the generated CSV results.
6. For a robustness section, vary traffic intensity, `risk_delta`, NDT residual
   standard deviations, battery capacity and the Lyapunov parameter.

## 8. Important interpretation

A lower objective is not sufficient on its own: compare it together with admission
rate, service utility, latency and constraint violations. In particular, robust
screening may reject more requests to reduce uncertain SLA violations. Discuss this
trade-off rather than describing one algorithm as universally best.

See `ASSUMPTIONS_AND_SCOPE.md` for the exact modeling limitations.
