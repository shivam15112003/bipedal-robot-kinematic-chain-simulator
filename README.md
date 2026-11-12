# Bipedal Robot Kinematic Chain Simulator

A humanoid rigid-body kinematics and joint-control application built on the Pinocchio C++ engine. The repository contains a deterministic 28-DOF biped model, damped-Jacobian inverse kinematics, computed-torque tracking, and a measured report — the application code lives in `portfolio/`.

![Humanoid tracking and inverse kinematics](portfolio/results/overview.png)

## What I built

- **Rigid-body biped model** — fixed-pelvis humanoid tree with 28 joint degrees of freedom, explicit link inertias, and a deterministic standing pose.
- **Inverse kinematics** — damped-Jacobian right-ankle position solver that converges from a 0.79 m initial error to below 0.1 µm.
- **Model-based control** — computed-torque tracking using recursive Newton–Euler inverse dynamics and articulated-body forward dynamics.
- **Measurement harness** — CSV trajectories and joint torques, JSON metrics, a headless report plot, and pytest checks for inertia positive-definiteness, gravity balance, and inverse/forward consistency.

## Measured results

From `portfolio/results/metrics.json` (Python 3.12.3, Pinocchio 4.1.0, 2.0 s run at 2 ms steps, 500 timing samples):

| Metric | Measured |
|---|---|
| Degrees of freedom | 28 |
| IK position error, start → finish | 0.7905 m → 9.88 × 10⁻⁸ m |
| Joint tracking RMSE | 0.003885 rad |
| Inverse/forward dynamics residual | 9.34 × 10⁻¹⁵ rad/s² |
| ABA forward dynamics, median | 0.014723 ms |
| ABA forward dynamics, p95 | 0.018351 ms |
| Hardware validation | none (`false`) |

## Resume targets vs. what this repo measures

The resume bullets are design targets from the project brief. This table maps each one to the evidence in this repository.

| Resume claim | Measured in this repo | Status |
|---|---|---|
| Reduce positioning errors by 14% | IK error 0.7905 m → 9.88 × 10⁻⁸ m | Exceeded on the IK metric; no separate 14% baseline was defined |
| Cut simulation latency by 22 ms | ABA median 0.0147 ms per call | Not comparable — no 22 ms baseline was recorded |
| Increase trajectory tracking accuracy by 18% | Tracking RMSE 0.003885 rad | Improvement not established; only the absolute error is measured |
| Validate torque models on physical test benches, closing the sim-to-real gap by 12% | No bench data; `hardware_validation: false` | Not measured |

## Scope

This is a fixed-pelvis joint-motion experiment. It does **not** include ground contact, walking balance, hardware test benches, ROS 2 integration, or PyTorch training. The runnable stack is Python, NumPy, Matplotlib, and Pinocchio's C++ engine. Numerical consistency (residuals near machine precision) is not physical validation.

## Run

Tested with Python 3.12 on Ubuntu under WSL. Install `libgomp1` if the native runtime requests it; native Windows wheels were unavailable in the tested installation path.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r portfolio/requirements.lock
python -m portfolio.biped
python -m pytest portfolio -q
```

Use `--output path/to/results` for a separate run. Default outputs go to `portfolio/results/`. Timings are warmed single-call latency and vary by machine.

## More detail

- [Verification report](doc/PROJECT_VERIFICATION.md)
- [Project requirements and design targets](doc/PROJECT_REQUIREMENTS.md)
- [Metrics JSON](portfolio/results/metrics.json) · [Trajectory CSV](portfolio/results/trajectory.csv)

## Attribution and license

Extends [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`stack-of-tasks/pinocchio`, BSD-2-Clause). Upstream copyright notices and the [LICENSE](LICENSE) text are retained; the application code in `portfolio/` is the work described above.
