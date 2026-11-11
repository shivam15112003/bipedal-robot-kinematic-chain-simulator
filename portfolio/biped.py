"""Fixed-pelvis humanoid kinematics and computed-torque tracking experiment."""

import argparse
import json
import platform
from pathlib import Path
from time import perf_counter_ns

import matplotlib
import numpy as np
import pinocchio as pin

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def make_model():
    """Create Pinocchio's deterministic humanoid with a fixed pelvis."""
    full = pin.buildSampleModelHumanoid(False)
    return pin.buildReducedModel(
        full, [full.getJointId("root_joint")], pin.neutral(full)
    )


def solve_foot(model, target, q0, max_iterations=200):
    """Solve right-ankle position IK with a damped world-frame Jacobian."""
    data = model.createData()
    joint = model.getJointId("rleg_wrist2_joint")
    q = q0.copy()
    errors = []
    for _ in range(max_iterations):
        pin.forwardKinematics(model, data, q)
        error = target - data.oMi[joint].translation
        errors.append(float(np.linalg.norm(error)))
        if errors[-1] < 1e-7:
            break
        jac = pin.computeJointJacobian(model, data, q, joint)
        jac = data.oMi[joint].rotation @ jac[:3]
        velocity = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(3), error)
        q = pin.integrate(model, q, 0.6 * velocity)
    return q, np.array(errors)


def run_experiment(duration=2.0, dt=0.002):
    """Measure tracking, forward/inverse dynamics agreement, and ABA runtime."""
    model = make_model()
    data = model.createData()
    q0 = pin.neutral(model)
    moving = model.joints[model.getJointId("rleg_shoulder2_joint")].idx_v
    target_q = q0.copy()
    target_q[moving] = 0.25
    target_q[model.joints[model.getJointId("rleg_elbow_joint")].idx_v] = 0.3
    pin.forwardKinematics(model, data, target_q)
    target = data.oMi[model.getJointId("rleg_wrist2_joint")].translation.copy()
    _, ik_errors = solve_foot(model, target, q0)
    t = np.arange(round(duration / dt) + 1) * dt
    q, v = q0.copy(), np.zeros(model.nv)
    records = []
    max_residual = 0.0
    for now in t:
        reference = q0.copy()
        reference[moving] += 0.15 * np.sin(2 * now)
        vr, ar = np.zeros(model.nv), np.zeros(model.nv)
        vr[moving] = 0.3 * np.cos(2 * now)
        ar[moving] = -0.6 * np.sin(2 * now)
        command = ar + 80 * pin.difference(model, q, reference) + 18 * (vr - v)
        torque = pin.rnea(model, data, q, v, command).copy()
        acceleration = pin.aba(model, data, q, v, torque).copy()
        max_residual = max(max_residual, float(np.max(np.abs(acceleration - command))))
        records.append([now, reference[moving], q[moving], *torque])
        q = pin.integrate(model, q, dt * v + 0.5 * dt**2 * acceleration)
        v += dt * acceleration
    for _ in range(20):
        pin.aba(model, data, q0, np.zeros(model.nv), np.zeros(model.nv))
    timings = []
    zero = np.zeros(model.nv)
    for _ in range(500):
        start = perf_counter_ns()
        pin.aba(model, data, q0, zero, zero)
        timings.append((perf_counter_ns() - start) / 1e6)
    records = np.array(records)
    metrics = {
        "experiment": "fixed-pelvis humanoid; no ground contact or walking balance",
        "python": platform.python_version(),
        "pinocchio": pin.__version__,
        "degrees_of_freedom": model.nv,
        "duration_s": float(t[-1]),
        "dt_s": dt,
        "ik_initial_position_error_m": float(ik_errors[0]),
        "ik_final_position_error_m": float(ik_errors[-1]),
        "tracking_rmse_rad": float(
            np.sqrt(np.mean((records[:, 1] - records[:, 2]) ** 2))
        ),
        "forward_inverse_dynamics_max_residual_rad_s2": max_residual,
        "aba_median_ms": float(np.median(timings)),
        "aba_p95_ms": float(np.percentile(timings, 95)),
        "timing_samples": len(timings),
        "hardware_validation": False,
    }
    return metrics, records, ik_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("portfolio/results"))
    args = parser.parse_args()
    metrics, records, errors = run_experiment()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    np.savetxt(
        args.output / "trajectory.csv",
        records,
        delimiter=",",
        comments="",
        header="time_s,reference_rad,actual_rad,"
        + ",".join(f"joint_{i}_torque_Nm" for i in range(records.shape[1] - 3)),
    )
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(records[:, 0], records[:, 1], label="Reference", color="#0072B2")
    axes[0].plot(
        records[:, 0],
        records[:, 2],
        "--",
        label="Computed-torque control",
        color="#D55E00",
    )
    axes[0].set(
        xlabel="Time (s)", ylabel="Hip angle (rad)", title="Humanoid joint tracking"
    )
    axes[0].legend()
    axes[1].semilogy(np.maximum(errors, 1e-12), color="#0072B2")
    axes[1].set(
        xlabel="IK iteration", ylabel="Position error (m)", title="Right-ankle IK"
    )
    fig.tight_layout()
    fig.savefig(args.output / "overview.png", dpi=160)
    plt.close(fig)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
