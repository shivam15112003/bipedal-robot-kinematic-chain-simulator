import numpy as np
import pinocchio as pin

from portfolio.biped import make_model, run_experiment


def test_physical_invariants_and_tracking():
    model = make_model()
    mass = pin.crba(model, model.createData(), pin.neutral(model))
    assert np.linalg.eigvalsh(mass).min() > 0
    metrics, records, _ = run_experiment(duration=0.5)
    assert np.isfinite(records).all()
    assert metrics["ik_initial_position_error_m"] > 0.01
    assert metrics["ik_final_position_error_m"] < 1e-6
    assert metrics["tracking_rmse_rad"] < 0.02
    assert metrics["forward_inverse_dynamics_max_residual_rad_s2"] < 1e-8


def test_static_gravity_balance():
    model = make_model()
    data = model.createData()
    q = pin.neutral(model)
    zero = np.zeros(model.nv)
    gravity_torque = pin.computeGeneralizedGravity(model, data, q).copy()
    assert np.allclose(pin.aba(model, data, q, zero, gravity_torque), zero, atol=1e-10)
