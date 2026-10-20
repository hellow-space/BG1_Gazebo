import importlib.util
import math
from pathlib import Path


MODULE = Path(__file__).resolve().parents[1] / "scripts" / "verify_runtime.py"


def load_module():
    spec = importlib.util.spec_from_file_location("verify_runtime", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(module, t, x=0.0, y=0.0, z=0.12, roll=0.0, pitch=0.0, yaw=0.0, linear=0.0, angular=0.0):
    return module.MotionSample(t, x, y, z, roll, pitch, yaw, linear, angular)


def test_stationary_window_rejects_drift_or_tip():
    module = load_module()
    stable = [sample(module, 0.0), sample(module, 2.0, x=0.004, y=-0.003, roll=0.01, pitch=-0.01)]
    drifting = [sample(module, 0.0), sample(module, 2.0, x=0.08)]
    tipped = [sample(module, 0.0), sample(module, 2.0, roll=0.25)]
    assert module.stationary_window_ok(stable)
    assert not module.stationary_window_ok(drifting)
    assert not module.stationary_window_ok(tipped)


def test_forward_motion_requires_positive_x_progress():
    module = load_module()
    assert module.forward_motion_ok([sample(module, 0.0), sample(module, 2.0, x=0.35)])
    assert not module.forward_motion_ok([sample(module, 0.0), sample(module, 2.0, x=-0.05)])
    assert not module.forward_motion_ok([sample(module, 0.0), sample(module, 2.0, y=0.35)])


def test_rotation_requires_positive_yaw_change():
    module = load_module()
    assert module.positive_rotation_ok([sample(module, 0.0, yaw=3.05), sample(module, 2.0, yaw=-2.75)])
    assert not module.positive_rotation_ok([sample(module, 0.0), sample(module, 2.0, yaw=-0.4)])
    assert not module.positive_rotation_ok([sample(module, 0.0), sample(module, 2.0, yaw=0.03)])


def test_stop_window_requires_near_zero_twist():
    module = load_module()
    stopped = [sample(module, 0.0, linear=0.01, angular=0.01), sample(module, 1.0, linear=0.0)]
    settling_then_stopped = [
        sample(module, 0.0, linear=0.18, angular=0.16),
        sample(module, 0.4, linear=0.07, angular=0.06),
        sample(module, 0.8, linear=0.01, angular=0.01),
        sample(module, 1.2, linear=0.0, angular=0.0),
    ]
    moving = [sample(module, 0.0, linear=0.08), sample(module, 1.0, linear=0.06)]
    assert module.stop_window_ok(stopped)
    assert module.stop_window_ok(settling_then_stopped, settling_seconds=0.75)
    assert not module.stop_window_ok(moving)


def test_speed_ceiling_rejects_overspeed():
    module = load_module()
    compliant = [sample(module, 0.0, linear=0.95), sample(module, 1.0, linear=1.01)]
    overspeed = [sample(module, 0.0, linear=0.9), sample(module, 1.0, linear=1.12)]
    assert module.speed_ceiling_ok(compliant, maximum=1.0, tolerance=0.03)
    assert not module.speed_ceiling_ok(overspeed, maximum=1.0, tolerance=0.03)
