"""
Tests for the TCP/world adapter over the unified analytical solver.

Several of these are regression tests for measured defects in the
dispatcher-and-numerical-fallback design this module replaced:

* an unreachable target returned a joint vector instead of None
* an Elfin solve returned q3 = 57.55 rad against a +/-2.61 limit
* the wrist "detection" read a link length, not a wrist offset
* both analytical branches raised TypeError and were silently swallowed
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from core.kinematics.geometric_extraction import UnsupportedArmError  # noqa: E402
from core.kinematics.ik_solver import IKSolver                       # noqa: E402
from core.kinematics.kinematic_model import KinematicModel           # noqa: E402
from core.kinematics.unified_ik_solver import UnifiedIKSolver        # noqa: E402

try:
    from utils.package_resolver import PackageResolver
    _RESOLVER = PackageResolver()
except Exception:                                    # pragma: no cover
    _RESOLVER = None

ROOT = Path(__file__).resolve().parents[2]
ARMS = {
    "UR10": ROOT / "assets/robots/ur_description/urdf/ur10_nominal.urdf",
    "FR5": ROOT / "assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf",
    "Elfin": ROOT / "assets/robots/E15_Pro/urdf/E15_Pro.urdf",
}


def _model(path):
    m = KinematicModel(str(path), package_resolver=_RESOLVER)
    m.load()
    return m


@pytest.fixture(params=list(ARMS))
def rigged(request):
    """A loaded model with the adapter attached, plus a test configuration."""
    m = _model(ARMS[request.param])
    ik = IKSolver(m)
    q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    m.update_state(q)
    return m, ik, q


def _pose_err(T1, T2):
    pos = float(np.linalg.norm(T1[:3, 3] - T2[:3, 3]))
    c = (float(np.trace(T1[:3, :3].T @ T2[:3, :3])) - 1.0) / 2.0
    return pos, float(np.arccos(np.clip(c, -1.0, 1.0)))


# ----------------------------------------------------------------------
# It really is the analytical solver, on every arm
# ----------------------------------------------------------------------

def test_uses_unified_solver_not_numerical(rigged):
    _, ik, _ = rigged
    assert isinstance(ik.unified, UnifiedIKSolver)
    assert type(ik.unified).__name__ == "UnifiedIKSolver"


def test_no_numerical_solver_is_imported():
    """The fallback must be gone, not merely unused by luck."""
    import core.kinematics.ik_solver as mod
    assert not hasattr(mod, "NumericalIKSolver")
    src = Path(mod.__file__).read_text()
    code = src.split('"""')[2]          # everything after the module docstring
    assert "numerical_ik_solver" not in code
    assert "NumericalIKSolver" not in code


def test_all_three_arms_attach():
    """The old dispatcher fell back to numerical on all three."""
    for name, path in ARMS.items():
        ik = IKSolver(_model(path))
        assert isinstance(ik.unified, UnifiedIKSolver), name


def test_no_wrist_type_label():
    """The taxonomy is a distance that may be zero, not two named types."""
    ik = IKSolver(_model(ARMS["UR10"]))
    assert not hasattr(ik, "wrist_type")


# ----------------------------------------------------------------------
# The TCP <-> flange conversion
# ----------------------------------------------------------------------

def test_tcp_round_trip_at_current_pose(rigged):
    m, ik, q = rigged
    T = m.get_tcp_pose()
    q_out = ik.solve_ik_for_tcp(T, q)
    assert q_out is not None
    pos, rot = _pose_err(m.forward_kinematics(q_out), T)
    assert pos < 1e-5 and rot < 1e-5


def test_tcp_round_trip_random_poses(rigged):
    m, ik, _ = rigged
    rng = np.random.default_rng(41)
    for _ in range(40):
        q = rng.uniform(-2.0, 2.0, 6)
        T = m.forward_kinematics(q)
        q_out = ik.solve_ik_for_tcp(T, q)
        assert q_out is not None, f"failed at a reachable pose {q}"
        pos, rot = _pose_err(m.forward_kinematics(q_out), T)
        assert pos < 1e-4 and rot < 1e-4


def test_conversion_is_invertible(rigged):
    m, ik, q = rigged
    T_tcp = m.forward_kinematics(q)
    T_fl = ik.tcp_to_flange(T_tcp, q)
    assert np.max(np.abs(T_fl - m.get_flange_pose(q))) < 1e-12
    assert np.max(np.abs(ik.flange_to_tcp(T_fl, q) - T_tcp)) < 1e-12


def test_forward_kinematics_matches_model(rigged):
    m, ik, _ = rigged
    rng = np.random.default_rng(42)
    for _ in range(10):
        q = rng.uniform(-2.5, 2.5, 6)
        assert np.max(np.abs(ik.forward_kinematics(q)
                             - m.forward_kinematics(q))) < 1e-12


def test_nontrivial_tool_transform_is_respected(rigged):
    """
    A bent torch puts the TCP off J6's axis. The tool must be removed
    before solving; if it is not, the answer is wrong by the tool offset.
    """
    m, ik, q = rigged
    tool = np.eye(4)
    tool[:3, 3] = [0.03, -0.05, 0.17]
    th = 0.4
    tool[:3, :3] = [[np.cos(th), 0, np.sin(th)],
                    [0, 1, 0],
                    [-np.sin(th), 0, np.cos(th)]]
    m.set_tool_transform(tool)
    m.update_state(q)
    T = m.get_tcp_pose()
    q_out = ik.solve_ik_for_tcp(T, q)
    assert q_out is not None
    pos, rot = _pose_err(m.forward_kinematics(q_out), T)
    assert pos < 1e-4 and rot < 1e-4


@pytest.mark.parametrize("name", list(ARMS))
def test_world_offset_is_respected(name, tmp_path):
    """
    The arm need not sit at the world origin. Mount it on a pedestal via
    a fixed joint above the file root, which is how a real cell is built.
    """
    m0 = _model(ARMS[name])
    file_root = m0.root_links[0]
    text = ARMS[name].read_text()
    pedestal = (
        '<link name="pedestal"/>'
        '<joint name="pedestal_joint" type="fixed">'
        '<parent link="pedestal"/>'
        f'<child link="{file_root}"/>'
        '<origin xyz="1.2 -0.7 0.35" rpy="0 0 0.6"/>'
        '</joint>'
    )
    p = tmp_path / "on_pedestal.urdf"
    p.write_text(text.replace("</robot>", pedestal + "</robot>"))

    m = _model(p)
    ik = IKSolver(m)
    q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    m.update_state(q)
    T = m.get_tcp_pose()
    assert np.linalg.norm(T[:3, 3] - m.get_flange_pose(q)[:3, 3]) > 0.5
    q_out = ik.solve_ik_for_tcp(T, q)
    assert q_out is not None
    pos, rot = _pose_err(m.forward_kinematics(q_out), T)
    assert pos < 1e-4 and rot < 1e-4


# ----------------------------------------------------------------------
# Regression: the numerical fallback's two measured defects
# ----------------------------------------------------------------------

def test_unreachable_target_returns_none(rigged):
    """The numerical solver returned a q with 3.01 m error, not None."""
    m, ik, q = rigged
    T = np.eye(4)
    T[:3, 3] = [3.0, 0.0, 0.5]
    assert ik.solve_ik_for_tcp(T, q) is None
    assert ik.solve_all_for_tcp(T, q) == []


def test_returned_joints_are_within_limits(rigged):
    """The numerical solver returned q3 = 57.55 rad on a +/-2.61 joint."""
    m, ik, _ = rigged
    limits = ik.unified.limits
    rng = np.random.default_rng(43)
    for _ in range(30):
        q = rng.uniform(-2.0, 2.0, 6)
        T = m.forward_kinematics(q)
        for seed in (np.zeros(6), q):
            q_out = ik.solve_ik_for_tcp(T, seed)
            if q_out is None:
                continue
            q_arm = ik._to_arm(q_out)
            for v, (lo, hi) in zip(q_arm, limits):
                if lo is not None and hi is not None:
                    assert lo - 1e-5 <= v <= hi + 1e-5, \
                        f"joint {v} outside [{lo}, {hi}]"
            assert np.max(np.abs(q_arm)) <= 2 * np.pi


def test_zero_seed_does_not_produce_unwrapped_angles(rigged):
    """Specifically the Elfin-from-zeros case."""
    m, ik, q = rigged
    T = m.forward_kinematics(q)
    q_out = ik.solve_ik_for_tcp(T, np.zeros(6))
    assert q_out is not None
    assert np.max(np.abs(ik._to_arm(q_out))) < 2 * np.pi


# ----------------------------------------------------------------------
# Purity and ordering
# ----------------------------------------------------------------------

def test_solve_does_not_mutate_model(rigged):
    m, ik, q = rigged
    m.update_state(q)
    before = m.get_current_joint_positions().copy()
    T_before = m.get_tcp_pose().copy()
    ik.solve_ik_for_tcp(m.forward_kinematics(q + 0.2), np.zeros(6))
    assert np.allclose(m.get_current_joint_positions(), before)
    assert np.allclose(m.get_tcp_pose(), T_before)


def test_joint_order_mapping_is_identity_on_bare_arms(rigged):
    _, ik, _ = rigged
    assert np.array_equal(ik._arm_idx, np.arange(6))


def test_joint_order_mapping_with_extra_actuated_joint():
    """
    A scene joint outside the arm chain must not shift the arm indices,
    and its value must survive the round trip through the solver.
    """
    m = _model(ARMS["UR10"])
    ik = IKSolver(m)
    # Simulate an extra actuated joint ahead of the arm in document order.
    ik._arm_idx = np.arange(1, 7)
    ik._n_full = 7
    q_full = np.array([0.123, 0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    assert np.allclose(ik._to_arm(q_full), q_full[1:])
    back = ik._from_arm(np.zeros(6), q_full)
    assert back[0] == 0.123           # non-arm joint preserved
    assert np.allclose(back[1:], 0.0)


def test_q_guess_defaults_to_model_state(rigged):
    m, ik, q = rigged
    m.update_state(q)
    T = m.get_tcp_pose()
    a = ik.solve_ik_for_tcp(T)
    b = ik.solve_ik_for_tcp(T, q)
    assert a is not None and b is not None
    assert np.max(np.abs(a - b)) < 1e-9


# ----------------------------------------------------------------------
# Branches and continuity
# ----------------------------------------------------------------------

def test_solve_all_exposes_branches(rigged):
    m, ik, q = rigged
    sols = ik.solve_all_for_tcp(m.forward_kinematics(q), q)
    assert len(sols) >= 2
    tags = [s.tags() for _, s in sols]
    assert len(tags) == len(set(tags))
    for q_full, s in sols:
        pos, rot = _pose_err(m.forward_kinematics(q_full), m.forward_kinematics(q))
        assert pos < 1e-4 and rot < 1e-4


def test_nearest_branch_is_first_and_preserves_configuration(rigged):
    m, ik, _ = rigged
    rng = np.random.default_rng(44)
    for _ in range(20):
        q = rng.uniform(-1.2, 1.2, 6)
        if abs(np.sin(q[4])) < 0.3 or abs(np.sin(q[2])) < 0.3:
            continue
        if ik.unified._fit_limits(q) is None:
            continue
        sols = ik.solve_all_for_tcp(m.forward_kinematics(q), q)
        assert sols
        q_best = ik._to_arm(sols[0][0])
        d = np.arctan2(np.sin(q_best - q), np.cos(q_best - q))
        assert np.max(np.abs(d)) < 1e-2, "nearest branch changed configuration"


def test_diagnostics_reported(rigged):
    m, ik, q = rigged
    ik.solve_ik_for_tcp(m.forward_kinematics(q), q)
    d = ik.diagnostics
    assert d["n_solutions"] >= 1
    assert d["worst_pos_error"] < 1e-4
    assert d["chosen_tags"] is not None


# ----------------------------------------------------------------------
# Refusal, with no substitute
# ----------------------------------------------------------------------

def test_refuses_out_of_class_arm_without_fallback(tmp_path):
    text = ARMS["UR10"].read_text()
    old = '<origin rpy="1.570796327 0 0" xyz="0 0 0"/>'
    assert old in text
    p = tmp_path / "puma.urdf"
    p.write_text(text.replace(old, '<origin rpy="1.570796327 0 0" xyz="0 -0.15 0"/>', 1))
    with pytest.raises(UnsupportedArmError):
        IKSolver(_model(p))


def test_requires_a_model():
    with pytest.raises(ValueError):
        IKSolver(None)


def test_refuses_when_tool_mount_is_not_flange():
    """
    The conversion composes inv(tool) onto a TCP pose. That only yields
    the flange pose if the tool is mounted on the flange. Pin the flange,
    then move the mount, so the mismatch is real.
    """
    m = _model(ARMS["UR10"])
    m._flange_link = m.get_flange()          # pin flange to wrist_3_link
    m.tool_mount_link = "tool0"              # a different link downstream
    assert m.get_flange() != m.tool_mount_link
    with pytest.raises(ValueError, match="tool mount"):
        IKSolver(m)


# ----------------------------------------------------------------------
# Integration: a Cartesian jog, the path the UI actually drives
# ----------------------------------------------------------------------

def test_cartesian_jog_keeps_configuration_and_limits(rigged):
    """
    Drag the TCP 0.5 mm at a time along world X for 200 steps. Every step
    must solve, stay in limits, and keep the same configuration — this is
    the behaviour that stops the arm sweeping through the workpiece.
    """
    m, ik, q = rigged
    limits = ik.unified.limits
    q = q.copy()
    m.update_state(q)
    tags0 = None
    for _ in range(200):
        T = m.forward_kinematics(q)
        T[0, 3] += 0.0005
        q_new = ik.solve_ik_for_tcp(T, q)
        assert q_new is not None, "jog step failed"
        q_arm = ik._to_arm(q_new)
        for v, (lo, hi) in zip(q_arm, limits):
            if lo is not None and hi is not None:
                assert lo - 1e-5 <= v <= hi + 1e-5
        tags = ik.diagnostics["chosen_tags"]
        if tags0 is None:
            tags0 = tags
        assert tags == tags0, f"configuration changed mid-jog: {tags0} -> {tags}"
        # no single step may jump
        d = np.arctan2(np.sin(q_arm - q), np.cos(q_arm - q))
        assert np.max(np.abs(d)) < 0.05, f"joint jump of {np.max(np.abs(d))} rad"
        q = q_new
        m.update_state(q)


def test_availability_probe_succeeds_on_every_arm():
    """What ui/panels/cartesian_control_panel._check_ik_available does."""
    for name, path in ARMS.items():
        m = _model(path)
        m.set_ik_solver(IKSolver(m))
        q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
        m.update_state(q)
        assert m.solve_ik_for_tcp(
            m.get_tcp_pose(), m.get_current_joint_positions()) is not None, name
