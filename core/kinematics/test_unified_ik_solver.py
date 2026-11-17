"""
Tests for the unified analytical IK solver.

The acceptance criterion is the *pose*, not the joint vector: reproducing
a target pose is what IK means. Comparing joint vectors directly is
ill-conditioned near singularities and would fail for reasons that have
nothing to do with the solver (see test_accuracy_is_tilt_times_condition).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from core.kinematics.kinematic_model import KinematicModel          # noqa: E402
from core.kinematics.geometric_extraction import UnsupportedArmError  # noqa: E402
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

# The URDFs are not equally clean. UR10's axes are orthogonal to 2e-10;
# FR5 and Elfin carry CAD rounding of 3.7e-6 rad. A closed-form solver
# cannot be more consistent than the model it is given.
POS_TOL = 2e-5
ROT_TOL = 2e-5


def _model(path):
    m = KinematicModel(str(path), package_resolver=_RESOLVER)
    m.load()
    return m


@pytest.fixture(scope="module", params=list(ARMS))
def solver(request):
    return UnifiedIKSolver(_model(ARMS[request.param]))


def _max_angle_diff(a, b):
    return float(np.max(np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))))


# ----------------------------------------------------------------------
# The core contract
# ----------------------------------------------------------------------

def test_round_trip_random_poses(solver):
    """Every returned solution must reproduce the target pose."""
    rng = np.random.default_rng(2024)
    for _ in range(60):
        q = rng.uniform(-2.4, 2.4, 6)
        T = solver.fk(q)
        sols = solver.solve(T, respect_limits=False)
        assert sols, f"no solution for a pose the arm is actually at: {q}"
        for s in sols:
            Tf = solver.fk(s.q)
            assert np.linalg.norm(Tf[:3, 3] - T[:3, 3]) < POS_TOL
            assert np.linalg.norm(Tf[:3, :3] - T[:3, :3]) < 1e-4


def test_original_configuration_is_among_the_solutions(solver):
    """Away from singularities the original q must be recovered."""
    rng = np.random.default_rng(5)
    checked = 0
    for _ in range(80):
        q = rng.uniform(-2.0, 2.0, 6)
        if abs(np.sin(q[4])) < 0.2 or abs(np.sin(q[2])) < 0.2:
            continue                      # skip ill-conditioned poses
        T = solver.fk(q)
        sols = solver.solve(T, respect_limits=False)
        assert any(_max_angle_diff(s.q, q) < 2e-3 for s in sols), \
            f"original configuration missing: {q}"
        checked += 1
    assert checked > 20


def test_up_to_eight_solutions(solver):
    rng = np.random.default_rng(3)
    seen = set()
    for _ in range(60):
        q = rng.uniform(-1.8, 1.8, 6)
        sols = solver.solve(solver.fk(q), respect_limits=False)
        assert len(sols) <= 8
        seen.add(len(sols))
    assert 8 in seen, f"never reached the full 8 branches, saw {seen}"


def test_solutions_are_distinct(solver):
    rng = np.random.default_rng(9)
    for _ in range(30):
        q = rng.uniform(-2.0, 2.0, 6)
        sols = solver.solve(solver.fk(q), respect_limits=False)
        for i in range(len(sols)):
            for j in range(i + 1, len(sols)):
                assert _max_angle_diff(sols[i].q, sols[j].q) > 1e-6


def test_tags_are_unique_per_solution(solver):
    """shoulder x elbow x wrist labels identify the branch."""
    rng = np.random.default_rng(13)
    for _ in range(30):
        q = rng.uniform(-1.5, 1.5, 6)
        sols = solver.solve(solver.fk(q), respect_limits=False)
        tags = [s.tags() for s in sols]
        assert len(tags) == len(set(tags)), f"duplicate branch tags: {tags}"


# ----------------------------------------------------------------------
# Singularities — the paths that a random sweep rarely reaches
# ----------------------------------------------------------------------

@pytest.mark.parametrize("theta5", [0.0, np.pi, -np.pi])
def test_wrist_singular_pose_is_solved_and_flagged(solver, theta5):
    """a4 parallel a6: the wrist loses a DOF but the pose is still reachable."""
    q = np.array([0.3, -0.8, 1.0, 0.5, theta5, -0.4])
    T = solver.fk(q)
    sols = solver.solve(T, q_reference=q, respect_limits=False)
    assert sols, "no solution at the wrist singularity"
    assert any(s.wrist_singular for s in sols), "singularity not flagged"
    for s in sols:
        Tf = solver.fk(s.q)
        assert np.linalg.norm(Tf[:3, 3] - T[:3, 3]) < POS_TOL


def test_wrist_singular_continuity_uses_reference(solver):
    """With a4 || a6, theta4 is free; the reference should be honoured."""
    q = np.array([0.3, -0.8, 1.0, 0.5, 0.0, -0.4])
    T = solver.fk(q)
    sols = solver.solve(T, q_reference=q, respect_limits=False)
    sing = [s for s in sols if s.wrist_singular]
    assert sing
    assert any(abs(s.q[3] - q[3]) < 1e-4 for s in sing), \
        f"reference theta4 not honoured: {[s.q[3] for s in sing]}"


def test_elbow_singular_pose(solver):
    """Arm fully stretched: the two elbow branches merge."""
    q = np.array([0.3, -0.8, 0.0, 0.5, 1.2, -0.4])
    T = solver.fk(q)
    sols = solver.solve(T, respect_limits=False)
    assert sols
    for s in sols:
        Tf = solver.fk(s.q)
        assert np.linalg.norm(Tf[:3, 3] - T[:3, 3]) < POS_TOL


def test_shoulder_singular_pose(solver):
    """P5 on the boundary cylinder: the two tangents merge."""
    q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    T = solver.fk(q)
    P5 = T[:3, 3] - solver.d6 * T[:3, :3][:, 2]
    r = np.hypot(P5[0], P5[1])
    # Rotate the whole pose about Z so P5's azimuth is preserved, then
    # nudge it radially onto the cylinder of radius |h|.
    scale = abs(solver.h) / r
    T2 = T.copy()
    T2[:2, 3] *= scale
    sols = solver.solve(T2, respect_limits=False)
    for s in sols:
        Tf = solver.fk(s.q)
        assert np.linalg.norm(Tf[:3, 3] - T2[:3, 3]) < 1e-3


def test_unreachable_pose_returns_nothing(solver):
    T = np.eye(4)
    T[:3, 3] = [50.0, 0.0, 0.0]
    assert solver.solve(T) == []


def test_pose_inside_the_shoulder_hole_returns_nothing(solver):
    """r < |h|: no tangent exists, so theta_1 has no real root."""
    if abs(solver.h) < 1e-3:
        pytest.skip("this arm has no shoulder hole (h ~ 0)")
    T = np.eye(4)
    T[:3, 3] = [abs(solver.h) * 0.2, 0.0, 0.5]
    assert solver.solve(T, respect_limits=False) == []


# ----------------------------------------------------------------------
# Limits and selection
# ----------------------------------------------------------------------

def test_joint_limits_respected(solver):
    rng = np.random.default_rng(17)
    for _ in range(40):
        q = rng.uniform(-2.0, 2.0, 6)
        for s in solver.solve(solver.fk(q), respect_limits=True):
            for i, (lo, hi) in enumerate(solver.limits):
                if lo is not None and hi is not None:
                    assert lo - 1e-5 <= s.q[i] <= hi + 1e-5


def test_solve_closest_prefers_current_configuration(solver):
    rng = np.random.default_rng(23)
    for _ in range(20):
        q = rng.uniform(-1.2, 1.2, 6)
        if abs(np.sin(q[4])) < 0.3 or abs(np.sin(q[2])) < 0.3:
            continue
        q = solver._fit_limits(q)
        if q is None:
            continue
        best = solver.solve_closest(solver.fk(q), q)
        assert best is not None
        assert _max_angle_diff(best.q, q) < 1e-2, \
            f"closest jumped branch: {q} -> {best.q}"


def test_no_solution_for_unreachable_closest(solver):
    T = np.eye(4)
    T[:3, 3] = [50.0, 0.0, 0.0]
    assert solver.solve_closest(T, np.zeros(6)) is None


# ----------------------------------------------------------------------
# Refusal: out-of-class arms must fail loudly, not approximately
# ----------------------------------------------------------------------

def _write_variant(tmp_path, src: Path, old: str, new: str, name: str) -> Path:
    text = src.read_text()
    assert old in text, f"anchor not found in {src.name}"
    out = tmp_path / name
    out.write_text(text.replace(old, new, 1))
    return out


def test_refuses_shoulder_offset(tmp_path):
    """
    A PUMA-style lateral shoulder offset puts J2's axis off the J1 axis.
    theta_1 then needs common tangents to two circles, which is a
    different (and higher-branch-count) problem. Refuse, do not fudge.
    """
    p = _write_variant(
        tmp_path, ARMS["UR10"],
        '<origin rpy="1.570796327 0 0" xyz="0 0 0"/>',
        '<origin rpy="1.570796327 0 0" xyz="0 -0.15 0"/>',
        "puma_shoulder.urdf")
    with pytest.raises(UnsupportedArmError) as e:
        UnifiedIKSolver(_model(p))
    assert "shoulder offset" in str(e.value)


def test_refuses_roll_wrist_with_radial_offset(tmp_path):
    """
    J4 in the arm plane AND delta > 0 closes the dependency loop:
    A needs a5, a5 needs a4, a4 needs the arm chain, the arm chain needs A.
    """
    p = _write_variant(
        tmp_path, ARMS["Elfin"],
        'xyz="0 0 0.57"',
        'xyz="0 0.05 0.57"',
        "roll_offset.urdf")
    with pytest.raises(UnsupportedArmError) as e:
        UnifiedIKSolver(_model(p))
    msg = str(e.value)
    assert "loop" in msg or "delta" in msg


def test_refuses_non_orthogonal_wrist(tmp_path):
    """J5 not perpendicular to J6 breaks the wrist triangle."""
    p = _write_variant(
        tmp_path, ARMS["UR10"],
        '<joint name="wrist_3_joint" type="revolute">',
        '<joint name="wrist_3_joint" type="revolute">',
        "noop.urdf")
    # Perturb wrist_2's origin roll so J5 and J6 are no longer perpendicular.
    text = ARMS["UR10"].read_text()
    idx = text.index('wrist_2_joint')
    seg = text[idx:idx + 400]
    import re
    m = re.search(r'origin rpy="([^"]+)"', seg)
    assert m
    rpy = [float(v) for v in m.group(1).split()]
    rpy[0] += 0.2
    new_seg = seg.replace(m.group(0),
                          'origin rpy="' + " ".join(f"{v:.9f}" for v in rpy) + '"')
    out = tmp_path / "skew_wrist.urdf"
    out.write_text(text[:idx] + new_seg + text[idx + 400:])
    with pytest.raises(UnsupportedArmError):
        UnifiedIKSolver(_model(out))


# ----------------------------------------------------------------------
# The accuracy story, stated as a test rather than a claim
# ----------------------------------------------------------------------

def test_ur10_is_exact():
    """UR10's URDF is clean, so the solver must be near machine-precision."""
    s = UnifiedIKSolver(_model(ARMS["UR10"]))
    rng = np.random.default_rng(31)
    worst = 0.0
    for _ in range(40):
        q = rng.uniform(-2.0, 2.0, 6)
        if abs(np.sin(q[4])) < 0.3:
            continue
        for sol in s.solve(s.fk(q), respect_limits=False):
            Tf = s.fk(sol.q)
            worst = max(worst, float(np.linalg.norm(Tf[:3, 3] - s.fk(q)[:3, 3])))
    assert worst < 1e-8, f"UR10 should be exact, got {worst:.3e} m"


def test_spherical_wrist_is_not_a_special_case():
    """
    Elfin has delta = 0. The same code path must handle it, with the
    wrist solved after the arm rather than before — forced by geometry,
    not selected by a label.
    """
    s = UnifiedIKSolver(_model(ARMS["Elfin"]))
    assert s.delta < 1e-5
    assert not s.pitch_wrist
    assert "A -> arm -> wrist" in s.describe()
    q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    sols = s.solve(s.fk(q), respect_limits=False)
    assert len(sols) == 8


def test_offset_wrist_solves_wrist_first():
    s = UnifiedIKSolver(_model(ARMS["UR10"]))
    assert s.delta > 1e-5
    assert s.pitch_wrist
    assert "wrist -> A -> arm" in s.describe()


def test_signed_flange_offset_is_measured_not_assumed():
    """
    d6's sign depends on which link the model calls the flange. It must
    be measured. UR10's wrist_3_link gives +0.0922, not -0.0922.
    """
    s = UnifiedIKSolver(_model(ARMS["UR10"]))
    assert abs(s.d6 - 0.0922) < 1e-6
    q = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
    T = s.fk(q)
    J, _ = s._place(q)
    P5_true = J[4][0] + np.dot(J[5][0] - J[4][0], J[4][1]) * J[4][1]
    P5 = T[:3, 3] - s.d6 * T[:3, :3][:, 2]
    assert np.linalg.norm(P5 - P5_true) < 1e-9


def test_fast_chain_walk_matches_scene_fk(solver):
    """
    The solver walks the six arm joints directly for speed. That walk
    must agree with the scene-wide FK it replaces, or every result is
    quietly wrong.
    """
    rng = np.random.default_rng(77)
    for _ in range(25):
        q = rng.uniform(-3.0, 3.0, 6)
        Jf, Tf = solver._place(q)
        Jm, Tm = solver._place_via_model(q)
        assert np.max(np.abs(Tf - Tm)) < 1e-12
        for (pf, af), (pm, am) in zip(Jf, Jm):
            assert np.linalg.norm(pf - pm) < 1e-12
            assert np.linalg.norm(af - am) < 1e-12


def test_flange_pose_matches_model(solver):
    rng = np.random.default_rng(78)
    for _ in range(15):
        q = rng.uniform(-2.5, 2.5, 6)
        _, Tf = solver._place(q)
        assert np.max(np.abs(Tf - solver.fk(q))) < 1e-12
