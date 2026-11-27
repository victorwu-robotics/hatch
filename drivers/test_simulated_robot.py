"""
Regression tests for SimulatedRobot.move_pose frame handling.

The bug these lock down: the Cartesian panel commands poses in the robot's
base frame, `move_pose` accepted a `frame` argument and ignored it, and the
local solver works in world. On an arm whose base is offset from world — a
mounting plate, a vehicle — the first pose jog after switching from joint
control swung the shoulder by the base yaw and moved the TCP about two metres.
Later jogs looked normal because the stored target had become numerically
world-frame, so the error was consumed once and then hid.

An unmounted arm cannot expose this: base and world coincide. Every test here
therefore runs on mounted arms as well.

Pure Python: no Qt, no hardware, no network, no xacro, no package path.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.ik_solver import IKSolver
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from drivers.simulated_robot import SimulatedRobot

UR10 = (Path(__file__).resolve().parents[1]
        / "assets" / "robots" / "ur_description" / "urdf" / "ur10_nominal.urdf")

# (xyz, rpy) of a fixed joint placing the arm's base_link in the world.
# "yawed" is the real bunker_with_arm mount: 0.1235 m and pi about Z.
MOUNTS = {
    "unmounted": None,
    "translated": ((0.1235, 0.0, 0.015), (0.0, 0.0, 0.0)),
    "yawed": ((0.1235, 0.0, 0.015), (0.0, 0.0, math.pi)),
    "tilted": ((0.4, -0.2, 0.3), (0.0, 0.349, 0.7)),
}

Q0 = np.array([0.3, -0.9, 1.1, -0.4, 0.7, 0.2])


def _build(mount, tmp_path):
    text = UR10.read_text()
    if mount is not None:
        (x, y, z), (r, p, yw) = mount
        text = text.replace(
            "</robot>",
            '<link name="mount"/>'
            '<joint name="mount_to_arm" type="fixed">'
            f'<origin xyz="{x} {y} {z}" rpy="{r} {p} {yw}"/>'
            '<parent link="mount"/><child link="base_link"/></joint>'
            "</robot>")
    path = tmp_path / "arm.urdf"
    path.write_text(text)
    model = KinematicModel(urdf_path=str(path))
    model.set_ik_solver(IKSolver(model))
    return model


@pytest.fixture(params=sorted(MOUNTS))
def rig(request, tmp_path):
    model = _build(MOUNTS[request.param], tmp_path)
    robot = SimulatedRobot(model, StateChannel())
    robot.move_joints(Q0.tolist())
    return robot, model, request.param


def _tcp(model, q):
    model.update_state(np.asarray(q, dtype=float))
    return model.get_tcp_pose().copy()


def _base_target(model, q, delta_axis=None, delta=0.0):
    """The pose the Cartesian panel publishes: current TCP in the base frame."""
    links = model.compute_fk(q)
    T_world_base = links[model.get_true_base()]
    target = np.linalg.inv(T_world_base) @ _tcp(model, q)
    if delta_axis is not None:
        target[delta_axis, 3] += delta
    return target


def _shoulder_change(q_before, q_after):
    d = q_after[0] - q_before[0]
    return abs(math.degrees(math.atan2(math.sin(d), math.cos(d))))


def test_zero_delta_jog_does_not_move(rig):
    """Asking for no motion must produce no motion, whatever the mount."""
    robot, model, mount = rig
    before = _tcp(model, Q0)
    target = _base_target(model, Q0)

    assert robot.move_pose(target, frame="base") is True

    after = _tcp(model, robot.get_state()["joint_positions"])
    moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    assert moved < 1e-6, f"{mount}: TCP moved {moved * 1000:.3f} mm on a zero-delta jog"
    assert _shoulder_change(Q0, np.asarray(robot.get_state()["joint_positions"])) < 0.01


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("delta", [0.001, 0.010])
def test_base_jog_moves_exactly_the_requested_delta(rig, axis, delta):
    """The TCP must travel the requested distance — not the base transform."""
    robot, model, mount = rig
    before = _tcp(model, Q0)
    target = _base_target(model, Q0, delta_axis=axis, delta=delta)

    assert robot.move_pose(target, frame="base") is True

    q_after = np.asarray(robot.get_state()["joint_positions"])
    after = _tcp(model, q_after)
    moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    assert moved == pytest.approx(delta, abs=1e-6), (
        f"{mount}: asked {delta * 1000:.1f} mm, moved {moved * 1000:.3f} mm")
    # A small Cartesian step must not flip the shoulder branch.
    assert _shoulder_change(Q0, q_after) < 1.0


def test_world_frame_is_not_transformed(rig):
    """frame='world' means the pose is already world; it must pass through."""
    robot, model, mount = rig
    before = _tcp(model, Q0)
    target = before.copy()
    target[0, 3] += 0.005

    assert robot.move_pose(target, frame="world") is True

    after = _tcp(model, robot.get_state()["joint_positions"])
    moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    assert moved == pytest.approx(0.005, abs=1e-6)


def test_base_and_world_agree_only_when_unmounted(rig):
    """The two frames coincide exactly when the base sits at the world origin."""
    robot, model, mount = rig
    links = model.compute_fk(Q0)
    T_world_base = links[model.get_true_base()]
    offset = float(np.linalg.norm(T_world_base[:3, 3]))
    yaw = abs(math.atan2(T_world_base[1, 0], T_world_base[0, 0]))
    if mount == "unmounted":
        assert offset < 1e-12 and yaw < 1e-12
    else:
        assert offset > 1e-6 or yaw > 1e-6


def test_unknown_frame_is_refused(rig):
    """A pose in an unnamed frame cannot be interpreted; refuse, do not assume."""
    robot, model, _ = rig
    target = _base_target(model, Q0)
    with pytest.raises(ValueError, match="unknown frame"):
        robot.move_pose(target, frame="tool")


def test_repeated_jogs_stay_continuous(rig):
    """After the first jog, further jogs must remain small and cumulative."""
    robot, model, mount = rig
    q = Q0.copy()
    travelled = 0.0
    for _ in range(5):
        before = _tcp(model, q)
        target = _base_target(model, q, delta_axis=0, delta=0.001)
        assert robot.move_pose(target, frame="base") is True
        q = np.asarray(robot.get_state()["joint_positions"])
        after = _tcp(model, q)
        step = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
        assert step == pytest.approx(0.001, abs=1e-6), f"{mount}: step {step * 1000:.3f} mm"
        travelled += step
    assert travelled == pytest.approx(0.005, abs=1e-5)


def test_unreachable_target_is_reported_not_raised(rig):
    """
    An unreachable pose must return False and publish an error, not raise.

    Added after a measured regression: a hand-applied edit dropped
    `ik_source = "local IK"` from the success path. Every other test here uses
    a reachable target, so all 44 passed while an out-of-reach command raised
    UnboundLocalError instead of reporting a refusal. StateChannel catches
    subscriber exceptions, so in the GUI that would have been silent: no
    motion, no refusal, only a log line.

    The solver documents None as a real answer for an unreachable pose, so
    this path is normal operation, not an edge case.
    """
    robot, model, mount = rig
    errors = []
    robot._channel.subscribe(EventType.ERROR_OCCURRED, lambda e: errors.append(e))

    before = np.asarray(robot.get_state()["joint_positions"]).copy()
    target = _base_target(model, Q0)
    target[0, 3] += 5.0          # UR10 reach is L2+L3+|d6| = 1.2765 m

    assert robot.move_pose(target, frame="base") is False, mount
    assert len(errors) == 1, f"{mount}: expected one ERROR_OCCURRED, got {len(errors)}"
    assert "IK failed" in errors[0].data["error"]

    after = np.asarray(robot.get_state()["joint_positions"])
    assert np.array_equal(before, after), f"{mount}: a refused move changed state"
