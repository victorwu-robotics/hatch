"""Does the panel's route to 'base' agree with the driver's route back to world?

The regression tests compute the base-frame target as inv(compute_fk(q)[true_base]).
The real panel does NOT do that: it asks TransformRegistry.transform_frame_pose,
using the frame name robot_manager stored as f"{asset_id}_{true_base}".

Those are two different code paths. If they disagree, the fix round-trips
imperfectly and the tests would not see it. This measures the real path,
end to end, on the real scene, with no Qt and no hardware.

Run from the repo root (needs vtk, which the GUI already requires):
      python measure_panel_driver_roundtrip.py
"""
import sys, math, tempfile, pathlib
sys.path.insert(0, '.')
import numpy as np

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.ik_solver import IKSolver
from core.world_state.state_channel import StateChannel
from core.world_state.transform_registry import TransformRegistry
from core.state_handler import StateHandler
from core.robot_manager import RobotManager
from drivers.simulated_robot import SimulatedRobot
from utils.xacro_expander import XacroExpander
from utils.package_resolver import PackageResolver

TMP = pathlib.Path(tempfile.mkdtemp(prefix="hatch-roundtrip-"))
SCENES = {
    "bunker_with_arm": "assets/scenes/bunker_with_arm/bunker_with_arm.urdf.xacro",
    "my_arm": "assets/scenes/my_arm.urdf.xacro",
}
Q0 = np.array([0.3, -0.9, 1.1, -0.4, 0.7, 0.2])


def rot_deg(A, B):
    """Relative rotation angle, via rotvec norm — well conditioned near 0."""
    R = A[:3, :3].T @ B[:3, :3]
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    # use the antisymmetric part for small angles, arccos for large
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(w) / 2.0
    return math.degrees(math.atan2(s, tr))


def load(scene_path):
    text = XacroExpander(PackageResolver()).expand(scene_path)
    p = TMP / (pathlib.Path(scene_path).stem + ".urdf")
    p.write_text(text)
    m = KinematicModel(urdf_path=str(p))
    m.set_ik_solver(IKSolver(m))
    return m


print("\n## P1 — panel route vs inverse-FK route to the base frame\n")
print("| scene | true_base | registry frame | position disagreement | angle disagreement |")
print("|---|---|---|---|---|")

rigs = {}
for name, path in SCENES.items():
    model = load(path)
    model.update_state(Q0)
    channel = StateChannel()
    registry = TransformRegistry()
    handler = StateHandler(channel, model, registry, asset_id=name)

    # Run the REAL initial frame registration from robot_manager. Only the
    # renderer is bypassed: _register_initial_transforms touches nothing but
    # self.transform_registry, so no visualizer engine is needed. This matters
    # because StateHandler alone registers only arm-chain links, and base_link
    # is NOT one of them (true_root is base_link_inertia).
    mgr = object.__new__(RobotManager)
    mgr.transform_registry = registry
    mgr._register_initial_transforms(name, model)

    robot = SimulatedRobot(model, channel)
    robot.move_joints(Q0.tolist())          # publishes ROBOT_STATE -> handler fills registry

    true_base = model.get_true_base()
    base_frame = f"{name}_{true_base}"      # exactly robot_manager.py:191
    tcp_world = model.get_tcp_pose()

    via_registry = registry.transform_frame_pose(
        tcp_world, from_frame="world", to_frame=base_frame)
    via_fk = np.linalg.inv(model.compute_fk(Q0)[true_base]) @ tcp_world

    dp = float(np.linalg.norm(via_registry[:3, 3] - via_fk[:3, 3]))
    da = rot_deg(via_registry, via_fk)
    print(f"| {name} | {true_base} | {base_frame} | {dp:.3e} m | {da:.3e} deg |")
    rigs[name] = (model, registry, robot, base_frame, handler)

print("\n## P2 — is T_world_base independent of q? (the static-base premise)\n")
print("| scene | max position drift over 200 random q | max angle drift |")
print("|---|---|---|")
rng = np.random.default_rng(7)
for name, (model, registry, robot, base_frame, handler) in rigs.items():
    tb = model.get_true_base()
    ref = model.compute_fk(Q0)[tb].copy()
    dp = da = 0.0
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        T = model.compute_fk(q)[tb]
        dp = max(dp, float(np.linalg.norm(T[:3, 3] - ref[:3, 3])))
        da = max(da, rot_deg(T, ref))
    print(f"| {name} | {dp:.3e} m | {da:.3e} deg |")

print("\n## P3 — end-to-end jog through the REAL panel route\n")
print("Panel publishes registry-derived base pose; driver converts back to world.\n")
print("| scene | asked | TCP moved | orientation change |")
print("|---|---|---|---|")
for name, (model, registry, robot, base_frame, handler) in rigs.items():
    for delta in (0.0, 0.001, 0.010):
        model.update_state(Q0)
        robot.move_joints(Q0.tolist())
        before = model.get_tcp_pose().copy()

        # --- exactly what the panel does ---
        tcp_world = model.get_tcp_pose()
        target = registry.transform_frame_pose(
            tcp_world, from_frame="world", to_frame=base_frame)
        target[0, 3] += delta
        ok = robot.move_pose(target, frame="base")

        q_after = np.asarray(robot.get_state()["joint_positions"])
        model.update_state(q_after)
        after = model.get_tcp_pose()
        moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
        ang = rot_deg(before, after)
        flag = "" if ok else "  (IK REFUSED)"
        print(f"| {name} | {delta*1000:.1f} mm | {moved*1000:.3f} mm | {ang:.4f} deg |{flag}")

print("\n## P4 — does the driver's own q matter? panel and driver at different q\n")
print("Panel reads model.link_transforms; driver uses its own _joint_positions.\n")
print("| scene | panel q vs driver q | asked | TCP moved |")
print("|---|---|---|---|")
for name, (model, registry, robot, base_frame, handler) in rigs.items():
    model.update_state(Q0)
    robot.move_joints(Q0.tolist())
    # Now desynchronise: move the model without telling the driver.
    q_other = np.array([0.9, -0.6, 0.8, -0.2, 1.0, -0.5])
    model.update_state(q_other)
    before = model.get_tcp_pose().copy()
    tcp_world = model.get_tcp_pose()
    target = registry.transform_frame_pose(
        tcp_world, from_frame="world", to_frame=base_frame)
    target[0, 3] += 0.001
    robot.move_pose(target, frame="base")
    q_after = np.asarray(robot.get_state()["joint_positions"])
    model.update_state(q_after)
    after = model.get_tcp_pose()
    moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    print(f"| {name} | Q0 vs desync | 1.0 mm | {moved*1000:.3f} mm |")
