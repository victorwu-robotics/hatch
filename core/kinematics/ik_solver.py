"""
IK entry point for Hatch: the TCP/world interface over the unified
analytical solver.

This module used to be a *dispatcher*. It inspected the last three joint
origins, guessed a wrist type, and instantiated one of three solvers —
with a silent fall back to a numerical Jacobian solver on any exception.
That design is gone, for reasons that were measured rather than assumed:

* **The discrimination was not measuring the wrist.** It read
  `origin_xyz[0]` of the last three joints. On UR10 that is −0.5723,
  which is the *forearm length* appearing in `wrist_1_joint`'s origin,
  not a wrist offset. The quantity it tested is a link length in an
  arbitrary frame, so it answers a question nobody asked.

* **Both analytical branches were dead.** `URIKSolver` no longer accepts
  `base_offset`, and `AnalyticalIKSolver` now requires four positional
  arguments. Each raised `TypeError`, which `except Exception` swallowed.
  All three supported arms therefore fell through to the numerical
  solver while logging "Wrist detection failed".

* **The fallback was unsafe, not merely approximate.** Measured on the
  numerical solver: an Elfin solve from a zero seed returned
  `q3 = 57.55 rad` against a ±2.61 limit (the right answer, unwrapped and
  unchecked), and a target 3 m beyond reach returned a joint vector with
  3.01 m of position error *without returning None* — i.e. reported to
  the caller as success.

There is now one solver, `UnifiedIKSolver`, which handles offset and
spherical wrists by the same derivation and refuses arms outside its
class. This module's remaining job is honest plumbing:

    TCP pose in world frame  <->  flange pose in true-root frame

and choosing among the analytical branches by continuity with the
current configuration.

**There is no numerical fallback and there must not be one.** An arm
outside the supported class raises `UnsupportedArmError`, naming the
violated condition and its measured value. A target the arm cannot reach
returns `None`. Both are answers; a plausible-looking joint vector that
is neither reachable nor in limits is not.

Principle: Everything in URDF.
"""

from __future__ import annotations

import logging
from typing import List, Optional

import numpy as np

from .geometric_extraction import UnsupportedArmError
from .kinematic_model import KinematicModel
from .unified_ik_solver import IKSolution, UnifiedIKSolver

logger = logging.getLogger(__name__)

# End-to-end acceptance in TCP space. Loose enough for the CAD axis tilt
# in FR5's and Elfin's URDFs (~3.7e-6 rad, giving ~6e-6 m), tight enough
# to catch a wrong frame conversion or an unreachable target.
TCP_POS_TOL = 1e-4   # metres
TCP_ROT_TOL = 1e-4   # radians


class IKSolver:
    """
    Analytical IK for one arm, addressed in TCP/world coordinates.

    Args:
        kinematic_model: a loaded KinematicModel. Required: a solver with
            no model is not a solver, and deferring construction is what
            let the old dispatcher fail silently.

    Raises:
        UnsupportedArmError: if the arm is outside the supported class.
            Nothing is substituted.
        ValueError: if the tool mount is not the flange, which would make
            the TCP <-> flange conversion wrong.
    """

    def __init__(self, kinematic_model: KinematicModel):
        if kinematic_model is None:
            raise ValueError("IKSolver requires a loaded KinematicModel")
        self.model = kinematic_model

        # The tool transform is defined from the tool mount link. The
        # unified solver works to the flange. If those differ, composing
        # inv(tool) onto a TCP pose does not yield the flange pose.
        mount = kinematic_model.tool_mount_link
        flange = kinematic_model.get_flange()
        if mount != flange:
            raise ValueError(
                f"tool mount link '{mount}' is not the flange '{flange}'. "
                f"The TCP <-> flange conversion assumes they coincide; "
                f"with a fixed offset between them the tool transform "
                f"alone does not relate TCP to flange.")

        # One solver, no alternatives, no fallback.
        self.unified = UnifiedIKSolver(kinematic_model)

        # Map between the model's joint ordering (every non-fixed joint,
        # document order) and the six arm joints. These coincide on a bare
        # arm URDF, but a scene with an extra actuated joint would silently
        # shift every index if this were assumed.
        order = [n for n, j in kinematic_model.joints.items()
                 if j['type'] != 'fixed']
        self._arm_idx = np.array([order.index(n) for n in self.unified.chain],
                                 dtype=int)
        self._n_full = len(order)

        self.diagnostics: dict = {}
        logger.info("IK ready: unified analytical solver on %s -> %s",
                    self.unified.root, self.unified.flange)

    # ------------------------------------------------------------------
    # Joint-vector ordering
    # ------------------------------------------------------------------

    def _to_arm(self, q_full: np.ndarray) -> np.ndarray:
        return np.asarray(q_full, dtype=float)[self._arm_idx]

    def _from_arm(self, q_arm: np.ndarray, template: np.ndarray) -> np.ndarray:
        """Put six arm angles back into a full joint vector, leaving the rest."""
        out = np.asarray(template, dtype=float).copy()
        out[self._arm_idx] = q_arm
        return out

    def _reference(self, q_guess: Optional[np.ndarray]) -> np.ndarray:
        if q_guess is not None:
            q = np.asarray(q_guess, dtype=float)
            if q.size == 6 and self._n_full != 6:
                return q            # already arm-only
            return self._to_arm(q)
        return self._to_arm(self.model.get_current_joint_positions())

    # ------------------------------------------------------------------
    # Frame conversion
    # ------------------------------------------------------------------

    def _root_world(self, q_full: np.ndarray) -> np.ndarray:
        return self.model.compute_fk(q_full)[self.unified.root]

    def tcp_to_flange(self,
                      T_tcp_world: np.ndarray,
                      q_full: np.ndarray) -> np.ndarray:
        """
        TCP pose in world -> flange pose in true root.

        Two removals, in order: the world transform of the root, and the
        tool transform. The tool must come off *before* the solver sees
        the pose: a bent welding torch puts the TCP off J6's axis, and
        `P6 - d6*a6` applied to a TCP is simply wrong.
        """
        T_tool = self.model.get_tool_transform()
        return np.linalg.inv(self._root_world(q_full)) @ T_tcp_world @ np.linalg.inv(T_tool)

    def flange_to_tcp(self,
                      T_flange_root: np.ndarray,
                      q_full: np.ndarray) -> np.ndarray:
        """Inverse of tcp_to_flange."""
        return (self._root_world(q_full) @ T_flange_root
                @ self.model.get_tool_transform())

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: Optional[np.ndarray] = None
                         ) -> Optional[np.ndarray]:
        """
        Joint vector placing the TCP at `target_pose`, or None.

        Args:
            target_pose: 4x4 TCP pose in the **world** frame — the same
                frame `forward_kinematics` and `get_tcp_pose` return.
            q_guess: current configuration, full or arm-only. Used to
                pick the branch by continuity, *not* as an optimisation
                seed: the solution is closed-form and does not iterate.
                Defaults to the model's current joint positions.

        Returns:
            A full joint vector in the model's joint order, within joint
            limits, or None if the pose is unreachable or no branch fits
            the limits. None is a real answer; callers already handle it.
        """
        sols = self.solve_all_for_tcp(target_pose, q_guess)
        if not sols:
            return None
        return sols[0][0]

    def solve_all_for_tcp(self,
                          target_pose: np.ndarray,
                          q_guess: Optional[np.ndarray] = None
                          ) -> List[tuple]:
        """
        Every viable branch, nearest first, as (q_full, IKSolution) pairs.

        Exposed because the eight solutions are the interesting part: a
        single-vector API hides the choice that decides whether the arm
        sweeps through the workpiece. Callers that care about
        configuration should use this.
        """
        q_ref_full = (np.asarray(q_guess, dtype=float)
                      if q_guess is not None
                      else self.model.get_current_joint_positions())
        if q_ref_full.size == 6 and self._n_full != 6:
            q_ref_full = self._from_arm(q_ref_full,
                                        self.model.get_current_joint_positions())
        q_ref = self._to_arm(q_ref_full)

        T_target = np.asarray(target_pose, dtype=float)

        # The root's world pose depends only on joints upstream of the arm,
        # so it is constant across branches. Compute it once.
        T_rw = self._root_world(q_ref_full)
        T_tool = self.model.get_tool_transform()
        T_flange = np.linalg.inv(T_rw) @ T_target @ np.linalg.inv(T_tool)

        sols = self.unified.solve(T_flange, q_reference=q_ref)
        if not sols:
            self.diagnostics = {"n_solutions": 0, "reason": "unreachable or out of limits"}
            return []

        # Order by continuity: same configuration first, then least travel.
        ref_tags = self.unified._tags_of(q_ref)

        def cost(s: IKSolution) -> float:
            d = np.arctan2(np.sin(s.q - q_ref), np.cos(s.q - q_ref))
            travel = float(np.sum(d ** 2))
            penalty = 0.0
            if ref_tags is not None:
                penalty = 100.0 * sum(a != b for a, b in zip(s.tags(), ref_tags))
            return travel + penalty

        sols = sorted(sols, key=cost)

        # End-to-end gate, in the frame the caller actually asked about.
        # The unified solver already verified in flange space; this check
        # additionally catches a wrong world or tool conversion, which a
        # flange-space check cannot see.
        #
        # A conversion error is *systematic*, so the nearest branch is
        # checked against the model's own TCP path — independent of the
        # arithmetic above — and the rest are checked through the now-
        # validated conversion. Doing the independent check on all eight
        # cost eight scene-wide FK evaluations for no extra information.
        out: List[tuple] = []
        rejected = 0
        worst_pos = worst_rot = 0.0
        for i, s in enumerate(sols):
            q_full = self._from_arm(s.q, q_ref_full)
            if i == 0:
                T_check = self.model.forward_kinematics(q_full)
            else:
                _, T_fl = self.unified._place(s.q)
                T_check = T_rw @ T_fl @ T_tool
            pos_err = float(np.linalg.norm(T_check[:3, 3] - T_target[:3, 3]))
            c = (float(np.trace(T_check[:3, :3].T @ T_target[:3, :3])) - 1.0) / 2.0
            rot_err = float(np.arccos(np.clip(c, -1.0, 1.0)))
            if pos_err > TCP_POS_TOL or rot_err > TCP_ROT_TOL:
                rejected += 1
                logger.debug("branch %s rejected in TCP space: pos %.3e rot %.3e",
                             s.tags(), pos_err, rot_err)
                continue
            worst_pos = max(worst_pos, pos_err)
            worst_rot = max(worst_rot, rot_err)
            out.append((q_full, s))

        self.diagnostics = {
            "n_solutions": len(out),
            "rejected_tcp": rejected,
            "worst_pos_error": worst_pos,
            "worst_rot_error": worst_rot,
            "chosen_tags": out[0][1].tags() if out else None,
            "wrist_singular": out[0][1].wrist_singular if out else False,
        }
        if rejected:
            logger.warning("%d branch(es) failed the TCP-space check and were "
                           "dropped; inspect the tool and world transforms",
                           rejected)
        if not out:
            logger.error("IK: %d analytical branch(es) found but none reproduces "
                         "the TCP target to tolerance", len(sols))
        return out

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """
        TCP pose in the world frame. Does not mutate the model.

        Delegates to the model so that FK and IK cannot drift apart.
        """
        return self.model.forward_kinematics(np.asarray(q, dtype=float))

    def describe(self) -> str:
        return self.unified.describe()
