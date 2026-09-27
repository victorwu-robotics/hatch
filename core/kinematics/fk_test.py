# fk_test.py
import numpy as np
from math import pi, cos, sin
import logging

logger = logging.getLogger(__name__)


# DH tables for the robots we have a published reference for.
# Standard DH: T_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)
_DH_TABLES = {
    "ur10_nominal": [
        # (a,      d,        alpha,  theta_offset)
        (0.0,      0.1273,   pi/2,   0.0),
        (-0.612,   0.0,      0.0,    0.0),
        (-0.5723,  0.0,      0.0,    0.0),
        (0.0,      0.163941, pi/2,   0.0),
        (0.0,      0.1157,  -pi/2,   0.0),
        (0.0,      0.0922,   0.0,    0.0),
    ],
}


def _dh_transform(a, d, alpha, theta):
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1],
    ])


def _reference_fk(dh_table, q):
    T = np.eye(4)
    for (a, d, alpha, offset), theta in zip(dh_table, q):
        T = T @ _dh_transform(a, d, alpha, theta + offset)
    return T


def run_fk_test(model, asset_id, atol_pos=1e-6, atol_rot=1e-6):
    """
    Compare model.forward_kinematics(q) against a published DH reference.

    Logs the result. Does not raise. If there is no reference table for
    this asset_id, logs at debug and returns.
    """
    table = _DH_TABLES.get(asset_id)
    if table is None:
        logger.debug(f"FK test: no DH reference for '{asset_id}', skipping")
        return

    # Zero the tool so the model returns the flange, not the TCP.
    saved_tool = model.get_tool_transform()
    model.set_tool_transform(np.eye(4))

    try:
        test_configs = [
            np.zeros(6),
            np.array([0.3, -0.5, 0.8, -0.4, 0.7, 0.2]),
            np.array([pi/4, -pi/3, pi/2, -pi/6, pi/3, pi/5]),
            np.array([0.5, 0.5, -0.5, 1.0, -0.8, 0.3]),
        ]

        all_ok = True
        for q in test_configs:
            T_model = model.forward_kinematics(q)
            T_ref = _reference_fk(table, q)

            pos_err = float(np.linalg.norm(T_model[:3, 3] - T_ref[:3, 3]))
            rot_err = float(np.linalg.norm(T_model[:3, :3] - T_ref[:3, :3]))

            ok = pos_err < atol_pos and rot_err < atol_rot
            all_ok = all_ok and ok

            logger.info(
                f"FK test [{asset_id}] q={np.round(q, 4).tolist()} "
                f"pos_err={pos_err:.3e} rot_err={rot_err:.3e} "
                f"{'OK' if ok else 'FAIL'}"
            )
            if not ok:
                logger.info(f"  model pos = {T_model[:3, 3].round(6).tolist()}")
                logger.info(f"  ref   pos = {T_ref[:3, 3].round(6).tolist()}")
                logger.info(f"  model rot =\n{T_model[:3, :3].round(6)}")
                logger.info(f"  ref   rot =\n{T_ref[:3, :3].round(6)}")

        logger.info(f"FK test [{asset_id}]: {'PASS' if all_ok else 'FAIL'}")

    finally:
        # Restore the tool transform, whatever happened.
        model.set_tool_transform(saved_tool)