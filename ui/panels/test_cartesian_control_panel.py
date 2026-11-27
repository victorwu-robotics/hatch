"""
Tests for the Cartesian jog control panel.

Acceptance criterion is the POSE, not the slider numbers and not joint vectors.
The panel's job is to turn a gesture into a TCP pose with the semantics agreed in
docs/cartesian_jog_design.md; these tests assert that pose directly.

Why the methods are extracted instead of imported
------------------------------------------------
The panel is a QWidget, so importing it needs PyQt5 and a display. The coordinate
semantics -- the part that was wrong, and the part worth guarding -- are pure
numpy/scipy and do not need Qt at all. So the class's methods are read from source
and bound to a stub carrying only the attributes they touch. That keeps these
tests runnable in a kernel-only environment (numpy + scipy + pytest), which is
also how the IK tests run.

Two real-Qt tests at the bottom cover the signal behaviour the design depends on.
They skip when PyQt5 is unavailable.

Regression history -- each of these was a measured defect, not a hypothesis:
  * repeated +/- clicks did not accumulate (anchor never cleared): 3 clicks of
    1 deg produced 1 deg.
  * a second axis discarded the first (one shared anchor): wheel X then Y gave
    12 deg instead of 16.955.
  * a wheel notch was quantised to integer slider units: a 0.1 mm step moved
    3 mm (30x), the default 1 mm step moved 3 mm (3x).
  * divergence subtracted rotation vectors, which wraps at +/-180 deg: two poses
    2 deg apart reported 6.248 rad.
  * the in-flight flag was cleared after an early return, so it could latch true
    forever and suppress divergence reporting permanently.
"""

import ast
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

PANEL_PATH = Path(__file__).with_name("cartesian_control_panel.py")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


# ----------------------------------------------------------------------
# Headless harness
# ----------------------------------------------------------------------

class _EventType:
    CARTESIAN_COMMAND = "cartesian_command"
    ROBOT_STATE = "robot_state"
    MODE_SWITCHED = "mode_switched"
    ROBOT_LOADED = "robot_loaded"
    ERROR_OCCURRED = "error_occurred"


class _QTimer:
    @staticmethod
    def singleShot(ms, fn):
        """Label-flash timers are cosmetic; do not fire them in tests."""


class _QSlider:
    TicksBelow = 1
    NoTicks = 0


def _load_methods():
    """Bind the panel's methods to a stub, without importing PyQt5."""
    src = PANEL_PATH.read_text()
    cls = next(n for n in ast.parse(src).body if isinstance(n, ast.ClassDef))
    ns = {"np": np, "R": R, "EventType": _EventType,
          "QTimer": _QTimer, "QSlider": _QSlider,
          "logger": __import__("logging").getLogger("test.cartesian")}
    for f in cls.body:
        if isinstance(f, ast.FunctionDef):
            exec(textwrap.dedent(ast.get_source_segment(src, f)), ns)
    return ns, src


METHODS, SOURCE = _load_methods()


class _Slider:
    def __init__(self): self._v = 500
    def blockSignals(self, b): pass
    def setValue(self, v): self._v = int(v)
    def value(self): return self._v
    def maximum(self): return 1000
    def minimum(self): return 0
    def setTickPosition(self, p): pass
    def setTickInterval(self, i): pass
    def setEnabled(self, b): pass


class _Label:
    def __init__(self): self.text = ""; self.style = ""
    def setText(self, t): self.text = t
    def setStyleSheet(self, t): self.style = t
    def setAlignment(self, a): pass
    def setToolTip(self, t): pass


class _Combo:
    def __init__(self, t): self.t = t
    def currentText(self): return self.t


class _Radio:
    def __init__(self, checked=True): self.c = checked
    def isChecked(self): return self.c


class _Timer:
    def __init__(self): self.active = False
    def start(self, *a): self.active = True
    def stop(self): self.active = False
    def isActive(self): return self.active


class _Group:
    def setTitle(self, t): self.title = t


class _Channel:
    def __init__(self): self.published = []
    def publish(self, *a, **k): self.published.append((a, k))


class _Delta:
    def __init__(self, y): self._y = y
    def y(self): return self._y


class WheelEvent:
    """Minimal stand-in for QWheelEvent: notches of 120."""
    def __init__(self, y): self._y = y
    def angleDelta(self): return _Delta(self._y)


class _Panel:
    pass


for _k, _v in METHODS.items():
    if callable(_v) and _k.startswith("_") and not _k.startswith("__"):
        setattr(_Panel, _k, _v)


START_RPY = (20.0, 35.0, 50.0)
START_P = np.array([0.4, 0.1, 0.6])


def start_pose():
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', START_RPY, degrees=True).as_matrix()
    T[:3, 3] = START_P
    return T


def make_panel(pos_frame="Base", orient_frame="Base", reach=1.5,
               linear_step="0.001", angular_step="1", degrees=True, model=None):
    """A panel stub with a known target pose and no Qt."""
    s = _Panel()
    s.orient_sliders = [_Slider() for _ in range(3)]
    s.position_sliders = [_Slider() for _ in range(3)]
    s.orient_labels = [_Label() for _ in range(3)]
    s.position_labels = [_Label() for _ in range(3)]
    s.pos_current_vals = [_Label() for _ in range(3)]
    s.pos_target_vals = [_Label() for _ in range(3)]
    s.orient_current_vals = [_Label() for _ in range(3)]
    s.orient_target_vals = [_Label() for _ in range(3)]
    s.orient_current_rpy = _Label()
    s.orient_target_rpy = _Label()
    s.status_label = _Label()
    s.orient_group = _Group()
    s.pos_mode_hint = _Label()
    s.orient_mode_hint = _Label()
    s.pos_frame_selector = _Combo(pos_frame)
    s.orient_frame_selector = _Combo(orient_frame)
    s.linear_step_combo = _Combo(linear_step)
    s.angular_step_combo = _Combo(angular_step)
    s.degrees_radio = _Radio(degrees)
    s._move_timer = _Timer()
    s._wheel_idle_timer = _Timer()
    s.state_channel = _Channel()
    s._orient_jog_full_scale = np.radians(30)
    s._pos_jog_full_scale = 0.1 * reach
    s.position_limits = (-reach, reach)
    s._urdf_reach = reach
    s._pos_anchor = [None, None, None]
    s._orient_anchor = [None, None, None]
    s._wheel_accumulator = {}
    s._wheel_active = {}
    s._wheel_burst = {}
    s._motion_in_flight = False
    s._ik_refused = False
    s.target_pose = start_pose()
    s._last_achieved_pose = start_pose()
    s.kinematic_model = model
    s.robot_manager = None
    s.asset_id = None
    s.transform_registry = None
    return s


def separation_deg(A, B):
    """
    True angle between two rotation matrices, in degrees.

    Uses the relative rotation vector, not arccos((trace-1)/2): arccos is
    ill-conditioned near 0 and 180 deg. A pose restored to machine precision
    (1.1e-16 in the matrix) reads as 1.2e-6 deg via arccos but 2.4e-15 deg via
    rotvec -- so arccos would make a correct implementation look broken.
    """
    return float(np.degrees(np.linalg.norm(R.from_matrix(A.T @ B).as_rotvec())))


def rotation_axis(A, B):
    v = R.from_matrix(B @ A.T).as_rotvec()
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros(3)


# ----------------------------------------------------------------------
# Defect 1 -- discrete steps must accumulate
# ----------------------------------------------------------------------

@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_orientation_steps_accumulate(frame):
    """Three clicks of 1 deg must give 3 deg, not 1 deg."""
    s = make_panel(orient_frame=frame)
    R0 = s.target_pose[:3, :3].copy()
    for _ in range(3):
        s._on_orientation_step(0, +1)
    assert separation_deg(R0, s.target_pose[:3, :3]) == pytest.approx(3.0, abs=1e-9)


@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_position_steps_accumulate(frame):
    s = make_panel(pos_frame=frame)
    p0 = s.target_pose[:3, 3].copy()
    for _ in range(3):
        s._on_position_step(0, +1)
    got_mm = np.linalg.norm(s.target_pose[:3, 3] - p0) * 1000
    assert got_mm == pytest.approx(3.0, abs=1e-6)


@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_steps_are_reversible(frame):
    """+ then - returns to the start: a step is a complete gesture."""
    s = make_panel(orient_frame=frame)
    R0 = s.target_pose[:3, :3].copy()
    s._on_orientation_step(0, +1)
    s._on_orientation_step(0, -1)
    # Assert the pose itself, which is the acceptance criterion.
    assert np.allclose(s.target_pose[:3, :3], R0, atol=1e-12)
    assert separation_deg(R0, s.target_pose[:3, :3]) == pytest.approx(0.0, abs=1e-9)


def test_step_leaves_no_anchor():
    """A button click must not leave a drag anchor behind."""
    s = make_panel(orient_frame="Tool")
    s._on_orientation_step(0, +1)
    assert s._orient_anchor == [None, None, None]
    assert s.orient_sliders[0].value() == 500


# ----------------------------------------------------------------------
# Defect 2 -- axes are independent
# ----------------------------------------------------------------------

def test_second_axis_does_not_discard_first_orientation():
    """Wheel X then Y must compose, not replace."""
    s = make_panel(orient_frame="Tool")
    R0 = s.target_pose[:3, :3].copy()
    s._handle_wheel(s.orient_sliders[0], 0, WheelEvent(120 * 12), is_position=False)
    s._handle_wheel(s.orient_sliders[1], 1, WheelEvent(120 * 12), is_position=False)
    want = (R0
            @ R.from_euler('x', 12, degrees=True).as_matrix()
            @ R.from_euler('y', 12, degrees=True).as_matrix())
    assert np.allclose(s.target_pose[:3, :3], want, atol=1e-9)
    assert separation_deg(R0, s.target_pose[:3, :3]) == pytest.approx(16.955, abs=1e-3)


def test_second_axis_does_not_discard_first_position():
    s = make_panel(pos_frame="Tool", linear_step="0.01")
    p0 = s.target_pose[:3, 3].copy()
    R0 = s.target_pose[:3, :3].copy()
    s._handle_wheel(s.position_sliders[0], 0, WheelEvent(120 * 6), is_position=True)
    s._handle_wheel(s.position_sliders[1], 1, WheelEvent(120 * 6), is_position=True)
    want = p0 + R0 @ np.array([0.06, 0.06, 0.0])
    assert np.allclose(s.target_pose[:3, 3], want, atol=1e-9)


def test_drag_anchor_is_per_axis():
    """Releasing axis 0 must not clear axis 1's live anchor."""
    s = make_panel(orient_frame="Tool")
    s._on_orient_slider_pressed(0)
    s._on_orient_slider_pressed(1)
    s._on_orient_slider_released(0)
    assert s._orient_anchor[0] is None
    assert s._orient_anchor[1] is not None


# ----------------------------------------------------------------------
# Defect 3 -- a wheel notch is exactly one chosen step
# ----------------------------------------------------------------------

@pytest.mark.parametrize("step", ["0.0001", "0.001", "0.01", "0.1"])
@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_wheel_notch_equals_one_linear_step(frame, step):
    s = make_panel(pos_frame=frame, linear_step=step)
    p0 = s.target_pose[:3, 3].copy()
    s._handle_wheel(s.position_sliders[0], 0, WheelEvent(120), is_position=True)
    got = np.linalg.norm(s.target_pose[:3, 3] - p0)
    assert got == pytest.approx(float(step), abs=1e-12)


@pytest.mark.parametrize("step", ["0.1", "1", "5", "10"])
def test_wheel_notch_equals_one_angular_step(step):
    s = make_panel(orient_frame="Base", angular_step=step)
    R0 = s.target_pose[:3, :3].copy()
    s._handle_wheel(s.orient_sliders[0], 0, WheelEvent(120), is_position=False)
    assert separation_deg(R0, s.target_pose[:3, :3]) == pytest.approx(float(step), abs=1e-9)


def test_partial_notch_does_nothing():
    s = make_panel()
    T0 = s.target_pose.copy()
    s._handle_wheel(s.orient_sliders[0], 0, WheelEvent(60), is_position=False)
    assert np.allclose(s.target_pose, T0, atol=1e-15)
    s._handle_wheel(s.orient_sliders[0], 0, WheelEvent(60), is_position=False)
    assert separation_deg(T0[:3, :3], s.target_pose[:3, :3]) == pytest.approx(1.0, abs=1e-9)


def test_wheel_idle_recentres_only_jog_sliders():
    s = make_panel(pos_frame="Tool", orient_frame="Base")
    s._handle_wheel(s.orient_sliders[0], 0, WheelEvent(120 * 3), is_position=False)
    s._handle_wheel(s.position_sliders[1], 1, WheelEvent(120 * 3), is_position=True)
    s._on_wheel_idle_timeout()
    assert s.orient_sliders[0].value() == 500
    assert s.position_sliders[1].value() == 500
    assert s._orient_anchor[0] is None and s._pos_anchor[1] is None


# ----------------------------------------------------------------------
# Defect 4 -- divergence is a true rotation distance
# ----------------------------------------------------------------------

@pytest.mark.parametrize("deg", [0, 5, 90, 179, 180])
def test_rotation_separation_is_true_angle(deg):
    s = make_panel()
    A = np.eye(3)
    B = R.from_euler('z', deg, degrees=True).as_matrix()
    assert np.degrees(s._rot_separation(A, B)) == pytest.approx(deg, abs=1e-6)


def test_separation_resolves_tiny_angles():
    """arccos would report exactly 0 here; the metric must not lose small angles."""
    s = make_panel()
    A = np.eye(3)
    B = R.from_euler('z', 1e-6, degrees=True).as_matrix()
    assert np.degrees(s._rot_separation(A, B)) == pytest.approx(1e-6, rel=1e-3)


def test_rotation_separation_does_not_wrap():
    """179 deg and -179 deg about X are 2 deg apart, not ~358 deg."""
    s = make_panel()
    A = R.from_euler('x', 179, degrees=True).as_matrix()
    B = R.from_euler('x', -179, degrees=True).as_matrix()
    assert np.degrees(s._rot_separation(A, B)) == pytest.approx(2.0, abs=1e-9)


def _red(labels):
    return any("e74c3c" in l.style for l in labels)


def test_no_false_divergence_while_motion_in_flight():
    s = make_panel(orient_frame="Base")
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)   # 12 deg ahead of the arm
    s._publish_command()
    s._update_target_display()
    assert not _red(s.orient_target_vals)


def test_divergence_reported_once_settled():
    s = make_panel(orient_frame="Base")
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    s._publish_command()
    s._move_timer.stop()
    # The arm has not moved: its TCP is still the start pose.
    s.kinematic_model = type("M", (), {"get_tcp_pose": staticmethod(start_pose)})()
    s._update_current_display()
    s._update_target_display()
    assert _red(s.orient_target_vals)


def test_in_flight_flag_cleared_even_without_a_pose():
    """It must not latch true when no pose is available (regression)."""
    s = make_panel()
    s._motion_in_flight = True
    s.kinematic_model = None          # _get_tcp_pose_in_base returns None
    s._move_timer.stop()
    s._update_current_display()
    assert s._motion_in_flight is False


def test_refusal_is_reported_in_status():
    s = make_panel()
    s._ik_refused = True
    s._update_target_display()
    assert "refused" in s.status_label.text


# ----------------------------------------------------------------------
# Design invariants, docs/cartesian_jog_design.md section 6
# ----------------------------------------------------------------------

@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_I1_orientation_jog_preserves_position(frame):
    s = make_panel(orient_frame=frame)
    p0 = s.target_pose[:3, 3].copy()
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    assert np.allclose(s.target_pose[:3, 3], p0, atol=1e-15)


@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_drag_magnitude_is_exact(frame):
    """Slider 700 is 0.4 of full scale: 0.4 * 30 deg = 12 deg exactly."""
    s = make_panel(orient_frame=frame)
    R0 = s.target_pose[:3, :3].copy()
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    assert separation_deg(R0, s.target_pose[:3, :3]) == pytest.approx(12.0, abs=1e-9)


def test_I4_base_jog_axis_is_a_base_axis():
    s = make_panel(orient_frame="Base")
    R0 = s.target_pose[:3, :3].copy()
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    axis = rotation_axis(R0, s.target_pose[:3, :3])
    assert np.allclose(np.abs(axis), [1, 0, 0], atol=1e-9)


def test_I5_tool_jog_axis_is_the_tool_axis():
    s = make_panel(orient_frame="Tool")
    R0 = s.target_pose[:3, :3].copy()
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    axis = rotation_axis(R0, s.target_pose[:3, :3])
    assert np.allclose(np.abs(axis), np.abs(R0[:, 0]), atol=1e-9)


@pytest.mark.parametrize("frame", ["Base", "Tool"])
def test_I6_return_to_centre_restores_the_pose(frame):
    s = make_panel(orient_frame=frame)
    R0 = s.target_pose[:3, :3].copy()
    s._on_orient_slider_pressed(0)
    s._on_orientation_slider_changed(0, 700)
    s._on_orientation_slider_changed(0, 500)
    assert np.allclose(s.target_pose[:3, :3], R0, atol=1e-12)


def test_I2_I3_tool_position_jog():
    s = make_panel(pos_frame="Tool")
    p0 = s.target_pose[:3, 3].copy()
    R0 = s.target_pose[:3, :3].copy()
    s._on_pos_slider_pressed(0)
    s._on_position_slider_changed(0, 700)
    disp = s.target_pose[:3, 3] - p0
    assert np.allclose(s.target_pose[:3, :3], R0, atol=1e-15)          # I2
    assert np.allclose(disp / np.linalg.norm(disp), R0[:, 0], atol=1e-9)  # I3
    assert np.linalg.norm(disp) == pytest.approx(0.4 * s._pos_jog_full_scale, abs=1e-12)


def test_I7_tcp_position_in_its_own_frame_is_zero():
    T = start_pose()
    p_own = (np.linalg.inv(T) @ np.append(T[:3, 3], 1.0))[:3]
    assert np.allclose(p_own, 0, atol=1e-12)


def _orient(T, frame, deg=10):
    Tn = T.copy()
    A = R.from_euler('z', deg, degrees=True).as_matrix()
    Tn[:3, :3] = A @ T[:3, :3] if frame == 'base' else T[:3, :3] @ A
    return Tn


def _translate(T, frame, d=np.array([0.02, 0.0, 0.0])):
    Tn = T.copy()
    Tn[:3, 3] = T[:3, 3] + (d if frame == 'base' else T[:3, :3] @ d)
    return Tn


@pytest.mark.parametrize("of", ["base", "tool"])
def test_I8_base_position_commutes_with_orientation(of):
    T = start_pose()
    a = _translate(_orient(T, of), 'base')
    b = _orient(_translate(T, 'base'), of)
    assert np.allclose(a[:3, 3], b[:3, 3], atol=1e-12)


@pytest.mark.parametrize("of,expected_mm", [("base", 2.856), ("tool", 3.486)])
def test_I9_tool_position_does_not_commute(of, expected_mm):
    """
    A PROPERTY, not a defect: 'along tool X' points somewhere new once the tool
    has been reoriented, so in Position/Tool mode the order of jogs matters.
    """
    T = start_pose()
    a = _translate(_orient(T, of), 'tool')
    b = _orient(_translate(T, 'tool'), of)
    gap_mm = np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000
    assert gap_mm == pytest.approx(expected_mm, abs=1e-3)


# ----------------------------------------------------------------------
# Section 7 -- ranges derived from the loaded URDF
# ----------------------------------------------------------------------

ARMS = {
    "UR10": ("assets/robots/ur_description/urdf/ur10_nominal.urdf", 1.2765),
    "FR5": ("assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf", 0.92001),
    "Elfin": ("assets/robots/E15_Pro/urdf/E15_Pro.urdf", 1.5000),
}


@pytest.mark.parametrize("name", sorted(ARMS))
def test_reach_comes_from_the_urdf(name):
    """
    L2 + L3 + |d6|, measured from the axis lines. The old hard-coded +/-1.5 m
    was 63% beyond FR5's reach.
    """
    rel, expected = ARMS[name]
    path = ROOT / rel
    if not path.exists():
        pytest.skip(f"URDF not present: {rel}")
    from core.kinematics.kinematic_model import KinematicModel
    model = KinematicModel(urdf_path=str(path))
    model.load()
    s = make_panel(model=model)
    assert s._compute_urdf_reach() == pytest.approx(expected, abs=1e-4)


def test_reach_falls_back_loudly_without_a_model(caplog):
    s = make_panel(model=None)
    assert s._compute_urdf_reach() == 1.5


# ----------------------------------------------------------------------
# Section 8 -- clipping preserves the jog direction
# ----------------------------------------------------------------------

def test_tool_jog_clips_at_the_box_without_changing_direction():
    s = make_panel(pos_frame="Tool", reach=0.5)
    s.target_pose[:3, :3] = np.eye(3)
    s.target_pose[:3, 3] = [0.49, 0.0, 0.0]
    p0 = s.target_pose[:3, 3].copy()
    s._on_pos_slider_pressed(0)
    s._on_position_slider_changed(0, 1000)      # asks +0.05 m, only 0.01 available
    disp = s.target_pose[:3, 3] - p0
    assert s.target_pose[0, 3] == pytest.approx(0.5, abs=1e-12)
    assert np.allclose(disp[1:], 0, atol=1e-15)  # still purely along X


def test_base_position_step_is_clipped():
    s = make_panel(pos_frame="Base", reach=0.5, linear_step="0.1")
    s.target_pose[0, 3] = 0.49
    s._on_position_step(0, +1)
    assert s.target_pose[0, 3] == pytest.approx(0.5, abs=1e-12)


# ----------------------------------------------------------------------
# Section 10/11 -- readout and units
# ----------------------------------------------------------------------

def test_readout_honours_the_unit_toggle():
    s = make_panel(degrees=True)
    s._update_target_display()
    deg_vals = [l.text for l in s.orient_target_vals]
    s.degrees_radio = _Radio(False)
    s._update_target_display()
    rad_vals = [l.text for l in s.orient_target_vals]
    assert deg_vals != rad_vals
    assert "rad" in s.orient_target_rpy.text


def test_rpy_readout_names_its_convention():
    s = make_panel()
    s._update_target_display()
    assert s.orient_target_rpy.text.startswith("RPY")


def test_no_quaternions_in_the_public_api():
    """Constitution Part 7: rotation vectors only."""
    assert "as_quat(" not in SOURCE
    assert "from_quat(" not in SOURCE


# ----------------------------------------------------------------------
# Real Qt: the signal behaviour the design relies on. Skips without PyQt5.
# ----------------------------------------------------------------------

def _qt_slider():
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    # Deliberately NOT pytest.importorskip. Two measured reasons:
    #   * Guarding "PyQt5" is not enough: the top-level package imports fine
    #     even when the Qt runtime libraries are missing, so the failure would
    #     surface inside the test instead of as a skip.
    #   * Guarding "PyQt5.QtWidgets" is not enough either. Since pytest 9.1 the
    #     default exc_type of importorskip is ModuleNotFoundError, so an
    #     installed-but-unloadable extension (ImportError: libGL.so.1 on a
    #     headless box) is re-raised and the test FAILS. Measured on pytest
    #     9.2.0. exc_type=ImportError fixes it but only exists from pytest 8.2,
    #     and this repo pins no pytest version at all.
    # A plain try/except ImportError skips correctly on every pytest version.
    try:
        from PyQt5.QtWidgets import QApplication, QSlider
        from PyQt5.QtCore import Qt
    except ImportError as exc:
        pytest.skip(f"Qt runtime unavailable: {exc}")
    try:
        app = QApplication.instance() or QApplication([])
    except Exception as exc:  # no usable QPA platform plugin
        pytest.skip(f"no usable Qt platform: {exc}")
    s = QSlider(Qt.Horizontal)
    s.setRange(0, 1000)
    s.setValue(500)
    s.resize(200, 20)
    s.show()
    return app, s


def test_qt_wheel_does_not_emit_slider_released():
    """
    Verified on Qt 5.15.14: a wheel notch fires valueChanged only. This is why
    jog spring-back cannot rely on sliderReleased and needs an idle timer.
    """
    app, s = _qt_slider()
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import Qt, QPointF, QPoint
    from PyQt5.QtGui import QWheelEvent
    seen = {"pressed": 0, "released": 0, "changed": 0}
    s.sliderPressed.connect(lambda: seen.__setitem__("pressed", seen["pressed"] + 1))
    s.sliderReleased.connect(lambda: seen.__setitem__("released", seen["released"] + 1))
    s.valueChanged.connect(lambda v: seen.__setitem__("changed", seen["changed"] + 1))
    ev = QWheelEvent(QPointF(100, 10), QPointF(100, 10), QPoint(0, 0), QPoint(0, 120),
                     Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
    QApplication.sendEvent(s, ev)
    assert seen["changed"] >= 1
    assert seen["released"] == 0
    assert seen["pressed"] == 0


def test_qt_handle_drag_emits_press_and_release():
    app, s = _qt_slider()
    from PyQt5.QtWidgets import QStyle, QStyleOptionSlider
    from PyQt5.QtCore import Qt, QPoint
    from PyQt5.QtTest import QTest
    opt = QStyleOptionSlider()
    opt.initFrom(s)
    opt.minimum, opt.maximum, opt.sliderPosition = 0, 1000, 500
    opt.orientation = Qt.Horizontal
    handle = s.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, s)
    seen = {"pressed": 0, "released": 0}
    s.sliderPressed.connect(lambda: seen.__setitem__("pressed", seen["pressed"] + 1))
    s.sliderReleased.connect(lambda: seen.__setitem__("released", seen["released"] + 1))
    c = handle.center()
    QTest.mousePress(s, Qt.LeftButton, Qt.NoModifier, c)
    QTest.mouseRelease(s, Qt.LeftButton, Qt.NoModifier, c)
    assert seen["pressed"] == 1
    assert seen["released"] == 1


# ----------------------------------------------------------------------
# Defect -- a tab switch left a stale Target readout on screen
# ----------------------------------------------------------------------

class _Model:
    """Minimal stand-in: the panel only asks the model for its TCP pose."""
    def __init__(self, T): self._T = T.copy()
    def get_tcp_pose(self): return self._T.copy()


def test_reset_target_refreshes_both_numeric_readouts():
    """
    Switching joint -> Cartesian must not leave a stale Target on screen.

    Measured on the maintainer's machine: the Target box read
    (-0.653, 0.256, 1.122) while the arm sat at (-0.714, 0.158, 0.613), and the
    per-axis slider labels were already correct. _reset_target refreshed the
    sliders but not the numeric readouts, which are otherwise written only on
    ROBOT_STATE -- and a tab switch causes no motion, so no event arrives. The
    box kept its old contents until the first jog.

    A student reading Target != Current would reasonably believe a move was
    pending and go looking for a way to complete it.
    """
    achieved = np.eye(4)
    achieved[:3, :3] = R.from_euler(
        'xyz', (177.8, 3.6, 89.9), degrees=True).as_matrix()
    achieved[:3, 3] = [-0.7144, 0.1581, 0.6134]
    s = make_panel(model=_Model(achieved))

    stale = achieved.copy()
    stale[:3, 3] = [-0.653, 0.256, 1.122]
    # Rotate it too. Without this the RPY half of the check would pass even
    # unfixed, and the test would be half degenerate.
    stale[:3, :3] = stale[:3, :3] @ R.from_euler(
        'xyz', (30, 20, -40), degrees=True).as_matrix()
    s.target_pose = stale
    s._last_achieved_pose = achieved.copy()
    s._update_current_display()
    s._update_target_display()

    # Precondition: the readouts really do disagree, or this proves nothing.
    assert [w.text for w in s.pos_target_vals] != [w.text for w in s.pos_current_vals]
    assert s.orient_target_rpy.text != s.orient_current_rpy.text

    # Exactly what motion_container._on_mode_changed calls on the switch.
    s._reset_target()

    assert [w.text for w in s.pos_target_vals] == [w.text for w in s.pos_current_vals]
    assert s.orient_target_rpy.text == s.orient_current_rpy.text
    assert [w.text for w in s.pos_target_vals] == ['-0.714', '0.158', '0.613']
    # And the reset must not have commanded anything.
    assert s.state_channel.published == []
