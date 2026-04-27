"""
UR RTDE robot arm driver with thread-safe Qt signals.

Communicates with Universal Robots via RTDE protocol.
Uses URRobotSignalHolder (composition) for thread-safe Qt signal emission.
No threading logic here — that's handled externally by the application.

Principle #8: Pure Python where possible. Qt used only for signal bridge.
"""

import time
import logging
from typing import Dict, List, Optional, Any

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False

from PyQt5.QtCore import QObject, pyqtSignal
from .base_robot_arm import BaseRobotArmDriver

logger = logging.getLogger(__name__)


class URRobotSignalHolder(QObject):
    """
    Separate QObject to hold Qt signals.

    Uses composition (not inheritance) to avoid metaclass conflicts
    and keep the driver class clean. Signals are emitted from the
    RTDE receive thread and delivered to main thread slots via
    Qt.QueuedConnection.
    """
    state_signal = pyqtSignal(dict)
    connection_signal = pyqtSignal(bool, str)
    error_signal = pyqtSignal(str, object)


class URRobotDriver(BaseRobotArmDriver):
    """
    UR robot driver using RTDE protocol.

    Thread-safe: emits Qt signals from internal RTDE thread.
    Application connects to these signals via Qt.QueuedConnection
    for safe cross-thread delivery.

    Supports both real hardware and mock mode (when RTDE unavailable).
    """

    DEFAULT_JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3"
    ]

    def __init__(self, joint_names: Optional[List[str]] = None):
        """Initialize UR driver."""
        super().__init__()

        self.signals = URRobotSignalHolder()
        self._mock_mode = not RTDE_AVAILABLE

        if self._mock_mode:
            logger.info("RTDE not available — operating in mock mode")

        self.joint_names = joint_names or self.DEFAULT_JOINT_NAMES.copy()

        # RTDE interfaces (initialized on connect)
        self._rtde_r = None
        self._rtde_c = None

        # Configuration
        self._frequency = 125
        self._ur_cap_port = 50002

        # State cache
        self._latest_state = {
            'joint_positions': [0.0] * len(self.joint_names),
            'joint_velocities': [0.0] * len(self.joint_names),
            'joint_currents': [0.0] * len(self.joint_names),
            'timestamp': 0.0,
            'tcp_pose': [0.0] * 6,
            'tcp_force': [0.0] * 6,
            'robot_status': 0,
            'safety_status': 0
        }

        self._last_receive_time = 0.0
        self._receive_timeout = 1.0

    # =================================================================
    # Connection Management
    # =================================================================

    def connect(self,
                ip: str,
                frequency: int = 125,
                ur_cap_port: int = 50002,
                max_retries: int = 10) -> bool:
        """
        Connect to UR robot with retry logic.

        Args:
            ip: Robot IP address.
            frequency: RTDE update frequency in Hz.
            ur_cap_port: UR control port.
            max_retries: Maximum connection attempts (0 = infinite).

        Returns:
            True if connection successful.
        """
        logger.info(f"Connecting to UR at {ip} "
                    f"(frequency={frequency}Hz, port={ur_cap_port})")

        if self._mock_mode:
            logger.info("Mock mode — simulating connection")
            self.is_connected = True
            self.robot_ip = ip
            self._frequency = frequency
            self.signals.connection_signal.emit(
                True, f"Mock connected to UR at {ip}"
            )
            return True

        self.robot_ip = ip
        self._frequency = frequency
        self._ur_cap_port = ur_cap_port

        retry_count = 0
        while max_retries == 0 or retry_count < max_retries:
            try:
                logger.info(f"Connection attempt {retry_count + 1}")

                self._rtde_c = RTDEControlInterface(ip, ur_cap_port=ur_cap_port)
                self._rtde_r = RTDEReceiveInterface(ip)

                # Test connection
                q_actual = self._rtde_r.getActualQ()
                if q_actual is None:
                    raise ConnectionError("Failed to get joint positions")
                if len(q_actual) != 6:
                    raise ConnectionError(
                        f"Expected 6 joints, got {len(q_actual)}"
                    )

                self.is_connected = True
                self._last_receive_time = time.time()
                self._update_state_from_rtde()
                self.signals.connection_signal.emit(
                    True, f"Connected to UR at {ip}"
                )
                logger.info(f"Connected successfully on attempt {retry_count + 1}")
                return True

            except Exception as e:
                logger.warning(f"Attempt {retry_count + 1} failed: {e}")
                self._cleanup_connections()
                retry_count += 1

                if max_retries == 0 or retry_count < max_retries:
                    time.sleep(2)

        logger.error(f"All {max_retries} connection attempts failed")
        self.is_connected = False
        self.signals.error_signal.emit(
            f"Failed to connect after {max_retries} attempts", None
        )
        self.signals.connection_signal.emit(
            False, f"Could not connect to {ip}"
        )
        return False

    def disconnect(self):
        """Disconnect from robot."""
        if self._mock_mode:
            self.is_connected = False
            self.signals.connection_signal.emit(False, "Mock disconnected")
            return

        self._cleanup_connections()
        self.is_connected = False
        self.signals.connection_signal.emit(False, "Disconnected")
        logger.info("Disconnected from robot")

    def _cleanup_connections(self):
        """Clean up RTDE connections."""
        if self._rtde_c is not None:
            try:
                self._rtde_c.disconnect()
            except Exception:
                pass
            self._rtde_c = None

        if self._rtde_r is not None:
            try:
                self._rtde_r.disconnect()
            except Exception:
                pass
            self._rtde_r = None

    # =================================================================
    # State Acquisition
    # =================================================================

    def _update_state_from_rtde(self):
        """Get latest state from RTDE and update cache."""
        if not self.is_connected or self._mock_mode or self._rtde_r is None:
            return

        try:
            q_actual = self._rtde_r.getActualQ()
            if q_actual is None:
                return

            self._last_receive_time = time.time()

            qd_actual = self._rtde_r.getActualQd()
            current = self._rtde_r.getActualCurrent()
            tcp_pose = self._rtde_r.getActualTCPPose()
            tcp_force = self._rtde_r.getActualTCPForce()
            robot_status = self._rtde_r.getRobotStatus()
            safety_status = self._rtde_r.getSafetyStatusBits()

            self._latest_state = {
                'joint_positions': (
                    q_actual if q_actual is not None
                    else self._latest_state['joint_positions']
                ),
                'joint_velocities': (
                    qd_actual if qd_actual is not None
                    else self._latest_state['joint_velocities']
                ),
                'joint_currents': (
                    current if current is not None
                    else self._latest_state['joint_currents']
                ),
                'timestamp': time.time(),
                'tcp_pose': (
                    tcp_pose if tcp_pose is not None
                    else self._latest_state['tcp_pose']
                ),
                'tcp_force': (
                    tcp_force if tcp_force is not None
                    else self._latest_state['tcp_force']
                ),
                'robot_status': robot_status,
                'safety_status': safety_status
            }

            self.signals.state_signal.emit(self._latest_state)

        except Exception:
            # Silently handle RTDE communication errors
            pass

    def get_current_state(self) -> Optional[Dict[str, Any]]:
        """
        Get latest cached robot state.

        Returns:
            State dictionary, or None if disconnected.
        """
        if not self.is_connected:
            return None

        if not self._mock_mode and self._rtde_r is not None:
            self._update_state_from_rtde()

        return self._latest_state.copy() if self._latest_state else None

    # =================================================================
    # Command Sending
    # =================================================================

    def send_joint_command(self, positions: List[float]) -> bool:
        """
        Send joint position command to robot.

        Args:
            positions: List of 6 joint angles in radians.

        Returns:
            True if command sent successfully.
        """
        if not self.is_connected:
            self.signals.error_signal.emit(
                "Cannot send command: Not connected", None
            )
            return False

        if not self.validate_joint_positions(positions):
            self.signals.error_signal.emit("Invalid joint positions", None)
            return False

        if self._mock_mode:
            self._latest_state['joint_positions'] = positions
            self._latest_state['timestamp'] = time.time()
            self.signals.state_signal.emit(self._latest_state)
            return True

        try:
            success = self._rtde_c.moveJ(positions, 0.5, 0.5)
            if success:
                self._latest_state['joint_positions'] = positions
            return success
        except Exception as e:
            self.signals.error_signal.emit(
                "Failed to send joint command", e
            )
            return False

    def send_cartesian_command(self,
                               pose: List[float],
                               speed: float = 0.5,
                               acceleration: float = 0.5) -> bool:
        """
        Send linear Cartesian move command (moveL).

        Args:
            pose: [x, y, z, rx, ry, rz] in meters and radians.
            speed: Tool speed in m/s.
            acceleration: Tool acceleration in m/s².

        Returns:
            True if command sent successfully.
        """
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            if self._mock_mode:
                logger.debug(f"Mock mode — would send Cartesian: {pose}")
                return True
            return False

        if len(pose) != 6:
            self.signals.error_signal.emit(
                f"Invalid pose length: {len(pose)}", None
            )
            return False

        try:
            self._rtde_c.moveL(pose, speed, acceleration)
            return True
        except Exception as e:
            self.signals.error_signal.emit(
                f"Failed to send Cartesian command: {e}", e
            )
            return False

    def get_inverse_kinematics(self,
                               target_pose: List[float],
                               q_guess: Optional[List[float]] = None
                               ) -> Optional[List[float]]:
        """
        Compute IK using robot's controller (no movement).

        Args:
            target_pose: [x, y, z, rx, ry, rz] in meters and radians.
            q_guess: Optional current joint angles for solution selection.

        Returns:
            List of 6 joint angles, or None if IK failed.
        """
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            if self._mock_mode:
                logger.debug(f"Mock mode — would compute IK for: {target_pose}")
                return q_guess if q_guess is not None else [0.0] * 6
            return None

        if len(target_pose) != 6:
            self.signals.error_signal.emit(
                f"Invalid pose length: {len(target_pose)}", None
            )
            return None

        if q_guess is None:
            q_guess = self._latest_state['joint_positions']

        try:
            q_solution = self._rtde_c.getInverseKinematics(
                target_pose, q_guess
            )
            return q_solution
        except Exception as e:
            self.signals.error_signal.emit(
                f"Failed to compute IK: {e}", e
            )
            return None

    # =================================================================
    # Status and Control
    # =================================================================

    def is_robot_moving(self) -> bool:
        """Check if robot is currently moving."""
        if not self.is_connected or self._mock_mode:
            return False
        try:
            qd = self._latest_state['joint_velocities']
            return any(abs(v) > 0.001 for v in qd)
        except Exception:
            return False

    def stop_motion(self):
        """Emergency stop — halt all motion."""
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            return
        try:
            self._rtde_c.stopScript()
        except Exception as e:
            self.signals.error_signal.emit("Failed to stop motion", e)

    # =================================================================
    # Convenience Methods
    # =================================================================

    def get_joint_positions_dict(self) -> Dict[str, float]:
        """Get joint positions as {name: value} dictionary."""
        state = self.get_current_state()
        if not state:
            return {}
        return dict(zip(self.joint_names, state['joint_positions']))

    def get_joint_positions(self) -> List[float]:
        """Get joint positions as list in standard UR order."""
        state = self.get_current_state()
        if not state:
            return [0.0] * 6
        return state.get('joint_positions', [0.0] * 6)

    def get_tcp_pose(self) -> Optional[List[float]]:
        """Get TCP pose [x, y, z, rx, ry, rz]."""
        state = self.get_current_state()
        return state['tcp_pose'] if state else None

    def get_robot_status_string(self) -> str:
        """Get human-readable robot status."""
        if not self.is_connected:
            return "Disconnected"
        if self._mock_mode:
            return "Mock Mode"
        return "Connected"

    # =================================================================
    # Callback Setters (using Qt signals)
    # =================================================================

    def set_state_callback(self, callback):
        """Connect to state signal."""
        self.signals.state_signal.connect(callback)

    def set_connection_callback(self, callback):
        """Connect to connection signal."""
        self.signals.connection_signal.connect(callback)

    def set_error_callback(self, callback):
        """Connect to error signal."""
        self.signals.error_signal.connect(callback)