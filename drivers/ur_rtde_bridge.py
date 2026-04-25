"""
UR RTDE robot arm driver implementation with thread-safe Qt signals.
Uses composition to avoid metaclass conflicts.
"""

import time
from typing import Dict, List, Optional, Any

# Import UR RTDE - with fallback for development without hardware
try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False
    pass

from PyQt5.QtCore import QObject, pyqtSignal
from .base_robot_arm import BaseRobotArmDriver


class URRobotSignalHolder(QObject):
    """Separate QObject to hold Qt signals (avoids metaclass conflicts)."""
    state_signal = pyqtSignal(dict)
    connection_signal = pyqtSignal(bool, str)
    error_signal = pyqtSignal(str, object)


class URRobotDriver(BaseRobotArmDriver):
    """
    UR robot driver using RTDE protocol.
    Thread-safe: emits Qt signals that can be connected to main thread slots.
    """
    
    # UR joint names in standard order (from base to tip)
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
        
        # Create signal holder (composition, not inheritance)
        self.signals = URRobotSignalHolder()
        
        if not RTDE_AVAILABLE:
            self._mock_mode = True
        else:
            self._mock_mode = False
        
        # Set joint names (respect URDF naming)
        self.joint_names = joint_names or self.DEFAULT_JOINT_NAMES.copy()
        
        # RTDE interfaces (initialized on connect)
        self._rtde_r = None  # RTDEReceiveInterface
        self._rtde_c = None  # RTDEControlInterface
        
        # Configuration
        self._frequency = 125  # Hz
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
        
        # Default thresholds
        self.joint_threshold = 0.001    # radians
        self.position_threshold = 0.0005    # half mm in meters
        self.rotation_threshold = 0.001     # radians for TCP orientation

        self._last_significant_state = None
        self._last_receive_time = 0
        self._receive_timeout = 1.0  # seconds
        
    def connect(self, ip: str, frequency: int = 125, ur_cap_port: int = 50002, max_retries: int = 10) -> bool:
        """Connect to UR robot with retry logic."""
        print(f"\n=== UR ROBOT DRIVER CONNECT ===")
        print(f"📡 Target IP: {ip}")
        print(f"   Frequency: {frequency} Hz")
        print(f"   Port: {ur_cap_port}")
        print(f"   Max retries: {max_retries}")
        print(f"   Mock mode: {self._mock_mode}")
        
        if self._mock_mode:
            print("🔧 MOCK MODE: Simulating connection")
            self.is_connected = True
            self.robot_ip = ip
            self._frequency = frequency
            self.signals.connection_signal.emit(True, f"Mock connected to UR at {ip}")
            return True
        
        self.robot_ip = ip
        self._frequency = frequency
        self._ur_cap_port = ur_cap_port
        
        retry_count = 0
        while max_retries == 0 or retry_count < max_retries:
            try:
                print(f"\n🔄 Attempt {retry_count + 1}/{max_retries if max_retries > 0 else '∞'}")
                
                # Try to create control interface
                print("   Creating RTDEControlInterface...")
                self._rtde_c = RTDEControlInterface(ip, ur_cap_port=ur_cap_port)
                print(f"   ✓ RTDEControlInterface created: {self._rtde_c}")
                
                # Try to create receive interface
                print("   Creating RTDEReceiveInterface...")
                self._rtde_r = RTDEReceiveInterface(ip)
                print(f"   ✓ RTDEReceiveInterface created: {self._rtde_r}")
                
                # Test connection by getting data
                print("   Testing connection - getting joint positions...")
                q_actual = self._rtde_r.getActualQ()
                print(f"   Received joint positions: {q_actual}")
                
                if q_actual is None:
                    print("   ❌ Received None from robot")
                    raise ConnectionError("Failed to get joint positions")
                
                if len(q_actual) != 6:
                    print(f"   ❌ Expected 6 joints, got {len(q_actual)}")
                    raise ConnectionError(f"Expected 6 joints, got {len(q_actual)}")
                
                # Success!
                print(f"✅ Connection successful on attempt {retry_count + 1}")
                self.is_connected = True
                self._last_receive_time = time.time()
                self._update_state_from_rtde()
                self.signals.connection_signal.emit(True, f"Connected to UR at {ip}")
                return True
                
            except Exception as e:
                print(f"❌ Attempt {retry_count + 1} failed: {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
                
                self._cleanup_connections()
                retry_count += 1
                
                if max_retries == 0 or retry_count < max_retries:
                    wait = 2
                    print(f"⏳ Waiting {wait} seconds before retry...")
                    time.sleep(wait)
        
        # All retries exhausted
        print(f"❌ All {max_retries} attempts failed")
        self.is_connected = False
        self.signals.error_signal.emit(f"Failed to connect after {max_retries} attempts", None)
        self.signals.connection_signal.emit(False, f"Could not connect to {ip}")
        return False
    
    def _cleanup_connections(self):
        """Clean up RTDE connections."""
        if self._rtde_c is not None:
            try:
                self._rtde_c.disconnect()
            except:
                pass
            self._rtde_c = None
        
        if self._rtde_r is not None:
            try:
                self._rtde_r.disconnect()
            except:
                pass
            self._rtde_r = None

    def set_thresholds(self, joint_rad=0.001, pos_m=0.0005, rot_rad=0.001):
        """Allow control panel to adjust sensitivity."""
        self.joint_threshold = joint_rad
        self.position_threshold = pos_m
        self.rotation_threshold = rot_rad
    
    def _significant_change(self, new_state) -> bool:
        """Check if state change is significant enough to emit."""
        if self._last_significant_state is None:
            return True
        
        # Check joint changes
        if 'joint_positions' in new_state and 'joint_positions' in self._last_significant_state:
            old_joints = self._last_significant_state['joint_positions']
            new_joints = new_state['joint_positions']
            
            max_joint_change = max(abs(new_joints[i] - old_joints[i]) 
                                for i in range(len(new_joints)))
            
            if max_joint_change > self.joint_threshold:
                return True
        
        # Check TCP position change
        if ('tcp_pose' in new_state and 'tcp_pose' in self._last_significant_state and
            new_state['tcp_pose'] is not None and self._last_significant_state['tcp_pose'] is not None):
            
            # Check position change (first 3 elements)
            old_pos = self._last_significant_state['tcp_pose'][:3]
            new_pos = new_state['tcp_pose'][:3]
            max_pos_change = max(abs(new_pos[i] - old_pos[i]) for i in range(3))
            
            if max_pos_change > self.position_threshold:
                return True
            
            # Check orientation change (last 3 elements)
            old_rot = self._last_significant_state['tcp_pose'][3:6]
            new_rot = new_state['tcp_pose'][3:6]
            max_rot_change = max(abs(new_rot[i] - old_rot[i]) for i in range(3))
            
            if max_rot_change > self.rotation_threshold:
                return True
        
        return False

    def _update_state_from_rtde(self):
        """Get latest state from RTDE and update cache."""
        if not self.is_connected or self._mock_mode or self._rtde_r is None:
            return
        
        try:
            # Get joint data
            q_actual = self._rtde_r.getActualQ()

            if q_actual is None:
                return
                
            # Update timestamp on successful receive
            self._last_receive_time = time.time()

            qd_actual = self._rtde_r.getActualQd()
            current = self._rtde_r.getActualCurrent()
            
            # Get TCP data
            tcp_pose = self._rtde_r.getActualTCPPose()
            tcp_force = self._rtde_r.getActualTCPForce()
            
            # Get status
            robot_status = self._rtde_r.getRobotStatus()
            safety_status = self._rtde_r.getSafetyStatusBits()
            
            # Update cache
            self._latest_state = {
                'joint_positions': q_actual if q_actual is not None else self._latest_state['joint_positions'],
                'joint_velocities': qd_actual if qd_actual is not None else self._latest_state['joint_velocities'],
                'joint_currents': current if current is not None else self._latest_state['joint_currents'],
                'timestamp': time.time(),
                'tcp_pose': tcp_pose if tcp_pose is not None else self._latest_state['tcp_pose'],
                'tcp_force': tcp_force if tcp_force is not None else self._latest_state['tcp_force'],
                'robot_status': robot_status,
                'safety_status': safety_status
            }
            
            # Only emit if significant change
            # if self._significant_change(self._latest_state):
            self._last_significant_state = self._latest_state.copy()
            print(f"[DRIVER] Going to emit state signal")
            self.signals.state_signal.emit(self._latest_state)
            print(f"[DRIVER] state signal EMITTED")
            
        except Exception as e:
            # Silently handle RTDE errors to avoid console spam
            pass
    
    def send_joint_command(self, positions: List[float]) -> bool:
        """Send joint position command to robot."""
        if not self.is_connected:
            self.signals.error_signal.emit("Cannot send command: Not connected", None)
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
            # Send joint position command using moveJ
            success = self._rtde_c.moveJ(positions, 0.5, 0.5)
            
            if success:
                self._latest_state['joint_positions'] = positions
            return success
            
        except Exception as e:
            self.signals.error_signal.emit("Failed to send joint command", e)
            return False
    
    # ===== NEW METHODS FOR CARTESIAN CONTROL =====
    
    def send_cartesian_command(self, pose: List[float], speed: float = 0.5, acceleration: float = 0.5) -> bool:
        """
        Send linear Cartesian move command (moveL).
        
        Args:
            pose: [x, y, z, rx, ry, rz] in meters and radians
            speed: m/s (default 0.5)
            acceleration: m/s² (default 0.5)
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            if self._mock_mode:
                print(f"🔧 MOCK MODE: Would send Cartesian command: {pose}")
                return True
            return False
        
        if len(pose) != 6:
            self.signals.error_signal.emit(f"Invalid pose length: {len(pose)}", None)
            return False
        
        try:
            print(f"📡 Sending moveL: {pose}")
            self._rtde_c.moveL(pose, speed, acceleration)   # , asynchronous=True)
            return True
        except Exception as e:
            self.signals.error_signal.emit(f"Failed to send Cartesian command: {e}", e)
            return False
    
    def get_inverse_kinematics(self, target_pose: List[float], q_guess: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Compute IK for target pose using robot's controller.
        
        Args:
            target_pose: [x, y, z, rx, ry, rz] in meters and radians
            q_guess: optional current joint angles (list of 6)
        
        Returns:
            List of 6 joint angles in radians, or None if failed
        """
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            if self._mock_mode:
                print(f"🔧 MOCK MODE: Would compute IK for pose: {target_pose}")
                # Return a mock solution (just return q_guess or zeros)
                if q_guess is not None:
                    return q_guess
                return [0.0] * 6
            return None
        
        if len(target_pose) != 6:
            self.signals.error_signal.emit(f"Invalid pose length: {len(target_pose)}", None)
            return None
        
        try:
            # Use current joint positions as guess if not provided
            if q_guess is None:
                q_guess = self._latest_state['joint_positions']
            
            print(f"📡 Computing IK for pose: {target_pose}")
            q_solution = self._rtde_c.getInverseKinematics(target_pose, q_guess)
            print(f"   IK solution: {q_solution}")
            return q_solution
        except Exception as e:
            self.signals.error_signal.emit(f"Failed to compute IK: {e}", e)
            return None
    
    # ===== End of new methods =====
    
    def get_current_state(self) -> Optional[Dict[str, Any]]:
        """Get latest robot state."""
        if not self.is_connected:
            return None
        
        # Don't report timeout if we're just starting up
        if not self._mock_mode and time.time() - self._last_receive_time > self._receive_timeout:
            self._last_receive_time = time.time()  # Reset timer to avoid error spam
            return self._latest_state.copy() if self._latest_state else None
        
        # Update state from RTDE if connected
        if not self._mock_mode and self._rtde_r is not None:
            self._update_state_from_rtde()
        
        return self._latest_state.copy() if self._latest_state else None
    
    def is_robot_moving(self) -> bool:
        """Check if robot is moving."""
        if not self.is_connected or self._mock_mode:
            return False
        
        try:
            qd = self._latest_state['joint_velocities']
            return any(abs(v) > 0.001 for v in qd)
        except:
            return False
    
    def stop_motion(self):
        """Emergency stop - halt all motion."""
        if not self.is_connected or self._mock_mode or self._rtde_c is None:
            return
        
        try:
            self._rtde_c.stopScript()
        except Exception as e:
            self.signals.error_signal.emit("Failed to stop motion", e)
    
    def disconnect(self):
        """Disconnect from robot."""
        if self._mock_mode:
            self.is_connected = False
            self.signals.connection_signal.emit(False, "Mock disconnected")
            return
        
        self._cleanup_connections()
        self.is_connected = False
        self.signals.connection_signal.emit(False, "Disconnected")
    
    # ===== Convenience methods =====
    
    def get_joint_positions_dict(self) -> Dict[str, float]:
        """
        Get joint positions as dictionary mapping joint names to values.
        
        Returns:
            Dict: {joint_name: position}
        """
        state = self.get_current_state()
        if not state:
            return {}
        
        positions = state['joint_positions']
        return dict(zip(self.joint_names, positions))

    def get_joint_positions(self) -> List[float]:
        """
        Get joint positions as a list in standard UR order.
        
        Returns:
            List of 6 joint positions in radians
        """
        state = self.get_current_state()
        if not state:
            return [0.0] * 6
        
        return state.get('joint_positions', [0.0] * 6)

    def get_tcp_pose(self) -> Optional[List[float]]:
        """Get TCP pose [x,y,z,rx,ry,rz]."""
        state = self.get_current_state()
        return state['tcp_pose'] if state else None
    
    def get_robot_status_string(self) -> str:
        """Get human-readable robot status."""
        if not self.is_connected:
            return "Disconnected"
        
        if self._mock_mode:
            return "Mock Mode"
        
        status = self._latest_state.get('robot_status', 0)
        safety = self._latest_state.get('safety_status', 0)
        
        # Decode status bits (simplified)
        if status & 1:  # Bit 0: Power on
            return "Powered On"
        elif status & 2:  # Bit 1: Program running
            return "Running"
        else:
            return f"Status: {status}, Safety: {safety}"
    
    # ===== Callback setters using signals =====
    
    def set_state_callback(self, callback):
        """Connect to state signal."""
        self.signals.state_signal.connect(callback)
    
    def set_connection_callback(self, callback):
        """Connect to connection signal."""
        self.signals.connection_signal.connect(callback)
    
    def set_error_callback(self, callback):
        """Connect to error signal."""
        self.signals.error_signal.connect(callback)