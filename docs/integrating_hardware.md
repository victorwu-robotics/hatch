# Integrating Hardware with Hatch

This guide covers how to connect external hardware to Hatch: robot arms,
cameras, and sensors. It focuses on the **event-driven integration pattern**
that Hatch uses for all hardware, and provides case studies of real
integrations.

**Recommended OS:** Ubuntu 20.04. Windows and macOS have not been tested.

---

## The Event-Driven Pattern

All hardware in Hatch follows the same pattern:

```
Hardware Driver (your code)
    ↓
Publishes events via StateChannel
    ↓
Hatch components receive and act on them
```

The driver does not call Hatch methods directly. It publishes events.
Hatch subscribes to those events. This keeps the driver decoupled from
the platform.

### What the Driver Must Do

1. **Connect to hardware** (network, USB, serial, etc.)
2. **Read data** continuously in a background thread
3. **Publish events** when data arrives or state changes
4. **Handle disconnections** gracefully

### What the Driver Must NOT Do

- Import Hatch UI components
- Call `RobotManager` or `KinematicModel` directly
- Modify the `TransformRegistry` directly
- Block the main thread

---

## Connecting to a UR Robot

Hatch uses `ur_rtde` to communicate with Universal Robots. The connection is automatic -- no program needs to run on the teach pendant.

**Requirements:**
- Robot powered on, brake released
- e-Series: Remote Control Mode enabled (PolyScope -> Settings -> System -> Remote Control)
- Same network, ports 30001-30004 open

**Steps:**
1. Enter robot IP in Hatch's Robot Connection panel
2. Click Connect

`ur_rtde` automatically uploads its control script to the robot. The robot moves when you command it.

**If connection fails:**
- Press **Stop** on teach pendant to clear stuck script from crashed session
- Verify network: `ping ROBOT_IP`
- Check firewall: ports 30001-30004

**Advanced: External Control URCap**

Only needed if you want to mix Python control with pendant logic (waits, I/O triggers). See [Universal Robots GitHub](https://github.com/UniversalRobots/Universal_Robots_ExternalControl) for installation.

---

## Case Study: The RTDE Driver

The Universal Robots RTDE (Real-Time Data Exchange) driver is the most
complex hardware integration in Hatch. It serves as a reference for all
other hardware drivers.

### The Challenge

UR robots communicate via RTDE, a high-frequency binary protocol. The
protocol requires:
- A persistent TCP connection
- Continuous data streaming at 125-500 Hz
- Thread-safe access from the main UI thread
- Graceful handling of connection drops and robot protective stops

### The Solution: Event-Driven Threading

class URRobotDriver(QObject):
    """UR robot driver using RTDE protocol."""

    # Qt signals for thread-safe communication
    state_received = pyqtSignal(dict)  # Emitted when new state arrives
    connection_established = pyqtSignal(str)
    connection_lost = pyqtSignal(str)

    def __init__(self, state_channel):
        super().__init__()
        self._channel = state_channel
        self._rtde_c = None
        self._rtde_r = None
        self._running = False
        self._thread = None

    def connect(self, ip, frequency=500.0):
        """Connect to robot and start background thread."""
        self._rtde_c = RTDEControlInterface(ip, frequency)
        self._rtde_r = RTDEReceiveInterface(ip, frequency)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop)
        self._thread.start()

        self.connection_established.emit(f"Connected to {ip}")

    def _read_loop(self):
        """Background thread: continuously read robot state."""
        while self._running:
            try:
                state = self._rtde_r.getRobotStatus()
                self.state_received.emit(state)
            except Exception as e:
                self.connection_lost.emit(str(e))
                break
            time.sleep(0.002)  # 500 Hz

    def _on_state_received(self, state):
        """Called in main thread via signal."""
        self._channel.publish(
            EventType.ROBOT_STATE,
            data={
                'joint_positions': state['actual_q'],
                'tcp_pose': state['actual_TCP_pose'],
                'timestamp': time.time(),
                'source': 'ur_rtde'
            },
            source='ur_rtde'
        )

    def move_joints(self, positions):
        """Send joint command to robot."""
        if self._rtde_c:
            self._rtde_c.moveJ(positions, speed=1.0, acceleration=1.0)

### Key Design Decisions

**Thread separation:** The background thread reads from RTDE continuously.
The main thread receives data via Qt signals. This prevents the UI from
freezing during network operations.

**Signal bridging:** `pyqtSignal` is the only Qt mechanism used in the
driver. It bridges the background thread to the main thread safely.

**Event publishing:** The driver never calls Hatch methods. It publishes
`ROBOT_STATE` events. `StateHandler` subscribes and updates the model.

**Connection state:** The driver emits `connection_established` and
`connection_lost` signals. The UI subscribes and updates the connection
panel.

---

## Case Study: The Camera Pipeline

The camera pipeline demonstrates how to integrate a sensor that produces
continuous data streams.

### The Challenge

RGB-D cameras produce:
- Color images (1920x1080, 30 FPS)
- Depth images (640x480, 30 FPS)
- Point clouds (variable size, 30 FPS)

All of this must happen without blocking the UI thread.

### The Solution: Three-Stage Pipeline

```
Camera Hardware
    ↓ (USB/Network)
Camera Driver (background thread)
    ↓ (raw frames)
Frame Processor (background thread)
    ↓ (point cloud)
StateChannel.publish(POINT_CLOUD)
    ↓
PointCloudDisplay (main thread, via signal)
```

### Stage 1: Camera Driver

class CameraDriver(QObject):
    """Background thread that reads from camera hardware."""

    frame_ready = pyqtSignal(np.ndarray, np.ndarray)  # color, depth

    def __init__(self, device_id=0):
        super().__init__()
        self._device_id = device_id
        self._running = False

    def start(self):
        self._cap = cv2.VideoCapture(self._device_id)
        self._running = True
        threading.Thread(target=self._capture_loop).start()

    def _capture_loop(self):
        while self._running:
            ret, color = self._cap.read()
            if ret:
                depth = self._capture_depth()  # Device-specific
                self.frame_ready.emit(color, depth)

### Stage 2: Frame Processor

class FrameProcessor(QObject):
    """Converts depth frames to point clouds."""

    point_cloud_ready = pyqtSignal(np.ndarray)  # Nx3 array

    def __init__(self, camera_intrinsics):
        super().__init__()
        self._intrinsics = camera_intrinsics

    def process_frame(self, color, depth):
        """Called when new frame arrives."""
        points = self._depth_to_pointcloud(depth, self._intrinsics)
        self.point_cloud_ready.emit(points)

    def _depth_to_pointcloud(self, depth, K):
        """Vectorized depth-to-point-cloud conversion."""
        # ... implementation ...
        return points  # Nx3 array

### Stage 3: Point Cloud Publisher

class PointCloudPublisher:
    """Publishes point clouds to Hatch's event system."""

    def __init__(self, state_channel, transform_registry, camera_frame):
        self._channel = state_channel
        self._registry = transform_registry
        self._camera_frame = camera_frame

    def publish(self, points):
        """Transform points to world frame and publish."""
        T = self._registry.get_transform(self._camera_frame, "world")
        points_world = (T[:3, :3] @ points.T + T[:3, 3:4]).T

        self._channel.publish(
            EventType.POINT_CLOUD,
            data={
                'points': points_world,
                'frame': 'world',
                'timestamp': time.time(),
                'source': self._camera_frame
            },
            source='camera_pipeline'
        )

### Wiring It Together

```python
# In MainWindow or your initialization code:
camera = CameraDriver(device_id=0)
processor = FrameProcessor(K=load_camera_intrinsics())
publisher = PointCloudPublisher(
    state_channel=state_channel,
    transform_registry=registry,
    camera_frame="camera_depth_optical_frame"
)

# Connect the pipeline
camera.frame_ready.connect(processor.process_frame)
processor.point_cloud_ready.connect(publisher.publish)

# Start capturing
camera.start()
```

### Key Design Decisions

**Thread separation:** Camera I/O happens in a background thread. Processing
happens in another thread. Only the final publish happens in the main thread.

**No direct calls:** The pipeline uses Qt signals between stages. The final
stage publishes to `StateChannel`. No component knows about the others.

**Transform lookup:** The publisher queries `TransformRegistry` for the
camera's current pose. As the robot moves, the point cloud automatically
follows.

---

## Case Study: The Keyence Laser Scanner

The Keyence LJ-V7200 integration demonstrates how to reverse-engineer a
proprietary protocol and integrate it into Hatch.

### The Challenge

Keyence provides a Windows-only SDK with no Linux support. The protocol is
proprietary binary over TCP. The documentation is in Japanese and English,
but omits critical details about coordinate systems and data formats.

### The Solution: Protocol Reverse-Engineering

class KeyenceDriver(QObject):
    """Keyence LJ-V7200 laser scanner driver."""

    profile_ready = pyqtSignal(np.ndarray)  # 2D profile points

    def __init__(self, state_channel):
        super().__init__()
        self._channel = state_channel
        self._socket = None

    def connect(self, ip, port=24685):
        """Connect to Keyence controller."""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((ip, port))

        # Send initialization command (reverse-engineered)
        init_cmd = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._socket.send(init_cmd)

        threading.Thread(target=self._read_loop).start()

    def _read_loop(self):
        """Read profiles from Keyence controller."""
        while True:
            header = self._socket.recv(16)
            if len(header) < 16:
                break

            data_len = struct.unpack('<I', header[4:8])[0]
            data = self._socket.recv(data_len)

            profile = self._parse_profile(data)
            self.profile_ready.emit(profile)

    def _parse_profile(self, data):
        """Parse binary profile data into point array."""
        # Keyence sends packed 16-bit unsigned integers
        # Each value is a Z height in 0.1 micrometer units
        # X positions are implied by the profile index

        values = np.frombuffer(data, dtype=np.uint16)
        z = values * 0.0001  # Convert to millimeters

        # X positions: Keyence sends 800 points per profile
        x = np.linspace(-20.0, 20.0, len(z))  # -20mm to +20mm

        # Filter invalid points (Keyence uses 0xFFFF for no data)
        valid = values != 0xFFFF
        return np.column_stack([x[valid], np.zeros(np.sum(valid)), z[valid]])

### Key Design Decisions

**Protocol reverse-engineering:** The Keyence protocol was captured using
Wireshark and analyzed byte-by-byte. The initialization command, data format,
and coordinate system were all determined experimentally.

**Coordinate transformation:** Keyence uses a right-handed coordinate system
with Z up. Hatch uses the same convention, but the scanner's mounting position
must be specified in the URDF.

**Error handling:** The driver handles connection drops, invalid data, and
timeouts gracefully. It publishes `ERROR_OCCURRED` events when something
goes wrong.

---

## Adding Your Own Hardware

To add a new sensor or actuator to Hatch:

1. **Create a driver class** that inherits `QObject` (if using Qt signals)
   or runs in a plain Python thread (if not)

2. **Connect to hardware** in a background thread

3. **Publish events** via `StateChannel` when data arrives

4. **Subscribe to events** if your hardware needs commands (e.g., `JOINT_COMMAND`)

5. **Clean up** on disconnect: stop threads, close sockets, unsubscribe

### Example: A Simple Force Sensor

class ForceSensor(QObject):
    """Simple force sensor driver."""

    force_received = pyqtSignal(np.ndarray)  # [fx, fy, fz, tx, ty, tz]

    def __init__(self, state_channel, serial_port='/dev/ttyUSB0'):
        super().__init__()
        self._channel = state_channel
        self._serial = serial.Serial(serial_port, baudrate=115200)
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._read_loop).start()

    def _read_loop(self):
        while self._running:
            line = self._serial.readline().decode().strip()
            if line:
                forces = np.array([float(x) for x in line.split(',')])
                self.force_received.emit(forces)

                self._channel.publish(
                    EventType.FORCE_TORQUE,
                    data={
                        'forces': forces[:3],
                        'torques': forces[3:],
                        'timestamp': time.time(),
                        'source': 'force_sensor'
                    },
                    source='force_sensor'
                )

    def stop(self):
        self._running = False
        self._serial.close()

---

## Common Pitfalls

### Blocking the Main Thread

**Don't:** Read from hardware in the main thread.

**Do:** Use a background thread and Qt signals to communicate with the main thread.

### Forgetting to Unsubscribe

**Don't:** Subscribe to events and never unsubscribe.

**Do:** Call `channel.unsubscribe()` in your cleanup method.

### Modifying Hatch State Directly

**Don't:** Call `robot_manager.move_joints()` from your driver.

**Do:** Publish `ROBOT_STATE` events and let `StateHandler` update the model.

### Hardcoded Paths and IPs

**Don't:** Hardcode device IPs or serial ports.

**Do:** Accept them as constructor arguments or read from configuration.

---

## Further Reading

| Document | What It Covers |
|----------|---------------|
| [User Guide](user_guide.md) | Using Hatch -- scene creation, robot control, 3D view |
| [Architecture](architecture.md) | How Hatch works internally -- components, principles, event flow |
| [Technical Notes](technical_notes.md) | Deep dives on coordinate systems, URDF parsing, mesh loading |

---

*This guide covers Hatch v1.0 hardware integration. Contributions welcome.*
