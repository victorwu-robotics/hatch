# Camera Integration in Hatch

## Overview

Hatch supports RGB-D cameras (Orbbec Gemini 335, Intel RealSense D435) as point cloud sources. The camera pipeline follows Hatch's event-driven architecture: a driver captures raw data, a processor filters and transforms it, and a renderer displays it in the 3D view. Everything runs in the main thread — no separate threads, no signal queues, no growing latency.

## Architecture

The pipeline has three stages, each in its own file:

```
drivers/camera/                   # Hardware interaction
├── base_camera.py                # Abstract interface
└── orbbec_capture.py             # Orbbec-specific driver

displays/pointcloud/              # Processing and display
├── pointcloud_display.py         # Orchestrator (capture → process → render)
├── pointcloud_processor.py       # ROI clipping, transform to world
├── pointcloud_renderer.py        # VTK actor management
└── streaming_pointcloud.py       # Zero-copy VTK point cloud

ui/managers/
└── camera_manager.py             # UI orchestration (dock, panel, lifecycle)
```

### Data Flow

```
OrbbecDriver.capture_raw_pointcloud()
    ↓ raw (N,3) points in optical frame
PointCloudProcessor.process_frame()
    ↓ ROI clipping, world transform
PointCloudRenderer.update_point_cloud()
    ↓ zero-copy VTK update, sets _needs_render = True
VisualizerEngine (60Hz timer)
    ↓ checks _needs_render, calls Render()
Screen
```

## Adding a Camera to Your Scene

### Step 1: Include the Camera in Your URDF

Your scene URDF must include the camera's URDF and mount it to the robot. For an Orbbec Gemini 335:

```xml
<!-- Include the camera definition -->
<xacro:include filename="$(find orbbec_camera)/urdf/gemini_335_336.urdf.xacro"/>

<!-- Mount the camera to the robot wrist -->
<joint name="wrist_to_camera" type="fixed">
  <parent link="wrist_3_link"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

The camera URDF defines several frames:
- `camera_link` — the physical camera body
- `camera_depth_frame` — depth sensor origin
- `camera_depth_optical_frame` — depth sensor in optical convention (X right, Y down, Z forward)
- `camera_color_frame` — RGB sensor origin
- `camera_color_optical_frame` — RGB sensor in optical convention

Hatch uses `camera_depth_optical_frame` for point cloud data because the Orbbec SDK outputs points in this frame.

### Step 2: Camera Frame Auto-Detection

When the camera starts, `PointCloudDisplay` automatically finds the depth optical frame by scanning the kinematic model for link names containing `depth_optical_frame`. No manual configuration is needed. The detected frame name is prefixed with the asset ID and used to query the `TransformRegistry` for the world transform.

If your camera uses a different naming convention, override by calling:
```python
pointcloud_display.set_camera_frame("your_frame_name")
```

### Step 3: Transform to World Frame

The `PointCloudProcessor` queries the `TransformRegistry` for the world transform of the camera optical frame on every processed frame. As the robot moves, the camera frame's world transform updates, and subsequent point clouds are correctly positioned in world coordinates.

The processor also supports ROI (Region of Interest) clipping in the X, Y, and Z axes to filter out points outside the work area.

## Camera Control Panel

The camera is controlled through a dock widget on the right side of the main window. The panel provides:

- **Start/Stop** — begin or end camera streaming
- **Camera Type** — switch between Orbbec and RealSense (when multiple cameras are available)
- **Resolution** — select from camera-supported resolutions
- **ROI Settings** — clip point cloud to a 3D bounding box
- **Transform to World** — toggle world-frame transformation
- **Show Camera Frames** — toggle visibility of camera coordinate axes
- **Point Cloud Visibility** — show/hide the point cloud

## Performance Characteristics

At 640×360 resolution, the Orbbec Gemini 335 produces approximately 210,000 points per frame. The single-threaded pipeline processes this at 17-20 FPS:

| Stage | Time |
|-------|------|
| Capture | ~9ms |
| Process (ROI + transform) | ~8ms |
| Render (VTK) | ~38ms |
| **Total** | **~55ms (18 FPS)** |

The render time is the dominant factor. For applications requiring higher frame rates, consider downsampling the point cloud before rendering (see Performance Tuning below).

## Performance Tuning

### Downsampling

If frame rate is more important than point density, randomly downsample before rendering:

```python
if len(points) > 50000:
    indices = np.random.choice(len(points), 50000, replace=False)
    points = points[indices]
    colors = colors[indices]
```

### Point Size

Adjust the rendered point size based on your working distance:

```python
# In StreamingPointCloud.__init__:
self.actor.GetProperty().SetPointSize(2)  # Pixels (default)
self.actor.GetProperty().SetPointSize(3)  # Larger for better visibility
```

### Dynamic Point Size

For applications with varying camera-to-scene distance, scale point size inversely with distance:

```python
if self.renderer:
    camera = self.renderer.GetActiveCamera()
    pos = np.array(camera.GetPosition())
    focal = np.array(camera.GetFocalPoint())
    distance = np.linalg.norm(pos - focal)
    size = max(1, min(6, 30.0 / distance))
    self.actor.GetProperty().SetPointSize(size)
```

## Adding a New Camera Type

To add support for a new depth camera:

1. Create a driver file in `drivers/camera/` (e.g., `new_camera.py`)
2. Inherit from `BaseCameraDriver` and implement `start_streaming()`, `capture_raw_pointcloud()`, `stop_streaming()`
3. Register the camera type in `CameraManager.available_cameras`
4. Add resolution presets in `CameraManager.camera_resolutions`
5. Add the import branch in `PointCloudDisplay.start_camera()`

The processor and renderer work with any camera that produces (N,3) float32 points and (N,3) uint8 colors — no changes needed.

## Installation Requirements

```bash
# For Orbbec Gemini 335:
pip install pyorbbecsdk2

# For Intel RealSense D435:
pip install pyrealsense2
```

Note: The Orbbec SDK package is named `pyorbbecsdk2` on PyPI but imported as `pyorbbecsdk`.

