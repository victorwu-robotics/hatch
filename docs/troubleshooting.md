# Troubleshooting

**Recommended OS:** Ubuntu 20.04. Windows and macOS have not been tested.

---

## VTK / 3D View Issues

### Black screen or no 3D view appears

**Symptoms:** Window opens but the central area is black or empty.

**Checks:**
1. Verify VTK and PyQt5 versions match `requirements.txt`
2. On Linux with Wayland: `export QT_QPA_PLATFORM=xcb` before running
3. On macOS: VTK Qt widgets may need `export VTK_USE_COCOA=ON`
4. Check console for VTK initialization errors

**Debug:**
```python
# Add to top of main_window.py
import vtk
print(vtk.vtkVersion.GetVTKVersion())
```

---

## URDF Loading Issues

### Robot loads but meshes don't appear (red boxes instead)

**Symptoms:** Robot structure is visible as red cubes, not actual meshes.

**Cause:** Mesh path resolution failed. Hatch searches:
- The URDF file's directory
- The URDF's parent directory
- `~/hatch/assets/` subdirectories
- `~/.cache/robot_descriptions/`

**Fix:**
- Use absolute paths in URDF: `file:///absolute/path/to/mesh.stl`
- Or organize files following ROS package conventions:
  ```
  package_name/
  ├── urdf/robot.urdf
  └── meshes/base_link.stl
  ```
- Check console for `"Mesh not found for ..."` warnings

### "Failed to load robot" dialog

**Symptoms:** Error dialog appears after selecting URDF.

**Common causes:**
- Invalid XML in URDF (run through `xmllint` or open in browser)
- Xacro features Hatch doesn't support (conditionals, nested macros)
- Missing `package://` resolution — place package in search path

**Workaround:** Preprocess xacro manually: `xacro robot.xacro > robot.urdf`, then load the `.urdf`.

---

## Inverse Kinematics Issues

### "IK failed" when using Cartesian control

**Symptoms:** Robot doesn't move, error dialog shows "IK failed".

**Checks:**
1. Is the target pose reachable? Try small movements (±10mm) from current position.
2. Check joint limits in URDF — some poses may be mathematically reachable but outside limits.
3. In `SIMULATE_LOCAL` mode: verify `IKSolver` detected correct wrist type (check console: "Detected spherical wrist" or "Detected offset wrist").

**Debug:**
```python
# In console or debug panel
from core.kinematics.ik_solver import IKSolver
ik = IKSolver(model)
print(ik.wrist_type)  # Should match your robot
```

---

## RTDE / Hardware Connection Issues

### Cannot connect to UR robot

**Symptoms:** "Connection failed after N attempts" or timeout.

**Checks:**
1. **Verify robot is in Remote Control Mode** (e-Series: PolyScope → Settings → System → Remote Control)
2. **Verify IP addresses** — robot and Hatch machine on same subnet, reachable via `ping`
3. **Press Stop on teach pendant** — clears stuck script from previous crashed session
4. **Check firewall** — ports 30001-30004 must be open
5. Try increasing `max_retries` in `URRobotDriver.connect()`

### Connection drops during operation

**Symptoms:** Robot stops, UI shows "Connection Lost".

**Causes:**
- Network instability (WiFi)
- Robot controller reboot or protective stop
- Previous session crashed, script still running

**Fix:** Press **Stop** on teach pendant, then reconnect. Use wired connection if possible.

---

## Performance Issues

### High CPU usage when robot is idle

**Expected:** ~0-2% CPU when stationary.

**If higher:**
1. Check for custom panels with `while` loops or `time.sleep()` polling
2. Check for leaked `StateChannel.subscribe()` calls without matching `unsubscribe()`
3. Use `state_channel.get_subscriber_count()` to detect leaks

### Laggy 3D view during motion

**Expected:** Smooth 60Hz rendering.

**If choppy:**
1. Check mesh complexity — very large meshes (>100k vertices) slow VTK
2. Disable anti-aliasing: `RenderConfig(anti_aliasing=False)`
3. Check if multiple displays are registered — each adds render overhead

---

## Mode Switching Issues

### Jump when switching REAL → SIMULATE

**Symptoms:** Virtual robot jumps to a different position.

**Cause:** Virtual robot state was not synced to real robot before switching.

**Fix:** This should happen automatically in `CommandHandler._on_mode_switch_request()`. If it doesn't, check that `real_robot.get_state()` returns valid joint positions before switching.

### Cannot switch to REAL mode

**Symptoms:** "Cannot switch to REAL mode: Robot not connected".

**Fix:** Connect first (Robots → Connect), then switch mode.

---

## Getting Help

If your issue isn't here:
1. Check the console output — Hatch logs extensively
2. Search [GitHub Issues](https://github.com/victorwu-robotics/hatch/issues)
3. Open a new issue with:
   - Hatch version
   - Python version
   - OS
   - Steps to reproduce
   - Console output
