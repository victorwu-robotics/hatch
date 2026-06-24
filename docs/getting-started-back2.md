# Getting Started with Hatch

**Time:** 30 minutes  
**Goal:** Load a robot, move it, and understand what you see.

---

## Before You Start

### What You Need

- A computer running Linux (Ubuntu 20.04+ recommended), Windows, or macOS
- Python 3.10 or higher
- A URDF or XACRO file for your robot (or use the example in `assets/robots/`)

### What You Will Learn

- How to start Hatch
- How to load a robot from its URDF
- How to move the robot in joint space and Cartesian space
- How to read the 3D view
- How to switch between simulation and real hardware

---

## Step 1: Install Hatch (5 minutes)

```bash
# Clone the repository
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

---

## Step 2: Start Hatch (2 minutes)

```bash
python -m ui.main_window
```

You will see:

![Startup Screen](images/startup_screen.png)

- A **3D viewport** with a grid floor (empty, no robot yet)
- A **View Controls** panel on the left (camera presets)
- A **menu bar** at the top (File, View, Robot, Camera, Help)

> **What you see:** The grid is your workspace. The axes in the corner show orientation (red = X, green = Y, blue = Z). The viewport is ready for your robot.

---

## Step 3: Load Your Robot (5 minutes)

**File → Load URDF**

Select your robot's URDF or XACRO file. For example:
- `assets/robots/ur_description/urdf/ur10_nominal.urdf`
- `assets/robots/ur10/ur10.xacro` (Hatch expands this automatically)

After loading:

![Robot Loaded](images/robot_loaded.png)

**What appears:**
- The robot model in the 3D viewport
- The **Motion Control** panel on the right
- The **Joint Control** tab active by default

> **Note:** If the robot appears too large or only partially visible (as in the screenshot above), the **Viewing Camera** is simply zoomed in too close. The robot is fully loaded.

## Adjust the Viewing Camera

| Button | Action |
|--------|--------|
| Zoom to Fit (left panel, Camera Controls) | Frames all objects in the view |
| Reset (toolbar) | Returns camera to default position |
| Scroll wheel | Zoom in/out |
| Left-click drag | Rotate camera around the scene |
| Middle-click drag | Pan the view |

**Try this:** Click **Zoom to Fit** to see the entire robot.

## What You See After Adjusting
- The **full robot arm** from base to tool
- The **grid floor** for spatial reference
- The **world axes** in the corner (red = X, green = Y, blue = Z)

> **What you learn:** The 3D view is a camera looking at the scene. You control the **Viewing Camera**, not the robot. The robot's state is shown; the camera's position is yours to adjust.


---

## Step 4: Check the Mode (1 minute)

Before moving anything, check the **Mode** dropdown in the Motion Control panel.

| Mode | What It Does | When to Use |
|------|-------------|-------------|
| **Simulate** | Local IK solver, no hardware connection | Learning, testing, debugging |
| **Real** | Connects to physical robot via RTDE | Production, actual movement |

**For this tutorial:** Select **Simulate**.

> **Why this matters:** In Simulate mode, you are safe. The robot exists only in software. In Real mode, your commands move physical hardware.

---

## Step 5: Move the Robot in Joint Space (5 minutes)

Click the **Joint Control** tab.

| Control | What It Does |
|---------|-------------|
| **Shoulder Pan Joint** | Rotates the base of the arm left/right |
| **Shoulder Lift Joint** | Lifts the upper arm up/down |
| **Elbow Joint** | Bends the elbow |
| **Wrist 1, 2, 3** | Orient the tool |

**Try this:**
1. Drag the **Shoulder Lift Joint** slider to -1.0
2. Watch the robot arm lift in the 3D view
3. Drag the **Elbow Joint** slider to -1.5
4. Watch the elbow bend

> **What you learn:** Each slider is your **intent** — a value you want the joint to reach. The 3D view shows the **result** — the robot's actual pose after applying your intent. The slider and the robot state are connected by the forward kinematics engine, but they are not the same thing. The slider is input; the 3D view is output.

**Key buttons:**
- **Home Position** — Returns all joints to neutral
- **Zero All** — Sets all joints to zero

---

## Step 6: Move the Robot in Cartesian Space (5 minutes)

Click the **Cartesian Control** tab.

Each axis has a **slider** (coarse) and **[-] [+]** buttons (fine). Both control the same target value.

| Control | What It Does |
|---------|-------------|
| **X, Y, Z sliders** | Drag or scroll to move the tool tip in 3D space (meters) |
| **RX, RY, RZ sliders** | Drag or scroll to rotate the tool (rotation vectors, radians) |
| **[-] [+] buttons** | Click to move one exact step in that axis |
| **Step size** | Choose precision: 0.0001, 0.001, 0.01, or 0.1 (meters or degrees) |
| **Reset to Current** | Snap target back to the robot's actual pose |

**Step size reference:**

| Step | Linear | Angular |
|------|--------|---------|
| **0.0001** | 0.1 mm | 0.001° |
| **0.001** | 1 mm | 0.01° |
| **0.01** | 1 cm | 0.1° |
| **0.1** | 10 cm | 1° |

**Try this:**
1. Ensure **Mode** is still **Simulate**
2. Select step size **0.01** (1 cm / 0.1°)
3. Drag the **X slider** right — robot reaches forward
4. Now click **[+]** next to X twice — robot moves exactly 2 cm more

> **What you learn:** The slider gets you close quickly. The buttons count exact steps. One click = one step size. No guessing. As you drag or click, Hatch continuously solves Inverse Kinematics to find joint angles that reach your target. The 3D view updates in real time. The **Auto-move enabled** label confirms the solver is running.

**What you do NOT see:**
- Joint angles. The Cartesian tab shows target pose, not joint values. The Joint Control and Cartesian Control tabs are mutually exclusive — you cannot see both at once.
- The 3D view and **Current TCP** display are your feedback.

**If the robot stops moving:**
- You may have dragged into an unreachable pose (IK failure)
- Click **Reset to Current** to recover

---

## Step 7: Inspect the Robot (5 minutes)

**Left panel: Joint Frames**

Check **Show All** to see coordinate frames on every link.

| Frame | Color | Meaning |
|-------|-------|---------|
| **Red axis** | X | Forward/back in that link's frame |
| **Green axis** | Y | Left/right in that link's frame |
| **Blue axis** | Z | Up/down in that link's frame |

**Hover over a frame** to see its real-time position and rotation.

> **What you learn:** Every frame comes from the URDF. What you see is exactly what the robot knows. There is no hidden state.

---

## Step 8: Switch to Real Hardware (Optional, 5 minutes)

When you are ready to control a physical robot:

1. **Robot Connection** panel → Enter IP address (default: 192.168.1.10)
2. **Mode** dropdown → Select **"Real"**
3. Click **Connect to Robot**
4. Move a slider — the physical robot moves

> **Warning:** Ensure the robot workspace is clear. Hatch does not have collision detection. You are responsible for safety.

---

## What You Have Learned

| Concept | Where You Saw It |
|---------|---------------|
| **Joint space** | Joint Control sliders |
| **Cartesian space** | Cartesian Control tab |
| **Inverse Kinematics** | Cartesian control — solver runs in real time |
| **Forward kinematics** | 3D view, Frame Panel |
| **Digital twin** | 3D view matching robot state |
| **URDF as scene** | Robot loaded from URDF, frames from URDF |
| **Event-driven** | Slider moves → robot updates (no polling) |
| **UI separate from services** | Panels publish events; they do not command |

---

## Next Steps

| I want to... | Go to... |
|-------------|----------|
| **Understand why Hatch exists** | [Philosophy](philosophy.md) |
| **Understand the full architecture** | [Architecture](architecture.md) |
| **Connect cameras or sensors** | [Integrating Hardware](integrating_hardware.md) |
| **Write code to control Hatch** | [API Reference](api-reference.md) |
| **See how IK works** | [Inverse Kinematics](inverse_kinematics.md) |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot does not appear | Check URDF path. Ensure meshes are in the same directory or use `package://` paths correctly. |
| Sliders do not move robot | Check that **Simulate** mode is selected. In **Real** mode, ensure robot is connected. |
| 3D view is blank | Try **View → Reset** or **View → Zoom to Fit**. |
| Chinese characters garbled | Install CJK fonts: `sudo apt install fonts-noto-cjk` |

---

*Hatch is pre-flight. The flight is yours.*
