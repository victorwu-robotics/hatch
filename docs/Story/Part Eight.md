## Part Eight: The Visualizer and the Render Loop

The visualizer is the window into the robot's mind. It does not control anything. It does not send commands. It observes. Whatever the kinematic model says is true, the visualizer shows. If the model is wrong, the visualizer shows the wrong thing — and the user knows the model needs fixing.

This is Principle #3: **Visualizer as Mind-Prying Tool.** The 3D view is not a separate simulation. It is the robot's internal state made visible.

### The Actors and the Scene

The visualizer is built on VTK, a 3D rendering library. Every visible object in the scene is an **actor** — a thing with a shape (a mesh) and a transform (a position and orientation in world space). The robot's links are actors. The sensors mounted to its wrist are actors. The grid on the floor is an actor. The coordinate axes are actors.

Each actor's transform is a 4×4 matrix — three columns for the X, Y, Z axes of the link's local frame, and a fourth column for its position in world coordinates. When the robot moves, these transforms change. The visualizer must update every affected actor.

### The KinematicDisplay

`KinematicDisplay` bridges the kinematic model and VTK. When a robot is loaded, it creates one actor per link. It reads the link's mesh file (STL or DAE) through the `MeshLoader` service, creates a VTK actor with that geometry, and positions it at the link's world transform from the kinematic model.

It then subscribes to `TransformRegistry` callbacks. Whenever a frame's transform changes — because a joint moved — the registry notifies `KinematicDisplay`. It reads the new world transform, updates the corresponding actor, and sets a flag: `_needs_render = True`.

This flag is the key to efficient rendering. The visualizer does not re-render on every transform change. Dozens of transforms might change in a single state update — every link from the moving joint outward gets a new world position. Setting `_needs_render = True` on each one is cheap. The actual render happens once, after all transforms are updated.

### The Render Loop

The `VisualizerEngine` runs a `QTimer` at 60Hz — roughly every 16 milliseconds. On each tick, it checks `_needs_render` on every registered display. If any display is dirty, it calls `Render()` on the VTK window. If no display is dirty, it does nothing. The CPU sleeps between ticks.

This is not polling. Polling would recompute transforms or re-render on every tick regardless of need. The render loop checks a single boolean per display — a memory read, not a computation. The flag is set only by `TransformRegistry` callbacks, which fire only when transforms actually change. When the robot is stationary, no callbacks fire, no flags are set, the render loop does nothing, and the CPU enters low-power states.

This is the hybrid at the heart of Principle #2: the render timer is a single centralized check, and every component that feeds it is purely event-driven. The timer does not poll for changes. It polls a flag that was set by an event.

### The Grid and the Camera

The visualizer provides spatial context. A ground grid at Z=0 shows the floor plane. Adjustable grid size and color help with different scales — millimeter precision for welding, meter precision for mobile robots. Preset camera views (Top, Front, Side, Isometric) let the user quickly orient themselves. The camera preserves its zoom distance when switching views, so the user doesn't lose their place.

The coordinate axes widget in the bottom-left corner shows the world frame orientation: red for X, green for Y, blue for Z. This is especially important when the user rotates the view and loses track of which way is up.

### The Joint Frame Display

An optional overlay shows the coordinate frame at every joint and at the TCP. Each frame is drawn as small RGB axes — red for the link's X-axis, green for Y, blue for Z (the joint rotation axis). The TCP frame is drawn in magenta to distinguish it.

These frames are hidden by default. The user enables them from a panel on the left side of the window. Each joint can be toggled individually. The panel also shows the numerical pose of each visible frame — position in meters, orientation as a rotation vector and as RPY Euler angles. This transforms the visualizer from a pretty picture into a measurement tool.

The frames update in real time as the robot moves. They reveal the kinematic structure that the URDF describes: which joints are coincident, which axes are orthogonal, where the wrist center truly sits. They are the bridge between the abstract URDF and the physical robot.

### The Event Flow to Render

Here is the complete chain from a joint command to a rendered frame:

1. User moves slider → `JOINT_COMMAND` published
2. `CommandHandler` routes to robot → robot moves
3. Robot publishes `ROBOT_STATE`
4. `StateHandler` updates kinematic model → all link transforms recomputed
5. `StateHandler` updates `TransformRegistry` → cache invalidated, callbacks fire
6. `KinematicDisplay._on_transform_updated` → updates VTK actor for each changed link
7. `KinematicDisplay` sets `_needs_render = True`
8. `VisualizerEngine` 60Hz timer fires → checks `_needs_render` → sees it's True → calls `Render()`
9. VTK draws the new frame
10. `_needs_render` cleared

Ten steps. No direct calls between UI and model. No polling. No wasted rendering. Every step happens because the previous step demanded it. This is the derived architecture in motion.

