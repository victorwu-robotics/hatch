## Part Two: Describing the Robot

A robot is a chain of rigid bodies connected by joints. Every industrial robot — from a small UR5 to a massive KUKA — follows this pattern. A base. A shoulder. An upper arm. A forearm. A wrist. At the end, a tool.

To put a robot on screen, I need to tell the computer exactly what those parts look like and how they connect. The robot's manufacturer already did this work. They created a file — a URDF — that lists every link, every joint, and every dimension. URDF stands for Unified Robot Description Format. It is the industry standard. Every major robot brand provides one.

### The URDF File

The first time I opened a URDF file, I saw things I understood and things I didn't. The links and joints made immediate sense. A `<link>` is a rigid piece of the robot — the base, the upper arm, the wrist. A `<joint>` connects two links and tells how they move relative to each other. Revolute joints rotate. Fixed joints don't move. Prismatic joints slide. That much was comfortable.

But there were strange things too. `<xacro>` blocks that looked like code but weren't quite code. `<collision>` tags that seemed to duplicate what `<visual>` already said. File references ending in `.stl` and `.dae` that I had never seen before. And `<parent>` and `<child>` elements that defined a family tree I didn't yet understand.

Each of these strange things exists for a reason. They are not accidents of the format. They solve real problems.

### The Parent-Child Chain

A joint connects a parent link to a child link. The parent is the link closer to the base. The child is the link further out. This creates a chain:

```
world → base_link → shoulder_link → upper_arm_link → forearm_link → wrist_1_link → wrist_2_link → wrist_3_link
```

When the shoulder joint rotates, the upper arm, forearm, and everything beyond it rotates too. The parent-child relationship captures this: a child moves with its parent, plus whatever movement its own joint adds. This simple rule — repeated for every joint — is all the kinematic model needs to compute where every link is at any moment.

### What is Xacro?

Xacro is a preprocessor. It is not part of the URDF standard — it is a convenience layer on top. A robot with many sensors and tools can have a URDF file that is thousands of lines long, with repeated blocks for similar components. Xacro lets you write that structure once and reuse it.

Hatch includes its own xacro preprocessor, just enough to handle the features that matter: including other files, defining variables, and instantiating macros. If you already have a URDF that works in ROS, it will work in Hatch. If you are building one from scratch, you can use plain URDF without xacro at all. The preprocessor is invisible — you load a `.xacro` file, and Hatch produces a clean URDF before parsing it.

### Meshes: STL and DAE

A link is a rigid body, but the URDF does not describe its shape in numbers. It points to a mesh file — a 3D model of the link. Two formats dominate: STL and DAE (Collada).

STL is the simplest. It lists triangles — three points, then another three, then another three — until the entire surface of the part is described. It contains no color, no material, no scene structure. Just triangles. STL files are small, fast to load, and universally supported. Every CAD program can export them.

DAE (Collada) is richer. It can contain color, texture, and multiple objects arranged in a scene. A single DAE file can hold an entire assembly of parts with their relative positions baked into a scene graph. This richness comes with complexity. A DAE file must be interpreted correctly — its scene transforms must be applied, or the mesh appears in the wrong place at the wrong size. This exact problem would later teach us something important about mesh loading that we will revisit in the appendix.

### Visual and Collision

Every link can have two representations: a visual mesh (what you see on screen) and a collision mesh (what the robot checks for impacts). They are often the same file, but they serve different purposes. The visual mesh can be detailed and beautiful. The collision mesh should be simple — a box or a cylinder — because collision checking is computationally expensive. Hatch uses the visual mesh for display. Collision detection is a future feature, but the URDF structure is ready for it.

