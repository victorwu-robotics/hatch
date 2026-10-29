# Inverse Kinematics in Hatch (孵)

## A Unified Derivation for Offset and Spherical Wrists

---

# Part 1 — The URDF: What It Is, and How to Read It

## 1.1 Why we start here

Everything in this document — every axis, every offset, every angle — is extracted from a URDF file.

The URDF is the single source of truth for the scene (Principle #4). The robot's geometry, its joints, its meshes, its limits: all of it lives in one XML file. If we do not understand what that file contains, we cannot understand what we extract from it, and we cannot check whether the extraction is correct.

So before we solve anything, we learn to read the file. Not to *write* it — writing is the robot manufacturer's job. Our job is to *read* it completely, and to know exactly which numbers in it become the geometry of the solver.

This part is a reading exercise. By the end, you will be able to open any industrial arm's URDF and point to the line that gives each joint its axis, each link its mesh, and each joint its limits.

## 1.2 What XML is

A URDF is written in **XML**. XML is a way of writing structured data as text. It has only a few rules, and they are worth knowing precisely, because a URDF is nothing but XML.

**An element** is a pair of tags with content between them:

```xml
<link name="base_link">
  ...
</link>
```

**An attribute** is a name-value pair inside a tag:

```xml
<joint name="shoulder_pan_joint" type="revolute">
```

**Nesting** is elements inside elements. **A self-closing element** is an element with no content:

```xml
<link name="base_link"/>
```

**A comment** begins with `<!--` and ends with `-->`.

**A processing instruction** begins with `<?` and ends with `?>`. Every XML file begins with one:

```xml
<?xml version="1.0"?>
```

It declares the file as XML version 1.0 and carries no robot information.

**Namespaces** — the `xacro:` prefix, the `xmlns` attributes — are a more advanced XML feature, used when a file is composed from parts. They matter for the `URDFPreprocessor`, which expands xacro templates into plain URDF. They do **not** matter here: by the time the file reaches the solver, it is plain URDF with no namespaces.

That is the whole of XML that a URDF reader needs. Five constructs: elements, attributes, nesting, self-closing elements, comments.

## 1.3 Links: the rigid bodies

A **link** is a rigid body. It is a physical object that does not deform — a piece of metal, a bracket, a mounting plate, a sensor housing.

```xml
<link name="base_link">
  <visual>   <geometry> <mesh filename="package://ur10/meshes/base_link.stl"/> </geometry> </visual>
  <collision> <geometry> <mesh filename="package://ur10/meshes/base_link.stl"/> </geometry> </collision>
  <inertial> <mass value="4.0"/> <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/> </inertial>
</link>
```

A link has three optional subsections:

- **`<visual>`** — how the link looks. Its geometry, and optionally its material.
- **`<collision>`** — the shape used for collision checking.
- **`<inertial>`** — the mass and inertia tensor, used for dynamics.

For the kinematic solver, only the link's **name** matters. The kinematics is entirely about *how links are connected*, not about what they look like or how heavy they are. Meshes and inertials are for visualization and simulation — they are not part of the solver's input.

## 1.4 Joints: the connections

A **joint** connects two links. It says: link A (the **parent**) and link B (the **child**) are attached, in this way, at this position, with this kind of motion allowed between them.

```xml
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="base_link_inertia"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.1273" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-6.283" upper="6.283" effort="150" velocity="3.15"/>
</joint>
```

**`type`** — what kind of motion the joint allows:

| Type | Motion | Used for |
|---|---|---|
| `fixed` | None. Rigid attachment. | Mounting plates, sensor brackets, the tool mount. |
| `revolute` | Rotation about an axis, with limits. | Most robot joints. |
| `continuous` | Rotation about an axis, no limits. | Wheels, some base joints. |
| `prismatic` | Translation along an axis, with limits. | Linear actuators, some gantries. |

The solver operates on **revolute and continuous** joints — the six that rotate the arm. Fixed joints define the boundary (the true root, the flange).

**`<parent>` and `<child>`** — the two links the joint connects.

**`<origin>`** — the position and orientation of the joint's frame, relative to the parent link's frame:

```xml
<origin xyz="0 0 0.1273" rpy="0 0 0"/>
```

`xyz` is a translation in meters. `rpy` is a rotation as roll-pitch-yaw angles in radians. Together they give a full 6-DOF transformation from the parent link to the joint's frame.

**This is the most important element in the URDF for kinematics.** The joint origin tells us *where the joint sits* relative to its parent.

**`<axis>`** — the direction of the joint's rotation, expressed in the joint's own frame:

```xml
<axis xyz="0 0 1"/>
```

The axis is a *direction*, not a position. The position of the axis is given by the joint origin.

**`<limit>`** — the joint's physical limits:

```xml
<limit lower="-6.283" upper="6.283" effort="150" velocity="3.15"/>
```

The solver uses `lower` and `upper` to filter out joint angles outside the joint's range.

### The Z-axis rule

**All revolute joints rotate about the Z axis of their own frame, according to the right-hand rule.**

This is the URDF convention, and it is the foundation of the entire kinematic description:

- The `<axis>` element specifies the direction of rotation *in the joint's frame*. For a revolute joint that rotates about its own Z, the axis is `0 0 1`.
- The joint angle θ is the angle of rotation about that Z, with the sign given by the right-hand rule (thumb along the axis, fingers curl in the positive direction).
- When a later part says "J2's axis is normal to the arm plane," it means J2's frame Z, expressed in the true-root frame, is the arm plane's normal. When it says "the twist between J2 and J3 is 0," it means their Z axes point in the same direction.

**The Z axis is the joint's axis of rotation, always.**

## 1.5 The tree structure

A URDF is a **tree**. There is exactly one root link — the link with no parent joint. Every other link has exactly one parent joint, and therefore exactly one parent link. No link is the child of two joints. No cycles exist.

This is not a convention. It is a consequence of physics. **A rigid body cannot be in two places at once.** The tree structure is what makes a URDF consistent: every link's pose is determined uniquely by the path from the root.

For a 6R arm, the tree contains **more than six joints in total** — but exactly six of them are movable. The six movable joints are the revolute joints of the arm chain. The rest are **fixed joints**: at the base (mounting the arm to the world) and at the flange (mounting the tool). There may also be fixed joints between consecutive revolute joints — for example, a `base_inertia` link between `base_link` and the first revolute joint, as in Universal Robots' URDFs.

This is why the **true kinematic root** is not always the intuitive `base_link`. The solver detects it: the true root is the **parent link of the first movable revolute joint**. For UR10, that is `base_inertia`, not `base_link`. Using the wrong root produces wrong kinematics.

## 1.6 Meshes: the visual geometry

A **mesh** is a piece of 3D geometry — a collection of triangles that, together, form the surface of a link.

```xml
<geometry>
  <mesh filename="package://ur10/meshes/base_link.stl"/>
</geometry>
```

The mesh is referenced by **filename**, not embedded in the URDF. A robot's meshes are large; embedding them would make the file megabytes long. Referencing them keeps the URDF small — a few hundred lines of XML a human can read and a version control system can diff.

The reference is a **URI**, most commonly `package://<package_name>/<relative_path>`. Hatch resolves it through the `PackageResolver` (Constitution Part 4): `~/hatch/assets/` first, then every directory in `HATCH_PACKAGE_PATH`.

### The four mesh formats

| Format | Stores | Encoding | Typical use |
|---|---|---|---|
| **STL** | Triangles only. No color. | ASCII or binary | The most common format for robot links. |
| **OBJ** | Triangles, optionally normals and texture coordinates. | ASCII | Less common; used when texture coordinates are needed. |
| **PLY** | Triangles, optionally per-vertex color and normals. | ASCII or binary | Used when per-vertex color is needed. |
| **DAE** (COLLADA) | A full scene graph: triangles, materials, colors, textures. | XML | Used by some manufacturers (notably Universal Robots) because it carries color without an extra file. |

For the kinematic solver, the mesh format does not matter. **The solver never reads a mesh.** Meshes are for visualization — the `MeshLoader` reads them, the `KinematicDisplay` renders them. The solver's input is the *joint origins and axes*.

## 1.7 A minimal 6R arm

Here is the **minimal** URDF of a 6R arm — the smallest file that still has the full structure. It has no meshes, and its links are short, but it is not a simplification: it is the **bone** of a real industrial arm. Six revolute joints, fixed joints at the base and the flange, and a true root that is not `base_link`. The structure is exactly the one the solver will consume; only the flesh — the mesh geometry and the long link lengths — has been removed so the structure is visible at a glance.

```xml
<?xml version="1.0"?>
<!-- A minimal 6R arm. Six revolute joints, with fixed joints at the base
     and the flange, mirroring the structure of a real industrial arm. -->
<robot name="minimal_6r_arm">

  <!-- The root link: the ground. No parent joint. -->
  <link name="world"/>

  <!-- Fixed joint: mounts the arm to the world. Not part of the 6R chain. -->
  <joint name="mount_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- base_link: the arm's mounting plate. -->
  <link name="base_link"/>
  <!-- A <visual> with a <mesh> would go here. Omitted for brevity. -->

  <!-- Fixed joint: the base_inertia link, with a 180° rotation about Z.
       This mirrors Universal Robots' URDFs, where the true kinematic
       root is base_inertia, not base_link. -->
  <joint name="base_inertia_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_inertia"/>
    <origin xyz="0 0 0" rpy="0 0 3.14159"/>
  </joint>

  <!-- base_inertia: the true kinematic root — the parent link of the
       first movable revolute joint. -->
  <link name="base_inertia"/>

  <!-- Joint 1: the first movable joint. Rotates about the vertical axis. -->
  <joint name="joint_1" type="revolute">
    <parent link="base_inertia"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="150" velocity="3.15"/>
  </joint>

  <link name="link_1"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Joint 2: shoulder. Rotates about the horizontal Y axis. -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="150" velocity="3.15"/>
  </joint>

  <link name="link_2"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Joint 3: elbow. Parallel to joint 2 — the arm plane condition. -->
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="3.15"/>
  </joint>

  <link name="link_3"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Joint 4: wrist 1. Perpendicular to joint 3. -->
  <joint name="joint_4" type="revolute">
    <parent link="link_3"/>
    <child link="link_4"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>

  <link name="link_4"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Joint 5: wrist 2. Perpendicular to joint 4. -->
  <joint name="joint_5" type="revolute">
    <parent link="link_4"/>
    <child link="link_5"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>

  <link name="link_5"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Joint 6: wrist 3. Parallel to joint 4 — the tool roll. -->
  <joint name="joint_6" type="revolute">
    <parent link="link_5"/>
    <child link="link_6"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>

  <link name="link_6"/>
  <!-- A <visual> with a <mesh> would go here. -->

  <!-- Fixed joint: the flange. The end of the 6R chain. -->
  <joint name="flange_joint" type="fixed">
    <parent link="link_6"/>
    <child link="flange"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- flange: the tool mount link. Everything past this point is the tool,
       which the solver never sees. -->
  <link name="flange"/>

</robot>
```

Read it against the concepts:

- **The tree.** `world` is the root; its single child is `base_link` through the fixed `mount_joint`. `base_link` has one child, `base_inertia`, through the fixed `base_inertia_joint`. `base_inertia` is the **true kinematic root** — the parent of the first movable joint. From there the chain is `link_1` through `link_6`, connected by six revolute joints, then a fixed `flange_joint` to the `flange` link. Every link has exactly one parent. No cycles.
- **The fixed joints.** There are three: `mount_joint`, `base_inertia_joint` (with a 180° rotation about Z), and `flange_joint`. None of them move. They define the boundary — the world below, the tool above.
- **The true root.** `base_inertia` is not the first link in the file, and it is not the intuitive `base_link`. It is the parent of the first *movable* joint. The 180° rotation in `base_inertia_joint` means the entire arm chain is flipped relative to `base_link`. Using `base_link` as the kinematic root would produce wrong kinematics.
- **The six movable joints.** `joint_1` through `joint_6` are `type="revolute"`. Each has an origin, an axis, and limits. The axes are arranged so that `joint_1` rotates about the vertical Z axis; `joint_2` and `joint_3` about the horizontal Y axis and are **parallel** — the arm-plane condition; `joint_4` about X, perpendicular to J3; `joint_5` about Y, perpendicular to J4; `joint_6` about X, parallel to J4 — the tool roll.
- **The origins.** Each joint origin places the joint relative to its parent. These translations are the link geometry: the shoulder offset, the upper arm length (0.5 m, in `joint_3`'s origin), the forearm length (0.4 m, in `joint_4`'s origin), and the wrist offsets (0.1 m each, in `joint_5`'s and `joint_6`'s origins).
- **The flange.** `flange_joint` is fixed, with origin `xyz="0.05 0 0"` — the flange sits 0.05 m past `link_6`. The solver's output is the pose of this frame; everything past it is the tool.
- **The meshes.** Each link in a real file would carry a `<visual>` with a `<mesh>` reference. They are omitted here to keep the example small.

A reader who can read this file can read any industrial arm's URDF. The real UR10 file has more links and more detail, but the structure is the same: a tree of links, connected by joints, with fixed joints at the base and the flange, exactly six movable revolute joints in between, and the true root one or two fixed joints below the first movable one.

## 1.8 The limitation of this document

This document is about **6R arms** — six-joint serial robots with revolute joints, of the class described in Part 9. Not every robot is a 6R arm:

- **SCARA arms** have two parallel revolute joints and a prismatic joint. Different structure; different solver.
- **Delta robots** have three parallel kinematic chains meeting at a common platform. Not a serial chain.
- **Parallel robots** (Stewart platforms, etc.) have closed kinematic loops, not a tree.
- **7-DOF redundant arms** have more than six joints; the solver's counting argument does not apply.
- **Mobile robots** have a kinematic structure that is not a fixed-base arm.

Hatch's analytical solver does not support these. The refusal is deliberate (Part 9, §9.1 Step 0). The URDF is general — it can describe any of these — but *our use of it* is specific to 6R arms.

## 1.9 What we have, and what comes next

We now know what a URDF contains: a tree of links, joints with origins and axes, meshes referenced by URI, a tree structure rooted at a single link.

And we know what the URDF does *not* give us directly: the **geometric quantities** the solver needs. Those are *derived* from the joint origins and axes. They are not named in the URDF, and they are not the DH parameters.

In the next part, we derive them. The direction of derivation is:

> **URDF → geometric quantities → solver**

not:

> **URDF → DH table → solver**

Part 2 begins the derivation.

---

# Part 2 — Seeing the Robot

## 2.1 What a robot arm is made of

A 6-DOF industrial arm is a chain of rigid bodies connected by joints. Each joint allows exactly one motion: rotation about a line. That line is the **joint axis**.

The robot's geometry is the collection of these axes and the fixed relationships between them: where each axis sits in space relative to the previous axis, how it is oriented, how far along the axis the next joint attaches.

Everything else — the meshes, the colors, the physical shape of the links — is decoration. **The kinematics is entirely captured by the axes.**

## 2.2 What the URDF gives us

The URDF describes each joint with three pieces of information:

- **The joint origin** (`xyz`, `rpy`): the position and orientation of the joint's frame relative to its parent.
- **The joint axis** (`xyz`): the direction the joint rotates about, in the joint's own frame.
- **The joint type** (`revolute`, `continuous`, `prismatic`, `fixed`).

For a revolute joint, this is complete. The origin tells us where the axis sits and how it is oriented; the axis tells us the direction of rotation; the type tells us the joint moves.

**This is all the kinematics we need.** The URDF is not a description of a robot that *happens to include* kinematics — it *is* the kinematics, plus visual geometry.

## 2.3 The wrist

The last three joints — J4, J5, J6 — form the wrist. Their geometry has a property that the rest of the arm does not: **J5 and J6 always intersect** (for the class of arms this document covers). Their intersection is a point — the **wrist center**, P5.

This is what makes the wrist separable from the arm:

- **P5 is fixed by the flange pose.** Given the target flange pose, P5 is determined: it is the flange position offset by the flange's own length along the flange's Z axis. (Part 5, Step 1.)
- **J4 may or may not pass through P5.** If it does, the wrist is *spherical*. If it does not, the wrist is *offset*.
- **The offset is the quantity that varies.** In the spherical wrist, J4, J5, J6 all intersect at P5. In the offset wrist, J4's axis passes at a perpendicular distance from P5. That distance is the wrist's defining quantity.

**The document derives the general case — the offset wrist — and lets the spherical wrist fall out at offset zero.** The conventional derivation does the reverse (spherical first, offset as a complication); this document reverses it (§9.5).

## 2.4 The three points

The wrist geometry is captured by three points:

- **A** — the **wrist root**. The point where J4's axis meets the **arm plane** (the plane the upper arm and forearm sweep through). A is the end of the forearm; it is the point the arm chain must reach.
- **B** — the **wrist's first bend**. The point where J4's axis meets J5's axis. B is on J4's axis, at the perpendicular distance from A that the wrist geometry requires.
- **C = P5** — the **wrist center**. The point where J5's axis meets J6's axis. C is on J5's axis.

These three points form a **right triangle at B**:

- **AB** is along J4's axis — **perpendicular** to the arm plane.
- **BC** is along J5's axis — **parallel** to the arm plane (since J5 ⊥ J4, and J4 is normal to the plane).
- **AC** is the hypotenuse — from the wrist root to the wrist center.

The three sides are the three wrist quantities (Part 3). The right angle at B is what makes them a triangle.

**When the wrist is spherical, A, B, and C coincide.** The triangle collapses to a point. But the triangle's *structure* — the right angle, the three sides, the hypotenuse — is still the structure of the geometry; it is just the structure at zero. **The spherical wrist is not a different geometry; it is the geometry at a limit.**

## 2.5 Why we do not need DH

The Denavit–Hartenberg convention compresses the relationship between two consecutive joint axes into four scalars, by assuming the frames are placed in a canonical way. The URDF gives us **more information than DH does**: each joint origin is a full 6-DOF transform. The compression is lossy, works only for the class DH was designed for, and is ambiguous when two consecutive axes are parallel.

Hatch does not build a DH table. It reads the joint origins and axes directly from the URDF and works with them. The geometric quantities that DH would encode are still useful — we will name them and use them — but they are *derived from* the URDF, not *substituted for* it.

The direction is:

> **URDF → geometric quantities → solver**

not:

> **URDF → DH table → solver**

The first direction works for every robot the URDF can describe. The second works only for robots whose geometry fits the DH assumptions, and it fails silently when it does not.

---

# Part 3 — The Five Geometric Quantities

## 3.1 The question we are answering

Every joint in a serial arm rotates about a line — its **joint axis**. To describe the robot, we need to describe how each joint axis relates to the next.

Two lines in space are related by four numbers: how far apart are they? how are they rotated relative to each other? where along the first line does the second one attach? how much has the joint rotated?

These are the four quantities. The wrist adds a fifth, because its axes do not all lie in a common plane the way the arm's do.

## 3.2 The four axis quantities

For each consecutive pair of joint axes (i, i+1), for i = 1 to 5:

**`axis_gap(i)`** — **how far apart are two consecutive joint axes?** The distance between two lines in space is measured along their **common perpendicular** — the shortest line segment connecting them. That segment's length is `axis_gap(i)`.

For UR10: `axis_gap(2)` (J2 to J3) is 0.612 m, the upper arm length. `axis_gap(3)` (J3 to J4) is 0.5723 m.

**`axis_twist(i)`** — **how are two consecutive joint axes rotated relative to each other?** Each joint axis is a line with a direction. The angle between the two directions is `axis_twist(i)`. It is signed, and measured about the common perpendicular.

For UR10: `axis_twist(1)` (J1 to J2) is π/2 — the base axis is vertical, the shoulder axis horizontal. `axis_twist(2)` (J2 to J3) is 0 — the shoulder and elbow axes are parallel. `axis_twist(3)` (J3 to J4) is π/2.

**`axis_shift(i)`** — **where along the first joint axis does the common perpendicular meet it?** The common perpendicular hits the first joint's axis at a specific point. The signed distance along that axis between the feet of the common perpendiculars with the previous and next axes is `axis_shift(i)`.

`axis_shift` is **origin-dependent**: it is measured from the joint's origin. For UR10, `axis_shift(3)` is 0.0, while `wrist_normal_offset` is 0.163941 — they are *not* the same quantity, and the solver uses `wrist_normal_offset` (see §3.4 and Appendix A).

**`joint_angle(i)`** — **how much has the joint rotated?** The rotation of the joint about its own axis. This is the variable we solve for.

**Indexing.** The quantities are indexed **1-based**, matching the joint numbers: `axis_gap(2)` is the gap between J2 and J3. (The extraction code uses 0-based indexing; this is a bug to fix — see Appendix C.)

## 3.3 Why these names

The classical names describe the **link** — the rigid body between the joints. But the quantities do not describe the link; they describe the relationship between two **joint axes**.

Consider the length of a link. A physical link is a solid object; if you measure it end to end, the joint axes are not at its very ends. The pin of each joint sits inside the link, with material past it. So the physical length of the link and the distance between its two joint axes are **different numbers**. Naming the distance between axes "link length" invites the reader to picture the wrong thing.

The same applies to the twist. The physical link does not twist. The two **axes** are rotated relative to each other, and the link is the material connecting them.

Naming the quantities after the axes makes the geometry visible: `axis_gap` is a distance between axes; `axis_twist` is an angle between axes.

## 3.4 The fifth quantity: `wrist_normal_offset`

The arm's four quantities describe how each axis relates to the next. The wrist needs a fifth, because J5's axis relates not to J4's axis alone, but to the **arm plane**.

**`wrist_normal_offset` = the perpendicular distance from the arm plane to J5's axis.**

This is the quantity the arm-chain derivation uses — for the θ₁ tangency (Part 5) and the θ₅ lateral offset (Part 6). It is computed from the axes as lines, not from the joint origins:

- **B** = J4's axis ∩ J5's axis.
- **`wrist_normal_offset` = |dot(B − J2's origin, n̂)|**, where `n̂` is J2's axis direction (normal to the arm plane).

**Why it is origin-independent.** B is an axis intersection — an invariant point. J2's origin is on the arm plane. The perpendicular distance from B to the plane is a geometric fact, independent of where any joint origin sits along its axis. This matters: UR and FR place J4's origin at opposite ends of the A–B segment, so any origin-derived computation would disagree between the two arms. The line-based computation does not.

## 3.5 The right triangle at B

The fifth quantity is one side of the **right triangle A–B–C**, right-angled at B. The three sides are the three wrist quantities:

| Symbol | Name | What it is | Role |
|---|---|---|---|
| `\|AB\|` | **`wrist_normal_offset`** | Perpendicular distance from the arm plane to J5's axis. AB is normal to the arm plane. | **θ₁ tangency, θ₅ lateral offset** |
| `\|BC\|` | **`wrist_radial_offset`** | Perpendicular distance from C to J4's axis. BC is parallel to the arm plane, in the plane through B at distance `\|AB\|` from the arm plane. | Wrist inversion geometry |
| `\|AC\|` | **`wrist_offset`** | The hypotenuse: `√(\|AB\|² + \|BC\|²)`. Distance from wrist root A to wrist center C. | Arm-chain target distance (`a₃` in conventional notation) |

**The solver's scalar inputs are `|AB|`, `flange_offset`, `axis_gap(2)`, and `elbow_to_wrist`. The other two sides of the triangle — `|BC|` and `|AC|` — are derived quantities.** They are stated here because they make the geometry visible (the triangle, the right angle at B, the spherical case as all three going to zero), and because the wrist inversion's origins encode the same information. But the solver does not take `|BC|` or `|AC|` as scalars.

**All three are zero for a spherical wrist.** They are distinct quantities, and the solver uses each for a different part of the derivation. The conventional term "wrist offset" is ambiguous — it is used sometimes for `|BC|` and sometimes for `|AC|`. This document names all three explicitly.

## 3.6 The derived lengths

Two lengths the arm chain uses are not among the five quantities, and are not `axis_gap` of anything. They are geometric distances involving a **point** (an axis intersection or an origin), not two axes:

**`elbow_to_wrist`** — the distance from J3's axis to A (the wrist root). This is the "forearm length" in the law of cosines (Part 7). It is **not** `axis_gap(3)` (the J3–J4 perpendicular distance), and it is **not** the physical forearm link length. It is the distance the arm chain must span with its forearm.

**`flange_offset`** — the distance from P5 to the flange along the flange's Z axis. This is `d₆` in the conventional notation. It is the offset the flange carries beyond the wrist center.

The law of cosines uses `axis_gap(2)` (the upper arm, J2 to J3) and `elbow_to_wrist` (the forearm, J3 to A). The θ₅ formula uses `flange_offset`.

## 3.7 What the five quantities describe — and what they do not

The five quantities describe the **class of arms this document covers**: arm-plane 6R arms with intersecting wrist axes. They plus the URDF's joint origin transforms are a complete kinematic description of that class.

They do **not** describe every 6R arm. For an arm outside the class — skew J2/J3, non-intersecting J5/J6 — the five quantities are not sufficient, and the solver refuses (Part 9, Step 0). The refusal is honest: the five quantities are not claimed to be universal.

**They are not a replacement for the URDF.** They are a *selection* from the URDF — the geometric quantities the derivation needs, named for what they mean. The solver still reads the URDF's origins for the wrist inversion (Part 8) and for the class check. Principle #4 is not violated; the five quantities are derived from the URDF, not substituted for it.

**DH is not needed and not used.** The five quantities are not DH parameters under different names — they are geometric quantities. `wrist_normal_offset` in particular is not a DH parameter at all; DH's `d₄` happens to equal it for these arms, but that is a frame-placement coincidence (Appendix A).

---

# Part 4 — The Wrist Root and the Arm Plane

## 4.1 The arm plane

The **arm plane** is the plane containing the origins of J2 and J3 — the plane the upper arm and forearm sweep through as J2 and J3 rotate. It is **normal** to J2's axis direction (and to J3's, and to J4's, when they are parallel to J2).

The plane also contains J1's axis (vertical), and rotates about it as θ₁ changes. **θ₁ determines the arm plane's azimuth.**

## 4.2 The wrist root A

**A is the point where J4's axis meets the arm plane.** This is the end of the forearm — the point the two-link arm chain must reach.

The unified computation of A, valid for both offset and spherical wrists:

- **B** = J4's axis ∩ J5's axis.
- **A = B − dot(B − J2_origin, n̂) · n̂**, where `n̂` is the arm plane's normal (J2's axis direction).

For the offset wrist, B is at perpendicular distance `wrist_normal_offset` from the arm plane, and A is the projection of B onto the plane. For the spherical wrist, B is on the plane, and **A = B = P5**.

**No `intersect_plane` call, no branch.** The projection of B onto the plane is well-defined for every arm in the class, whether J4's axis pierces the plane or lies in it.

## 4.3 `|AP5|` — the quantity that collapses

The distance from A to P5:

```
|AP5| = √(|AB|² + |BC|²) = √(wrist_normal_offset² + wrist_radial_offset²)
```

- For the offset wrist: A and P5 are distinct, and `|AP5|` is the length of the hypotenuse.
- For the spherical wrist: A = B = C, and `|AP5| = 0`.

`|AP5|` is **the quantity that collapses to zero for the spherical wrist**. The arm chain reaches A; the wrist begins at A; the distance from A to P5 is the wrist's offset from its root. At zero, the wrist root and the wrist center coincide.

## 4.4 The spherical case

When `wrist_normal_offset = 0` and `wrist_radial_offset = 0`:

- A = B = C = P5.
- `|AP5| = 0`.
- The wrist triangle collapses to a point.

**The formulas of Parts 5–8 remain valid.** They degenerate to the spherical case as the geometry requires: the tangency radius goes to zero, the lateral offset loses its wrist contribution, the arm-chain target becomes P5 itself, and the wrist inversion's origins become identity. **The spherical case is the value at zero, not a branch.**

## 4.5 The class condition for the arm chain

The projection argument for θ₁ (Part 5) requires the perpendicular direction between the arm plane and the J5-plane to be **horizontal** — so that the X-Y projection preserves `|AB|`.

- **When `wrist_normal_offset > 0`:** J4's axis must be horizontal. (The perpendicular direction between the arm plane and the J5-plane is J4's axis.)
- **When `wrist_normal_offset = 0`:** no condition — P5 is on the arm plane, the distance is zero, and zero is preserved by any projection.

This is a **conditional class condition**, checked at extraction time (Part 9, Step 0). For the three arms:

| Arm | `wrist_normal_offset` | J4 direction | Condition |
|---|---|---|---|
| UR10 | 0.163941 | `(0, −1, 0)` — horizontal | ✓ |
| FR5 | 0.102100 | `(0, −1, −4e-6)` — horizontal | ✓ |
| Elfin | 0 | `(0, 0, 1)` — vertical, but offset is zero | ✓ |

---

# Part 5 — Solving θ₁

## 5.1 What we need to find

θ₁ is the rotation of the base joint. It determines the **arm plane's azimuth** — the vertical plane through the base axis in which the upper arm and forearm move. Everything after θ₁ is expressed relative to this plane, so θ₁ must be solved first.

The geometry is entirely in the **top view**: the X-Y plane of the true-root frame, looking down J1's axis. In the top view, the arm plane is a line through the origin, and θ₁ is the angle of that line.

## 5.2 The line and the circle

In the top view:

- **The arm plane is a line through the origin**, at azimuth θ₁. The azimuth is measured from the J1 frame's X axis, with `φ₀` as the zero reference — the azimuth of the J1 frame's X axis in the true-root frame (`φ₀ = 0` for UR10, FR5, and Elfin).
- **P5 projects to a point** at `(x₅, y₅)`. Let `r = √(x₅² + y₅²)` and `φ = atan2(y₅, x₅)`.
- **The perpendicular distance from P5's projection to the arm-plane line must equal `|AB| = wrist_normal_offset`.**

**Why `|AB|`?** P5 lies in the J5-axis plane, which is parallel to the arm plane. The perpendicular distance between the two planes is `|AB|` — the distance from the arm plane to B, measured along J4's axis. When `|AB| > 0`, J4's axis is horizontal (the class condition, Part 4.5), so the perpendicular direction is horizontal, and the X-Y projection preserves the distance: the perpendicular distance from P5's projection to the arm-plane line in the top view is also `|AB|`. When `|AB| = 0`, P5 is on the arm plane, and the distance is zero — preserved trivially.

## 5.3 The tangent condition

The arm-plane line through the origin must be at perpendicular distance `|AB|` from P5's projection. This is the tangency condition: **the line through the origin is tangent to a circle of radius `|AB|` centered at P5's projection.**

At the tangent point, the radius (from P5's projection to the tangent point) is perpendicular to the tangent line (the arm plane). This gives a right triangle with the origin, the tangent point, and P5's projection as vertices, right-angled at the tangent point. Its hypotenuse is `r`; the side opposite the angle at the origin is `|AB|`. So:

```
sin(α) = |AB| / r
α = arcsin(|AB| / r)
```

where `α` is the angle between the line from the origin to P5's projection and the tangent line (the arm plane).

## 5.4 The two tangent lines

The line from the origin to P5's projection is at azimuth `φ`. The tangent line is offset from it by `α`. There are two tangent lines:

- One rotated by `+α` from `φ` — **shoulder left** (`σ = −1`).
- One rotated by `−α` from `φ` — **shoulder right** (`σ = +1`).

The sign convention: `σ = +1` for the right-shoulder branch (P5 on the +n̂ side of the arm plane), `σ = −1` for the left-shoulder branch, where `n̂ = (−sin θ₁, cos θ₁, 0)` is the arm plane's normal.

## 5.5 The line-symmetry branch

A line through the origin at azimuth `θ` is the same line as one at `θ + π`. The tangency condition is satisfied by two **lines**, each of which can be traversed in two directions. So the full solution set is:

```
θ₁ ∈ { a1_sign · (φ − φ₀) ± α,  a1_sign · (φ − φ₀) ± α + π }
```

where `a1_sign = +1` if J1's axis points up in the true-root frame, `−1` if down (`+1` for all three arms). The `+π` solutions are the arm plane pointing in the opposite direction — the arm reaches toward P5 or away from it. Both are valid configurations, and both must be returned.

## 5.6 The reachability condition

If `r < |AB|`, then `|AB| / r > 1` and `α` is undefined. No tangent line exists — the circle surrounds the base axis entirely, and the arm cannot reach. **The reachability condition is `r ≥ |AB|`.** When `|AB| = 0`, the condition is `r ≥ 0`, always satisfied.

## 5.7 The spherical case

When `wrist_normal_offset = 0`, `α = 0`. The formula gives:

```
θ₁ ∈ { a1_sign · (φ − φ₀),  a1_sign · (φ − φ₀) + π }
```

These are the two shoulder solutions for the spherical wrist: the arm plane points toward P5 or away from it. **No branch — the general formula at `|AB| = 0`.**

## 5.8 The formula

```
P5 = P_flange − flange_offset · a_flange
r  = √(x₅² + y₅²)
φ  = atan2(y₅, x₅)

if r < wrist_normal_offset: no solution on this arm
α = arcsin(wrist_normal_offset / r)

θ₁ ∈ { a1_sign · (φ − φ₀) ± α,  a1_sign · (φ − φ₀) ± α + π }
```

with `wrist_normal_offset = |AB|` from the extraction, and `φ₀`, `a1_sign` as properties of the true-root frame.

---

# Part 6 — Solving θ₅

## 6.1 What we need to find

θ₅ is the second wrist joint. It rotates the flange about J5's axis. After θ₁, the arm plane is fixed; θ₅ determines how far the flange tilts away from J5's axis. It is solved after θ₁ and before θ₂ and θ₃ because θ₅ is determined by the **position** of the flange's lateral offset from the arm plane — position information that is available before the full arm chain is solved.

## 6.2 The two contributions to the flange's lateral offset

The arm plane's normal, pointing left of the plane's direction, is `n̂ = (−sin θ₁, cos θ₁, 0)`. A point `p` has signed offset from the arm plane:

```
o(p) = dot(p − origin, n̂)
```

The flange's signed offset has two contributions:

**The wrist contribution.** P5 is at perpendicular distance `|AB|` from the arm plane, on the side selected by the shoulder branch. With `σ = +1` for right shoulder and `σ = −1` for left shoulder, the wrist contributes `σ · |AB|`.

**The flange contribution.** The flange is offset from P5 by `flange_offset` along the flange's Z axis. The flange's Z axis is J6's axis, tilted from J5's axis by θ₅. Its component perpendicular to the arm plane is `flange_offset · cos(θ₅)`.

**The total:**

```
o_flange = σ · wrist_normal_offset + flange_offset · cos(θ₅)
```

where `o_flange = −x_flange · sin θ₁ + y_flange · cos θ₁`.

## 6.3 Solving for θ₅

Equating the two expressions and solving:

```
cos(θ₅) = (o_flange − σ · wrist_normal_offset) / flange_offset
```

**Reachability condition:** `|o_flange − σ · wrist_normal_offset| ≤ flange_offset`.

**Two solutions:** `θ₅ = ± arccos(...)` — **no-flip** (`+`) and **flip** (`−`). Both produce the same flange position; they differ in how the wrist is rotated about J5's axis.

## 6.4 The `flange_offset = 0` case

If `flange_offset = 0` — the flange coincides with P5 — the position equation gives `σ · wrist_normal_offset = o_flange`, independent of θ₅. Then θ₅ is determined from the flange's **orientation**:

```
cos(θ₅) = dot(a_flange, n̂) = −a_x · sin θ₁ + a_y · cos θ₁
```

where `a_flange` is the flange's approach vector (the third column of `R_flange`). Two solutions: `± arccos(...)`, same tag convention. **This is a branch in the solver, not a fallback** — both the position-determined and orientation-determined forms are exact, and the condition is `flange_offset > tolerance`.

## 6.5 The spherical case

When `wrist_normal_offset = 0`:

```
cos(θ₅) = o_flange / flange_offset
```

Same formula, at `|AB| = 0`. **No branch.**

## 6.6 The formula

```
o_flange = −x_flange · sin θ₁ + y_flange · cos θ₁

if flange_offset > tol:
    c = (o_flange − σ · wrist_normal_offset) / flange_offset
    if |c| > 1: no solution on this branch
    θ₅ ∈ { +arccos(c) (no-flip),  −arccos(c) (flip) }

if flange_offset ≤ tol:
    c = −a_x · sin θ₁ + a_y · cos θ₁
    if |c| > 1: no solution
    θ₅ ∈ { +arccos(c) (no-flip),  −arccos(c) (flip) }
```

---

# Part 7 — Solving θ₂ and θ₃

## 7.1 What we need to find

We now have θ₁ and θ₅. The arm plane is fixed; the flange's lateral position is accounted for. The remaining arm-chain problem is **2D**: place the wrist root A at a specific point **within the arm plane**, using the upper arm (J2) and forearm (J3).

## 7.2 The wrist root A

A is the point where J4's axis meets the arm plane. It is computed from B (J4 ∩ J5) by projection onto the arm plane along the plane's normal:

```
A = B − dot(B − J2_origin, n̂_plane) · n̂_plane
```

where `n̂_plane = axes[2].direction` (J2's direction, normalized) is the arm plane's normal. For the offset wrist, B is at distance `wrist_normal_offset` from the plane, and A is the projection. For the spherical wrist, B is on the plane, and A = B = P5.

**A is the wrist root.** The arm chain reaches A; the wrist begins at A.

## 7.3 The two-link problem

Within the arm plane, with the shoulder at J2's origin:

- **Shoulder point:** J2's origin.
- **Target point:** A.
- **Upper arm length:** `axis_gap(2)` — the perpendicular distance between J2's axis and J3's axis. (1-based index: `axis_gap(2)` is the gap between J2 and J3.)
- **Forearm length:** `elbow_to_wrist` — the distance from J3's axis to A, `|A − J3_origin|`.

Compute the vector from the shoulder to A, in the arm plane:

```
Δu = dot(A − J2_origin, ê_u)
Δw = dot(A − J2_origin, ê_w)
D² = Δu² + Δw²
D  = √D²
```

where `ê_u`, `ê_w` are two orthogonal unit vectors spanning the arm plane. Concretely, `ê_w` can be J1's axis direction — the vertical direction within the plane — and `ê_u = cross(ê_w, n̂_plane)` the horizontal direction within the plane.

## 7.4 Law of cosines for θ₃

The triangle has sides `axis_gap(2)` (upper arm), `elbow_to_wrist` (forearm), and `D` (shoulder to target). The law of cosines:

```
cos(θ₃) = (D² − axis_gap(2)² − elbow_to_wrist²) / (2 · axis_gap(2) · elbow_to_wrist)
```

**Reachability:** `|axis_gap(2) − elbow_to_wrist| ≤ D ≤ axis_gap(2) + elbow_to_wrist`. If not, no solution exists.

**Two solutions:** `θ₃ = ± arccos(cos θ₃)`. `+` is **elbow-up**; `−` is **elbow-down**.

## 7.5 Law of sines for θ₂

The angle of the shoulder-to-target line, measured from `ê_u`:

```
ψ = atan2(Δw, Δu)
```

The angle between the upper arm and the shoulder-to-target line:

```
β = arcsin(elbow_to_wrist · sin(θ₃) / D)
```

Then:

```
θ₂ = ψ − β · sign(θ₃)
```

The exact form of the sign convention depends on how `ê_u`, `ê_w` are oriented and how θ₂ is defined relative to the J2 frame. **The code's convention is authoritative** — for the Elfin, where J2 and J3 are anti-parallel (`axis_twist(2) = π` in 1-based indexing), the sign of θ₂ must respect the antiparallel relationship; the code handles this from the axis directions directly.

## 7.6 The spherical case

When `wrist_normal_offset = 0` (spherical wrist), B is on the arm plane, so A = B = P5. The forearm length becomes `elbow_to_wrist = |P5 − J3_origin|`. **Same formula, with A = P5.** The law of cosines is unchanged — no branch.

## 7.7 The formula

```
A = B − dot(B − J2_origin, n̂_plane) · n̂_plane
Δu = dot(A − J2_origin, ê_u)
Δw = dot(A − J2_origin, ê_w)
D  = √(Δu² + Δw²)

if not (|axis_gap(2) − elbow_to_wrist| ≤ D ≤ axis_gap(2) + elbow_to_wrist):
    no solution

cos(θ₃) = (D² − axis_gap(2)² − elbow_to_wrist²) / (2 · axis_gap(2) · elbow_to_wrist)
θ₃ ∈ { +arccos(...) (elbow-up),  −arccos(...) (elbow-down) }

ψ = atan2(Δw, Δu)
β = arcsin(elbow_to_wrist · sin(θ₃) / D)
θ₂ = ψ − β · sign(θ₃)
```

with `axis_gap(2)` = upper arm, `elbow_to_wrist = |A − J3_origin|`.

---

# Part 8 — Solving θ₄ and θ₆

## 8.1 The equation

The flange orientation is the product of all six joint rotations, each about its own axis, **each expressed back in the base frame** by the origin transform between consecutive joint frames:

```
R_flange = R₁(θ₁) · R₂(θ₂) · R₃(θ₃) · R₄(θ₄) · R₅(θ₅) · R₆(θ₆)
```

The first three are known (θ₁, θ₂, θ₃ are solved). Define:

```
R_frame4 = R₁(θ₁) · R₂(θ₂) · R₃(θ₃) · R_o4
R_diff   = R_frame4ᵀ · R_flange
```

where `R_o4` is the rotation part of J4's origin transform. Then the wrist subchain must produce:

```
R_z(θ₄) · R_o5 · R_z(θ₅) · R_o6 · R_z(θ₆) = R_diff
```

Each `R_z(θᵢ)` is the rotation about joint i's local Z axis. Each `R_oi` is the **re-expression** — the origin transform that carries the next joint's axis into the base frame after the previous joint's rotation.

**The axes can be apart.** J4's axis and J5's axis need not intersect; J5's and J6's need not intersect. When they are apart, the re-expression is not identity, and the `R_oi` carry the offset's orientation.

**This is the general case: the offset wrist.**

## 8.2 The spherical limit

When all the wrist axes intersect at one point — the wrist center P5 — the re-expression becomes identity:

- Rotating about one axis carries the other axes with it, but if the axes intersect at a common point, the **position** of the intersection does not change, and the next axis's direction relative to the current frame is unchanged by the rotation.
- In frame terms: `R_o5 = R_o6 = I` when the axes intersect at a common point.
- The composition reduces to `R_z(θ₄) · R_z(θ₅) · R_z(θ₆) = R_diff` — the **ZYZ** form.

**The ZYZ form is the spherical limit of the origin-included form.** It is not a separate derivation; it is what the general equation becomes when the re-expression is identity.

## 8.3 The solution — Paden-Kahan subproblem 2

Define:

```
A = R_o5 · R_z(θ₅) · R_o6
B = R_diff
```

**`A` is known** — a fixed 3×3 rotation computed from the origins and θ₅. The equation becomes:

```
R_z(θ₄) · A · R_z(θ₆) = B
```

**This is Paden-Kahan subproblem 2:** rotation about two axes (both local Z) to achieve a given rotation, with a fixed rotation `A` between them.

**Key observation:** the equation is linear in `R_z(θ₄)` on the left and `R_z(θ₆)` on the right:

```
R_z(θ₄) · A = B · R_z(−θ₆)
```

The unknown θ₄ appears only on the left; θ₆ only on the right.

### Step 1 — θ₄ from the third column

Consider the **third column** of both sides. The right side `B · R_z(−θ₆)` has third column `b₃` (the third column of `B`), independent of θ₆. The left side `R_z(θ₄) · A` has third column `R_z(θ₄) · a₃`, where `a₃ = A[:,2]`. So:

```
R_z(θ₄) · a₃ = b₃
```

`R_z` preserves the Z component, so this requires `a₃_z = b₃_z` — a **consistency condition** on the input. Given that, θ₄ is determined by the X-Y components:

```
θ₄ = atan2(b₃_y, b₃_x) − atan2(a₃_y, a₃_x)
```

**θ₄ is solved from the third column alone.**

### Step 2 — θ₆ from the first column

With θ₄ known, the equation becomes `A · R_z(θ₆) = R_z(−θ₄) · B`. The third column is automatically satisfied. Consider the **first column**: `A · R_z(θ₆)` has first column `cos θ₆ · a₁ + sin θ₆ · a₂`, where `a₁`, `a₂` are the first two columns of `A`. The right side has first column `c₁ = R_z(−θ₄) · b₁`. Since `a₁`, `a₂` are orthogonal:

```
θ₆ = atan2(dot(c₁, a₂), dot(c₁, a₁))
```

**θ₆ is solved from the first column, given θ₄.**

### Step 3 — the spherical limit

When `R_o5 = R_o6 = I` (spherical wrist, canonical frame placement):

```
A = R_z(θ₅)   →   a₃ = (0, 0, 1)ᵀ
```

The third column is invariant under θ₄ (`R_z(θ₄) · (0,0,1)ᵀ = (0,0,1)ᵀ`), so the third-column method degenerates. **The solution must come from the other columns**, and it gives the standard **ZYZ formulas**:

```
θ₄ = atan2(B[1,2], B[0,2])       (with sign flips for sin θ₅ < 0)
θ₆ = atan2(B[2,1], −B[2,0])
```

**These are not a separate derivation.** They are the `A[:,2] ∥ ẑ` limit of the general method — the spherical case, where the third column is invariant and the solution uses the other columns.

### Step 4 — the singularity

When `sin θ₅ = 0` (θ₅ = 0 or π), the axes `a₄` and `a₆` become **parallel in the effective composition** — the rotations about J4 and J6 are about the same axis (spherical) or about parallel axes (offset with `sin θ₅ = 0`). **θ₄ and θ₆ are not independent.** Only their **sum** (θ₅ = 0) or **difference** (θ₅ = π) is determined.

**The solver sets θ₄ to a convenient value** — its current value, for continuity — and solves θ₆ from the remaining matrix entries. The choice of θ₄ is arbitrary; the flange reaches the correct orientation regardless, but θ₄ may not match the robot's previous state.

## 8.4 The formula

```
R_frame4 = R₁(θ₁) · R₂(θ₂) · R₃(θ₃) · R_o4
B        = R_frame4ᵀ · R_flange
A        = R_o5 · R_z(θ₅) · R_o6

if |A[:,2]_xy| > tol:   # offset wrist: third-column method
    θ₄ = atan2(B[1,2], B[0,2]) − atan2(A[1,2], A[0,2])
else:                    # spherical limit: ZYZ formulas
    θ₄ = atan2(B[1,2], B[0,2])    (sign flips for sin θ₅ < 0)

if |sin θ₅| > tol:       # non-singular
    c₁ = R_z(−θ₄) · B[:,0]
    θ₆ = atan2(dot(c₁, A[:,1]), dot(c₁, A[:,0]))
else:                    # singular: θ₅ = 0 or π
    θ₄ = current θ₄      # for continuity
    θ₆ = solve from remaining entries
```

## 8.5 What the wrist inversion uses

The wrist inversion uses:

- **`R_frame4`** — J4's frame in the base frame, from the arm chain and `R_o4`.
- **The origins `R_o5`, `R_o6`** — the rotation parts of J5's and J6's origin transforms.
- **`θ₅`** — from Part 6.
- **`R_flange`** — the target orientation.

It does **not** use `wrist_normal_offset`, `wrist_radial_offset`, or `wrist_offset` as scalars. The wrist inversion uses the **axes and origins** directly, because the geometry of the wrist is the geometry of the origins and axes — not of the scalar distances. **The distances are for the arm chain (Parts 5–7); the origins and axes are for the wrist inversion (Part 8).**

---

# Part 9 — The Full Algorithm and Worked Examples

## 9.1 The algorithm, stated as a sequence

The input is the **flange pose in the true-root frame**: a 4×4 homogeneous transformation `T_flange`. The output is a list of up to 8 tagged joint vectors.

**Step 0 — Precondition check.** Verify the arm is in the supported class:

- Six revolute joints.
- J5 and J6 intersect (within tolerance) — this is what makes P5 exist.
- J4 ⊥ J5, J5 ⊥ J6, J4 ∥ J6 (within tolerance).
- If `wrist_normal_offset > 0`: J4's axis is horizontal.

If any fails, refuse loudly with a message naming the violation and the measured value.

**Step 1 — Compute P5:**

```
P5 = P_flange − flange_offset · a_flange
```

**Step 2 — Solve θ₁:**

```
r = √(x₅² + y₅²);  φ = atan2(y₅, x₅)
if r < wrist_normal_offset: no solution
α = arcsin(wrist_normal_offset / r)
θ₁ ∈ { a1_sign·(φ−φ₀) ± α,  a1_sign·(φ−φ₀) ± α + π }
```

Each θ₁ carries a shoulder sign `σ = +1` (right) or `σ = −1` (left).

**Step 3 — Solve θ₅:** For each θ₁ branch:

```
o_flange = −x_flange · sin θ₁ + y_flange · cos θ₁

if flange_offset > tol:
    c = (o_flange − σ · wrist_normal_offset) / flange_offset
else:
    c = −a_x · sin θ₁ + a_y · cos θ₁
if |c| > 1: no solution on this branch
θ₅ ∈ { +arccos(c) (no-flip),  −arccos(c) (flip) }
```

**Step 4 — Solve θ₂, θ₃:** For each (θ₁, θ₅):

```
A = B − dot(B − J2_origin, n̂) · n̂
Δu = dot(A − J2_origin, ê_u);  Δw = dot(A − J2_origin, ê_w)
D  = √(Δu² + Δw²)
if not (|axis_gap(2) − elbow_to_wrist| ≤ D ≤ axis_gap(2) + elbow_to_wrist): no solution
cos(θ₃) = (D² − axis_gap(2)² − elbow_to_wrist²) / (2·axis_gap(2)·elbow_to_wrist)
θ₃ ∈ { +arccos (elbow-up),  −arccos (elbow-down) }
ψ = atan2(Δw, Δu);  β = arcsin(elbow_to_wrist·sin θ₃ / D)
θ₂ = ψ − β·sign(θ₃)
```

**Step 5 — Solve θ₄, θ₆:** For each (θ₁, θ₂, θ₃, θ₅):

```
R_frame4 = R₁(θ₁)·R₂(θ₂)·R₃(θ₃)·R_o4
B = R_frame4ᵀ · R_flange
A = R_o5 · R_z(θ₅) · R_o6

if |A[:,2]_xy| > tol:  θ₄ = atan2(B[1,2], B[0,2]) − atan2(A[1,2], A[0,2])
else:                   θ₄ = atan2(B[1,2], B[0,2])

if |sin θ₅| > tol:
    c₁ = R_z(−θ₄)·B[:,0]
    θ₆ = atan2(dot(c₁, A[:,1]), dot(c₁, A[:,0]))
else:
    θ₄ = current θ₄;  θ₆ = solve from remaining
```

**Step 6 — Assemble and tag.**

| Level | Tag values |
|---|---|
| Shoulder | `left`, `right` |
| Elbow | `up`, `down` |
| Wrist | `flip`, `no-flip` |

Up to 8 solutions total.

**Step 7 — Filter by joint limits**, modulo 2π:

```
if not (lower ≤ θ ≤ upper):
    θ += 2π  (or − 2π)
    if not (lower ≤ θ ≤ upper): reject
```

**Step 8 — Return** all valid, tagged solutions. Do not select. The caller selects by continuity — preferring the branch that matches the current configuration.

## 9.2 Worked example: UR10 (offset wrist)

**Parameters** (from the extraction):

| Quantity | Value |
|---|---|
| `wrist_normal_offset` (`\|AB\|`) | 0.163941 |
| `wrist_radial_offset` (`\|BC\|`) | 0.115700 |
| `wrist_offset` (`\|AC\|`) | 0.200657 |
| `flange_offset` | 0.092200 |
| `axis_gap(2)` (upper arm) | 0.612000 |
| `elbow_to_wrist` (J3→A) | 0.572300 |
| J4 axis direction | `(0, −1, 0)` — horizontal ✓ |

**Target** (in true-root frame): `P_flange = (0.5, 0.3, 0.8)`, `R_flange = identity`.

**Step 1:** `P5 = (0.5, 0.3, 0.8 − 0.0922) = (0.5, 0.3, 0.7078)`.

**Step 2:** `r = 0.5831`, `φ = 0.5404`, `α = arcsin(0.163941/0.5831) = 0.2852`. So `θ₁_right ≈ 0.2552` (`σ = +1`), `θ₁_left ≈ 0.8256` (`σ = −1`), plus the `+π` branches.

**Step 3:** For the right-shoulder branch: `o_flange ≈ 0.1640`, `c ≈ 0.0006`, `θ₅ ≈ ±1.5702` (flip and no-flip).

**Step 4:** Compute A, Δu, Δw, D, then the law of cosines. The exact values depend on the full computation; the test suite (`test_unified_ik.py`) is the authoritative round-trip check: `q_true → FK → IK → set containing q_true`.

**Step 5:** `A[:,2]` is not along ẑ (the origins are not identity), so the third-column method applies. `θ₄` from the third column; `θ₆` from the first.

**Steps 6–8:** Assemble, tag, filter by limits, return.

**What the example shows:** every step uses the settled quantities (`wrist_normal_offset`, `axis_gap(2)`, `elbow_to_wrist`, `flange_offset`). **No DH symbols.**

## 9.3 Worked example: FR5 (offset wrist)

**Parameters:**

| Quantity | Value |
|---|---|
| `wrist_normal_offset` | 0.102100 |
| `wrist_radial_offset` | 0.102000 |
| `wrist_offset` | 0.144319 |
| `flange_offset` | 0.100000 |
| `axis_gap(2)` | 0.425000 |
| `elbow_to_wrist` | 0.395010 |
| J4 axis direction | `(0, −1, −4e-6)` — horizontal ✓ |

**Same target, same algorithm.** The values differ; the steps are identical. The wrist inversion uses the third-column method (`A[:,2]` not along ẑ).

**Note on the origin trap:** for FR5, `axis_shift(3) = wrist_normal_offset = 0.1021` (J4's origin is at A), while for UR10 `axis_shift(3) = 0.0`. **The algorithm is immune to this** because it uses `wrist_normal_offset`, not `axis_shift(3)`. This is why the origin trap was important to identify: the algorithm never touches the origin-dependent quantity.

## 9.4 Worked example: Elfin (spherical wrist)

**Parameters:**

| Quantity | Value |
|---|---|
| `wrist_normal_offset` | 0 |
| `wrist_radial_offset` | 0 |
| `wrist_offset` | 0 |
| `flange_offset` | 0.200000 |
| `axis_gap(2)` | 0.730000 |
| `elbow_to_wrist` (J3→P5) | 0.570000 |
| J4 axis direction | `(0, 0, 1)` — vertical, but offset is zero ✓ |
| `axis_twist(2)` (J2–J3, 1-based) | π (anti-parallel) |

**Same target, same algorithm.**

**Step 2:** `α = arcsin(0/r) = 0`, so `θ₁ ∈ { φ, φ + π }`.

**Step 3:** `c = o_flange / 0.2` (since `wrist_normal_offset = 0`). Two solutions: ± arccos.

**Step 4:** A = B = P5, so `elbow_to_wrist = |P5 − J3_origin| = 0.570000`. The law of cosines uses `axis_gap(2) = 0.730000` and `elbow_to_wrist = 0.570000`.

**Step 5:** For Elfin, `A = R_o5 · R_z(θ₅) · R_o6`. With the canonical spherical frame placement, `R_o5 = R_o6 = I`, so `A = R_z(θ₅)` and `A[:,2] = (0,0,1)ᵀ` — the spherical limit. The third-column method degenerates and the ZYZ formulas apply.

**What the example shows:** the spherical wrist is the **limit** of the general algorithm. Every step is the same; the quantities that are zero (`wrist_normal_offset`, `wrist_radial_offset`, `wrist_offset`) make the formulas degenerate, and the ZYZ formulas are the `A[:,2] ∥ ẑ` limit of the wrist inversion. **No branch — the spherical case is the value at zero.**

## 9.5 The reversal, stated

The conventional derivation treats the **spherical wrist** as the natural case and the **offset wrist** as a complication. It derives the spherical formulas first, and the offset wrist is a fourth branch, a perturbation, a separate derivation.

**This document reverses that.** The general case is the **offset wrist** — the axes are apart, and each joint's rotation must be re-expressed in the base frame by the origin transform. The **spherical wrist** is the case where the axes intersect at one point, and the re-expression becomes identity. The ZYZ formulas are the spherical limit of the origin-included form; they are not a separate derivation.

**The algorithm is one.** Every step uses the settled quantities, and the spherical case falls out at `wrist_normal_offset = 0`, `wrist_radial_offset = 0`, `wrist_offset = 0`. No branch, no special case — the general formula degenerates to the spherical formulas as the geometry requires.

**This is the reversal:** the offset wrist is the general case; the spherical wrist is the value at zero. The conventional derivation has it backwards.

## 9.6 The full formula set

```
# Step 1
P5 = P_flange − flange_offset · a_flange

# Step 2
r = √(x₅² + y₅²);  φ = atan2(y₅, x₅)
α = arcsin(wrist_normal_offset / r)   (if r ≥ wrist_normal_offset)
θ₁ ∈ { a1_sign·(φ−φ₀) ± α,  a1_sign·(φ−φ₀) ± α + π }

# Step 3
o_flange = −x_flange·sin θ₁ + y_flange·cos θ₁
c = (o_flange − σ·wrist_normal_offset) / flange_offset   (if flange_offset > tol)
c = −a_x·sin θ₁ + a_y·cos θ₁                              (if flange_offset ≤ tol)
θ₅ ∈ { ± arccos(c) }   (if |c| ≤ 1)

# Step 4
A = B − dot(B − J2_origin, n̂)·n̂
D = |A − J2_origin|  (in the arm plane)
cos(θ₃) = (D² − axis_gap(2)² − elbow_to_wrist²) / (2·axis_gap(2)·elbow_to_wrist)
θ₃ ∈ { ± arccos(cos θ₃) }
ψ = atan2(Δw, Δu);  β = arcsin(elbow_to_wrist·sin θ₃ / D)
θ₂ = ψ − β·sign(θ₃)

# Step 5
A = R_o5 · R_z(θ₅) · R_o6
B = (R₁·R₂·R₃·R_o4)ᵀ · R_flange
θ₄ = atan2(B[1,2], B[0,2]) − atan2(A[1,2], A[0,2])   (offset)
θ₄ = atan2(B[1,2], B[0,2])                            (spherical limit)
c₁ = R_z(−θ₄)·B[:,0]
θ₆ = atan2(dot(c₁, A[:,1]), dot(c₁, A[:,0]))          (non-singular)
θ₄ = current θ₄;  θ₆ = solve from remaining           (singular, sin θ₅ = 0)

# Steps 6–8
Tag (shoulder, elbow, wrist), filter by joint limits modulo 2π, return all valid.
```

**No DH symbols anywhere.** `wrist_normal_offset`, `axis_gap(2)`, `elbow_to_wrist`, `flange_offset` — all geometric, all named for what they are.

---

# Appendix A — DH Comparison

## A.1 What DH encodes

The Denavit–Hartenberg convention describes the relationship between two consecutive joint axes with four numbers: `d` (offset along the previous joint's axis), `a` (distance along the common perpendicular), `α` (twist angle between the axes), `θ` (joint angle, the variable).

## A.2 How our quantities relate to DH

| Hatch quantity | DH analog | Relationship |
|---|---|---|
| `axis_gap(i)` | `a` (link length) | Perpendicular distance between axes i and i+1. Same value, different name. |
| `axis_twist(i)` | `α` (link twist) | Angle between axes i and i+1. Same value, different name. |
| `axis_shift(i)` | `d` (link offset) | Distance along axis i to the common perpendicular. Origin-dependent. |
| `joint_angle(i)` | `θ` (joint variable) | The joint's rotation. Same value. |
| `wrist_normal_offset` | **no direct DH analog** | DH `d₄` happens to equal it for these arms, but only because of DH's frame placement (see A.4). The geometric quantity is the plane-to-J5-axis distance. |

## A.3 Why Hatch does not use DH

1. **DH is lossy for the general case.** The DH convention assumes the joint frames are placed canonically; the placement is not unique and is ambiguous when two consecutive axes are parallel.
2. **URDF gives more information.** A URDF joint origin is a full 6-DOF transform. DH compresses it into four scalars.
3. **DH requires a convention choice the solver does not need.** Hatch works directly with the joint axes as lines in space.

## A.4 The joint-4 offset: DH `d₄` vs. `wrist_normal_offset`

In the standard UR DH table, `d₄` is labelled "the wrist offset." For UR10, `d₄ = 0.163941 m`. The extraction reports `wrist_normal_offset = 0.163941 m`. **They are numerically equal** — but for a frame-placement reason, not a geometric one.

The geometric quantity is the **perpendicular distance from the arm plane to J5's axis**. For these arms, DH's frame placement happens to make `d₄` equal to it. The solver uses `wrist_normal_offset`, computed from the axes as lines. The equality is a coincidence of frame placement, not an identity. For an arm whose DH frames are placed differently, `d₄` might not equal `wrist_normal_offset`.

**`axis_shift` is not `wrist_normal_offset` either — at any index.** The `axis_shift(i)` quantity is the signed distance along joint `i`'s axis between the feet of the common perpendiculars with joints `i-1` and `i+1`. It is origin-dependent: it is measured from joint `i`'s origin, and the URDF may place that origin anywhere along the axis. `wrist_normal_offset`, by contrast, is computed from axis intersections and is origin-independent.

The two differ, and the numbers make the difference visible:

| Arm | `wrist_normal_offset` | `axis_shift(3)` (along J3) | `axis_shift(4)` (along J4) |
|---|---|---|---|
| UR10 | 0.163941 | 0.163941 | 0.000000 |
| FR5 | 0.102100 | 0.000000 | 0.102100 |
| Elfin | 0.000002 | 0.000000 | 0.570000 |

For UR10, `axis_shift(3)` happens to equal `wrist_normal_offset` — a coincidence of J3's origin placement and the geometry. For FR5, `axis_shift(3)` is 0.0, and the "along J4" quantity `axis_shift(4)` is 0.1021 — close to `wrist_normal_offset` but at a different index. **The three quantities are distinct, and their numerical coincidences depend on where the URDF places the joint origins.**

**The algorithm uses `wrist_normal_offset`, never `axis_shift(i)`** — which is why it is immune to the origin placement, and why it gives the same answer for UR10 and FR5 (the same geometric quantity, computed from the axes as lines, independent of where the origins sit).

## A.5 Summary

For the reader who knows DH: the four axis quantities are the same physical information, renamed. At joint 4, DH's `d₄` coincides numerically with `wrist_normal_offset` for these arms, but the geometric quantity is the plane-to-J5-axis distance, and the solver uses it. DH is not used anywhere in the derivation.

---

# Appendix B — What Hatch Refuses to Do

## B.1 The supported class

Hatch's analytical IK solver applies to **arm-plane 6R arms with intersecting wrist axes**:

1. **Six revolute joints.** Not 5, not 7.
2. **J5 and J6 intersect** (within tolerance). This is what gives the wrist a center P5.
3. **J4 ⊥ J5, J5 ⊥ J6, J4 ∥ J6** (within tolerance).
4. **If `wrist_normal_offset > 0`: J4's axis is horizontal.** (The projection condition for θ₁.)
5. **The arm chain is in the arm plane** — J2 and J3 are parallel in the general sense (same or opposite direction), and the arm plane is well-defined.

Robots in this class: UR10, UR5, FR5, Elfin E15 Pro, and most industrial 6-axis arms. The class covers both spherical wrists (`wrist_normal_offset = 0`) and offset wrists (`wrist_normal_offset ≠ 0`).

## B.2 The refusal policy

If a robot violates any condition, Hatch **refuses loudly**. It does not approximate the geometry to fit the solver, fall back to a numerical solver silently, or return a solution that is "close" but not exact.

The refusal is a `RuntimeError` with a message that states the violation and the measured value:

```
Unsupported arm geometry: J4 and J5 are not perpendicular
(axis_twist(3) = 0.523 rad, expected π/2 within 1.0e-04).
The analytical solver requires perpendicular wrist axes.
```

The message names the violated condition and the measured value. The caller knows exactly what is wrong.

## B.3 Why refuse instead of approximate

**Because a plausible-looking wrong answer is worse than no answer.** If the solver approximated a non-class robot, it would produce joint angles that place the flange near the target but not exactly at it. The error might be small for some poses and large for others. The caller would have no way to know which. In a safety-critical context, that is unacceptable.

The refusal is the safe behavior. It says: "This robot is not in the class I can solve. I will not guess."

## B.4 What is not in the class

- **Robots with skew J2 and J3.** Some painting and welding robots use non-parallel shoulder-elbow axes.
- **Robots with non-intersecting J5 and J6.** Some collaborative robots and some 7-DOF arms.
- **7-DOF arms.** Redundant arms have more than 6 joints.
- **Parallel robots, delta robots, SCARA arms.** Different kinematic structures entirely.

For all of these, Hatch refuses. The word is **unified**, not **universal**. Unified across the supported class — spherical and offset wrists, UR and FR and Elfin — not universal across all robots.

---

# Appendix C — Code Obligations

The extraction and solver code on disk does not yet match this document. The gaps, for the next session:

**Extraction (`geometric_extraction.py`):**

1. **`axis_gap`, `axis_twist`, `axis_shift` indexing.** The code uses 0-based indexing; this document uses 1-based (matching joint numbers). Change `axis_gap(i)` to the gap between J`i` and J`i+1`, for i = 1..5.
2. **`wrist_offset` rename.** The code's `wrist_offset` is `|BC|`; this document calls that `wrist_radial_offset`, and reserves `wrist_offset` for `|AC|`. Add `wrist_radial_offset` and `wrist_offset` as derived properties.
3. **`elbow_to_wrist`.** Add a method returning `|A − J3_origin|`, where A = B projected onto the arm plane along the normal.
4. **Class check.** Add the conditional J4-horizontal check (only when `wrist_normal_offset > 0`). The current check requires J4 ∥ J2 unconditionally, which fails for Elfin; Elfin should pass.
5. **`describe()`.** Print `wrist_normal_offset`, `wrist_radial_offset`, `wrist_offset` side by side, with the triangle's meaning.

**Solver (`unified_ik_solver.py`):**

1. **θ₁.** Use `wrist_normal_offset` (not the old `wrist_offset`), and return all four solutions (both `±α` and both `+π` branches).
2. **θ₅.** Use `wrist_normal_offset` in the lateral offset; `flange_offset` in the denominator.
3. **θ₂, θ₃.** Use `axis_gap(2)` (1-based) and `elbow_to_wrist`; target A (the projection of B).
4. **θ₄, θ₆.** Replace the ZYZ-only path with the origin-included Paden-Kahan form; ZYZ is the `A[:,2] ∥ ẑ` limit.
5. **Tags.** Return `(shoulder, elbow, wrist)` tags on every solution; select by continuity, not proximity.

**Tests:**

1. **Round-trip.** `q_true → FK → IK → set containing q_true` for all three arms, with enough random poses to exercise both wrist branches and the singularities.
2. **Triangle identity.** For each arm: `wrist_offset² = wrist_normal_offset² + wrist_radial_offset²`.
3. **Origin-trap regression.** Confirm `axis_shift(3)` differs from `wrist_normal_offset` for UR10 and equals it for FR5 — documenting the trap, not relying on it.

---

*Hatch (孵) 🐣 — built to understand, built to see.*
*The offset wrist is the general case; the spherical wrist is the value at zero.*
*No DH symbols. No branches. The geometry is what it is.*
