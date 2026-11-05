# Inverse Kinematics in Hatch (孵)

## A Unified Derivation for Offset and Spherical Wrists

---

# Part 1 — The URDF: What It Is, and How to Read It

## 1.1 Why we start here

Everything in this document — every axis, every offset, every angle — is extracted from a URDF file.

The URDF is the single source of truth for the scene (Principle #4). If we do not understand what that file contains, we cannot understand what we extract from it, and we cannot check whether the extraction is correct.

So before we solve anything, we learn to read the file. This part is a reading exercise. By the end, you will be able to open any industrial arm's URDF and point to the line that gives each joint its axis, each link its mesh, and each joint its limits.

## 1.2 What XML is

A URDF is written in **XML**. It has only a few rules, and they are worth knowing precisely, because a URDF is nothing but XML.

**An element** is a pair of tags with content between them. **An attribute** is a name-value pair inside a tag. **Nesting** is elements inside elements. **A self-closing element** is an element with no content (`<link name="base_link"/>`). **A comment** begins with `<!--` and ends with `-->`. **A processing instruction** begins with `<?` and ends with `?>` — every XML file begins with `<?xml version="1.0"?>`, which declares the version and carries no robot information.

**Namespaces** — the `xacro:` prefix, the `xmlns` attributes — are a more advanced XML feature, used when a file is composed from parts. They matter for the `URDFPreprocessor`, which expands xacro templates into plain URDF. They do **not** matter here: by the time the file reaches the solver, it is plain URDF with no namespaces.

That is the whole of XML that a URDF reader needs.

## 1.3 Links: the rigid bodies

A **link** is a rigid body. It has three optional subsections:

- **`<visual>`** — how the link looks. Its geometry, and optionally its material.
- **`<collision>`** — the shape used for collision checking.
- **`<inertial>`** — the mass and inertia tensor, used for dynamics.

For the kinematic solver, only the link's **name** matters. The kinematics is entirely about *how links are connected*, not about what they look like or how heavy they are.

## 1.4 Joints: the connections

A **joint** connects two links — a **parent** and a **child**. It has:

- **`type`** — `fixed` (rigid attachment), `revolute` (rotation with limits), `continuous` (rotation without limits), `prismatic` (translation with limits). **The solver operates on revolute and continuous joints.** Fixed joints define the boundary (the true root, the flange).
- **`<parent>` and `<child>`** — the two links the joint connects.
- **`<origin>`** — the position and orientation of the joint's frame, relative to the parent link's frame. `xyz` is a translation in meters, `rpy` is a rotation as roll-pitch-yaw in radians. **This is the most important element in the URDF for kinematics.**
- **`<axis>`** — the direction of the joint's rotation, expressed in the joint's own frame.
- **`<limit>`** — the joint's physical limits (`lower`, `upper`, `effort`, `velocity`). The solver uses `lower` and `upper` to filter joint angles.

### The Z-axis rule

**All revolute joints rotate about the Z axis of their own frame, according to the right-hand rule.**

This is the URDF convention, and it is the foundation of the entire kinematic description:

- The `<axis>` element specifies the direction of rotation *in the joint's frame*. For a revolute joint that rotates about its own Z, the axis is `0 0 1`.
- The joint angle θ is the angle of rotation about that Z, with the sign given by the right-hand rule.
- When a later part says "J2's axis is normal to the arm plane," it means J2's frame Z, expressed in the true-root frame, is the arm plane's normal. When it says "the twist between J2 and J3 is 0," it means their Z axes point in the same direction.

**The Z axis is the joint's axis of rotation, always.**

## 1.5 The tree structure

A URDF is a **tree**. There is exactly one root link — the link with no parent joint. Every other link has exactly one parent joint. No cycles exist. This is a consequence of physics: a rigid body cannot be in two places at once.

For a 6R arm, the tree contains **more than six joints in total** — but exactly six of them are movable. The six movable joints are the revolute joints of the arm chain. The rest are fixed joints: at the base, at the flange, and possibly between consecutive revolute joints (e.g., a `base_inertia` link between `base_link` and the first revolute joint, as in Universal Robots' URDFs).

**The true kinematic root** is the **parent link of the first movable revolute joint**. For UR10, that is `base_inertia`, not `base_link`. Using the wrong root produces wrong kinematics.

## 1.6 Meshes: the visual geometry

A **mesh** is a piece of 3D geometry. It is referenced by **filename**, not embedded in the URDF:

```xml
<mesh filename="package://ur10/meshes/base_link.stl"/>
```

The reference is a **URI**, resolved through `PackageResolver`: `~/hatch/assets/` first, then every directory in `HATCH_PACKAGE_PATH`.

**The four mesh formats:** STL (triangles only), OBJ (triangles with materials), PLY (triangles with per-vertex color), DAE/COLLADA (a full scene graph). **The solver never reads a mesh.** Meshes are for visualization; the solver's input is the joint origins and axes.

## 1.7 A minimal 6R arm

Here is the **minimal** URDF of a 6R arm — the smallest file that still has the full structure. It has no meshes, and its links are short, but it is not a simplification: it is the **bone** of a real industrial arm. Six revolute joints, fixed joints at the base and the flange, and a true root that is not `base_link`.

```xml
<?xml version="1.0"?>
<robot name="minimal_6r_arm">
  <link name="world"/>
  <joint name="mount_joint" type="fixed">
    <parent link="world"/> <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="base_link"/>
  <joint name="base_inertia_joint" type="fixed">
    <parent link="base_link"/> <child link="base_inertia"/>
    <origin xyz="0 0 0" rpy="0 0 3.14159"/>
  </joint>
  <link name="base_inertia"/>
  <joint name="joint_1" type="revolute">
    <parent link="base_inertia"/> <child link="link_1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="150" velocity="3.15"/>
  </joint>
  <link name="link_1"/>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/> <child link="link_2"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="150" velocity="3.15"/>
  </joint>
  <link name="link_2"/>
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/> <child link="link_3"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="3.15"/>
  </joint>
  <link name="link_3"/>
  <joint name="joint_4" type="revolute">
    <parent link="link_3"/> <child link="link_4"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>
  <link name="link_4"/>
  <joint name="joint_5" type="revolute">
    <parent link="link_4"/> <child link="link_5"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>
  <link name="link_5"/>
  <joint name="joint_6" type="revolute">
    <parent link="link_5"/> <child link="link_6"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="3.15"/>
  </joint>
  <link name="link_6"/>
  <joint name="flange_joint" type="fixed">
    <parent link="link_6"/> <child link="flange"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>
  <link name="flange"/>
</robot>
```

Read it against the concepts: the tree, the fixed joints, the true root (`base_inertia`), the six movable joints, the origins, the flange, the meshes (omitted for brevity).

A reader who can read this file can read any industrial arm's URDF. The real UR10 file has more links and more detail, but the structure is the same.

## 1.8 The limitation of this document

This document is about **6R arms** — six-joint serial robots with revolute joints, of the class described in Part 9. Not every robot is a 6R arm: SCARA arms, delta robots, parallel robots, 7-DOF redundant arms, and mobile robots are outside the class. Hatch's analytical solver does not support them. The refusal is deliberate (Part 9, Step 0).

## 1.9 What we have, and what comes next

We now know what a URDF contains. And we know what the URDF does *not* give us directly: the **geometric quantities** the solver needs. Those are *derived* from the joint origins and axes. The direction is:

> **URDF → geometric quantities → solver**

not:

> **URDF → DH table → solver**

Part 2 begins the derivation.

---

# Part 2 — Seeing the Robot

## 2.1 What a robot arm is made of

A 6-DOF industrial arm is a chain of rigid bodies connected by joints. Each joint allows exactly one motion: rotation about a line — its **joint axis**. The robot's geometry is the collection of these axes and the fixed relationships between them. Everything else — the meshes, the colors, the physical shape of the links — is decoration. **The kinematics is entirely captured by the axes.**

## 2.2 What the URDF gives us

For each joint: the **origin** (`xyz`, `rpy`), the **axis** (`xyz`), and the **type**. For a revolute joint, this is complete. The origin tells us where the axis sits and how it is oriented; the axis tells us the direction of rotation; the type tells us the joint moves. **This is all the kinematics we need.**

## 2.3 The wrist

The last three joints — J4, J5, J6 — form the wrist. Their geometry has a property the rest of the arm does not: **J5 and J6 always intersect** (for the class this document covers). Their intersection is a point — the **wrist center**, P5.

This is what makes the wrist separable from the arm:

- **P5 is fixed by the flange pose.** Given the target, `P5 = P_flange − flange_offset · a_flange`.
- **J4 may or may not pass through P5.** If it does, the wrist is *spherical*. If it does not, the wrist is *offset*.
- **The offset is the quantity that varies.** In the spherical wrist, J4, J5, J6 all intersect at P5. In the offset wrist, J4's axis passes at a perpendicular distance from P5.

**The document derives the general case — the offset wrist — and lets the spherical wrist fall out at offset zero.** The conventional derivation does the reverse; this document reverses it (Part 9, §9.5).

## 2.4 The three points

The wrist geometry is captured by three points:

- **A** — the **wrist root**. The point where J4's axis meets the **arm plane** (the plane the upper arm and forearm sweep through). The end of the forearm.
- **B** — the **wrist's first bend**. J4's axis ∩ J5's axis. On J4's axis, at distance `|AB|` from A.
- **C = P5** — the **wrist center**. J5's axis ∩ J6's axis.

These three points form a **right triangle at B**:

- **AB** is along J4's axis — **perpendicular** to the arm plane.
- **BC** is along J5's axis — **parallel** to the arm plane.
- **AC** is the hypotenuse — from the wrist root to the wrist center.

**When the wrist is spherical, A, B, and C coincide.** The triangle collapses to a point. But the triangle's *structure* is still the structure of the geometry — it is just the structure at zero. **The spherical wrist is not a different geometry; it is the geometry at a limit.**

## 2.5 Why we do not need DH

The Denavit–Hartenberg convention compresses the relationship between two consecutive joint axes into four scalars, by assuming the frames are placed in a canonical way. The URDF gives us **more information**: each joint origin is a full 6-DOF transform. The compression is lossy, works only for the class DH was designed for, and is ambiguous when two consecutive axes are parallel.

Hatch does not build a DH table. It reads the joint origins and axes directly from the URDF. The direction is:

> **URDF → geometric quantities → solver**

not:

> **URDF → DH table → solver**

---

# Part 3 — The Five Geometric Quantities

## 3.1 The question we are answering

Every joint in a serial arm rotates about a line. To describe the robot, we need to describe how each joint axis relates to the next. Two lines in space are related by four numbers: how far apart, how rotated, where along the first line the second attaches, and how much the joint has rotated. **These are the four quantities.** The wrist adds a fifth.

## 3.2 The four axis quantities

For each consecutive pair of joint axes (i, i+1), for i = 1 to 5:

**`axis_gap(i)`** — the perpendicular distance between axes i and i+1, measured along their **common perpendicular**. For UR10, `axis_gap(2)` (J2 to J3) is 0.612 m — the upper arm.

**`axis_twist(i)`** — the angle between the two axis directions, signed, measured about the common perpendicular. For UR10, `axis_twist(1)` (J1 to J2) is π/2. `axis_twist(2)` (J2 to J3) is 0. For Elfin, `axis_twist(2)` is π (anti-parallel).

**`axis_shift(i)`** — the signed distance along axis i between the feet of the common perpendiculars with the previous and next axes. **Origin-dependent.** For UR10, `axis_shift(3)` is 0.163941; for FR5 it is 0.0. It is **not** `wrist_normal_offset` — see §3.4 and Appendix A.

**`joint_angle(i)`** — the rotation of joint i. The variable we solve for.

**Indexing.** The quantities are indexed **1-based**, matching the joint numbers: `axis_gap(2)` is the gap between J2 and J3.

## 3.3 Why these names

The classical names describe the **link** — the rigid body between the joints. But the quantities do not describe the link; they describe the relationship between two **joint axes**. The physical length of the link and the distance between its two joint axes are different numbers. Naming the distance between axes "link length" invites the reader to picture the wrong thing.

Naming the quantities after the axes makes the geometry visible: `axis_gap` is a distance between axes; `axis_twist` is an angle between axes.

## 3.4 The fifth quantity: `wrist_normal_offset`

**`wrist_normal_offset` = the perpendicular distance from the arm plane to J5's axis.**

This is the quantity the arm-chain derivation uses — for the θ₁ rectangle (Part 5) and the θ₅ lateral offset (Part 6). It is computed from the axes as lines, not from the joint origins:

- **B** = J4's axis ∩ J5's axis.
- **`wrist_normal_offset` = |dot(B − J2's origin, n̂)|**, where `n̂` is J2's axis direction (normal to the arm plane).

**Why it is origin-independent.** B is an axis intersection — an invariant point. J2's origin is on the arm plane. The perpendicular distance from B to the plane is a geometric fact, independent of where any joint origin sits along its axis. This matters: UR and FR place J4's origin at opposite ends of the A–B segment, so any origin-derived computation would disagree between the two arms. The line-based computation does not.

## 3.5 The right triangle at B

The fifth quantity is one side of the **right triangle A–B–C**, right-angled at B. The three sides are the three wrist quantities:

| Symbol | Name | What it is | Role |
|---|---|---|---|
| `\|AB\|` | **`wrist_normal_offset`** | Perpendicular distance from the arm plane to J5's axis. AB is normal to the arm plane. | **θ₁ rectangle, θ₅ lateral offset** |
| `\|BC\|` | **`wrist_radial_offset`** | Perpendicular distance from C to J4's axis. BC is parallel to the arm plane, in the plane through B at distance `\|AB\|` from the arm plane. | Wrist inversion geometry |
| `\|AC\|` | **`wrist_offset`** | The hypotenuse: `√(\|AB\|² + \|BC\|²)`. Distance from wrist root A to wrist center C. | Derived; not used as a scalar by the solver |

**The solver's scalar inputs are `wrist_normal_offset`, `flange_offset`, `axis_gap(2)`, and `forearm_length()`.** The other two sides of the triangle — `|BC|` and `|AC|` — are derived quantities. They are stated here because they make the geometry visible (the triangle, the right angle at B, the spherical case as all three going to zero), but the solver does not take them as scalars.

**All three are zero for a spherical wrist.** They are distinct quantities, and the conventional term "wrist offset" is ambiguous — it is used sometimes for `|BC|` and sometimes for `|AC|`. This document names all three explicitly.

## 3.6 The derived lengths

Two lengths the arm chain uses are not among the five quantities:

**`forearm_length()`** — the distance from J3's axis to **Q** (the arm chain's target, P5's projection onto the arm plane; Part 7). This is the "forearm" in the law of cosines. It is **not** `axis_gap(3)` (the J3–J4 perpendicular distance), and it is **not** `elbow_to_wrist` (the distance to A).

**`flange_offset`** — the distance from P5 to the flange along the flange's Z axis. This is `d₆` in the conventional notation.

The law of cosines uses `axis_gap(2)` (the upper arm, J2 to J3) and `forearm_length()` (the forearm, J3 to Q). The θ₅ formula uses `flange_offset`.

## 3.7 What the five quantities describe — and what they do not

The five quantities describe the **class of arms this document covers**: arm-plane 6R arms with intersecting wrist axes. They plus the URDF's joint origin transforms are a complete kinematic description of that class.

They do **not** describe every 6R arm. For an arm outside the class — skew J2/J3, non-intersecting J5/J6 — the five quantities are not sufficient, and the solver refuses (Part 9, Step 0). The refusal is honest: the five quantities are not claimed to be universal.

**They are not a replacement for the URDF.** They are a *selection* from the URDF — the geometric quantities the derivation needs, named for what they mean. The solver still reads the URDF's origins for the wrist inversion (Part 8) and for the class check.

**DH is not needed and not used.** The five quantities are not DH parameters under different names — they are geometric quantities. `wrist_normal_offset` in particular is not a DH parameter at all; DH's `d₄` happens to equal it for these arms, but that is a frame-placement coincidence (Appendix A).

---

# Part 4 — The Wrist Root and the Arm Plane

## 4.1 The arm plane

The **arm plane** is the plane containing the origins of J2 and J3 — the plane the upper arm and forearm sweep through as J2 and J3 rotate. It is **normal** to J2's axis direction (and to J3's). The plane also contains J1's axis (vertical), and rotates about it as θ₁ changes. **θ₁ determines the arm plane's azimuth.**

## 4.2 The wrist root A

**A is the point where J4's axis meets the arm plane.** This is the end of the forearm.

The unified computation of A, valid for both offset and spherical wrists:

- **B** = J4's axis ∩ J5's axis.
- **A = B − dot(B − J2_origin, n̂) · n̂**, where `n̂` is the arm plane's normal (J2's axis direction).

For the offset wrist, B is at perpendicular distance `wrist_normal_offset` from the arm plane, and A is the projection of B onto the plane. For the spherical wrist, B is on the plane, and **A = B = P5**.

**No `intersect_plane` call, no branch.** The projection of B onto the plane is well-defined for every arm in the class.

## 4.3 `|AP5|` — the quantity that collapses

The distance from A to P5:

```
|AP5| = √(|AB|² + |BC|²) = √(wrist_normal_offset² + wrist_radial_offset²)
```

- For the offset wrist: A and P5 are distinct, and `|AP5|` is the length of the hypotenuse.
- For the spherical wrist: A = B = C, and `|AP5| = 0`.

`|AP5|` is **the quantity that collapses to zero for the spherical wrist**.

## 4.4 The spherical case

When `wrist_normal_offset = 0` and `wrist_radial_offset = 0`:

- A = B = C = P5.
- `|AP5| = 0`.
- The wrist triangle collapses to a point.

**The formulas of Parts 5–9 remain valid.** They degenerate to the spherical case as the geometry requires. **The spherical case is the value at zero, not a branch.**

## 4.5 The class condition for the arm chain

The rectangle argument for θ₁ (Part 5) requires the perpendicular direction between the arm plane and the J5-plane to be **horizontal** — so that the X-Y projection preserves `|AB|`.

- **When `wrist_normal_offset > 0`:** J4's axis must be horizontal.
- **When `wrist_normal_offset = 0`:** no condition.

This is checked at extraction time (Part 9, Step 0). For the three arms:

| Arm | `wrist_normal_offset` | J4 direction | Condition |
|---|---|---|---|
| UR10 | 0.163941 | `(0, −1, 0)` — horizontal | ✓ |
| FR5 | 0.102100 | `(0, −1, −4e-6)` — horizontal | ✓ |
| Elfin | 0 | `(0, 0, 1)` — vertical, but offset is zero | ✓ |

---

# Part 5 — Solving θ₁

## 5.1 What we need to find

θ₁ is the rotation of the base joint. It determines the **arm plane's azimuth** — the vertical plane through the base axis in which the upper arm and forearm move. Everything after θ₁ is expressed relative to this plane, so θ₁ must be solved first.

The geometry is entirely in the **X-Y projection**: the plane of the true-root frame's X and Y axes, looking down J1's axis. In this projection, the arm plane is a line through the origin, and θ₁ is the angle of that line.

## 5.2 The projection

Project three points onto the X-Y plane:

- **O′** — the base frame origin.
- **A′** — the wrist root A.
- **C′ = P5′** — the wrist center P5.

**C′ is known from the flange pose.** P5 is computed as `P5 = P_flange − flange_offset · a_flange`, and its X and Y are the projection C′. **A′ is not yet known** — it is on the arm-plane line, and its position follows once θ₁ is known.

The arm plane is vertical, so its projection is a **line through O′**. Its angle from the X axis is **θ₁**.

## 5.3 The rectangle

The rectangle has four corners:

- **O′** — the base origin.
- **F** — the foot of the perpendicular from C′ to the arm-plane line. F is on the arm-plane line.
- **C′ = P5′** — the wrist center's projection, on the far side of the rectangle.
- **D** — the fourth corner.

**The four sides:**

- **O′–F** — the long side, on the arm-plane line.
- **F–C′** — the short side, of length `|AB| = wrist_normal_offset`, perpendicular to the arm-plane line.
- **C′–D** — the other long side, parallel to O′–F.
- **D–O′** — the other short side.

**The diagonal is O′–C′.**

**A′ is on the long side O′–F, between O′ and F.** It is **not** a corner of the rectangle. **Its position along O′–F is determined by the wrist geometry — by θ₄ — and comes in a later part.**

## 5.4 The angle at O′

In the right triangle O′–F–C′, the angle at F is a right angle. **The angle at O′ between the long side O′–F and the diagonal O′–C′ is:**

```
sin(∠C′O′F) = |FC′| / |O′C′| = wrist_normal_offset / r
∠C′O′F = arcsin(wrist_normal_offset / r)
```

This is the angle between the diagonal and the arm-plane line.

## 5.5 The three constants of the projection

Three constants enter the formula:

**`α` — the diagonal's azimuth.** `α = atan2(y, x)`, where `(x, y)` are C′'s coordinates, and `r = √(x² + y²)` is the diagonal's length.

**`β` — the angle between the diagonal and the arm-plane line.** `β = arcsin(wrist_normal_offset / r)`.

**`φ₀` — the azimuth zero of the J1 frame.** The angle from the true-root X axis to the X axis of the J1 frame, projected onto the X-Y plane. **For UR10, FR5, and Elfin, `φ₀ = 0`.**

**`a1_sign` — the direction of J1's axis.** `+1` if J1's axis points up (positive Z), `−1` if down. **For UR10, FR5, and Elfin, `a1_sign = +1`.**

## 5.6 Finding θ₁

**θ₁ is the angle of the arm-plane line O′–F from the X axis.** It is the diagonal's azimuth adjusted by β:

```
θ₁ = a1_sign · (α − φ₀) ∓ β
```

with the `−` for the shoulder branch where the arm plane is on one side of the diagonal, the `+` for the other.

**The line-symmetry branch.** The arm-plane line is a **line**, not a ray. The rectangle can be traversed in either direction, giving two more solutions:

```
θ₁ = a1_sign · (α − φ₀) ∓ β + π
```

**So θ₁ has up to four solutions:** the two sides of the diagonal (`±`), each with the two directions of the line (`+π` or not).

## 5.7 The reachability condition

The rectangle exists only if the short side `wrist_normal_offset` fits within the diagonal `r`:

```
r ≥ wrist_normal_offset
```

If `r < wrist_normal_offset`, the arm plane cannot be placed — the target is outside the arm's reach. When `wrist_normal_offset = 0` (spherical wrist), the condition is `r ≥ 0`, always satisfied.

## 5.8 The spherical case

When `wrist_normal_offset = 0`, `β = 0`. The formula gives:

```
θ₁ ∈ { a1_sign · (α − φ₀),  a1_sign · (α − φ₀) + π }
```

The two shoulder solutions for the spherical wrist: the arm plane points toward C′ or away from it. **No branch — the general formula at `wrist_normal_offset = 0`.**

## 5.9 What θ₁ gives

**θ₁ fixes the arm-plane line O′–F.** With θ₁ known, the rectangle's corner F is determined, and **A′'s position along O′–F comes from the wrist geometry in Part 8** — it is not determined by θ₁ alone.

**A′'s Z — the vertical coordinate — is not in the rectangle.** The rectangle is the X-Y projection. A's Z also comes from the wrist geometry, in Parts 6–8.

---

# Part 6 — Solving θ₅

## 6.1 Why θ₅ comes first

The flange pose gives two points directly:

- **P6** — the flange position, from `T_flange`.
- **P5** — the wrist center, `P5 = P6 − flange_offset · ẑ_flange`.

**θ₅ depends only on these two points.** It is the tilt of the flange's Z axis out of the P5-plane, and that tilt is measured by the flange's lateral offset from the arm plane — the difference between P6's lateral distance and P5's. **No other joint angle enters.** θ₂ and θ₃ rotate within the arm plane and do not change lateral distances. θ₄ rotates about the arm-plane normal and does not change the distance along it. θ₆ rotates about the flange's own Z axis and does not change the flange's position at all. **So θ₅ is the first angle that can be solved, and it is solved from the pose alone.**

## 6.2 Definition

**θ₅ is the angle between J5's Z axis and J6's Z axis.**

J6's Z axis is the flange's Z axis — the axis along which `flange_offset` is measured. J5's Z axis is the axis about which J5 rotates. **θ₅ is the rotation of J5 about its own Z axis, which carries J6's Z axis around by that angle.**

At θ₅ = 0, J6's Z axis is parallel to J5's Z axis (the URDF's zero pose). At θ₅ = π/2, J6's Z is perpendicular to J5's Z. At θ₅ = π, J6's Z is anti-parallel.

## 6.3 J5's Z axis lies in the P5-plane

**J5's Z axis is perpendicular to J4's Z axis** (J4 ⊥ J5). **J4's Z axis is the arm-plane normal** (the arm-plane condition). **Therefore J5's Z axis lies in a plane perpendicular to the arm-plane normal — a plane parallel to the arm plane.**

**That plane passes through P5**, because J5's axis passes through P5. It is the **P5-plane**: the plane through P5 parallel to the arm plane.

**So J5's Z axis lies in the P5-plane.** Since J6's Z is the flange's Z, **θ₅ is the angle by which the flange's Z axis is tilted out of the P5-plane.**

## 6.4 The lateral offset `lat`

Define **`lat`** — the **lateral offset of the flange**:

> **`lat` is the signed distance from the arm plane to the flange position P6, measured along the arm-plane normal.**

The arm-plane normal is the unit vector:

```
n̂ = (−sin θ₁, cos θ₁, 0)
```

It is horizontal and perpendicular to the arm-plane line. **`lat` is the component of P6 along `n̂`:**

```
lat = dot(P6, n̂) = −P6_x · sin θ₁ + P6_y · cos θ₁
```

**`lat` is positive if the flange is on the `n̂` side of the arm plane, negative on the other side.** It is computed from P6's X and Y coordinates and θ₁. **It does not depend on Z — `n̂` is horizontal.**

## 6.5 The lateral offset equation

The flange's position P6 is related to the wrist center P5 by:

```
P6 = P5 + flange_offset · ẑ_flange
```

where `ẑ_flange` is the flange's Z axis (J6's Z axis).

**Project onto `n̂`:**

```
dot(P6, n̂) = dot(P5, n̂) + flange_offset · dot(ẑ_flange, n̂)
lat        = dot(P5, n̂) + flange_offset · cos(θ₅)
```

**The two terms:**

- **`dot(P5, n̂)`** — P5's lateral offset from the arm plane. **P5 is at perpendicular distance `wrist_normal_offset` from the arm plane** (B is at `wrist_normal_offset` from the arm plane, and P5 is in the P5-plane through B). So `dot(P5, n̂) = ±wrist_normal_offset`, with the sign from the shoulder branch.
- **`flange_offset · cos(θ₅)`** — the flange's lateral component. **Since `ẑ_flange` is J6's Z, and θ₅ is the angle between J5's Z and J6's Z, and J5's Z is in the P5-plane (perpendicular to `n̂`), `dot(ẑ_flange, n̂) = cos(θ₅)`.** The flange's Z axis has a component `cos(θ₅)` along the normal — full at θ₅ = 0, zero at θ₅ = π/2.

**So:**

```
lat = σ · wrist_normal_offset + flange_offset · cos(θ₅)
```

where `σ = +1` or `−1` is the shoulder branch's sign.

## 6.6 Solving for θ₅

```
cos(θ₅) = (lat − σ · wrist_normal_offset) / flange_offset
```

**Reachability:** `|lat − σ · wrist_normal_offset| ≤ flange_offset`.

**Two solutions:** `θ₅ = ± arccos(...)` — **no-flip** (`+`) and **flip** (`−`).

## 6.7 Why the other angles do not enter

**The lateral offset is measured along the arm-plane normal `n̂`.**

- **θ₂, θ₃** rotate the upper arm and forearm **within the arm plane**. They do not change any distance along `n̂`.
- **θ₄** rotates the wrist **about J4's axis**, which **is** the arm-plane normal. **A rotation about an axis does not change distances along that axis.**
- **θ₆** rotates the flange **about its own Z axis**, which does not move P6.

**So `lat` depends only on the flange pose and θ₁.** **That is why θ₅ can be solved before the arm chain.**

## 6.8 The spherical case

When the wrist is spherical, `wrist_normal_offset = 0`. **The equation becomes `lat = flange_offset · cos(θ₅)`.** Same formula, at `wrist_normal_offset = 0`.

## 6.9 The formula

```
lat = −P6_x · sin θ₁ + P6_y · cos θ₁
cos(θ₅) = (lat − σ · wrist_normal_offset) / flange_offset
if |cos(θ₅)| > 1: no solution on this shoulder branch
θ₅ ∈ { +arccos(...) (no-flip),  −arccos(...) (flip) }
```

with `σ` from the shoulder branch of θ₁, `wrist_normal_offset` from the extraction, and `flange_offset` from the extraction.

---

# Part 7 — Solving θ₂ and θ₃

## 7.1 Why the arm chain comes after θ₁ and θ₅

By Part 6, θ₅ is known. By Part 5, θ₁ is known — the arm plane is fixed. **The arm chain's job is to place the forearm's end at the target.**

**The arm chain moves within the arm plane — the vertical plane at azimuth θ₁ — and it reaches a point in that plane.**

**P5 is not on the arm plane.** It is at perpendicular distance `wrist_normal_offset` from the arm plane. **The arm chain cannot reach P5 directly.**

**The arm chain reaches a point in the arm plane, and the wrist bridges the gap to P5.** What the arm chain reaches, in the arm plane, is **P5's projection onto the arm plane** — the foot of the perpendicular from P5 to the arm plane. Call it **Q**.

**Q is computable from P5 and θ₁**, and it does not depend on θ₂, θ₃, θ₄. **So the arm chain targets Q.**

## 7.2 The target Q

**Q is the projection of P5 onto the arm plane, along the arm-plane normal.** The arm-plane normal is `n̂ = (−sin θ₁, cos θ₁, 0)`. **P5 is at distance `wrist_normal_offset` from the arm plane, along `n̂`** (with the shoulder-branch sign `σ`). **So:**

```
Q = P5 − σ · wrist_normal_offset · n̂
```

**Q is on the arm plane.** Its coordinates in the X-Y projection are P5's X and Y adjusted by the offset; its Z is P5's Z.

**Q is computed from P5, θ₁, `wrist_normal_offset`, and `σ`.** All known after Parts 5 and 6.

## 7.3 The arm plane's coordinates

The arm plane is vertical. It has two in-plane coordinates:

- **`u`** — the horizontal direction along the arm-plane line. Unit vector `ê_u = (cos θ₁, sin θ₁, 0)`.
- **`w`** — the vertical direction. Unit vector `ê_w = (0, 0, 1)`.

**The shoulder is at J2's origin.** Its coordinates in the arm plane:

```
u_shoulder = dot(J2_origin − O, ê_u)
w_shoulder = dot(J2_origin − O, ê_w)
```

where `O` is the base origin. For UR10, J2's origin is at `(0, 0, 0.1273)`, and `u_shoulder = 0`, `w_shoulder = 0.1273`.

**The target Q's coordinates in the arm plane:**

```
u_target = dot(Q − O, ê_u)
w_target = dot(Q − O, ê_w)
```

**The vector from the shoulder to the target:**

```
Δu = u_target − u_shoulder
Δw = w_target − w_shoulder
```

**The distance:**

```
D = √(Δu² + Δw²)
```

## 7.4 The two lengths

The arm chain has two lengths:

- **`axis_gap(2)`** — the upper arm. The perpendicular distance between J2's axis and J3's axis.
- **`forearm_length()`** — the forearm. **The distance from J3's axis to Q.** **Not** `axis_gap(3)`, and **not** `elbow_to_wrist` (to A). **The arm chain reaches Q, so the forearm's length is the distance from J3's axis to Q.**

**`forearm_length()` is `|Q − J3_origin|`.** It is computed from Q and J3's origin.

## 7.5 Law of cosines for θ₃

The triangle has sides `axis_gap(2)`, `forearm_length()`, and `D`. The law of cosines:

```
cos(θ₃) = (D² − axis_gap(2)² − forearm_length()²) / (2 · axis_gap(2) · forearm_length())
```

**Reachability:** `|axis_gap(2) − forearm_length()| ≤ D ≤ axis_gap(2) + forearm_length()`. If not, no solution.

**Two solutions:** `θ₃ = ± arccos(cos θ₃)`. The `+` is **elbow-up**; the `−` is **elbow-down**.

## 7.6 Law of sines for θ₂

**The angle of the shoulder-to-target line**, measured from `ê_u`:

```
ψ = atan2(Δw, Δu)
```

**The angle between the upper arm and the shoulder-to-target line:**

```
β = arcsin(forearm_length() · sin(θ₃) / D)
```

**Then:**

```
θ₂ = ψ − β · sign(θ₃)
```

**The exact form depends on how `ê_u` and `ê_w` are oriented and how θ₂ is defined relative to the J2 frame.** **The code's convention is authoritative** — for Elfin, where J2 and J3 are anti-parallel (`axis_twist(2) = π` in 1-based indexing), the sign of θ₂ must respect the antiparallel relationship; the code handles this from the axis directions directly.

## 7.7 The spherical case

For the spherical wrist, `wrist_normal_offset = 0`. **P5 is on the arm plane**, so **Q = P5**. The arm chain targets P5 directly, and `forearm_length()` is the distance from J3's axis to P5. **Same formulas, with Q = P5.** No branch — the general formula at `wrist_normal_offset = 0`.

## 7.8 What θ₂ and θ₃ give

**θ₂ and θ₃ place the forearm's end at Q — P5's projection onto the arm plane.** The arm chain has done its job: the forearm's end is at the point in the arm plane closest to P5. **The wrist bridges the gap from Q to P5, and that bridge is the wrist's offset.**

**The next step is the wrist's orientation.** With θ₁, θ₂, θ₃, θ₅ known, and the flange pose, **θ₄ and θ₆ follow** from the wrist inversion. Part 8.

## 7.9 The formula

```
θ₁ from Part 5; θ₅ from Part 6; σ from the shoulder branch.
P5 = P6 − flange_offset · ẑ_flange
n̂ = (−sin θ₁, cos θ₁, 0)
Q = P5 − σ · wrist_normal_offset · n̂

ê_u = (cos θ₁, sin θ₁, 0);  ê_w = (0, 0, 1)

Δu = dot(Q − J2_origin, ê_u)
Δw = dot(Q − J2_origin, ê_w)
D = √(Δu² + Δw²)

if not (|axis_gap(2) − forearm_length()| ≤ D ≤ axis_gap(2) + forearm_length()):
    no solution

cos(θ₃) = (D² − axis_gap(2)² − forearm_length()²) / (2 · axis_gap(2) · forearm_length())
θ₃ ∈ { +arccos(...) (elbow-up),  −arccos(...) (elbow-down) }

ψ = atan2(Δw, Δu)
β = arcsin(forearm_length() · sin θ₃ / D)
θ₂ = ψ − β · sign(θ₃)
```

with `axis_gap(2)` = upper arm, `forearm_length()` = `|Q − J3_origin|`.

---

# Part 8 — Solving θ₄ and θ₆

## 8.1 What is known, and what remains

By Parts 5–7:

- **θ₁** — the arm plane's azimuth, from the rectangle.
- **θ₅** — the flange's tilt, from the lateral offset.
- **θ₂, θ₃** — the arm chain, targeting Q.

**The flange pose is given.** The arm chain has placed the forearm, so **J4's frame in the base frame is known** — it is `R₀₃ · R_o4`, where `R₀₃ = R₁(θ₁) · R₂(θ₂) · R₃(θ₃)` and `R_o4` is the rotation part of J4's origin transform.

**Two angles remain: θ₄ and θ₆.** This part finds them **geometrically** — by projecting the flange's axes onto the planes perpendicular to J5's and J4's axes, and reading the angles. **No Paden-Kahan, no quaternions, no numerical inversion.**

## 8.2 θ₆ — the polar angle of the flange's Y axis around J5's axis

**J5's axis is the axis about which θ₅ rotates.** The flange's Y axis — call it `Y_flange`, the second column of `R_flange` — is carried around J5's axis by θ₆.

**Project `Y_flange` onto the plane perpendicular to J5's axis.** Its polar angle in that plane, measured from a reference direction, is **θ₆** — up to the reference and the sign, which come from the frame conventions.

**Concretely:** let `a₅` be J5's axis (unit vector, in the base frame). **The component of `Y_flange` perpendicular to `a₅` is:**

```
Y_perp = Y_flange − dot(Y_flange, a₅) · a₅
```

**Choose two orthonormal vectors `e₁`, `e₂` spanning the plane perpendicular to `a₅`.** **The natural frame is the frame after θ₅ — the frame whose Z axis is J5's axis.** **And that frame's X axis is the reference for θ₆.**

**In `ArmGeometry` terms:** `e₁` and `e₂` are the X and Y axes of J5's frame — **which are computable from the origin transforms and the arm chain's placement.** **The formula becomes:**

```
θ₆ = atan2( dot(Y_flange, e₂), dot(Y_flange, e₁) )
```

where `e₁`, `e₂` are J5's frame's X and Y axes in the base frame, and `Y_flange` is the flange's Y axis.

**The singularity:** when `sin θ₅ = 0`, J5's axis and J6's axis are aligned (or anti-aligned), and `Y_perp` degenerates — θ₆ is indeterminate. **The solver sets θ₆ to an arbitrary value** (often 0, or the current value for continuity), and the flange reaches the correct orientation regardless.

## 8.3 θ₄ — the roll about J4's axis

**θ₄ rotates J4's frame about J4's axis.** **Once θ₆ is known, θ₄ is the roll that aligns J4's frame's X axis with the direction that produces the flange's orientation.**

**Concretely:** compute the flange's orientation with θ₄ = 0 — call it `R_flange(θ₄ = 0)`. **The actual flange orientation is `R_flange`, and the difference is a rotation about J4's axis by θ₄.** **So:**

```
R_diff = R_flange · R_flange(θ₄ = 0)ᵀ
```

**`R_diff` should be a rotation about J4's axis by θ₄.** **Extract θ₄ from it:**

```
θ₄ = atan2( dot(R_diff · J4_frame_X, J4_frame_Y),
            dot(R_diff · J4_frame_X, J4_frame_X) )
```

where `J4_frame_X` and `J4_frame_Y` are J4's frame's X and Y axes (in the base frame, with θ₄ = 0), and `R_diff · J4_frame_X` is the rotated X axis.

**Geometrically, θ₄ is the roll of J4's frame about J4's axis.** **It is measured by how far J4's X axis has rotated.**

**The singularity:** when `sin θ₅ = 0`, θ₄ and θ₆ are coupled (only their sum or difference is determined). **The solver handles this by setting one of them to a convenient value** (as in §8.2).

## 8.4 The spherical case — no branch

**The wrist inversion uses only:**

- The flange's orientation.
- θ₁, θ₅.
- The wrist geometry (J4, J5, J6 origins and axes).
- θ₂, θ₃ (to place J4's frame).

**It does not use `wrist_normal_offset`.** The wrist inversion is about the flange's **orientation**, which is determined by the wrist geometry and the arm chain's placement — not by where P5 sits relative to the arm plane.

**So the spherical wrist and the offset wrist are solved identically.** **The spherical case is the limit `wrist_normal_offset = 0` in the arm chain (Q = P5) and the lateral offset (`σ · wrist_normal_offset` term vanishes) — but the wrist inversion itself does not change.**

## 8.5 The formula

```
# θ₆ — the polar angle of the flange's Y axis around J5's axis
a₅ = J5's axis (from the extraction)
e₁, e₂ = J5's frame's X, Y axes in the base frame (from the origins and arm chain)
Y_flange = R_flange[:, 1]
Y_perp = Y_flange − dot(Y_flange, a₅) · a₅

if |sin θ₅| > tol:
    θ₆ = atan2( dot(Y_perp, e₂), dot(Y_perp, e₁) )
else:
    θ₆ = current θ₆ (or 0)   # singularity: arbitrary; the flange orientation is still correct

# θ₄ — the roll about J4's axis
R_flange_θ₄_0 = (the flange orientation with θ₄ = 0, computed from θ₁, θ₂, θ₃, θ₅, θ₆)
R_diff = R_flange · R_flange_θ₄_0ᵀ
J4_X, J4_Y = J4's frame's X, Y axes (in the base frame)

if |sin θ₅| > tol:
    θ₄ = atan2( dot(R_diff · J4_X, J4_Y), dot(R_diff · J4_X, J4_X) )
else:
    θ₄ = current θ₄ (or 0)   # singularity: arbitrary; the flange orientation is still correct
```

## 8.6 Why this is geometric, and why it works for both wrists

**θ₆** — the polar angle of the flange's Y axis around J5's axis. **A projection and an atan2.**

**θ₄** — the roll of J4's frame about J4's axis, measured once θ₆ is known. **A projection and an atan2.**

**The method works for both wrists** because the wrist's rotation geometry is the same — only the arm chain's target Q and the lateral offset's `σ · wrist_normal_offset` term differ.

**This is the unification, stated precisely:** **the wrist is one geometry; the arm chain is where the offset enters; and the spherical case falls out at `wrist_normal_offset = 0`.**

## 8.7 The singularities

**At `sin θ₅ = 0`** (θ₅ = 0 or π): J5's axis and J6's axis are aligned, and θ₄, θ₆ are not independent — only their sum (θ₅ = 0) or difference (θ₅ = π) is determined. **The solver sets one of them to a convenient value** (the current value, for continuity), and solves the other. **The choice is arbitrary; the flange reaches the correct orientation regardless, but the selected θ₄ or θ₆ may not match the robot's previous state.**

**This is a known and unavoidable singularity** — it is the wrist singularity that every 6R arm with a spherical (or offset) wrist has. **The solver handles it explicitly, with a documented convention, not silently.**

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

**Step 2 — Solve θ₁ (Part 5):**

```
α = atan2(P5_y, P5_x);  r = √(P5_x² + P5_y²)
β = arcsin(wrist_normal_offset / r)      (if r ≥ wrist_normal_offset)
θ₁ ∈ { a1_sign·(α − φ₀) ± β,  a1_sign·(α − φ₀) ± β + π }
```

Each θ₁ carries a shoulder sign `σ = +1` (right) or `σ = −1` (left), and a line-symmetry branch.

**Step 3 — Solve θ₅ (Part 6):** For each θ₁ branch:

```
lat = −P6_x · sin θ₁ + P6_y · cos θ₁
cos(θ₅) = (lat − σ · wrist_normal_offset) / flange_offset
if |cos(θ₅)| > 1: no solution on this branch
θ₅ ∈ { +arccos (no-flip),  −arccos (flip) }
```

**Step 4 — Solve θ₂, θ₃ (Part 7):** For each (θ₁, θ₅):

```
Q = P5 − σ · wrist_normal_offset · n̂        (n̂ = (−sin θ₁, cos θ₁, 0))
Δu = dot(Q − J2_origin, ê_u)               (ê_u = (cos θ₁, sin θ₁, 0))
Δw = dot(Q − J2_origin, ê_w)               (ê_w = (0, 0, 1))
D = √(Δu² + Δw²)

if not (|axis_gap(2) − forearm_length()| ≤ D ≤ axis_gap(2) + forearm_length()):
    no solution on this branch
cos(θ₃) = (D² − axis_gap(2)² − forearm_length()²) / (2 · axis_gap(2) · forearm_length())
θ₃ ∈ { +arccos (elbow-up),  −arccos (elbow-down) }

ψ = atan2(Δw, Δu)
β = arcsin(forearm_length() · sin θ₃ / D)
θ₂ = ψ − β · sign(θ₃)
```

where `forearm_length() = |Q − J3_origin|`.

**Step 5 — Solve θ₆ (Part 8):** For each (θ₁, θ₂, θ₃, θ₅):

```
a₅ = J5's axis (from the extraction and the arm chain's placement)
e₁, e₂ = J5's frame's X, Y axes in the base frame
Y_flange = T_flange[:3, 1]
Y_perp = Y_flange − dot(Y_flange, a₅) · a₅

if |sin θ₅| > tol:
    θ₆ = atan2(dot(Y_perp, e₂), dot(Y_perp, e₁))
else:
    θ₆ = current θ₆ (or 0)   # singularity
```

**Step 6 — Solve θ₄ (Part 8):**

```
R_flange_θ₄_0 = FK with θ₁, θ₂, θ₃, θ₅, θ₆ known, θ₄ = 0
R_diff = R_flange · R_flange_θ₄_0ᵀ
J4_X, J4_Y = J4's frame's X, Y axes in the base frame

if |sin θ₅| > tol:
    θ₄ = atan2(dot(R_diff·J4_X, J4_Y), dot(R_diff·J4_X, J4_X))
else:
    θ₄ = current θ₄ (or 0)   # singularity
```

**Step 7 — Assemble and tag.**

| Level | Tag values |
|---|---|
| Shoulder | `left`, `right` |
| Elbow | `up`, `down` |
| Wrist | `flip`, `no-flip` |

Up to 8 solutions total.

**Step 8 — Filter by joint limits**, modulo 2π:

```
for each joint i:
    if not (lower_i ≤ θ_i ≤ upper_i):
        θ_i += 2π (or − 2π)
        if not (lower_i ≤ θ_i ≤ upper_i): reject
```

**Step 9 — Return** all valid, tagged solutions. The caller selects by continuity.

## 9.2 Worked example: UR10 (offset wrist)

**Parameters** (from the extraction):

| Quantity | Value |
|---|---|
| `wrist_normal_offset` (`\|AB\|`) | 0.163941 |
| `wrist_radial_offset` (`\|BC\|`) | 0.115700 |
| `wrist_offset` (`\|AC\|`) | 0.200657 |
| `flange_offset` | 0.092200 |
| `axis_gap(2)` (upper arm) | 0.612000 |
| `forearm_length()` (J3→Q) | 0.572300 |
| J4 axis direction | `(0, −1, 0)` — horizontal ✓ |

**Target** (in true-root frame): `P_flange = (0.5, 0.3, 0.8)`, `R_flange = identity`.

**Step 1 — P5:** `P5 = (0.5, 0.3, 0.8 − 0.0922) = (0.5, 0.3, 0.7078)`.

**Step 2 — θ₁:** `α = atan2(0.3, 0.5) = 0.5404`, `r = 0.5831`, `β = arcsin(0.163941 / 0.5831) = 0.2852`. So `θ₁_right ≈ 0.2552`, `θ₁_left ≈ 0.8256`, plus the `+π` branches.

**Step 3 — θ₅:** For the right-shoulder branch: `lat = −0.5·sin(0.2552) + 0.3·cos(0.2552) ≈ 0.1640`, `cos(θ₅) = (0.1640 − 0.163941)/0.0922 ≈ 0.0006`, `θ₅ ≈ ±1.5702` (flip and no-flip).

**Step 4 — θ₂, θ₃:** Compute Q, then `Δu`, `Δw`, `D`, the law of cosines, and the law of sines. The test suite (`test_arm_chain.py`) is the authoritative round-trip check.

**Step 5–6 — θ₆, θ₄:** The geometric wrist inversion, from the flange orientation, given θ₁, θ₂, θ₃, θ₅.

**Steps 7–9:** Assemble, tag, filter by limits, return.

**What the example shows:** every step uses the settled quantities, the ordering is the paper's (θ₁ → θ₅ → arm chain → wrist), and the wrist inversion is geometric.

## 9.3 Worked example: FR5 (offset wrist)

**Parameters:**

| Quantity | Value |
|---|---|
| `wrist_normal_offset` | 0.102100 |
| `wrist_radial_offset` | 0.102000 |
| `wrist_offset` | 0.144321 |
| `flange_offset` | 0.100000 |
| `axis_gap(2)` | 0.425000 |
| `forearm_length()` | 0.395010 |
| J4 axis direction | `(0, −1, −4e-6)` — horizontal ✓ |

**Same target, same algorithm.** The values differ; the steps are identical. The wrist inversion is the same geometric operation.

**Note on the origin trap:** for FR5, `axis_shift(3)` is 0.0 and `wrist_normal_offset` is 0.1021 — the algorithm uses `wrist_normal_offset`, so it is immune to the origin placement.

## 9.4 Worked example: Elfin (spherical wrist)

**Parameters:**

| Quantity | Value |
|---|---|
| `wrist_normal_offset` | 0 |
| `wrist_radial_offset` | 0 |
| `wrist_offset` | 0 |
| `flange_offset` | 0.200000 |
| `axis_gap(2)` | 0.730000 |
| `forearm_length()` (J3→P5) | 0.570000 |
| J4 axis direction | `(0, 0, 1)` — vertical, but offset is zero ✓ |
| `axis_twist(2)` (J2–J3, 1-based) | π (anti-parallel) |

**Same target, same algorithm.**

**Step 2 — θ₁:** `β = arcsin(0/r) = 0`, so `θ₁ ∈ { α, α + π }`.

**Step 3 — θ₅:** `cos(θ₅) = lat / flange_offset` (since `wrist_normal_offset = 0`). Two solutions.

**Step 4 — θ₂, θ₃:** Q = P5 (since `wrist_normal_offset = 0`), so `forearm_length() = |P5 − J3_origin| = 0.570000`. Same law of cosines.

**Step 5–6 — θ₆, θ₄:** Same wrist inversion. **The wrist inversion does not use `wrist_normal_offset` — so it is identical to the offset case.**

**What the example shows:** the spherical wrist is the **limit** of the general algorithm. Every step is the same; `wrist_normal_offset = 0` makes Q = P5 and removes the `σ·wrist_normal_offset` term from the lateral offset. **The wrist inversion is unchanged.**

## 9.5 The reversal, stated

The conventional derivation treats the **spherical wrist** as the natural case and the **offset wrist** as a complication. **This document reverses that.** The general case is the **offset wrist** — the wrist center is off the arm plane, the arm chain targets Q, and the lateral offset has the `σ·wrist_normal_offset` term. The **spherical wrist** is the case where `wrist_normal_offset = 0` — the arm chain targets P5, and the lateral offset loses its wrist term. **The wrist inversion is the same geometric operation for both.**

**The algorithm is one.** Every step uses the settled quantities, and the spherical case falls out at `wrist_normal_offset = 0`. No branch, no special case — the general formula degenerates to the spherical formulas as the geometry requires.

## 9.6 The full formula set

```
# Step 1
P5 = P_flange − flange_offset · a_flange

# Step 2
α = atan2(P5_y, P5_x);  r = √(P5_x² + P5_y²)
β = arcsin(wrist_normal_offset / r)      (if r ≥ wrist_normal_offset)
θ₁ ∈ { a1_sign·(α − φ₀) ± β,  a1_sign·(α − φ₀) ± β + π }

# Step 3
lat = −P6_x·sin θ₁ + P6_y·cos θ₁
cos(θ₅) = (lat − σ·wrist_normal_offset) / flange_offset
θ₅ ∈ { +arccos (no-flip),  −arccos (flip) }

# Step 4
Q = P5 − σ·wrist_normal_offset·n̂
Δu = dot(Q − J2_origin, ê_u);  Δw = dot(Q − J2_origin, ê_w)
D = √(Δu² + Δw²)
cos(θ₃) = (D² − axis_gap(2)² − forearm_length()²) / (2·axis_gap(2)·forearm_length())
θ₃ ∈ { +arccos (elbow-up),  −arccos (elbow-down) }
ψ = atan2(Δw, Δu);  β₂ = arcsin(forearm_length()·sin θ₃ / D)
θ₂ = ψ − β₂·sign(θ₃)

# Step 5–6 — wrist inversion (Part 8)
θ₆ = polar angle of Y_flange around J5's axis (if sin θ₅ ≠ 0)
θ₄ = roll about J4's axis (if sin θ₅ ≠ 0)

# Steps 7–9
Tag (shoulder, elbow, wrist), filter by joint limits modulo 2π, return all valid.
```

**No DH symbols anywhere.** `wrist_normal_offset`, `axis_gap(2)`, `forearm_length()`, `flange_offset` — all geometric, all named for what they are.

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
2. **URDF gives more information.** A URDF joint origin is a full 6-DOF transform.
3. **DH requires a convention choice the solver does not need.** Hatch works directly with the joint axes as lines in space.

## A.4 The joint-4 offset: DH `d₄` vs. `wrist_normal_offset`

In the standard UR DH table, `d₄` is labelled "the wrist offset." For UR10, `d₄ = 0.163941 m`. The extraction reports `wrist_normal_offset = 0.163941 m`. **They are numerically equal** — but for a frame-placement reason, not a geometric one.

The geometric quantity is the **perpendicular distance from the arm plane to J5's axis**. For these arms, DH's frame placement happens to make `d₄` equal to it. The solver uses `wrist_normal_offset`, computed from the axes as lines. The equality is a coincidence of frame placement, not an identity.

**`axis_shift` is not `wrist_normal_offset` either — at any index.** The `axis_shift(i)` quantity is the signed distance along joint `i`'s axis between the feet of the common perpendiculars with joints `i-1` and `i+1`. It is origin-dependent. `wrist_normal_offset`, by contrast, is computed from axis intersections and is origin-independent.

| Arm | `wrist_normal_offset` | `axis_shift(3)` (along J3) | `axis_shift(4)` (along J4) |
|---|---|---|---|
| UR10 | 0.163941 | 0.163941 | 0.000000 |
| FR5 | 0.102100 | 0.000000 | 0.102100 |

**The algorithm uses `wrist_normal_offset`, never `axis_shift(i)`** — which is why it is immune to the origin placement.

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
5. **The arm chain is in the arm plane** — J2 and J3 are parallel in the general sense (same or opposite direction).

Robots in this class: UR10, UR5, FR5, Elfin E15 Pro, and most industrial 6-axis arms. The class covers both spherical wrists (`wrist_normal_offset = 0`) and offset wrists (`wrist_normal_offset ≠ 0`).

## B.2 The refusal policy

If a robot violates any condition, Hatch **refuses loudly**. It does not approximate the geometry to fit the solver, fall back to a numerical solver silently, or return a solution that is "close" but not exact.

The refusal is a `RuntimeError` with a message that states the violation and the measured value.

## B.3 Why refuse instead of approximate

**Because a plausible-looking wrong answer is worse than no answer.** If the solver approximated a non-class robot, it would produce joint angles that place the flange near the target but not exactly at it. The error might be small for some poses and large for others. The caller would have no way to know which. In a safety-critical context, that is unacceptable.

## B.4 What is not in the class

- **Robots with skew J2 and J3.** Some painting and welding robots use non-parallel shoulder-elbow axes.
- **Robots with non-intersecting J5 and J6.** Some collaborative robots and some 7-DOF arms.
- **7-DOF arms.** Redundant arms have more than 6 joints.
- **Parallel robots, delta robots, SCARA arms.** Different kinematic structures entirely.

For all of these, Hatch refuses. The word is **unified**, not **universal**. Unified across the supported class — spherical and offset wrists, UR and FR and Elfin — not universal across all robots.

---

# Appendix C — Code Obligations

The extraction and solver code on disk does not yet match Parts 5–9. The gaps, in the order they should be closed.

## C.1 The extraction (`core/kinematics/geometric_extraction.py`)

**Already committed and correct:**

- 1-based indexing for `axis_gap`, `axis_twist`, `axis_shift`.
- `wrist_normal_offset`, `wrist_radial_offset`, `wrist_offset` — the three triangle quantities.
- `wrist_root()` — A = B projected onto the arm plane.
- `elbow_to_wrist()` — `|A − J3_origin|`.
- The conditional class check (J3 ∥ J4 only when `wrist_normal_offset > 0`).
- The `describe()` output.

**Still to do:**

1. **`forearm_length()` — restore it, targeting Q.** The arm chain's forearm is the distance from J3's axis to **Q**, not to P5 and not to A. **Q is P5's projection onto the arm plane** — the arm chain's target (Part 7). The extraction's `forearm_length()` should return `|Q − J3_origin|`, computed for a given Q. Since Q depends on the target pose (not on the fixed geometry), `forearm_length()` may need to take Q as an argument, or the solver computes it directly. **The extraction's `elbow_to_wrist()` (to A) is a different quantity — it is not the arm chain's forearm.**

2. **`describe()` should print both `forearm_length()` (when Q is given) and `elbow_to_wrist()`,** so the distinction is visible.

## C.2 The solver (`core/kinematics/solver_v2.py`)

**The file must be rewritten to follow Parts 5–9. The sequence:**

1. **P5 = P_flange − flange_offset · a_flange.**
2. **θ₁** from the rectangle (Part 5): `α = atan2(P5_y, P5_x)`, `β = arcsin(wrist_normal_offset / r)`, `θ₁ = a1_sign · (α − φ₀) ± β`, with the `+π` branches.
3. **θ₅** from the lateral offset (Part 6): `lat = −P6_x · sin θ₁ + P6_y · cos θ₁`, `cos(θ₅) = (lat − σ · wrist_normal_offset) / flange_offset`.
4. **θ₂, θ₃** from the arm chain targeting Q (Part 7): `Q = P5 − σ · wrist_normal_offset · n̂`, the law of cosines with `axis_gap(2)` and `forearm_length()`, the law of sines.
5. **θ₆** from the polar angle of the flange's Y axis around J5's axis (Part 8, §8.2).
6. **θ₄** from the roll about J4's axis (Part 8, §8.3).
7. **Tag** (shoulder, elbow, wrist), filter by joint limits, return all valid.
8. **Handle the singularities** (`sin θ₅ = 0`) with the documented convention.

**The key changes from the current solver:**

- **θ₂, θ₃ target Q, not A.** The solver's `_solve_theta2_theta3` must use Q, not `wrist_root()`.
- **θ₅ uses `lat` and `n̂`.** The old formula had the arm-plane normal and the flange offset combined — the new formula separates them.
- **θ₆ and θ₄ are added** (Part 8) — they are not in the current solver.

## C.3 The test (`test_arm_chain.py`)

**The test's `q_check` must reflect the paper's ordering.** In the paper's ordering, θ₁, θ₅, θ₂, θ₃, θ₆, θ₄ are solved in that order. **The test should check:**

1. **The arm chain** — `q_check = [θ₁, θ₂, θ₃, θ_true[3], θ₅, θ_true[5]]` — the arm chain's job is to place the forearm's end at Q, so **the test should check that the forearm's end is at Q**, not that P5 is at the truth's P5 (which would require the wrist).

2. **The wrist** — `q_check = [θ₁, θ₂, θ₃, θ₄, θ₅, θ₆]` from the solver — and check that the flange pose from `compute_fk(q_check)` matches the target `T_flange`.

**The test's `_extract_p5_from_fk` helper** — it computes P5 from the FK. It is still useful for checking the arm chain's placement of the forearm's end (which is Q, not P5). **The helper should be renamed to `_extract_Q_from_fk` and compute the forearm's end, not P5.** Or kept as `_extract_p5_from_fk` for the wrist check (comparing the solver's P5 to the truth's P5), and a separate `_extract_Q_from_fk` for the arm chain check.

## C.4 The document's Parts 1–4 and Appendices A–B

**These are correct and should not change.** The five quantities, the triangle, the naming, the class check, the DH comparison — all stand.

**Appendix A.4 was already corrected in the extraction commit** (the `axis_shift` indices). **Appendix A.1–A.3, A.5, and Appendix B are correct.**

## C.5 The commit sequence

1. **`core: restore forearm_length targeting Q, and update describe`** — the extraction's forearm length.
2. **`core: rewrite solver_v2 with Parts 5–9 ordering`** — the solver, following the paper's sequence.
3. **`test: arm chain targets Q, wrist check matches flange pose`** — the test, reflecting the new ordering.
4. **`docs: correct Appendix C`** — this appendix, after the code is done.

**Each commit follows the Constitution Part 5 format** — `<scope>: <one-line summary>`, with the body naming principles, docs, and smoke test.

## C.6 The verification

**After the code is corrected:**

1. **`check_extraction.py`** — passes for UR10, FR5, Elfin, with the new `forearm_length()` (to Q) and `elbow_to_wrist()` (to A) both printed.
2. **`test_arm_chain.py`** — passes for all three arms: the arm chain places the forearm's end at Q, and the wrist matches the flange pose.
3. **The document's worked examples** — the UR10 example's numbers (θ₁, θ₅, θ₂, θ₃, θ₄, θ₆) should match the test's output for the example target.

**The test is the acceptance criterion.** **The document is the specification; the code follows it; the test proves the code follows it.**

---

*Hatch (孵) 🐣 — built to understand, built to see.*
*The offset wrist is the general case; the spherical wrist is the value at zero.*
*No DH symbols. No branches. The geometry is what it is.*

