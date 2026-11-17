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

**The true kinematic root** is the **parent link of the first movable revolute joint**. For UR10, that is **`base_link_inertia`**, not `base_link`. Using the wrong root produces wrong kinematics.

The link's exact name varies between URDF releases — the file in `assets/robots/ur_description/urdf/ur10_nominal.urdf` names it `base_link_inertia`, while older documentation calls it `base_inertia`. **Detect the root structurally, by walking the tree; never match on a link name.**

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

**The document derives the general case — the offset wrist — and lets the spherical wrist fall out at offset zero.** The conventional derivation does the reverse; this document reverses it (Part 9, §9.8).

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

This is the quantity that governs θ₁ (Part 5, where it appears as the signed `h`) and the ρ term of the wrist root A (Part 7). It is computed from the axes as lines, not from the joint origins:

- **B** = J4's axis ∩ J5's axis.
- **`wrist_normal_offset` = |dot(B − J2's origin, n̂)|**, where `n̂` is J2's axis direction (normal to the arm plane).

**Why it is origin-independent.** B is an axis intersection — an invariant point. J2's origin is on the arm plane. The perpendicular distance from B to the plane is a geometric fact, independent of where any joint origin sits along its axis. This matters: UR and FR place J4's origin at opposite ends of the A–B segment, so any origin-derived computation would disagree between the two arms. The line-based computation does not.

## 3.5 The right triangle at B

The fifth quantity is one side of the **right triangle A–B–C**, right-angled at B. The three sides are the three wrist quantities:

| Symbol | Name | What it is | Role |
|---|---|---|---|
| `\|AB\|` | **`wrist_normal_offset`** (ρ) | Perpendicular distance from the arm plane to J5's axis. AB is normal to the arm plane. | **θ₁ (§5.4); the ρ term of A (§7.2); radius of the wrist circle (§7.4)** |
| `\|BC\|` | **`wrist_radial_offset`** (δ) | Perpendicular distance from C to J4's axis. BC is parallel to the arm plane, in the plane through B at distance `\|AB\|` from the arm plane. | **the δ term of A (§7.2) — the term that forces the wrist to be solved before the arm chain (§7.6)** |
| `\|AC\|` | **`wrist_offset`** | The hypotenuse: `√(\|AB\|² + \|BC\|²)`. Distance from wrist root A to wrist center C. | the slant distance from P5 to the wrist circle (§7.4). **Not** the radius of the θ₁ circle, and **not** the quantity that decides the ordering |

**The solver's scalar inputs are ρ, δ, `flange_offset`, `axis_gap(2)`, `elbow_to_wrist`, and the signed `h` (§5.2).** An earlier version of this document demoted `|BC|` to a derived quantity "not used as a scalar by the solver." That was wrong: **δ is the bridge from P5 to A**, and without it the arm-chain target is misplaced by δ — 115.7 mm on UR10, 102.0 mm on FR5 (§7.8). Only `|AC|` is genuinely unused as a scalar; it is stated because it makes the cone of §7.4 visible.

**All three are zero for a spherical wrist.** They are distinct quantities, and the conventional term "wrist offset" is ambiguous — it is used sometimes for `|BC|` and sometimes for `|AC|`. This document names all three explicitly.

## 3.6 The derived lengths

Two lengths the arm chain uses are not among the five quantities:

**`elbow_to_wrist`** — the distance from J3's axis to **A**, the wrist root. This is the forearm in the law of cosines, and it is a **constant of the machine**: 0.572300 (UR10), 0.395010 (FR5), 0.570000 (Elfin). For UR10 and FR5 it equals `axis_gap(3)`, because J4's origin sits at A for those arms.

**`flange_offset`** — the **signed** distance from P5 to the flange along â₆. This is `d₆` in the conventional notation. Measured against `KinematicModel.get_flange()`: **+0.092200 (UR10)**, +0.100000 (FR5), +0.200000 (Elfin).

**The sign is a property of the flange *link choice*, not of the arm.** An earlier measurement recorded −0.092200 for UR10 because it walked to the end of the fixed-joint chain (`tool0`), whose Z is flipped relative to `wrist_3_link`. Both are correct for their own link. Therefore `d₆` must be **computed per model**, as `dot(P_flange − P5, â₆)`, never taken from a table and never assumed positive. Note also that `ArmGeometry.flange_offset` is an *unsigned* norm and cannot supply this sign.

The law of cosines uses `axis_gap(2)` (the upper arm, J2 to J3) and `elbow_to_wrist` (the forearm, J3 to A). Both are constants, which is what makes the law of cosines legal.

**There is no `forearm_length()`.** An earlier version defined it as `|Q − J3_origin|`, where Q is P5's projection onto the arm plane. That quantity is not constant — Q orbits A as θ₄ varies — and it cannot be computed in advance anyway, since J3's origin depends on θ₂, the very unknown being solved for. The arm chain targets **A**, not Q (§7.8, §8.1).

## 3.7 What the five quantities describe — and what they do not

The five quantities describe the **class of arms this document covers**: arm-plane 6R arms with intersecting wrist axes. They plus the URDF's joint origin transforms are a complete kinematic description of that class.

They do **not** describe every 6R arm. For an arm outside the class — skew J2/J3, non-intersecting J5/J6 — the five quantities are not sufficient, and the solver refuses (Part 9, Step 0). The refusal is honest: the five quantities are not claimed to be universal.

**They are not a replacement for the URDF.** They are a *selection* from the URDF — the geometric quantities the derivation needs, named for what they mean. The solver still reads the URDF's origins and axes for the wrist angles (Part 6), for the arm chain (Part 8), and for the class check.

**DH is not needed and not used.** The five quantities are not DH parameters under different names — they are geometric quantities. `wrist_normal_offset` in particular is not a DH parameter at all; DH's `d₄` happens to equal it for these arms, but that is a frame-placement coincidence (Appendix A).

---

# Part 4 — The Wrist Root and the Arm Plane

## 4.1 The arm plane

The **arm plane** is the plane containing the origins of J2 and J3 — the plane the upper arm and forearm sweep through as J2 and J3 rotate. It is **normal** to J2's axis direction (and to J3's). The plane also contains J1's axis (vertical), and rotates about it as θ₁ changes. **θ₁ determines the arm plane's azimuth.**

## 4.2 The wrist root A

**A is the projection of B onto the arm plane.** This is the end of the forearm.

For UR10 and FR5, J4's axis is parallel to the plane's normal, so it pierces the plane at one point and A is equally "J4's axis ∩ arm plane." **For Elfin, J4's axis lies *in* the plane**, so that intersection is the entire line and the phrase is undefined. The projection is well defined for every arm in the class; the intersection is a special case. Always compute the projection (§7.1).

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

Two conditions, and they are independent.

**(a) The arm plane must contain J1's axis** — the shoulder offset ρ_arm must be zero, so that the θ₁ construction is a tangent from a point rather than a common tangent to two circles (§5.10, Appendix B.3).

**(b) When δ > 0, J4's axis must be normal to the arm plane** — so that â₄ is fixed by θ₁ alone and â₅ can be found before the arm chain. This is not a preference; without it the dependencies close into a loop (§9.3, Appendix B.1). When δ = 0 there is no condition, because the δ term of A vanishes.

One measured number decides (b): **`|â₄ · n̂|`** — 1 for a pitch wrist normal to the plane, 0 for a roll wrist lying in it.

| Arm | ρ | δ | ρ_arm | \|â₄·n̂\| | J4 is | verdict |
|---|---|---|---|---|---|---|
| UR10 | 0.163941 | 0.115700 | ≈0 | 1.000000 | pitch, normal to plane | ✓ wrist before arm |
| FR5 | 0.102100 | 0.102000 | ≈0 | 1.000000 | pitch, normal to plane | ✓ wrist before arm |
| Elfin | ≈0 | 0 | ≈0 | ≈0 | roll, in the plane | ✓ arm before wrist |

Note that "J4 is horizontal" is a *consequence* of J4 ∥ J2 ∥ J3, not the condition itself. Stating it as horizontality obscures the reason and would admit a horizontal roll wrist, which does not satisfy (b).

---

# Part 5 — Solving θ₁: Placing the Arm Plane

## 5.1 What θ₁ does

θ₁ is the rotation of the base joint. It fixes the **azimuth of the arm plane** — the vertical plane in which the upper arm and forearm move. Everything after θ₁ is expressed relative to that plane, so θ₁ comes first.

The geometry is in the **X-Y projection**: looking down J1's axis. In that projection the arm plane is a **line**, and θ₁ is its angle.

## 5.2 The single constraint

There is exactly one condition on the arm plane:

> **The signed perpendicular distance from P5 to the arm plane is a constant of the machine.**

Call it **h**:

```
h = dot(P5 − O, n̂)
```

where `O` is the true-root origin and `n̂` is the arm-plane normal — **J2's axis direction**, carried around by θ₁:

```
n̂(θ₁) = Rz(θ₁) · n̂₀ ,     n̂₀ = J2's axis at zero pose
```

`h` is **signed**. It is not a branch and not a choice. It is measured once from the URDF and never changes.

## 5.3 h is a machine constant — measured

Measured on all three arms at two different configurations:

| Arm | h at q₁=0.3 | h at q₁=0.9 | ρ = \|AB\| | δ = \|BC\| |
|---|---|---|---|---|
| UR10 | +0.163941000 | +0.163941000 | 0.163941 | 0.115700 |
| FR5 | +0.102099067 | +0.102099067 | 0.102100 | 0.102000 |
| Elfin | −0.000002392 | −0.000001566 | ≈0 | 0 |

**Identical across configurations.** For UR10 and FR5, `h = ρ`. For Elfin, `h = 0` to within the URDF's CAD rounding (~2×10⁻⁶ m), and must be treated as zero under a tolerance.

Why it is constant: for UR10 and FR5, J4 ∥ J3 ∥ J2, so `B = A + ρâ₄` is displaced purely along the normal, and `P5 = B + δâ₅` is displaced along J5's axis, which is perpendicular to the normal. Neither displacement changes with the wrist angles. For Elfin, B lies on J4's axis, which lies *in* the arm plane, so B — and therefore P5, since δ = 0 — is always in the plane.

## 5.4 The circle

In the X-Y projection, let **C′** be P5's projection, **O′** the base origin, `r = |O′C′|`, `α = atan2(P5_y, P5_x)`.

The arm-plane line passes through O′ and must sit at perpendicular distance |h| from C′. So:

> **The arm-plane line is a tangent from O′ to the circle of radius |h| centred at C′.**

```
        top view, down J1's axis

         O′ ●
            |\ \
            | \  \      the two tangents = the two arm planes
            |  \   \
            |   (|h|)  ← circle centred at C′, NOT at O′
            |     \  \
            └──────●─── C′  (P5 projected)
```

**The circle is centred at the wrist centre, not at the base.** This is the point on which an earlier version of this document was wrong, and it is worth stating twice. A circle of radius |h| around O′ would give lines at distance |h| from the *base*, which is a different condition — that one is the *shoulder offset* (§5.10), not this.

The angle at O′ between the diagonal O′C′ and the tangent:

```
sin β = |h| / r        →     β = arcsin(|h| / r)
```

## 5.5 The formula

Working with the signed h directly, and writing `az₂` for the azimuth of `n̂₀`:

```
cos(α − az₂ − θ₁) = h / r

θ₁ = α − az₂ ∓ arccos(h / r)
```

**Two roots.** Using `arccos` with the signed `h` absorbs the side automatically — no `σ`, no `±h`, no separate sign bookkeeping.

### 5.5.1 The exact form, when J2 is not quite horizontal

The form above assumes `n̂₀` lies in the XY plane. FR5 and Elfin tilt J2 out of horizontal by **3.7×10⁻⁶ rad**, so `n̂₀ = (s·cos az₂, s·sin az₂, n_z)` with `s = |(n₀ₓ, n₀ᵧ)|` and `n_z ≠ 0`. Writing the constraint out in full:

```
dot(P5, Rz(θ₁)·n̂₀) = s·r·cos(θ₁ + az₂ − α) + n_z·P5_z = h

                h − n_z·P5_z
cos(θ₁+az₂−α) = ─────────────
                     s·r

θ₁ = α − az₂ ∓ arccos( (h − n_z·P5_z) / (s·r) )
```

**The `n_z·P5_z` term is not optional.** It is tiny, but θ₁'s sensitivity to it is `1/(s·r·sin β)`, which diverges at the shoulder singularity. On an FR5 pose with `r − |h| = 35 µm`, dropping the term produced a **7.6×10⁻⁴ rad** error in θ₁ — and since the arm plane normal is `Rz(θ₁)·n̂₀`, that same error rotates the entire plane and propagates into every downstream joint. It was the single largest error source found during implementation.

Reachability is `|h − n_z·P5_z| ≤ s·r`. When `n_z = 0` and `s = 1` this reduces exactly to `r ≥ |h|`.

Verified: this recovers the true q₁ on all three arms (Appendix C).

## 5.6 Why two roots and not four

An earlier version claimed up to four θ₁ values, adding `+π` for "the two directions of the line." That is wrong, and it contradicted the document's own count of eight total solutions (4 × 2 elbow × 2 wrist = 16).

The reason: the constraint is **signed**. `dot(P5 − O, n̂) = h` with h fixed by the machine. Replacing θ₁ by θ₁ + π reverses n̂ and therefore reverses the sign of the left-hand side, violating the constraint. The `+π` candidates satisfy `−h`, which is a *different machine* — one whose wrist is offset to the other side.

Equally, the shoulder branch is **not** a sign `σ` attached to each solution. Both valid roots give the same h. `σ` cannot distinguish them, and treating it as a free choice doubles the solution set with configurations that do not satisfy the geometry.

> **σ is not a parameter. Define the normal from the measured axis, and it disappears.**

## 5.7 The two roots are not π apart

Measured separation between the two θ₁ roots:

| Arm | separation |
|---|---|
| UR10 (q₁=0.3) | 159.635° |
| UR10 (q₁=0.9) | 161.285° |
| FR5 (q₁=0.3) | 161.224° |
| Elfin | 180.000° |

For the offset wrist the separation depends on the pose. **Only when h = 0 does it become exactly π** — the familiar spherical-wrist pair `{α, α + π}`. The spherical case is the value at zero, as claimed, but the general case is genuinely not a `±π` pair.

## 5.8 Reachability

```
r ≥ |h|
```

If `r < |h|` no tangent exists: the target is unreachable. Geometrically, **there is a cylindrical hole of radius |h| around the base axis that the wrist centre can never enter.** At `r = |h|` the two tangents merge — the **shoulder singularity**, visible as a line touching a circle rather than as a matrix losing rank.

When h = 0 the condition is `r ≥ 0`, always satisfied, and the hole closes.

## 5.9 What must be measured, never hardcoded

**`n̂₀` is J2's axis read from the URDF.** An earlier version hardcoded `n̂ = (−sin θ₁, cos θ₁, 0)`, which presumes `n̂₀ = (0, +1, 0)`. Measured in the true-root frame:

| Arm | n̂₀ | azimuth az₂ |
|---|---|---|
| UR10 | (0, −1, 0) | −90° |
| FR5 | (0, −1, −4×10⁻⁶) | −90° |
| Elfin | (0, −1, −4×10⁻⁶) | −90° |

**All three are (0, −1, 0), so the hardcoded normal had the wrong sign for every arm in the repository.** Transport the measured axis; never write the literal.

**Use J2's axis, not J4's, for the azimuth.** For UR10 and FR5 the two coincide. For Elfin, J4's axis is vertical at zero pose, so its azimuth is `atan2(0, 0)` — undefined. J2's axis is horizontal for every arm in the class, so `az₂` always exists.

## 5.10 The shoulder offset — a second, independent offset

The formula above assumes the arm plane **contains J1's axis**. Define

```
ρ_arm = perpendicular distance from J1's axis to the arm plane
```

Measured (as the perpendicular distance of the J2 and J3 origins from the plane through O with normal n̂):

| Arm | ρ_arm |
|---|---|
| UR10 | 2.6×10⁻¹¹ |
| FR5 | 5.6×10⁻⁷ |
| Elfin | 9.6×10⁻⁷ |

Zero for all three, to CAD precision. **When ρ_arm ≠ 0 the arm-plane line does not pass through O′**, and the construction becomes *common tangents to two circles* — radius ρ_arm around O′ and radius |h| around C′ — giving up to four lines instead of two. That is a different derivation, and Hatch refuses it (Appendix B).

ρ_arm is independent of ρ and δ. Three offsets, three different jobs. Do not let one name stand for another.

---

# Part 6 — The Wrist Directions, and θ₅, θ₆

## 6.1 Three axes, one notation

| Symbol | Meaning |
|---|---|
| **â₄** | J4's axis direction, true-root frame, current configuration |
| **â₅** | J5's axis direction, same |
| **â₆** | J6's axis direction = flange Z = the direction the tool points |

`â₆` is **not** the base Z and **not** the vertical. It is wherever the tool happens to point. Writing it as `ẑ` invites exactly that confusion; this document uses `â₆` throughout.

## 6.2 θ₅, defined correctly

> **θ₅ is the angle from â₄ to â₆, measured about â₅.**

Both â₄ and â₆ are perpendicular to â₅ (J4 ⊥ J5 and J5 ⊥ J6), so both lie in the plane perpendicular to â₅, and the angle between them in that plane is well defined. That angle is what θ₅ sweeps.

**The definition "the angle between J5's Z and J6's Z" is impossible.** J5 ⊥ J6 is a structural invariant — measured `â₅·â₆ = 0` on all three arms, at every configuration, because it is built into the links. That definition would make θ₅ identically 90°: a joint that cannot move. The perpendicularity of J5 and J6 lives at *their* crossing point P5; it says nothing about the angle θ₅ sweeps.

Check: `â₄·â₆ = +1.000000` at zero pose on all three arms, i.e. θ₅ = 0. ✓

## 6.3 â₅ comes from the pose

â₅ is perpendicular to â₄ (J4 ⊥ J5) and perpendicular to â₆ (J5 ⊥ J6). Two perpendicularity conditions fix a direction up to sign:

```
â₅ = ± normalize(â₄ × â₆)
```

**The ± is the wrist branch** — `flip` / `no-flip`. Nothing else in the derivation carries a wrist sign. In particular, δ is a positive length; the branch lives in â₅ and nowhere else. This is the single-place rule that keeps the bookkeeping honest.

Inputs: â₄ (from θ₁, when available — §6.7) and â₆ (from the target orientation). **No arm-chain angle is needed.**

## 6.4 θ₅

```
cos θ₅ = dot(â₆, â₄)
θ₅ = signed angle from â₄ to â₆ about â₅
```

The sign follows the chosen â₅. Two branches, `±θ₅`.

**No lateral-offset equation is needed.** An earlier version derived θ₅ from `lat = dot(P6, n̂)` and then computed `cos θ₅ = (lat − σρ)/flange_offset`. That argument is circular: θ₁ was solved precisely to make `dot(P5, n̂) = h`, so `lat = h + d₆·dot(â₆, n̂)` and the expression collapses identically to `dot(â₆, n̂)`. Its reachability guard `|lat − σρ| ≤ d₆` reduces to `|dot(â₆, n̂)| ≤ 1`, which can never fail. A guard that cannot guard is not a guard. The dot product above is the whole content, stated directly.

## 6.5 θ₆ — and why it does not need the arm chain

θ₆ rotates the flange about â₆. A reference direction in the plane perpendicular to â₆ is needed, and **â₅ is such a direction** (â₅ ⊥ â₆). Both â₅ and the flange's X axis at θ₆ = 0 are fixed in link 5, so the angle between them is a constant of the machine:

```
θ₆ = angle(â₅ → X̂_flange about â₆) − c₆
```

where `angle(u → v about n) = atan2(dot(cross(u, v), n), dot(u, v))`.

**Verified invariant.** UR10, θ₅ = 1.2 and θ₆ = −0.4 held fixed while θ₂, θ₃, θ₄ varied:

| θ₂, θ₃, θ₄ | angle(â₅ → X̂_flange about â₆) |
|---|---|
| −0.8, 1.0, 0.5 | 1.170796327 |
| −0.5, 0.7, 0.9 | 1.170796327 |
| −1.1, 1.3, −0.7 | 1.170796327 |

Bit-identical. So θ₆ is available as soon as â₅ is — **before the arm chain**. For UR10, `c₆ = π/2`.

Note what θ₆ *cannot* do: it is a rotation about â₆, and P5 lies on that axis while A and B lie upstream of it. **θ₆ moves nothing upstream, so it plays no part in locating A.**

## 6.6 The wrist constants are machine constants

`c₅` and `c₆` are the angles between the reference directions at the joint's zero pose. Measured: c₅ = 0 on all three arms; c₆ = +π/2 on UR10 and FR5 but **−π/2 on Elfin**. **Extract them from the URDF; do not hardcode π/2** — Elfin's sign alone would produce a silent, uniform orientation error. θ₄ needs no constant at all, because its reference is â₅ itself at zero (§8.5).

## 6.7 When are these directions available?

â₆ always comes from the pose. â₄ depends on which links carry J4's axis:

- **J4 ∥ J2, J3** (a *pitch* continuing the planar chain): â₄ = ±n̂, fixed by θ₁ alone. Available immediately.
- **J4 in the arm plane** (a *roll* about the forearm): â₄ follows the forearm, so it needs θ₂ and θ₃.

One measured number discriminates them:

| Arm | \|â₄ · n̂\| | J4 is | â₄ after θ₁? |
|---|---|---|---|
| UR10 | 1.000000 | pitch, normal to arm plane | yes |
| FR5 | 1.000000 | pitch, normal to arm plane | yes |
| Elfin | 0.000003 | roll, in the arm plane | **no** |

Measured on Elfin, θ₁/θ₅/θ₆ fixed, only θ₂/θ₃ changed:

```
q₂,q₃ = −0.8, 1.0   →   â₄ = (0.930351, 0.287795, −0.227202)
q₂,q₃ = −0.5, 0.7   →   â₄ = (0.890411, 0.275436,  0.362358)
```

â₄ moves. So on Elfin the wrist angles **must** follow the arm chain. The *form* of every formula in this Part is universal; its *position in the sequence* is not. Part 9 makes the ordering a consequence rather than a choice.

## 6.8 The wrist singularity

When **â₆ ∥ â₄** — the tool pointing along J4's axis — the cross product vanishes and â₅ cannot be recovered from the pose. Equivalently `sin θ₅ = 0`.

This is an **alignment of two axes with each other**, not a statement about either being vertical. On UR10 and FR5, â₄ is horizontal for every configuration (J2's axis is horizontal and fixed in link 1, which rotates only about the vertical J1; J3 and J4 rotate about axes parallel to it, which leaves the direction unchanged). At the UR10 home pose:

```
â₄ = (0, −1, 0)
â₆ = (0, −1, 0)      identical — both horizontal
```

**The home pose of all three arms is a wrist singularity.** Any test suite whose first case is `q = 0` is testing the one configuration where the method is undefined. Use a pose with θ₅ near ±π/2 for the general tests, and keep `q = 0` as a separate, explicitly labelled singularity case.

At the singularity θ₄ and θ₆ are determined only as a sum (θ₅ = 0) or difference (θ₅ = π). The solver fixes one by continuity with the current state and **reports that it did so**. Part 7 §7.9 shows what this does to A.

---

# Part 7 — Locating the Wrist Root A

## 7.1 A, defined robustly

> **A is the projection of B onto the arm plane.**

```
A = B − dot(B − O, n̂) · n̂
```

For UR10 and FR5, J4's axis is parallel to n̂, so it pierces the plane at exactly one point and A is also "J4's axis ∩ arm plane." For Elfin, J4's axis lies *in* the plane, so that intersection is the whole line — **the intersection definition is undefined there, while the projection is always well defined.** Use the projection. (This is not a cosmetic point: computing A by intersecting the line with the plane divides by `â₄·n̂`, which is ~3×10⁻⁶ on Elfin and produces garbage.)

Both definitions are origin-independent: B is an intersection of axes, and the projection uses only the plane. The *origin of J4's frame* is not used anywhere — measured, it sits at B on UR10 and at A on FR5, which is precisely why it must not be used.

## 7.2 The formula

```
A = P5 − ρ·â₄ − δ·â₅
```

with `ρ = |AB| > 0`, `δ = |BC| > 0`, and â₄, â₅ the measured/chosen axis directions. Walk back from P5 along J5's axis to B, then along J4's axis to the plane. All three coordinates, directly.

**Verified against forward kinematics at a non-singular pose** (q = [0.3, −0.8, 1.0, 0.5, 1.2, −0.4]):

| Arm | ρ | δ | \|P5 − A\| | error in A |
|---|---|---|---|---|
| UR10 | 0.163941 | 0.115700 | 0.200657 | **1.6×10⁻¹¹ m** |
| FR5 | 0.102100 | 0.102000 | 0.144320 | **4.5×10⁻⁷ m** |
| Elfin | 1.4×10⁻⁶ | 0 | 1.4×10⁻⁶ | **2.0×10⁻⁶ m** |

FR5's and Elfin's residuals are their URDFs' CAD rounding (axis tilts of ~4×10⁻⁶), not the formula.

## 7.3 A distance is not a position

`|P5 − A| = √(ρ² + δ²)` is a fixed length. **Knowing it does not locate A** — a distance from a known point is a sphere. What pins A is the *direction* â₅, and choosing â₅ is choosing the wrist branch (§6.3).

So the honest sequence is: the length tells you *whether* there is work to do; the direction does the work. A presentation that says "find the distance, then find the position" hides the only step that matters.

## 7.4 The circle

Because â₄ ⊥ â₅:

```
dot(P5 − A, â₅) = ρ·dot(â₄, â₅) + δ = δ      — a constant
```

So A is confined to a **plane with normal â₅**, at distance δ from P5. Intersecting that with the fixed length |P5 − A| gives a circle:

- **centre: B** (= P5 − δâ₅)
- **radius: ρ**
- **plane: perpendicular to â₅, offset δ from P5** — *not* through P5

Every point X on it satisfies `|X − P5|² = ρ² + δ²`, since X − B ⊥ â₅ while B − P5 ∥ â₅. So |P5A| is the **slant distance** from P5 to the circle: P5 is the apex of a right circular cone over it.

```
            P5  ← apex
            |\
          δ | \  |P5A| = √(ρ²+δ²)  = slant, constant
            |  \
            B---A      circle: centre B, radius ρ, plane ⊥ â₅
             ρ
```

The sphere of radius |P5A| around P5 is the locus when â₅ is *free*. Once the branch is chosen, â₅ is fixed and the locus is this circle. The solver always chooses the branch first, so **the circle is the object the solver reasons about.**

## 7.5 The arm plane is tangent to that circle — at A

Let `ê₃ = n̂ × â₅`. The circle's plane (normal â₅, through B) and the arm plane (normal n̂, through O) meet in a line L through A with direction ê₃. Since `B − A = ρn̂ ⊥ ê₃`:

```
dist(B, L) = ρ = radius        →      L is tangent to the circle
```

and the foot of the perpendicular from B to L is A itself.

> **The arm plane touches the wrist circle at exactly one point, and that point is the wrist root.**

Two consequences worth having. First, the tangency is the *right* place for the tangency intuition — not the θ₁ top view, which is a different circle (§5.4). Second, a tangent meets a circle once, so **A is unique once θ₁ and the wrist branch are fixed**: no quadratic, no residual ±, no ambiguity deferred to a later part. The uniqueness is visible in the picture rather than proved by counting roots.

## 7.6 The fork is δ, not |P5A|

Look at which term needs what:

```
A = P5 − ρ·â₄ − δ·â₅
         ↑        ↑
    needs â₄   needs â₅
   (from θ₁)  (wrist branch)
```

**Only the δ term forces the wrist to be solved before the arm chain.** So the length that governs the ordering is **δ**, not |P5A|.

This matters because the two conditions are not the same:

- **spherical wrist** ⟺ δ = 0 (B and P5 coincide; the three wrist axes concur)
- **A = P5** ⟺ ρ = 0 **and** δ = 0

A machine with δ = 0 but ρ ≠ 0 has a spherical wrist yet A ≠ P5 — and locating A needs only â₄, no wrist information at all. Keying the fork on |P5A| would route such a machine through an unnecessary wrist solve. UR10, FR5 and Elfin all happen to make the two conditions coincide, which is exactly why the simplification is dangerous: it passes every example in this repository and fails on the next arm.

*(No commercial instance of δ = 0 with ρ ≠ 0 has been verified here. The argument rests on the structure of the formula, not on an example — and it is stated as such rather than illustrated with an invented machine.)*

## 7.7 The degenerate cases

| ρ | δ | what happens |
|---|---|---|
| ≠0 | ≠0 | full circle; â₅ needed first (UR10, FR5) |
| ≠0 | 0 | plane passes through P5, B = P5; circle centre P5, radius ρ; â₅ not needed |
| 0 | ≠0 | circle shrinks to the point B |
| 0 | 0 | circle = point = P5 = A; the whole step vanishes (Elfin) |

One formula. The step disappears when the lengths say it should. No `if spherical` anywhere.

## 7.8 What must not be used as the arm-chain target

**Q**, the projection of P5 onto the arm plane, is *not* A. Since `Q = A + δâ₅`:

```
|A − Q| = δ
```

Measured: **115.7 mm on UR10, 102.0 mm on FR5.** At the test pose the height alone differs by 88 mm. Using Q as the arm-chain target misplaces it by a tenth of a metre — which is why a solver built on Q does not converge. Elfin conceals the error because δ = 0 makes Q = A.

## 7.9 At the singularity, A is a circle

When â₆ ∥ â₄, â₅ is undetermined, so `δâ₅` may point anywhere in the plane perpendicular to â₄. That plane is parallel to the arm plane and passes through Q, so it *is* the arm plane. Therefore:

> **At the wrist singularity A is not a point but a circle of radius δ centred at Q, lying in the arm plane.**

The same circle as §7.4, read the other way: normally â₅ pins A to one point on it; at the singularity the pin is gone and the whole circle is available. The honest response is to choose a point by continuity with the current θ₄ and report the choice — not to return an arbitrary one silently.

This is also the practical warning for seam tracing: near the alignment, â₅ is ill-conditioned, so A becomes hypersensitive to tiny orientation changes. A seam can be entirely reachable and still be untraceable at constant speed. The validation pass must check that `|â₄ × â₆|` stays away from zero **along the whole path**, not merely that each point is reachable.

---

# Part 8 — The Arm Chain: θ₂, θ₃, and the Roll θ₄

## 8.1 The target and the two lengths

A is now a complete 3D point, known before the arm chain is touched. The chain is a **planar 2R** with two **constant** sides:

| Length | Meaning |
|---|---|
| `axis_gap(2)` | perpendicular distance between J2's and J3's axes — the upper arm |
| `elbow_to_wrist` | distance from J3's axis to **A** — the forearm |

Measured: `elbow_to_wrist` = 0.572300 (UR10), 0.395010 (FR5), 0.570000 (Elfin).

**Both are constants of the machine.** That is what makes the law of cosines legal. An earlier version used `forearm_length() = |Q − J3_origin|`, which is not constant: Q orbits A as θ₄ varies, and J3's origin depends on θ₂ — the very unknown being solved for. The document's own worked-example tables listed 0.572300 and 0.395010, which are the distances to **A**, contradicting the text's definition. The tables were right; the definition was wrong.

## 8.2 In-plane coordinates

```
ê_u = (cos θ₁, sin θ₁, 0)      horizontal, along the arm-plane line
ê_w = (0, 0, 1)                vertical

Δu = dot(A − p₂, ê_u)
Δw = dot(A − p₂, ê_w)
D  = √(Δu² + Δw²)
```

`p₂` is J2's origin. Note that **A's height enters here**, and it came from the `−δâ₅` term of Part 7 — the term the earlier derivation deferred and then dropped.

## 8.3 θ₃ — law of cosines

```
cos θ₃ = (D² − axis_gap(2)² − elbow_to_wrist²) / (2 · axis_gap(2) · elbow_to_wrist)
θ₃ ∈ { +arccos(...)  elbow-up ,  −arccos(...)  elbow-down }
```

Reachability: `|axis_gap(2) − elbow_to_wrist| ≤ D ≤ axis_gap(2) + elbow_to_wrist`. Two circles failing to meet — visible, not a numerical failure.

## 8.4 θ₂

```
ψ  = atan2(Δw, Δu)
β₂ = atan2(elbow_to_wrist · sin θ₃,  axis_gap(2) + elbow_to_wrist · cos θ₃)
θ₂ = ψ − β₂
```

The `atan2` form is preferred over `arcsin(.../D)`: it is correct for obtuse β₂ and does not divide by D, which vanishes when the target coincides with the shoulder.

**Elfin's J2 and J3 are anti-parallel** (measured `â₂·â₃ = −1.000000`, i.e. `axis_twist(2) = π`). The sign of θ₃ relative to the plane's orientation must therefore be taken from the measured axis directions, not assumed from the joint index. UR10 and FR5 have `â₂·â₃ = +1.000000`.

## 8.5 θ₄ — the roll, read against â₅ at zero

The reference direction for θ₄ must be **the thing being rotated**, not an arbitrary direction in the parent link. Place the chain with θ₂ and θ₃ solved and **θ₄ = 0**, and read â₅ there:

```
â₅⁰ = â₅ measured with θ₄ = 0, arm joints already solved
θ₄  = angle(â₅⁰ → â₅ about â₄)
```

**No constant.** Because the reference is â₅ itself at zero, `c₄` is identically zero and never needs extracting.

**Two wrong references, both tried and both rejected:**

1. **The parent link's X̂ axis.** This is fixed in the parent frame and perpendicular to â₄ after projection, so it looks usable — but it is not â₅⁰, and the angle differs from θ₄ by a per-arm constant. Measured on UR10: exactly **π/2**. Implementing this produced θ₁, θ₂, θ₃, θ₅, θ₆ all exact and θ₄ wrong by −π/2, which is the signature of a reference-direction error rather than a geometry error.
2. **The forearm direction f̂ = normalize(A − p₃).** On UR10 and FR5 this works and also gives π/2. On **Elfin it is degenerate**: â₄ lies along the forearm, so f̂ ∥ â₄ and the angle about â₄ is undefined. A formula that fails on one supported arm is not the formula.

Reading â₅⁰ costs one chain placement and works on all three arms without a constant, without a projection, and without a degenerate case.

For Elfin, â₄ and â₅ become available only here, so θ₅ and θ₆ (Part 6) are evaluated at this point rather than earlier. Same formulas, later position.

## 8.6 The full solution count

```
2 (shoulder, §5.5)  ×  2 (wrist, §6.3)  ×  2 (elbow, §8.3)  =  8
```

Eight, as the machine has. Every factor is a named geometric choice, and each degenerates at an identifiable locus: the shoulder pair merges at `r = |h|`, the elbow pair at full stretch or fold, the wrist pair at `â₆ ∥ â₄`. **The branch signs and the singularity set are the same geometry read twice** — which is what makes configuration continuity checkable rather than hopeful.

---

# Part 9 — The Algorithm

## 9.1 Inputs

**From the URDF, extracted once:** ρ, δ, d₆ (signed), `axis_gap(2)`, `elbow_to_wrist`, `n̂₀`, `az₂`, `c₅`, `c₆`, the joint limits, the model's own axis-tilt defect (§C.4), and the class measurements (§9.2). `c₄` is not extracted: it is zero by construction (§8.5).

**From the command:** the **flange** pose in the true-root frame. If the command is a TCP pose with a tool attached, **remove the tool transform first**. A welding torch is usually bent, so its TCP is not on â₆, and `P6 − d₆â₆` applied to a TCP is simply wrong.

## 9.2 Step 0 — the class check, measured

Refuse loudly unless all of the following hold, reporting the measured value on failure:

1. Six movable revolute/continuous joints in the arm chain.
2. J5 and J6 intersect within tolerance — this is what makes P5 exist.
3. `|â₄·â₅| ≈ 0` and `|â₅·â₆| ≈ 0`.
4. `|â₄·â₆| ≈ 1` **at the zero pose** — a structural statement about the frame conventions, *not* about a live configuration. During motion `â₄·â₆ = cos θ₅` varies freely; checking this on a live pose would refuse almost everything.
5. **ρ_arm ≈ 0** — the arm plane contains J1's axis (§5.10).
6. **h is configuration-independent.** Verify by measuring it at two distinct poses.
7. If `δ > 0`, then `|â₄·n̂| ≈ 1` — J4 normal to the arm plane, so â₄ is available after θ₁ (§9.3).

Condition 7 is not an arbitrary restriction; it is the absence of a circular dependency. Appendix B derives it.

## 9.3 The availability rule

> **There is one formula for the wrist root: `A = P5 − ρâ₄ − δâ₅`. Each term needs a direction. A direction is available once the joints that fix it are known. The solving order is not chosen by wrist type — it is forced by which directions exist yet.**

| J4 ∥ J2,J3? | δ | locating A needs â₅? | forced order |
|---|---|---|---|
| yes | ≠ 0 | yes | θ₁ → wrist → A → arm |
| yes | = 0 | no | θ₁ → A → arm → wrist |
| no (roll) | = 0 | no | θ₁ → arm → wrist |
| no (roll) | ≠ 0 | yes | **circular — refuse** |

UR10 and FR5 occupy row 1; Elfin row 3. No brand names, no wrist-type label, no `if spherical`.

## 9.4 The sequence

```
Step 1   P5 = P6 − d₆·â₆                         (d₆ signed; tool already removed)

Step 2   r = |P5_xy| ;  α = atan2(P5_y, P5_x)
         if r < |h|: unreachable on this pose
         θ₁ = α − az₂ ∓ arccos((h − n_z·P5_z)/(s·r))   two roots (§5.5.1)
         n̂ = Rz(θ₁)·n̂₀

Step 3   if |â₄·n̂| ≈ 1:  â₄ = ±n̂ (sign from the URDF)
         else:            â₄ deferred to Step 6

Step 4   if δ > 0:                                (â₄ must be available; else refuse)
             â₅ = ±normalize(â₄ × â₆)             two branches
             θ₅ = signed angle(â₄ → â₆ about â₅)
             θ₆ = angle(â₅ → X̂_flange about â₆) − c₆

Step 5   A = P5 − ρ·â₄ − δ·â₅                     (δ = 0 ⇒ the last term vanishes)

Step 6   planar 2R to A with axis_gap(2), elbow_to_wrist
             θ₃ = ±arccos(...)                    elbow-up / elbow-down
             θ₂ = ψ − β₂
         f̂ = normalize(A − p₃)
         if â₄ was deferred: recover â₄ from f̂ and the URDF, then do Step 4
         θ₄ = angle(â₅⁰ → â₅ about â₄)          (â₅⁰ read with θ₄ = 0; no constant)

Step 7   tag (shoulder, elbow, wrist) ; filter by joint limits modulo 2π

Step 8   return all valid tagged solutions; the caller selects by continuity
```

**Singularities.** `r = |h|` → shoulder pair merges. `D` at a stretch/fold limit → elbow pair merges. `|â₄ × â₆| ≈ 0` → â₅ undetermined, A becomes the circle of §7.9; fix one angle by continuity and **report it**. No silent arbitrary choices.

## 9.5 Worked example — UR10

Constants: ρ = 0.163941, δ = 0.115700, d₆ = **+0.092200**, `axis_gap(2)` = 0.612000, `elbow_to_wrist` = 0.572300, n̂₀ = (0, −1, 0), az₂ = −90°, c₅ = 0, c₆ = π/2, c₄ = 0 (§8.5).

**d₆ is signed**, and its sign follows the flange link the model designates (§4). With `get_flange()` = `wrist_3_link` it is positive here. Measure it; do not assume either sign.

At q = [0.3, −0.8, 1.0, 0.5, 1.2, −0.4], forward kinematics gives P5 = (−0.823527, −0.426352, 0.364131). Then:

```
h        = +0.163941                        (machine constant, verified at two poses)
r        = 0.927…                           α = atan2(−0.426352, −0.823527)
θ₁       ∈ {−2.486155, 0.300000}            → 0.3 recovered exactly
|â₄ × â₆| = 0.932039                        well away from the singularity
A        = (−0.943181, −0.291760, 0.452623)  error 1.6×10⁻¹¹ m
```

The two θ₁ roots are 159.635° apart — not 180°.

## 9.6 Worked example — FR5

ρ = 0.102100, δ = 0.102000, d₆ = +0.100000, `axis_gap(2)` = 0.425000, `elbow_to_wrist` = 0.395010, n̂₀ = (0, −1, −4×10⁻⁶).

Same target pose, same steps. P5 = (−0.559773, −0.280032, 0.300386); θ₁ ∈ {−2.513884, 0.300002}; A = (−0.652720, −0.201911, 0.378400), error 4.5×10⁻⁷ m.

**The origin trap, measured:** for UR10 the origin of J4's frame sits at **B**; for FR5 it sits at **A**. Opposite ends of the same segment. Every quantity in this derivation is computed from axes as lines and from intersections, so it is immune. Any origin-derived computation would disagree between these two arms.

## 9.7 Worked example — Elfin E15 Pro (the step that vanishes)

ρ ≈ 0 (1.4×10⁻⁶), δ = 0, d₆ = 0.200000, `axis_gap(2)` = 0.730000, `elbow_to_wrist` = 0.570000, `â₂·â₃ = −1` (anti-parallel), `|â₄·n̂| ≈ 0` (roll wrist).

```
h ≈ 0        →  arccos(0) = π/2  →  θ₁ ∈ {α − az₂ − π/2, α − az₂ + π/2}
             →  measured {0.299998, 3.441595}: q₁ and q₁ + π, exactly 180° apart
δ = 0        →  Step 4 skipped; A = P5 − ρâ₄ ≈ P5
             →  A = (1.030582, 0.318795, 0.641091), error 2.0×10⁻⁶ m
|â₄·n̂| ≈ 0  →  â₄ deferred; θ₅, θ₆, θ₄ evaluated after the arm chain
```

**Same formulas throughout.** What changes is that a term is zero and a direction arrives later. Nothing is special-cased.

## 9.8 The reversal, restated

The conventional derivation treats the spherical wrist as the natural case and the offset wrist as a complication. This document reverses that: the general machine has ρ and δ nonzero, and the spherical wrist is the value at zero.

But the reversal is stronger than "one code path." It is:

> **One formula, one vocabulary, and an order dictated by the machine's own dependency structure — with a principled refusal when those dependencies close into a loop.**

The word is **unified**, not **universal**.

---

# Appendix A — DH Comparison

## A.1 What DH encodes

Four numbers per consecutive axis pair: `d` (offset along the previous axis), `a` (distance along the common perpendicular), `α` (twist between the axes), `θ` (the joint variable).

## A.2 How the quantities relate

| Hatch quantity | DH analog | Relationship |
|---|---|---|
| `axis_gap(i)` | `a` | perpendicular distance between axes i and i+1 |
| `axis_twist(i)` | `α` | angle between axes i and i+1 |
| `axis_shift(i)` | `d` | distance along axis i; **origin-dependent** |
| `joint_angle(i)` | `θ` | the joint's rotation |
| `wrist_normal_offset` (ρ) | no direct analog | DH's `d₄` equals it for UR10 by frame-placement coincidence |
| `wrist_radial_offset` (δ) | no direct analog | the second wrist offset; DH hides it inside `d₅` |
| ρ_arm | no direct analog | shoulder offset; appears as `d₂`/`d₃` depending on convention |

## A.3 Why Hatch does not use DH

DH assumes canonically placed frames; the placement is non-unique and ambiguous for parallel axes. A URDF joint origin is a full 6-DOF transform — strictly more information. And the PUMA 560 illustrates the cost of the compression: its shoulder offset appears as `d₃` in one convention and `d₂` in another, for the same physical 150 mm. The geometric quantity — *the arm plane misses the base axis by 150 mm* — is convention-free. That is the quantity Hatch measures.

## A.4 `axis_shift` is not ρ, at any index

| Arm | ρ | `axis_shift(3)` (along J3) | `axis_shift(4)` (along J4) |
|---|---|---|---|
| UR10 | 0.163941 | 0.163941 | 0.000000 |
| FR5 | 0.102100 | 0.000000 | 0.102100 |

The algorithm uses ρ, computed from axis intersections, never `axis_shift(i)`. That is why it is immune to the origin placement (§9.6).

---

# Appendix B — What Hatch Refuses, and Why

## B.1 The refusal is derived, not enumerated

The deepest restriction is not a list of brands. It is a **circular dependency**:

> If δ ≠ 0, locating A requires â₅. â₅ requires â₄. If J4 is a roll about the forearm, â₄ requires θ₂ and θ₃. θ₂ and θ₃ require A. **The dependency closes into a loop, so no closed-form solution exists in this order.**

That is row 4 of §9.3. The refusal is not "this brand is unsupported" — it is "the geometry you have described cannot be unwound in sequence." A user can *see* why, and can see what would have to change.

## B.2 The supported class

Arm-plane 6R arms with intersecting J5/J6, zero shoulder offset, configuration-independent h, and — when δ ≠ 0 — J4 normal to the arm plane. UR10, FR5 and Elfin E15 Pro all qualify, by measurement (Appendix C).

## B.3 The offset shoulder

**ρ_arm ≠ 0: the arm plane does not contain J1's axis.** In top view the arm-plane line misses O′, so the construction becomes common tangents to two circles (radius ρ_arm at O′, radius |h| at C′) — up to four lines, hence up to four shoulder branches and a different solution count.

**This is a real and common commercial arrangement, not a curiosity.** The canonical example is the **PUMA 560**, whose shoulder offset is 0.15005 m and which also carries a 0.0203 m elbow offset; Corke's toolbox documents `ikine6s` as covering "the Puma 560 arms with shoulder and elbow offsets," alongside a KUKA model [1](https://www.petercorke.com/RTB/r9/html/SerialLink.html). The PUMA's wrist is spherical, so it is *not* an example of δ = 0 with ρ ≠ 0 — it is an example of a **third, independent offset** that this derivation does not cover.

Note that the PUMA's offset arises even though J1's and J2's axes intersect: the upper-arm plane is displaced *along* the shoulder axis. Whether an offset shows up as an `a` or a `d` in DH is bookkeeping; whether the arm plane misses the base axis is physics. Measure the latter.

Hatch v1 refuses ρ_arm ≠ 0. The extension is well-defined (two-circle tangency) and is a natural next step, but it is not implemented and must not be implied.

## B.4 Also refused

- **Skew J2/J3** — no arm plane exists.
- **Non-intersecting J5/J6** — no P5 exists.
- **Nonzero `a₄`/`a₅`** (skew wrist axes, e.g. the PUMA's 20.3 mm elbow offset) — B does not exist as an intersection.
- **7-DOF, SCARA, delta, parallel** — different structure entirely.

## B.5 Why refuse rather than approximate

**A plausible-looking wrong answer is worse than no answer.** An approximated non-class robot yields angles that place the flange near the target, with an error the caller cannot bound. In a safety-critical cell that is unacceptable. The refusal is a `RuntimeError` naming the violated condition and its measured value.

---

# Appendix C — Measured Constants and Verification Record

All values measured from the URDFs in `assets/robots/`, in each arm's **true kinematic root** frame. The true root is the parent link of the first movable joint — for UR10 the link is named **`base_link_inertia`** (not `base_inertia` as earlier documents stated; name-based assumptions will miss it).

## C.1 Axis directions at zero pose

| | â₂ | â₃ | â₄ | â₅ | â₆ | â₂·â₃ | â₂·â₄ | â₄·â₆ |
|---|---|---|---|---|---|---|---|---|
| UR10 | (0,−1,0) | (0,−1,0) | (0,−1,0) | (0,0,1) | (0,−1,0) | +1 | +1 | +1 |
| FR5 | (0,−1,−4e−6) | (0,−1,−4e−6) | (0,−1,−4e−6) | (0,0,−1) | (0,−1,−4e−6) | +1 | +1 | +1 |
| Elfin | (0,−1,−4e−6) | (0,+1,+4e−6) | **(0,0,1)** | (0,1,4e−6) | (0,0,1) | **−1** | **≈0** | +1 |

`â₄·â₆ = +1` at zero pose on all three: **the home pose is a wrist singularity for every arm here.**

## C.2 Scalar constants

| Quantity | UR10 | FR5 | Elfin |
|---|---|---|---|
| ρ = \|AB\| | 0.163941 | 0.102100 | ≈0 (1.4e−6) |
| δ = \|BC\| | 0.115700 | 0.102000 | 0 |
| \|P5A\| = √(ρ²+δ²) | 0.200657 | 0.144320 | ≈0 |
| h (signed) | +0.163941 | +0.102099 | ≈0 (−2e−6) |
| d₆ (signed, vs `get_flange()`) | **+0.092200** | +0.100000 | +0.200000 |
| `axis_gap(2)` | 0.612000 | 0.425000 | 0.730000 |
| `elbow_to_wrist` | 0.572300 | 0.395010 | 0.570000 |
| ρ_arm | ≈0 (2.6e−11) | ≈0 (5.6e−7) | ≈0 (9.6e−7) |
| \|â₄·n̂\| | 1.000000 | 1.000000 | ≈0 (3e−6) |
| az₂ | −90° | −90° | −90° |
| c₅, c₆ | 0, +π/2 | 0, +π/2 | 0, **−π/2** |

## C.3 Verification performed

**Full round-trip, all three arms, implemented solver.** 1000 random configurations per arm drawn uniformly from ±2.6 rad on every joint, each converted to a flange pose by the model's FK, solved, and every returned branch checked back against the target pose:

| | UR10 | FR5 | Elfin |
|---|---|---|---|
| poses with no solution | 0 / 1000 | 0 / 1000 | 0 / 1000 |
| worst position error, **all** branches | **1.9e−11 m** | 7.3e−7 m | 5.4e−6 m |
| worst orientation error, all branches | **3.7e−8 rad** | 7.3e−6 rad | 7.3e−6 rad |
| branch-count histogram | 8:768, 6:42, 4:158, 2:32 | 8:738, 6:51, 4:176, 2:35 | 8:1000 |
| original configuration recovered | ✓ | ✓ | ✓ |
| cost per solve (8 branches, verified) | 2.0 ms | 1.9 ms | 1.7 ms |

Also verified, as tests in `core/kinematics/test_unified_ik_solver.py` (60 passing):

| Check | Result |
|---|---|
| every returned branch reproduces the pose | ✓ all arms |
| up to 8 distinct solutions, distinct branch tags | ✓ all arms |
| joint limits respected after one 2π wrap | ✓ all arms |
| `solve_closest` preserves configuration | ✓ all arms |
| wrist singularity (θ₅ = 0, ±π) solved **and flagged** | ✓ all arms |
| wrist-singular continuity honours the reference | ✓ all arms |
| elbow singularity (full stretch) | ✓ all arms |
| shoulder singularity (r = \|h\|) | ✓ all arms |
| unreachable pose → no solution | ✓ all arms |
| pose inside the shoulder hole → no solution | ✓ UR10, FR5 |
| refusal: PUMA-style shoulder offset | ✓ names ρ_arm |
| refusal: roll wrist with δ > 0 | ✓ names the loop |
| refusal: non-orthogonal wrist | ✓ |
| fast chain walk == scene FK | ✓ to 1e−12 |

### C.3.1 Where the residual comes from

The solver is exact. **The residual is the URDF's own inconsistency, multiplied by the geometric condition number of the pose.** UR10's axes are orthogonal to 2×10⁻¹⁰; FR5's and Elfin's are tilted by 3.7×10⁻⁶ by CAD rounding. Bucketing 2000 poses per arm by `cond = max(1/sin β, 1/r, 1/|sin θ₅|, 1/|sin θ₃|)`:

| arm | model tilt | cond<10 | cond<100 | cond<1000 | cond≥1000 |
|---|---|---|---|---|---|
| UR10 | 2.1e−10 | 1.9e−9 | 9.2e−9 | 2.9e−8 | 1.3e−7 |
| FR5 | 3.7e−6 | 3.4e−5 | 1.2e−4 | 5.6e−4 | 5.7e−4 |
| Elfin | 3.7e−6 | 1.3e−4 | 1.2e−3 | 3.2e−3 | 2.3e−2 |

The error tracks `tilt × cond` across three orders of magnitude. Two consequences:

1. **Joint-vector comparison is the wrong acceptance test.** Near a singularity the pose is well determined but the joint vector is not; two different q can give the same pose to model precision. Test the *pose*.
2. **No closed-form solver can do better on these URDFs.** Driving the residual to machine precision would require either fixing the URDF's axes or a numerical polish step, which is a different kind of solver.

**Scope.** All of the above is against the URDFs' own forward kinematics. There is still **no comparison against a physical robot or a controller's own IK**, and no calibration. Those remain to be done.

## C.4 Tolerances

FR5 and Elfin carry axis tilts of ~3.7×10⁻⁶ from CAD rounding, giving residuals up to ~5×10⁻⁶ m. Class checks must use a tolerance no tighter than ~1×10⁻⁵, and `|h|` below that must be treated as zero — otherwise Elfin is misclassified as an offset wrist and routed down a path its geometry does not need.

**The singularity threshold must be derived from the model, not fixed.** A fixed `|â₄ × â₆| < 1e−7` test *fails to detect* FR5's and Elfin's wrist singularities, because their axes are tilted by 3.7×10⁻⁶ and the cross product never gets that small. At θ₅ = 0 the solver would treat â₅ as recoverable from the pose and return an unflagged, arbitrary member of the singular family. The threshold is therefore

```
singular_tol = max(1e−7, 10 × model_tilt)
model_tilt   = max(|â₄·â₅|, |â₅·â₆|, ||â₄·â₆|−1|, ||â₂·â₃|−1|, |n₀_z|)  at zero pose
```

giving 1e−7 on UR10 and 3.7e−5 on FR5 and Elfin. **A direction cannot be resolved more finely than the model defines it.** This was found by a failing test, not by reasoning.

---

# Appendix D — Code Obligations

`core/kinematics/unified_ik_solver.py` now implements this document, and the corrections found while implementing it have been folded back into the text above (§5.5.1, §8.5, §C.3.1, §C.4). The obligations below are recorded as the specification the code was written against.

**Known stale callers:** `check_solver.py` and `mini_solver_check.py` at the repository root still call the abolished API (`UnifiedIKSolver(geom)`, `.forearm_to_root`). They embody the superseded derivation and should be deleted or rewritten.

## D.1 Extraction (`core/kinematics/geometric_extraction.py`)

Already correct and to be kept: 1-based indexing; the three triangle quantities; the line-based, origin-independent computation of B and ρ; the conditional class check.

Implemented in the solver's `_extract_constants()` rather than in extraction, so that `ArmGeometry` stays a pure description of the axes: `h`, `ρ_arm`, `az₂`, signed `d₆`, `c₅`, `c₆`, `model_tilt`. Note that `ArmGeometry.flange_offset` and `wrist_normal_offset` are **unsigned norms** and cannot supply the signs the solver needs.

The original obligations, for the record:

1. **`wrist_root()` must use the projection of B, not a line/plane intersection.** The intersection divides by `â₄·n̂`, which is ~3e−6 on Elfin (§7.1).
2. **Replace `forearm_length()` with `elbow_to_wrist` = |A − J3 axis|,** a constant. Delete the Q-targeting definition entirely (§8.1).
3. **Add `h`** as a signed extracted constant, and **verify it at two poses** as a class check.
4. **Add `ρ_arm`** and refuse when it exceeds tolerance (§5.10, B.3).
5. **Add `az₂` from the measured J2 axis,** and `|â₄·n̂|` as the pitch/roll discriminator.
6. **Extract `c₅` and `c₆`** from the zero-pose frame conventions; do not hardcode π/2. `c₄` is unnecessary (§8.5).
7. **`describe()` should print ρ, δ, |P5A|, h, ρ_arm, d₆ (signed), `elbow_to_wrist`, and the discriminator,** so a reader can see which case the machine is.

## D.2 Solver (`core/kinematics/unified_ik_solver.py`)

**Implemented.** `UnifiedIKSolver(model)` follows §9.4. The substantive changes from the code that preceded it:

- **θ₁ from `arccos(h/r)` with signed h and measured az₂.** Remove the four-root `±β, +π` enumeration and remove `σ` entirely (§5.5–5.6).
- **θ₅ from `dot(â₆, â₄)`.** Delete the `lat` derivation and its vacuous guard (§6.4).
- **θ₆ computed early** from `angle(â₅ → X̂_flange about â₆) − c₆`, when â₅ is available (§6.5).
- **A = P5 − ρâ₄ − δâ₅,** then the arm chain targets **A**, never Q (§7.2, §7.8).
- **θ₄ from `angle(â₅⁰ → â₅ about â₄)`,** where â₅⁰ is read with θ₄ = 0. No constant. Delete the `R_diff` approach; do not use f̂ (degenerate on Elfin) or the parent X̂ (off by π/2) — §8.5.
- **Ordering driven by the availability rule,** not by a wrist-type flag (§9.3).
- **Singularity handling that reports its choice** (§7.9, §9.4).

## D.3 Tests

1. **Never use `q = 0` as the general test case** — it is a wrist singularity on all three arms (§6.8). Use θ₅ ≈ ±π/2; keep q = 0 as a labelled singularity test.
2. **Round-trip:** for sampled poses across the workspace, `FK(IK(pose)) == pose` for every returned branch.
3. **Branch integrity:** each returned solution's tags must match the geometric signs it was built from; no branch may silently change identity across a sweep.
4. **Constants:** h, ρ, δ, ρ_arm measured at several poses must agree to tolerance.
5. **Refusal:** a synthetic URDF with ρ_arm ≠ 0, and one with δ ≠ 0 and a roll J4, must both be refused with the offending value named.
6. **Degeneracy:** at `r = |h|`, at full stretch, and at `â₆ ∥ â₄`, the solution count must drop as predicted and the solver must report which locus was hit.

## D.4 What this document does not claim

No configuration management, no calibration, no collision checking, no trajectory or feedrate layer, and no verification against physical hardware. The derivation and its implementation are verified against the URDFs' own forward kinematics over 3000 random configurations plus targeted singularity and refusal tests (C.3). Everything beyond that is still to be built.

---

*Hatch (孵) 🐣 — built to understand, built to see.*
*One formula: A = P5 − ρâ₄ − δâ₅. The order is forced by the geometry, not chosen by a label.*
*The spherical wrist is the value at zero.*
