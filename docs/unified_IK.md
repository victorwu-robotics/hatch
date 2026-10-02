# Inverse Kinematics in Hatch (孵)

## A Unified Derivation for Spherical and Offset Wrists

---

# Part I: Seeing the Robot

## 1. What a Robot Arm Is Made Of

Before we solve anything, we must see what we are solving.

A 6-DOF industrial arm is a chain of rigid bodies connected by joints. Each joint allows exactly one motion: rotation about a line. That line is the **joint axis**.

The robot's geometry is the collection of these axes and the fixed relationships between them:

- Where each axis sits in space, relative to the previous axis
- How each axis is oriented, relative to the previous axis
- How far along each axis the next joint attaches

Everything else — the meshes, the colors, the physical shape of the links — is decoration. The kinematics is entirely captured by the axes.

## 2. What the URDF Gives Us

The URDF describes each joint with three pieces of information:

- **The joint origin** (`xyz`, `rpy`): the position and orientation of the joint's frame relative to its parent
- **The joint axis** (`xyz`): the direction the joint rotates about
- **The joint type** (`revolute`, `continuous`, `prismatic`, `fixed`): what kind of motion the joint allows

For a revolute joint, this is complete. The origin tells us where the axis sits and how it is oriented; the axis tells us the direction of rotation; the type tells us the joint moves.

**This is all the kinematics we need.** The URDF is not a description of a robot that *happens to include* kinematics — it *is* the kinematics, plus visual geometry.

## 3. Why We Do Not Need DH

The Denavit–Hartenberg (DH) convention is a way of encoding the relationship between two consecutive joint axes into four numbers. It was invented in 1955, for a world where robots were specified on paper data sheets and the geometry had to be reconstructed from a small set of scalar parameters.

The URDF gives us **more information than DH does**. Each joint origin is a full 6-DOF transform. DH compresses that transform into four scalars by assuming the frames are placed in a canonical way. The compression is lossy: it works only for the class of serial arms DH was designed for, and it becomes ambiguous when two consecutive axes are parallel.

Hatch does not build a DH table. It reads the joint origins and axes directly from the URDF and works with them. The four quantities that DH would encode are still useful — we will name them and use them — but they are *derived from* the URDF, not *substituted for* it.

The direction of derivation is:

> **URDF → geometric quantities → solver**

not:

> **URDF → DH table → solver**

This matters because the first direction works for every robot the URDF can describe. The second direction works only for robots whose geometry fits the DH assumptions, and it fails silently when it does not.

## 4. What We Will Use

From the URDF, we will extract four geometric quantities for each joint pair:

- **shift** — the distance along a joint axis to where the next joint attaches
- **gap** — the distance between two consecutive joint axes
- **twist** — the angle between two consecutive joint axes
- **joint_angle** — the rotation of the joint itself (the variable we solve for)

These four quantities are the same physical information DH encodes. We name them for what they are, not for the letters `d`, `a`, `α`, `θ`. The names tell the reader what the quantity *means*, and they make the derivation readable without a DH reference table.

In the next part, we define each quantity geometrically.

---

# Part II: The Four Geometric Quantities

## 5. The Question We Are Answering

Every joint in a serial arm rotates about a line — its **joint axis**. To describe the robot, we need to describe how each joint axis relates to the next one.

Two lines in space are related by four numbers:

- How far apart are they?
- How are they rotated relative to each other?
- Where along the first line does the second one attach?
- How much has the joint rotated?

These are the four quantities. We will name them for what they mean.

## 6. The Four Quantities

### axis_gap

**How far apart are two consecutive joint axes?**

The distance between two lines in space is measured along their **common perpendicular** — the shortest line segment connecting them. That segment's length is the `axis_gap`.

For a UR arm, the `axis_gap` between J2 and J3 is the distance between the shoulder axis and the elbow axis. The `axis_gap` between J3 and J4 is the distance between the elbow axis and the wrist-1 axis.

### axis_twist

**How are two consecutive joint axes rotated relative to each other?**

Each joint axis is a line in space, and each line has a direction. The angle between the two directions is the `axis_twist`. It is measured about the common perpendicular, and it is signed — the sign tells you the direction of the twist.

For a UR arm:

- Between J1 and J2, the `axis_twist` is 90° — the base axis is vertical, the shoulder axis is horizontal.
- Between J2 and J3, the `axis_twist` is 0° — the shoulder and elbow axes are parallel. This is what makes the upper arm and forearm move in a single plane.
- Between J4 and J5, the `axis_twist` is 90° — the forearm axis and the wrist axis are perpendicular.

### axis_shift

**Where along the first joint axis does the common perpendicular meet it?**

The common perpendicular hits the first joint axis at a specific point. The distance from the joint's origin to that point is the `axis_shift`.

For a UR arm:

- The `axis_shift` at J1 is the height from the base to the shoulder — the distance along the vertical axis to where the shoulder axis attaches.
- The `axis_shift` at J4 is the **J4 wrist offset** — the distance along J4's axis to where P5 sits. This offset has a special significance: it determines whether the wrist is *spherical* or *offset*. We will define both terms in Part III.

### joint_angle

**How much has the joint rotated?**

This is the variable we solve for. It is the rotation of the joint about its own axis.

## 7. Why These Names

The classical names for these quantities describe the **link** — the rigid body between the joints. But the four quantities do not describe the link. They describe the **relationship between two joint axes**.

Consider the length of a link. A physical link is a solid object. If you pick it up and measure it, you measure from one end to the other. But the joint axes are not at the very ends of the link. They sit inside it, at the points where the "pin" of the joint passes through. The link extends a little past each axis — enough material to hold the pin, enough clearance to allow rotation.

So the physical length of the link and the distance between its two joint axes are **different numbers**. The link is longer than the gap between the axes. Naming the distance between axes "link length" invites the reader to picture the wrong thing.

The same applies to the twist. The physical link does not twist. The two **axes** are rotated relative to each other, and the link is the material that connects them.

Naming the quantities after the axes, not the links, is more accurate. It also makes the geometry visible: when you read `axis_gap`, you know it is a distance between axes; when you read `axis_twist`, you know it is an angle between axes.

## 8. The Two Pairs

The four quantities form two natural pairs:

**The "shape" pair** — `axis_gap` and `axis_twist`:
These describe how two consecutive axes are arranged relative to each other. Given the shape pair, you can picture the geometry of the joint pair: how far apart the axes are, and how they are rotated. These quantities are fixed by the robot's design; they do not change when the joint moves.

**The "attachment" pair** — `axis_shift` and `joint_angle`:
These describe where along the current axis the next joint attaches, and how much the current joint has rotated. Given the attachment pair, you can determine where the next joint sits in space. `axis_shift` is fixed; `joint_angle` is the variable.

## 9. What the URDF Tells Us

The URDF does not name these quantities. It describes each joint with a full 6-DOF origin transform and an axis direction. The four geometric quantities are **derived** from those:

- `axis_gap` is the distance between the current joint's axis and the next joint's axis.
- `axis_twist` is the angle between those two axis directions.
- `axis_shift` is the position along the current joint's axis where the common perpendicular meets it.
- `joint_angle` is the variable we solve for.

The derivation from URDF to these quantities is geometric, not conventional. It does not require placing frames in a canonical way. It reads the geometry that is already there.

This is why Hatch can support both UR-style and FR-style wrists with the same code: it never assumed a specific axis convention in the first place. It reads where the axes are, computes the four quantities, and proceeds.

## 10. What the URDF Describes, and What the Four Quantities Describe

The URDF tells us **what the robot looks like**. It says where each link is, where each joint sits, what shape each mesh has. It is a static description — a snapshot of the robot's geometry.

The four quantities tell us **how the robot moves**. They describe the relationships between joint axes: how far apart the axes are, how they are rotated, where along each axis the next joint attaches. From these, we can compute where any link will be, given the joint angles. They are a kinematic model — a description of the robot's motion, not its appearance.

Both are derived from the same URDF. But they serve different purposes. The URDF is what the robot *is*. The four quantities are what the robot *can do*.

The rest of this document works with the four quantities, not the raw URDF. The derivation that follows is a derivation of motion.

## 11. What Comes Next

We now have the vocabulary. In the next part, we use it to see the wrist — and to see why the spherical wrist and the offset wrist are the same thing, with one quantity set to zero.

---

# Part III: Working Backward from the Flange

## 1. The Solver Boundary

Before we solve anything, we must fix what we are solving for. The solver's input and output are entirely inside the **6R chain**: from the **true root** (the parent of the first movable revolute joint) to the **flange** (the child of the last movable revolute joint). Everything before the true root — the world, the mounting, the pedestal — and everything after the flange — the fixture, the tool, the TCP — is a fixed transform the user defines and the controller composes. The solver never sees it.

This is stated as a rule in `docs/CONSTITUTION.md` Part 3.5. In this document, it means:

- The pose we are given is the **flange pose in the true-root frame**.
- The distance we will call `d₆` is the distance from the **wrist center P5** to the **flange**, measured along the flange's Z axis. It is not the distance to the TCP.
- The quantities `d₆`, `wrist_offset`, `a₂`, `a₃`, `d₁` are all properties of the 6R chain, computed by the extraction layer from the URDF between the two boundaries.

## 2. Why Six Joints

When we use a robot arm, we want to place the flange at a specific position and with a specific orientation.

**Position** is three numbers: where the flange is, along three independent directions.

**Orientation** is three more numbers: how the flange is rotated, about three independent axes.

Together, that is **six numbers**. A flange pose has six degrees of freedom.

To control six degrees of freedom, we need six joints. Each joint contributes one independent motion. With six joints, we can independently control all six numbers. With fewer, we cannot reach every pose. With more, we have redundancy — more joints than we need, which is a different problem.

This is why industrial arms have six joints. It is not a convention. It is a counting argument.

## 3. The First Three Joints Reach a Point

To reach a specific **position** — ignoring orientation for a moment — we need only three joints.

Think of your own arm. You can reach a point in front of you by:

- Rotating your body (the base joint)
- Raising your upper arm (the shoulder joint)
- Bending your elbow (the elbow joint)

Three joints, three links, one point. The base rotates the whole arm. The shoulder and elbow position the hand in the vertical plane that the base has aimed.

Industrial arms work the same way. The first three joints — J1, J2, J3 — are arranged to reach any point within the arm's workspace. J1 rotates the arm plane. J2 and J3 position the end of the forearm within that plane.

The first three joints handle **position**.

## 4. Adding Orientation Changes the Problem

If we only wanted to reach a point, three joints would be enough. But we need to reach a point *with a specific orientation*.

The orientation is set by the last three joints — the wrist. J4, J5, and J6 rotate the flange so that it points in the desired direction.

But here is the difficulty: **the wrist's job depends on where the arm has placed it.**

If the arm places the wrist at a different point, the wrist has to work differently to achieve the same flange orientation. Position and orientation are not independent — they are coupled through the wrist.

This is why the problem is hard. If we could solve position first and orientation second, the solution would be simple. But the two are linked.

## 5. Solving in Reverse: Starting from the Flange

The way to untangle the coupling is to work **backward** from the flange.

The user specifies the flange pose. That is the input. Everything else follows from it.

**Step 1: The flange pose is given.** We know the position and orientation of the flange.

**Step 2: The flange's relationship to J6 is fixed.** The flange is attached to the last joint with a fixed transformation — known from the URDF. So we can apply the inverse of that transformation to find J6's pose.

**Step 3: J6's pose is now known.** We know both the position and the orientation of the last joint's frame.

**Step 4: J5 and J6 intersect at a point.** Call it P5. The position of P5 can be found from J6's pose and the geometry of the wrist.

This is the key: **the flange pose determines P5 completely.** P5 is fixed. It cannot move.

## 6. The Wrist Root and the Wrist-1 Link

Now we have P5 fixed. The question is: where is J4?

J4 is the beginning of the wrist. It sits at the end of the forearm. We call its position the **wrist root**.

Between J4 and P5, there is a rigid connection — the **wrist-1 link**. It connects the end of the forearm to the point where J5 and J6 intersect.

The length of the wrist-1 link is the **J4 wrist offset** — the perpendicular distance from J4's axis to P5.

## 7. The Circle

P5 is fixed. The wrist-1 link has fixed length. It is rigid.

So the only freedom left in the link is its **direction**. As J4 rotates, the link swings around P5. The far end of the link — the J4 end — traces a **circle**:

- **Center**: P5
- **Radius**: the J4 wrist offset, `wrist_offset`
- **Plane**: perpendicular to J4's axis direction

This circle is fixed. It does not change as we search for solutions. It is a property of the given flange pose.

**The wrist root — the position of J4 — must lie on this circle.**

## 8. The Spherical Wrist: When the Circle Has Radius Zero

If `wrist_offset` is zero, the wrist-1 link has length zero. J4 and P5 are the **same point**.

The circle has radius zero. It is not a circle at all — it is a single point.

This is the **spherical wrist**: J4, J5, and J6 all intersect at P5. The wrist root and the wrist center are the same point.

The spherical wrist is not a different design. It is the offset wrist with a zero-length wrist-1 link. The geometry is the same. The circle is just smaller.

## 9. The Arm Plane

Now consider the arm — J1, J2, and J3.

The upper arm and forearm move in a single vertical plane. The reason is that J2 and J3 are parallel: the `axis_twist` between them is zero. Because they are parallel, the two links they connect sweep through a common plane.

This plane is the **arm plane**. It contains:

- The base axis (J1)
- The upper arm
- The forearm
- **The wrist root (J4)**

The last point is crucial. J4 sits at the end of the forearm, and the forearm moves within the arm plane. So **J4 must lie on the arm plane**.

J1 rotates the arm plane about the base axis. The plane is not fixed; it is rotated by the `joint_angle` of J1. As J1 rotates, the plane sweeps around the base.

## 10. The Constraint

Now we have two facts:

1. **J4 must lie on the circle** traced by the wrist-1 link.
2. **J4 must lie on the arm plane.**

Therefore: **the circle and the arm plane must meet at a point, and that point is J4.**

This is the geometric heart of the solution. Everything else follows from it.

## 11. Tangency, Not Crossing

A circle and a plane can meet in three ways:

| Relationship | Result |
|---|---|
| Plane too far from circle's center | No intersection |
| Plane at distance exactly equal to radius | One point (tangency) |
| Plane closer than radius | Two points (crossing) |

Which case applies to our robot?

The answer comes from the wrist-1 link itself. The link extends from J4 to P5 **along J4's axis direction** — which is perpendicular to the arm plane. The link has fixed length: the J4 wrist offset.

So the perpendicular distance from P5 to the arm plane is exactly `wrist_offset` — the radius of the circle.

That is the **tangency** case. The circle and the plane meet at exactly one point. That point is J4.

**The arm plane must be tangent to the circle.**

## 12. Two Solutions, from Two Tangent Planes

The circle is fixed once the flange pose is fixed. The arm plane rotates about the base axis. For the arm to reach J4, the plane must be tangent to the circle.

There are exactly **two** tangent planes:

- One touching the circle on the near side — **shoulder left**
- One touching the circle on the far side — **shoulder right**

These correspond to the two solutions for J1's angle.

When the circle has radius zero (spherical wrist), the tangency condition becomes trivial: any plane through P5 is tangent to the point. The two tangent planes become the two planes through P5 — one pointing toward it, one pointing away. These are still two solutions, and they still correspond to shoulder left and shoulder right.

**The spherical wrist does not have fewer configurations than the offset wrist.** It has the same configuration structure. The difference is only in how the shoulder angle is computed — from tangency in the offset case, from projection in the spherical case.

## 13. What Comes Next

We have established the geometric foundation:

- The flange pose determines P5 completely.
- The wrist-1 link traces a circle around P5, of radius equal to the J4 wrist offset.
- The wrist root — the position of J4 — must lie on this circle and on the arm plane.
- The arm plane must be tangent to the circle.
- There are two tangent planes — shoulder left and shoulder right — giving two solutions for J1.
- The spherical wrist is the special case where the radius is zero.

In the next part, we solve J1 precisely, using the tangent-line construction. Then we move to the remaining joints.

## 14. What We Have Assumed

The derivation in this part works for a specific class of arms. It is worth being explicit about which class, now that you have seen the derivation.

**We assumed J2 and J3 are parallel.** Their `axis_twist` is zero. This is what makes the upper arm and forearm move in a single plane — what we called the arm plane. If J2 and J3 are not parallel, there is no arm plane, and the derivation does not apply.

**We assumed J5 and J6 intersect.** They meet at a point, which we called P5. This is what gives the wrist a well-defined center. If J5 and J6 do not intersect, there is no P5, and the derivation does not apply.

Most industrial arms are in this class. Not all. Some arms — particularly painting robots, welding robots, and some collaborative robots — use different arrangements for cable routing, safety, or structural reasons. These are not edge cases. They are different kinematic classes, and they require different solutions.

If you are working with a robot that violates these assumptions, Hatch's analytical solver will refuse it. That refusal is deliberate. It is better to say "this robot is not in the class I can solve" than to produce a solution that is subtly wrong.

---

# Part IV: Solving θ₁

## 1. What We Need to Find

In Part III, we established the geometric picture:

- The flange pose determines P5 completely.
- The wrist-1 link traces a circle around P5, of radius equal to `wrist_offset`.
- The arm plane must be tangent to this circle.
- The point of tangency is J4.

Now we need to turn this picture into a number: **the angle of J1**.

We will do this in three steps:

1. Compute P5 from the flange pose.
2. Find the two tangent planes to the circle.
3. Read off the angle of J1 for each tangent plane.

## 2. Computing P5

The flange pose is given as a 4×4 transformation matrix. We need to find P5 — the point where J5 and J6 intersect.

P5 is offset from the flange by the length of the last link, along the flange's Z axis. In matrix form:

**p₅ = p_flange − d₆ · a**

where:

- **p_flange** is the position part of the flange pose (the fourth column of the transformation matrix)
- **d₆** is the **flange offset** — the distance from P5 to the flange along the flange's Z axis
- **a** is the **flange approach vector** — the direction the flange's Z axis points, which is the third column of the rotation part of the flange pose

This is the same reverse transformation described in Part III, §5. The flange's pose relative to J6 is fixed; we undo it to find P5.

**Note:** If the flange is mounted with a rotation relative to J6 (not just a translation), the full inverse transformation must be applied, not just a translation. The extraction layer handles this; for the derivation, the translation form is sufficient to see the geometry.

## 3. The Arm Chain as a Planar Mechanism

The arm chain — J2, J3, J4 — is a planar mechanism:

- **J2** rotates the upper arm about an axis perpendicular to the plane.
- **J3** rotates the forearm about an axis parallel to J2.
- **J4** rotates the wrist about an axis parallel to J2 and J3.

The upper arm has length **a₂** — the `axis_gap` between J2 and J3.

The forearm has length **a₃** — the **distance from J3's axis to the wrist center P5**, measured in the arm plane.

**This is not the same as `axis_gap(2)`.** The `axis_gap` between J3 and J4 is the perpendicular distance between their axes. For a spherical wrist, J3 and J4 may intersect (`axis_gap(2) = 0`), but the forearm is still nonzero — it extends from J3 to P5, which is the same point as J4 for a spherical wrist. For an offset wrist, J3 and J4 are separated by `axis_gap(2)`, and P5 is further offset from J4 by `wrist_offset`. The forearm length is the total distance from J3 to P5, which includes both contributions.

The solver must compute **a₃** as the distance from J3's axis to P5, not as `axis_gap(2)`.

The shoulder is at the origin of the arm plane — where J2's axis sits. The distance from the base to the shoulder, along the base axis, is `d₁`.

So the planar problem is:

> Given a target point **(u₅, w₅)** for P5, find the angles θ₂, θ₃, θ₄ of a three-link chain with link lengths a₂ (J2 to J3), a₃ (J3 to P5), and zero (the last "link" is the rotation at J4, which does not move P5).

The third joint, J4, does not change P5's position — it only rotates the wrist about its own axis. So the **position** problem involves only J2 and J3. The **orientation** problem — determining θ₄ — comes afterward.

## 4. The Circle in the Horizontal Plane

In Part III, the circle was described in 3D: centered at P5, radius `wrist_offset`, in a plane perpendicular to J4's axis.

For solving J1, it is easier to work with the circle's **projection** onto the horizontal plane. This projection is where the tangency condition becomes visible.

Two facts about this projection:

**Fact 1:** The center of the projected circle is at **(r, φ)** in polar coordinates. That is, the projected circle is centered at the horizontal projection of P5.

**Fact 2:** The projected circle is a circle, not an ellipse, only in the special case where J4's axis is vertical. In general, the projection of a tilted circle is an ellipse.

For now, we will work with the simplified case — the circle projects to a circle. We will handle the general case at the end of this part.

The radius of the projected circle is still `wrist_offset`.

So in the horizontal plane, we have:

- A circle of radius **wrist_offset**, centered at **(r, φ)**.
- The base axis, which is a point at the origin.

## 5. The Tangent Lines

The arm plane, viewed from above, is a **line** through the origin. It is rotated by the angle of J1.

We need this line to be **tangent** to the circle.

A line through the origin that is tangent to a circle centered at distance **r** with radius **wrist_offset** makes an angle **α** with the line from the origin to the circle's center, where:

**sin α = wrist_offset / r**

This comes from the right triangle formed by:

- The origin
- The circle's center
- The tangent point

The tangent from an external point to a circle makes this angle with the line to the center. This is a standard result in Euclidean geometry.

Therefore:

**α = arcsin(wrist_offset / r)**

## 6. The Two Solutions for J1 (Offset Case)

The line from the origin to the circle's center is at angle **φ**. The tangent line is offset from this by **α**.

There are two tangent lines:

- One rotated by **+α** from φ — **shoulder left**
- One rotated by **−α** from φ — **shoulder right**

So the two solutions for J1, when `wrist_offset > 0`, are:

**θ₁_left  = φ + α**
**θ₁_right = φ − α**

Or, written out:

**θ₁ = atan2(y₅, x₅) ± arcsin(wrist_offset / r)**

## 7. The Reachability Condition

The formula contains a reachability condition. If **r < wrist_offset**, then:

**wrist_offset / r > 1**

and the arcsine is undefined. No tangent line exists.

Physically: if P5 is too close to the base axis, the circle surrounds the base axis entirely, and no line through the origin can be tangent to it. The arm cannot reach.

**The wrist center must be at least `wrist_offset` from the base axis.**

This is a real, physical constraint — and it is the reason a UR cannot reach points directly above its base in certain configurations.

## 8. The Spherical Case: `wrist_offset = 0`

When the wrist is spherical, the J4 wrist offset is zero. The circle has radius zero. It is a point.

The angle **α** becomes:

**α = arcsin(0 / r) = 0**

And the two solutions from the tangent construction become:

**θ₁ = φ + 0 = φ**
**θ₁ = φ − 0 = φ**

They coincide.

This is the correct behavior for a spherical wrist: the two tangent lines collapse to a single line through P5's projection. There is only one J1 angle that points the arm plane *at* P5.

But the spherical wrist still has **two shoulder solutions**. They do not come from the tangent angle — they come from the fact that the arm plane can point **toward** P5 or **away** from it.

A line through the origin at angle **φ** passes through P5's projection. A line through the origin at angle **φ + π** also passes through P5's projection — because a line is infinite in both directions. Both lines satisfy the constraint that the arm plane contains P5's projection. They are different configurations of the arm:

- **θ₁ = φ** — the arm plane points toward P5. The upper arm and forearm extend in the direction of P5.
- **θ₁ = φ + π** — the arm plane points away from P5. The arm reaches "backward" to put J4 at P5's projection.

These are the two shoulder solutions for the spherical case: **φ** and **φ + π**.

The formula **φ ± arcsin(wrist_offset / r)** with `wrist_offset = 0` gives only φ, missing the "pointing away" solution. The solver must add the second solution explicitly for the spherical case.

**This is not a special case in the algebra — it is a singularity in the limit.** As `wrist_offset → 0`, the two tangent lines of the offset case collapse to the single line θ₁ = φ. The "second" solution does not emerge from the tangent construction; it emerges from the symmetry of the line itself. The spherical case has an additional discrete symmetry — the arm plane can point either way — that is not present in the offset case (where the two tangent lines are genuinely distinct).

**The solver must handle the spherical case separately.** When `wrist_offset` is below a tolerance (say, 1e-6 m), the two shoulder solutions are **φ** and **φ + π**. When `wrist_offset` is above the tolerance, the two solutions are **φ ± α**.

This is **a branch between two exact regimes**, not a fallback. Both branches are exact. The tolerance is a numerical threshold between them, chosen small enough that the spherical formula is correct for robots whose geometry is spherical, and the offset formula is correct for robots whose geometry is offset. It is not a graceful degradation, and it does not violate the constitution's prohibition on silent fallbacks (Part 7).

**Joint limits must be checked modulo 2π.** The two solutions φ and φ + π differ by π, but both may be valid angles for J1 if the limits allow. The limit check must wrap the angle into the joint's range (or check both the angle and the angle ± 2π).

## 9. The General Case: A Tilted Circle

In the derivation above, we assumed that the circle projects to a circle in the horizontal plane. This is true only when J4's axis is vertical.

In general, J4's axis is tilted, and the circle's projection is an **ellipse**. The tangency condition becomes a tangency-to-ellipse problem, which is more complex.

There are two ways to handle this:

**Option A: Work in a rotated frame.** If we rotate the coordinate system so that J4's axis is vertical, the circle projects to a circle in the new frame, and the derivation above applies. Then we transform the resulting θ₁ back to the original frame. This is the cleaner approach.

**Option B: Work with the ellipse directly.** The tangency condition for an ellipse is a quadratic equation, and its solution is more involved. This is the approach used in the standard UR IK derivations.

For this document, we present Option A, because it preserves the geometric clarity of the tangent-line construction. The rotation is a technical detail handled in the code.

**However:** for the class of arms Hatch supports (J2 parallel to J3, J5 intersecting J6), J4's axis is always perpendicular to the arm plane by construction. The arm plane contains the base axis and J2's axis; J4's axis is perpendicular to both. So in the frame of the arm plane, J4's axis is always vertical *relative to the arm plane*, and the circle always projects to a circle *in that frame*. The tilt only matters when working in the base frame directly. The cleaner approach is to work in the arm-plane frame from the start, and Option B is not needed for the supported class.

## 10. The Order of Computation

We have now derived the formula for θ₁:

**Offset case (`wrist_offset > tol`):**
**θ₁ = atan2(y₅, x₅) ± arcsin(wrist_offset / r)**

with the reachability condition **r ≥ wrist_offset**.

**Spherical case (`wrist_offset ≤ tol`):**
**θ₁ ∈ { φ, φ + π }** where **φ = atan2(y₅, x₅)**.

The next steps, which will be covered in later parts, are:

1. Solve θ₅ using the lateral position of the flange relative to the arm plane.
2. Solve θ₃, θ₂ using the law of cosines in the arm plane.
3. Solve θ₄ and θ₆ from the wrist orientation.

The order matters: θ₁ must be solved first, because the remaining angles are defined relative to the arm plane that θ₁ determines.

## 11. What Comes Next

We have the first joint angle. In the next part, we will solve θ₅ — the wrist angle — using the same geometric picture. Once θ₁ and θ₅ are known, the remaining problem reduces to a planar one, and the last four angles follow from the law of cosines and the orientation constraints.

---

# Part V: Solving θ₅

## 1. What We Need to Find

In Part IV, we found θ₁. We now know which vertical plane the arm lies in — the **arm plane**.

The next joint to solve is **θ₅**, the second wrist joint. The reason we solve it before θ₂, θ₃, and θ₄ is that θ₅ is determined by the *position* of the flange, while θ₂, θ₃, and θ₄ require both position and orientation. Position information is available first, so we use it first.

## 2. What θ₅ Controls

J5 rotates about an axis that passes through P5 — the point where J5 and J6 intersect. This is true for both spherical and offset wrists: P5 is where J5 and J6 meet.

The wrist-1 link connects J4 to P5. It has length equal to `wrist_offset` — the J4 wrist offset.

The `axis_shift` at J5 — the distance from P5 to the next wrist joint along J5's axis — is **zero by construction** for the class of robots Hatch supports. J5 and J6 intersect at P5, and there is no offset between them along J5's axis.

This is not the same quantity as the J4 wrist offset. The J4 wrist offset is the perpendicular distance from J4's axis to P5. It is nonzero for UR and FR robots and zero for the Elfin. It is the radius of the circle in the tangency condition. When this document says "wrist offset" without qualification, it means the **J4 wrist offset**. The J5 offset is always zero for the supported class.

So P5 is *on* J5's axis. And it is also *on* J6's axis. J5 and J6 share this point.

θ₅ rotates the link that connects P5 to the next frame — the flange frame. This link has length **d₆** — the **flange offset**, the distance from P5 to the flange along the flange's Z axis.

## 3. The Lateral Offset from the Arm Plane

Here is the key observation.

In Part III, we established that P5 lies at perpendicular distance **wrist_offset** from the arm plane. This is the radius of the circle — the perpendicular offset that created the whole tangency problem.

But `wrist_offset` is only the offset of P5 from the arm plane. What about the flange itself?

The flange is offset from P5 by **d₆**, along the flange's Z axis. The flange's Z axis has some component **perpendicular to the arm plane**. This component, multiplied by d₆, gives the flange's additional lateral offset from the arm plane.

So the flange's total lateral offset from the arm plane is:

**offset_flange = ± wrist_offset + d₆ · cos(θ₅)**

The sign of the `wrist_offset` term depends on which shoulder branch we are on (left or right). We will fix the sign convention in §6.

## 4. Why cos(θ₅)?

The angle θ₅ tilts the flange's Z axis relative to J5's axis. When θ₅ = 0, the flange's Z axis is aligned with the direction away from the forearm — and the flange sits at maximum distance from the arm plane, at offset `wrist_offset + d₆`.

As θ₅ increases, the flange's Z axis tilts. The projection of the flange along the direction perpendicular to the arm plane decreases as **cos(θ₅)**. At θ₅ = π/2, the flange's Z axis is parallel to the arm plane, and the flange sits at offset exactly `wrist_offset` from the arm plane.

At θ₅ = π, the flange's Z axis points back toward the arm plane, and the flange sits at offset `wrist_offset − d₆` from the arm plane.

This is why the lateral offset from the arm plane is a function of cos(θ₅).

## 5. Computing the Flange's Lateral Offset

We know the flange position in the true-root frame. We know the arm plane — it was determined by θ₁ in Part IV.

The flange's lateral offset from the arm plane is the component of the flange's position that is perpendicular to the arm plane.

The arm plane contains the base Z axis and is rotated by θ₁ about it. Define the unit normal to the arm plane, pointing to the "left" of the arm plane (the direction from the arm plane toward the left-shoulder solution):

**n̂ = (−sin θ₁, cos θ₁, 0)**

A point **(x, y, z)** has signed perpendicular offset from the arm plane of:

**o(p) = p · n̂ = −p_x · sin(θ₁) + p_y · cos(θ₁)**

This is the signed distance from the point to the plane, measured in the direction perpendicular to the plane within the horizontal plane.

Call the flange's offset **o_flange**. We can compute it directly from the flange position and θ₁.

## 6. The Equation for θ₅

Now we have two expressions for the flange's lateral offset from the arm plane:

**From geometry:** o_flange = σ · wrist_offset + d₆ · cos(θ₅)

where σ = +1 for the right-shoulder branch (P5 on the +n̂ side of the arm plane) and σ = −1 for the left-shoulder branch (P5 on the −n̂ side).

**From computation:** o_flange = −x_flange · sin(θ₁) + y_flange · cos(θ₁)

Equating them:

**σ · wrist_offset + d₆ · cos(θ₅) = o_flange**

Solving for cos(θ₅):

**cos(θ₅) = (o_flange − σ · wrist_offset) / d₆**

**This is the single, unambiguous formula.** The sign convention is fixed by the definitions above:

- **σ = +1** for the right-shoulder branch, where P5 is on the +n̂ side of the arm plane.
- **σ = −1** for the left-shoulder branch, where P5 is on the −n̂ side.

The n̂ direction is `(−sin θ₁, cos θ₁, 0)`, fixed in §5.

## 7. The Two Solutions for θ₅

The equation cos(θ₅) = c has two solutions in the range [−π, π]:

**θ₅ = ± arccos(c)**

These correspond to the **wrist flip** and **wrist no-flip** configurations. The two values of θ₅ give the same flange position, because the flange's Z axis points in the same direction but the wrist is rotated differently.

**Tag convention:** θ₅ = +arccos(c) is tagged **no-flip**. θ₅ = −arccos(c) is tagged **flip**. The code uses these tags to select by continuity.

The reachability condition is:

**|c| ≤ 1**

that is:

**|o_flange − σ · wrist_offset| ≤ d₆**

If this condition is violated, no solution for θ₅ exists with the given θ₁ branch. The solver must try the other shoulder branch — but if the robot is in a valid configuration, at least one of the two shoulder branches will give a valid θ₅.

## 8. The Special Case: d₆ = 0

If the flange coincides with P5 — that is, the flange offset d₆ is zero — then the equation becomes:

**σ · wrist_offset = o_flange**

which is independent of θ₅. In this case, θ₅ cannot be determined from the position alone. Instead, θ₅ is determined from the **orientation** of the flange.

The angle between the flange's Z axis and the arm plane's normal is exactly θ₅ (by the definition of J5, since the flange's Z axis is J6's axis, tilted by θ₅ out of the arm plane):

**cos(θ₅) = a_flange · n̂ = −a_x · sin(θ₁) + a_y · cos(θ₁)**

where **a_flange** is the flange's approach vector (the third column of R_flange).

This is a **branch in the solver**, not a fallback. The branch condition is `d₆ > tol`: if true, solve θ₅ from position; if false, solve θ₅ from orientation. Both branches are exact. Both produce two solutions (`± arccos`).

**FR5 is the canonical example of the `d₆ = 0` case.** Its flange (`j6_Link`) is at P5, so d₆ = 0 and θ₅ must be solved from orientation. See the worked example in Part VII §4.

## 9. The Spherical Wrist Case

For a spherical wrist, `wrist_offset = 0`. The equation becomes:

**cos(θ₅) = o_flange / d₆**

The geometry is simpler — there is no offset contribution from the wrist-1 link — but the formula is the same. The spherical case is not a special case of the algebra; it is the offset case with `wrist_offset = 0`.

## 10. What Comes Next

We now have θ₁ and θ₅. The remaining joints are θ₂, θ₃, and θ₄ (the arm chain) and **θ₆** (the flange roll).

With θ₁ and θ₅ known, the problem becomes **planar**. The arm chain — J2, J3, J4 — moves entirely within the arm plane. We can project the problem onto that plane and solve for θ₂, θ₃, and θ₄ using the law of cosines and simple trigonometry.

Then θ₆ is extracted from the orientation of the flange — once θ₁ through θ₅ are known, the only remaining degree of freedom is J6's rotation about its own axis.

In the next part, we solve the arm chain.

---

# Part VI: Solving the Arm Chain

## 1. What We Need to Find

We now have θ₁ and θ₅. The arm plane is fixed — θ₁ determined it. And the flange's lateral position is fixed — θ₅ accounted for it.

What remains is the arm chain: **θ₂, θ₃, and θ₄**. These three joints move the wrist root — the position of J4 — within the arm plane.

Because θ₁ and θ₅ are known, the problem has collapsed from 3D to **2D**. We are now solving a planar problem: place J4 at a specific point within a plane, using a three-joint chain whose joints rotate about parallel axes.

This is the same problem your own arm solves when you reach for something in front of you — shoulder, elbow, and wrist roll, all moving within a single plane.

## 2. Reducing the Problem to a Point

Before solving the three joints, we need to know **where J4 must be** within the arm plane.

We know P5 — the wrist center — from the flange pose. We know θ₅. And we know the wrist-1 link has length **wrist_offset**, extending from J4 to P5.

The relationship between J4 and P5 is:

**p_J4 = p_P5 − wrist_offset · ẑ₄**

where **ẑ₄** is J4's axis direction — perpendicular to the arm plane, and whose orientation depends on θ₁ (not θ₅, as was incorrectly stated in an earlier version of this document; J4's axis direction is determined by the arm plane, and the arm plane is determined by θ₁).

So J4's position is determined once we know P5 and θ₁.

In the arm plane, J4 sits at some 2D coordinates. Call them **(u₄, w₄)** — where u is the direction along the plane's horizontal projection, and w is the vertical direction. These are the coordinates we need to reach with the first three joints.

## 3. The Arm Chain as a Planar Mechanism

The arm chain — J2, J3, J4 — is a planar mechanism:

- **J2** rotates the upper arm about an axis perpendicular to the plane.
- **J3** rotates the forearm about an axis parallel to J2.
- **J4** rotates the wrist about an axis parallel to J2 and J3.

The upper arm has length **a₂** — the `axis_gap` between J2 and J3.

The forearm has length **a₃** — the **distance from J3's axis to the wrist center P5**, measured in the arm plane. This is not `axis_gap(2)`. See Part IV §3 for the full statement of why.

The shoulder is at the origin of the arm plane — where J2's axis sits. The distance from the base to the shoulder, along the base axis, is `d₁`.

So the planar problem is:

> Given a target point **(u₄, w₄)** for J4, find the angles θ₂, θ₃, θ₄ of a three-link chain with link lengths a₂, a₃, and zero (the last "link" is the rotation at J4, which does not move J4).

The third joint, J4, does not change J4's position — it only rotates the wrist about its own axis. So the **position** problem involves only J2 and J3. The **orientation** problem — determining θ₄ — comes afterward.

## 4. The Two-Link Position Problem

For position, we only need J2 and J3.

Define the coordinates within the arm plane:

- **u** — the horizontal direction along the plane
- **w** — the vertical direction (parallel to the base axis)

The shoulder sits at **(0, d₁)** in these coordinates.

J4's target is at **(u₄, w₄)**.

The vector from the shoulder to J4's target is:

**Δu = u₄ − 0 = u₄**
**Δw = w₄ − d₁**

The distance is:

**D = √(Δu² + Δw²)**

## 5. The Law of Cosines for θ₃

The elbow angle θ₃ is determined by the law of cosines. The triangle has:

- Side **a₂** — the upper arm
- Side **a₃** — the forearm
- Side **D** — the distance from shoulder to J4's target

The angle at the elbow — the interior angle between the two links — satisfies:

**D² = a₂² + a₃² − 2 · a₂ · a₃ · cos(θ₃)**

Solving for cos(θ₃):

**cos(θ₃) = (a₂² + a₃² − D²) / (2 · a₂ · a₃)**

Then:

**θ₃ = ± arccos[(a₂² + a₃² − D²) / (2 · a₂ · a₃)]**

The two signs correspond to **elbow up** and **elbow down** — the two ways the forearm can reach the same target point. Tag convention: θ₃ = +arccos(...) is **elbow-up**, θ₃ = −arccos(...) is **elbow-down**.

**Reachability condition:** |D − a₂| ≤ a₃ ≤ D + a₂. Equivalently, the target must be within the workspace of the two-link chain. If not, no solution exists.

## 6. The Law of Sines for θ₂

Once θ₃ is known, θ₂ follows from the angle of the vector from shoulder to target.

Let:

**ψ = atan2(Δw, Δu)**

This is the angle of the line from shoulder to target, measured from the horizontal direction within the arm plane.

The angle between the upper arm and this line is:

**β = arcsin(a₃ · sin(θ₃) / D)**

This comes from the law of sines in the same triangle.

Then:

**θ₂ = ψ − β · sign(θ₃)**

**Sign convention note:** The exact form depends on how θ₂ is defined relative to the reference direction, and how θ₃ is defined (positive = elbow-up or elbow-down). The code's convention is authoritative; the formula above is one consistent choice.

## 7. θ₄ from Orientation

We now have θ₂ and θ₃. Together with θ₁ and θ₅ (from Parts IV and V), we know where every joint is positioned — except θ₄ and θ₆.

θ₄ is the wrist roll — the rotation of the wrist about the forearm's axis. It does not affect the position of anything; it only affects the orientation of the wrist.

To find θ₄, we use the **known orientation of the flange** and work backward.

The orientation of the flange is the product of rotations from J1 through J6:

**R_flange = R₁(θ₁) · R₂(θ₂) · R₃(θ₃) · R₄(θ₄) · R₅(θ₅) · R₆(θ₆)**

With θ₁, θ₂, θ₃, θ₅ known, and R_flange known from the input, we can solve for θ₄ and θ₆. The standard approach is to **isolate the wrist subchain** — the product R₄ · R₅ · R₆ — and solve for θ₄ and θ₆ from its entries.

The exact formulas depend on the geometry. We present them for the structure used by UR, FR, and Elfin robots — where the wrist is a Z-Y-Z rotation in the frame of the forearm.

## 8. The Z-Y-Z Wrist

In UR, FR, and Elfin robots, the wrist is a **Z-Y-Z rotation** — θ₄ rotates about Z, θ₅ about Y, and θ₆ about Z again, all in the frame of the forearm.

Define **R₀₃** as the rotation from the true-root frame to the frame after applying θ₁, θ₂, θ₃:

**R₀₃ = R(a₁, θ₁) · R(a₂, θ₂) · R(a₃, θ₃)**

where **aᵢ** is the direction of joint `i`'s axis in the true-root frame (available from `ArmGeometry.axes[i].direction`), and **R(a, θ)** is the Rodrigues rotation about `a` by `θ`. This definition is convention-independent: it uses the actual URDF axis directions, not canonical frame choices.

**Claim:** For an arm in the supported class, `R_wrist = R₀₃ᵀ · R_flange` has the form `R_z(θ₄) · R_y(θ₅) · R_z(θ₆)` in the frame whose Z axis is J4's direction.

**Proof sketch:** J4's axis is perpendicular to J5's axis, which is perpendicular to J6's axis, and J4 and J6 are parallel (for the supported class, both are perpendicular to the arm plane). The composition of rotations about these three axes, starting and ending with rotations about parallel axes, is a Z-Y-Z rotation. ∎

So we can write:

**R_wrist = R_z(θ₄) · R_y(θ₅) · R_z(θ₆)**

Given R_wrist, and using the **known** θ₅ from Part V, we extract θ₄ and θ₆ directly:

**θ₄ = atan2(R_wrist[1,2] / sin θ₅, R_wrist[0,2] / sin θ₅)**
**θ₆ = atan2(R_wrist[2,1] / sin θ₅, −R_wrist[2,0] / sin θ₅)**

Since θ₅ is already known from Step 3 of the algorithm (Part VII §1), we do not re-extract it. We use the known value, which is consistent with the branch we selected.

This is the cleaner form: the wrist decomposition is consistent with the θ₅ already chosen.

## 9. The Wrist Singularity

When θ₅ = 0 or π, the axes of J4 and J6 become parallel. This is the **wrist singularity** — the wrist has lost a degree of freedom, and θ₄ and θ₆ are no longer independent. Only their sum (or difference) is determined by the orientation.

The standard approach:

- Set θ₄ to a convenient value (usually its current value, to maintain continuity).
- Solve for θ₆ from the remaining matrix entries.

The code does this with a check on sin(θ₅): if it is near zero, the solver picks a convention (like θ₄ = 0) and proceeds. The user should be aware that the choice of θ₄ in this case is arbitrary — the flange achieves the correct orientation regardless, but θ₄ may not match the robot's previous state.

## 10. The Full Solution Set

Combining all the choices:

| Level | Choice | Tag values | Options |
|---|---|---|---|
| Shoulder | Which tangent plane (or which direction for spherical) | `left`, `right` | 2 |
| Elbow | Up or down | `up`, `down` | 2 |
| Wrist | Flip or no-flip | `flip`, `no-flip` | 2 |
| **Total** | | | **Up to 8** |

Each combination gives a distinct joint vector (θ₁, θ₂, θ₃, θ₄, θ₅, θ₆), and each vector places the flange at the same pose.

For the spherical wrist, the situation is the same — up to 8 configurations — but the "shoulder" tag values are computed differently, as discussed in Part IV §8.

## 11. Selection by Continuity, Not Proximity

The solver returns **all valid solutions, tagged with their branch**. It does not select one silently.

Each solution carries three tags:

- **shoulder**: `left` or `right`
- **elbow**: `up` or `down`
- **wrist**: `flip` or `no-flip`

The caller selects by **continuity** — preferring the solution whose branch tags match the current configuration. This is the only safe selection criterion for multi-waypoint motion: it keeps the arm in the same kinematic branch across waypoints, preventing the arm from swinging through space to reach a different branch.

Proximity — choosing the solution with the smallest joint-space distance — is a **fallback**, not the primary criterion. It can select a solution in a different branch if that branch happens to be closer in joint space, which is exactly the unsafe behavior the branch tagging is designed to prevent.

**Why tagging matters.** A solver returning one untagged solution cannot support safe multi-waypoint motion for 6-DOF arms. The caller has no way to know whether the solution it received is in the same branch as the previous waypoint's solution. If it is not, the arm will move through a different configuration, possibly crossing a singularity or exceeding a joint limit along the way. Tagging makes the branch structure explicit, and continuity-based selection keeps the motion safe.

This is the design lesson that Hatch's solver encodes: the branch is not a property of a solution in isolation, but of the *relationship* between consecutive solutions. Returning tags is what makes the relationship expressible.

## 12. What Comes Next

We have now derived all six joint angles:

- θ₁ from the tangency condition (Part IV)
- θ₅ from the lateral position of the flange (Part V)
- θ₂, θ₃ from the law of cosines in the arm plane (Part VI, §5–6)
- θ₄ from the wrist orientation (Part VI, §8)
- θ₆ from the wrist orientation (Part VI, §8)

The next part presents the complete algorithm — the order of computation and the handling of special cases — and worked examples on real robots.

---

# Part VII: The Full Algorithm and Worked Examples

## 1. The Algorithm, Stated as a Sequence

We have the geometric picture (Parts I–III), the derivation for θ₁ (Part IV), θ₅ (Part V), and the arm chain (Part VI). Now we state the algorithm as a sequence of steps — the order in which a computer would execute it.

The input is the **flange pose in the true-root frame**: a 4×4 homogeneous transformation `T_flange`. The output is a list of up to 8 tagged joint vectors.

**Step 0 — Precondition check.** Verify the arm is in the supported class:
- J2 and J3 are parallel (`axis_twist(1) ≈ 0` or `≈ π`).
- J5 and J6 intersect (`distance between axes[4] and axes[5] < tol`).
- The arm has exactly 6 revolute joints.

If any fails, refuse loudly. Do not approximate.

**Step 1 — Compute P5 (the wrist center).**

The flange is offset from P5 by the **flange offset** `d₆` along the flange's Z axis. The flange's Z axis is the third column of the flange rotation. So:

```
P5 = P_flange − d₆ · a_flange
```

where `P_flange` is the translation part of `T_flange`, `a_flange = T_flange[:3, 2]`, and `d₆` is the **flange offset** from `ArmGeometry`.

**Step 2 — Solve the shoulder (θ₁).**

Project P5 onto the horizontal plane:

```
r = √(x₅² + y₅²)
φ = atan2(y₅, x₅)
```

If `wrist_offset > tol` (offset wrist):

```
α = arcsin(wrist_offset / r)
θ₁_left  = φ + α     (σ = −1)
θ₁_right = φ − α     (σ = +1)
```

If `wrist_offset ≤ tol` (spherical wrist):

```
θ₁_right = φ          (σ = +1)
θ₁_left  = φ + π      (σ = −1)
```

For each θ₁, check `r ≥ wrist_offset` (offset case only). If violated, this branch is invalid.

**Step 3 — Solve the wrist angle (θ₅).**

For each θ₁ branch, with branch sign `σ = +1` for right-shoulder and `σ = −1` for left-shoulder, compute the flange's signed offset from the arm plane:

```
o_flange = −x_flange · sin(θ₁) + y_flange · cos(θ₁)
```

**Case `d₆ > tol` (position-determined θ₅):**

```
cos(θ₅) = (o_flange − σ · wrist_offset) / d₆
```

Check `|cos(θ₅)| ≤ 1`. If violated, this (θ₁) branch is invalid.

Two solutions: `θ₅ = ± arccos(cos(θ₅))`. Tag: `+` is **no-flip**, `−` is **flip**.

**Case `d₆ ≤ tol` (orientation-determined θ₅):**

```
cos(θ₅) = a_flange · n̂ = −a_x · sin(θ₁) + a_y · cos(θ₁)
```

where `a_flange = T_flange[:3, 2]` and `n̂ = (−sin θ₁, cos θ₁, 0)`.

Check `|cos(θ₅)| ≤ 1`. Two solutions: `θ₅ = ± arccos(cos(θ₅))`. Same tag convention.

**Step 4 — Solve the arm chain (θ₂, θ₃).**

For each (θ₁, θ₅), compute P5's coordinates in the arm plane:

```
u = x₅ · cos(θ₁) + y₅ · sin(θ₁)
w = z₅
```

Subtract the shoulder offset `d₁`:

```
Δu = u
Δw = w − d₁
D² = Δu² + Δw²
```

Law of cosines for θ₃:

```
cos(θ₃) = (D² − a₂² − a₃²) / (2 · a₂ · a₃)
```

Check `|cos(θ₃)| ≤ 1`. Two solutions: `θ₃ = ± arccos(cos(θ₃))`. Tag: `+` is **elbow-up**, `−` is **elbow-down**.

Law of sines for θ₂:

```
ψ = atan2(Δw, Δu)
β = arcsin(a₃ · sin(θ₃) / D)
θ₂ = ψ − β · sign(θ₃)
```

**Step 5 — Solve θ₄ and θ₆ (wrist roll and tool roll).**

Compute `R₀₃` from the base-frame axis directions:

```
R₀₃ = R(a₁, θ₁) · R(a₂, θ₂) · R(a₃, θ₃)
```

where `aᵢ = ArmGeometry.axes[i].direction` and `R(a, θ)` is the Rodrigues rotation about `a` by `θ`.

Isolate the wrist rotation:

```
R_wrist = R₀₃ᵀ · R_flange
```

Using the **known** θ₅ from Step 3, extract θ₄ and θ₆:

```
θ₄ = atan2(R_wrist[1,2] / sin θ₅, R_wrist[0,2] / sin θ₅)
θ₆ = atan2(R_wrist[2,1] / sin θ₅, −R_wrist[2,0] / sin θ₅)
```

**Singularity at θ₅ ≈ 0 or π:** J4 and J6 axes become parallel. Only θ₄ + θ₆ (or θ₄ − θ₆) is determined. Set θ₄ to its current value (or 0 if no current value), and solve for θ₆ from the remaining matrix entries. The choice of θ₄ is arbitrary; the flange reaches the correct orientation regardless.

**Step 6 — Assemble and tag.**

Combine the choices:

| Level | Tag values |
|---|---|
| Shoulder | `left`, `right` |
| Elbow | `up`, `down` |
| Wrist | `flip`, `no-flip` |

Each combination gives one solution. Up to 8 total.

**Step 7 — Filter by joint limits.**

For each solution, check each joint angle against its limits **modulo 2π**:

```
for each joint i:
    θ = solution[i]
    if not (lower_i ≤ θ ≤ upper_i):
        θ += 2π  (or − 2π)
        if not (lower_i ≤ θ ≤ upper_i):
            reject
```

This is critical: the two shoulder solutions φ and φ + π differ by π, and both may be valid if the limits allow. The modulo-2π check must be done for every joint.

**Step 8 — Return.**

Return all valid, tagged solutions. Do not select. The caller selects by continuity.

---

## 2. Worked Example: Elfin E15 Pro (Spherical Wrist)

**Robot parameters** (representative values — exact numbers from extraction):

- `d₁` (shoulder height) = 0.450 m
- `a₂` (upper arm) = 0.730 m
- `a₃` (forearm, J3 to P5) = 0.570 m
- `wrist_offset` = 0 (spherical)
- `d₆` (flange offset) = 0.100 m

**Target** (in true-root frame):

```
P_flange = (0.5, 0.3, 0.8)
R_flange = identity   (flange Z axis = +Z)
```

### Step 1 — Compute P5

```
a_flange = (0, 0, 1)
P5 = P_flange − d₆ · a_flange = (0.5, 0.3, 0.7)
```

### Step 2 — Solve θ₁

```
r = √(0.5² + 0.3²) = √0.34 ≈ 0.5831
φ = atan2(0.3, 0.5) ≈ 0.5404 rad
```

Spherical case (`wrist_offset = 0`):

```
θ₁_right = φ ≈ 0.5404    (σ = +1)
θ₁_left  = φ + π ≈ 3.6820  (σ = −1)
```

Check limits: θ₁_right is valid; θ₁_left = 3.6820 rad wraps to −2.6012 rad, which is within [−π, π] and likely valid.

### Step 3 — Solve θ₅

**For θ₁_right = 0.5404:**

```
o_flange = −0.5 · sin(0.5404) + 0.3 · cos(0.5404)
         = −0.5 · 0.5142 + 0.3 · 0.8577
         = −0.2571 + 0.2573
         ≈ 0.0002
```

```
cos(θ₅) = (0.0002 − (+1) · 0) / 0.100 = 0.002
θ₅ = ± arccos(0.002) ≈ ± 1.5688 rad
```

Two solutions: θ₅ = +1.5688 (**no-flip**), θ₅ = −1.5688 (**flip**).

**For θ₁_left = 3.6820:**

```
o_flange = −0.5 · sin(3.6820) + 0.3 · cos(3.6820)
         = −0.5 · (−0.5142) + 0.3 · (−0.8577)
         = 0.2571 − 0.2573
         ≈ −0.0002
```

```
cos(θ₅) = (−0.0002 − (−1) · 0) / 0.100 = −0.002
θ₅ = ± arccos(−0.002) ≈ ± 1.5728 rad
```

Same two solutions up to sign. The values are nearly equal to those from the right branch because the target is nearly on the arm plane (a degenerate-ish case chosen for clean numbers).

### Step 4 — Solve θ₂, θ₃

**For θ₁_right = 0.5404, θ₅ = +1.5688:**

```
u = 0.5 · cos(0.5404) + 0.3 · sin(0.5404) = 0.5 · 0.8577 + 0.3 · 0.5142 = 0.4289 + 0.1543 = 0.5832
w = 0.7
Δu = 0.5832
Δw = 0.7 − 0.450 = 0.250
D² = 0.5832² + 0.250² = 0.3401 + 0.0625 = 0.4026
D ≈ 0.6345
```

```
cos(θ₃) = (0.4026 − 0.730² − 0.570²) / (2 · 0.730 · 0.570)
        = (0.4026 − 0.5329 − 0.3249) / 0.8322
        = −0.4552 / 0.8322
        ≈ −0.5470
θ₃ = ± arccos(−0.5470) ≈ ± 2.1571 rad
```

For θ₃ = +2.1571 (**elbow-up**):

```
ψ = atan2(0.250, 0.5832) ≈ 0.4054 rad
β = arcsin(0.570 · sin(2.1571) / 0.6345) = arcsin(0.570 · 0.8325 / 0.6345)
  = arcsin(0.7477) ≈ 0.8457 rad
θ₂ = 0.4054 − 0.8457 = −0.4403 rad
```

For θ₃ = −2.1571 (**elbow-down**):

```
β = arcsin(0.570 · (−0.8325) / 0.6345) = −0.8457
θ₂ = 0.4054 − (−0.8457) = 1.2511 rad
```

### Step 5 — Solve θ₄, θ₆

For each (θ₁, θ₂, θ₃, θ₅), compute `R₀₃` and extract θ₄, θ₆.

For θ₁ = 0.5404, θ₂ = −0.4403, θ₃ = +2.1571, θ₅ = +1.5688:

```
R₀₃ = R(a₁, 0.5404) · R(a₂, −0.4403) · R(a₃, 2.1571)
R_wrist = R₀₃ᵀ · I = R₀₃ᵀ
```

With θ₅ = +1.5688, sin θ₅ ≈ 1.0, so:

```
θ₄ = atan2(R_wrist[1,2], R_wrist[0,2])
θ₆ = atan2(R_wrist[2,1], −R_wrist[2,0])
```

The specific numerical values depend on the axis directions from the URDF. The extraction layer provides them; the code computes the matrices.

### Step 6 — Assemble

Up to 8 solutions. For this target, all 8 may be valid (no branch violates limits or reachability). The full table:

| Shoulder | Elbow | Wrist | θ₁ | θ₂ | θ₃ | θ₅ |
|---|---|---|---|---|---|---|
| right | up | no-flip | 0.5404 | −0.4403 | +2.1571 | +1.5688 |
| right | up | flip | 0.5404 | −0.4403 | +2.1571 | −1.5688 |
| right | down | no-flip | 0.5404 | +1.2511 | −2.1571 | +1.5688 |
| right | down | flip | 0.5404 | +1.2511 | −2.1571 | −1.5688 |
| left | up | no-flip | 3.6820 | ... | ... | +1.5728 |
| left | up | flip | 3.6820 | ... | ... | −1.5728 |
| left | down | no-flip | 3.6820 | ... | ... | +1.5728 |
| left | down | flip | 3.6820 | ... | ... | −1.5728 |

Filter by joint limits (modulo 2π). Return all valid ones, tagged.

### What this example shows

1. **The algorithm is geometric.** Every step is a distance, an angle, or a projection.
2. **The spherical case is clean.** With `wrist_offset = 0`, the two shoulder solutions are φ and φ + π.
3. **The offset case follows the same steps.** Replace `{φ, φ + π}` with `{φ ± arcsin(wrist_offset/r)}`. The rest is identical.
4. **Tagging is essential.** The 8 solutions are distinct branches. Without tags, the caller cannot know which branch it received.

---

## 3. Worked Example: UR10 (Offset Wrist)

The UR10 is an **offset-wrist** arm: `wrist_offset ≈ 0.1157 m`, nonzero. The derivation is the same as the spherical case, with the offset formula in Step 2 and the sign convention σ in Step 3.

The full numerical example is deferred to the test suite (`tests/test_unified_ik.py`), where the UR10 parameters are read directly from the extraction layer rather than quoted here. The reason is that the UR10's `a₃` (forearm length, J3 → P5) is a computed quantity — the distance from J3's axis to P5 — and quoting an approximate value in the document would invite confusion with `axis_gap(2) = 0.5723`. The extraction layer computes `a₃` correctly; the test suite verifies it against FK.

**Summary of the UR10's expected behavior under the algorithm:**

- Step 2 produces two distinct θ₁ values (`φ ± α`), not `{φ, φ + π}`.
- The reachability condition `r ≥ wrist_offset` can fail for targets near the base axis, and the solver must report the failure.
- Step 3 uses the sign convention σ to distinguish the two shoulder branches.
- Steps 4–8 are identical to the spherical case.

The **test suite** (`tests/test_unified_ik.py`) is the authoritative worked example for the UR10. It uses the actual URDF parameters, reproduces the Elfin example above, validates on UR10 with the offset included, and validates on FR5 with the `d₆ = 0` case.

---

## 4. Worked Example: FR5 (Offset Wrist with `d₆ = 0`)

The FR5 is an **offset-wrist** arm with an important property: its flange coincides with the wrist center. The flange offset `d₆` is **zero**.

**Robot parameters** (computed from the URDF in the previous session):

- `d₁` (shoulder height) = 0.152 m
- `a₂` (upper arm, J2 → J3) = 0.425 m
- `a₃` (forearm, J3 → P5) ≈ 0.4080 m
- `wrist_offset` (J4 → P5) = 0.102 m
- `d₆` (flange offset, P5 → flange) = **0**

**Target** (in true-root frame):

```
P_flange = (0.5, 0.3, 0.7)
R_flange = Rx(0.4) · Rz(0.3)   (non-trivial orientation)
```

The orientation is chosen non-identity because with `d₆ = 0`, θ₅ must be solved from the flange's approach direction, not from its position. A non-identity orientation exercises this branch.

### Step 1 — Compute P5

Since `d₆ = 0`:

```
P5 = P_flange = (0.5, 0.3, 0.7)
```

### Step 2 — Solve θ₁

```
r = √(0.5² + 0.3²) = √0.34 ≈ 0.5831
φ = atan2(0.3, 0.5) ≈ 0.5404 rad
α = arcsin(wrist_offset / r) = arcsin(0.102 / 0.5831) ≈ 0.1758 rad
```

Offset case:

```
θ₁_left  = 0.5404 + 0.1758 = 0.7162 rad   (σ = −1)
θ₁_right = 0.5404 − 0.1758 = 0.3646 rad   (σ = +1)
```

Both branches are valid (`r ≥ wrist_offset`).

### Step 3 — Solve θ₅ (orientation-determined)

Since `d₆ = 0`, θ₅ cannot be determined from position. Use the orientation branch.

Compute the flange's approach vector:

```
R_flange = Rx(0.4) · Rz(0.3)
        ≈ [[0.9553, −0.2955, 0],
           [0.2722, 0.8796, −0.3894],
           [0.1150, 0.3719,  0.9211]]
a_flange = R_flange · (0,0,1) = (0, −0.3894, 0.9211)
```

**For θ₁_right = 0.3646:**

```
n̂ = (−sin(0.3646), cos(0.3646), 0) ≈ (−0.3565, 0.9343, 0)
cos(θ₅) = a_flange · n̂ = 0 · (−0.3565) + (−0.3894) · 0.9343 + 0.9211 · 0
        ≈ −0.3638
θ₅ = ± arccos(−0.3638) ≈ ± 1.9435 rad
```

**For θ₁_left = 0.7162:**

```
n̂ = (−sin(0.7162), cos(0.7162), 0) ≈ (−0.6566, 0.7543, 0)
cos(θ₅) = 0 · (−0.6566) + (−0.3894) · 0.7543 + 0.9211 · 0
        ≈ −0.2937
θ₅ = ± arccos(−0.2937) ≈ ± 1.8692 rad
```

Both θ₁ branches give valid θ₅ solutions. Tags: `+` is **no-flip**, `−` is **flip**.

### Step 4 — Solve θ₂, θ₃

For θ₁_right = 0.3646:

```
u = 0.5 · cos(0.3646) + 0.3 · sin(0.3646) ≈ 0.5 · 0.9343 + 0.3 · 0.3565 ≈ 0.5741
w = 0.7
Δu = 0.5741
Δw = 0.7 − 0.152 = 0.548
D² = 0.5741² + 0.548² ≈ 0.3296 + 0.3003 = 0.6299
D ≈ 0.7937
```

```
cos(θ₃) = (0.6299 − 0.425² − 0.4080²) / (2 · 0.425 · 0.4080)
        = (0.6299 − 0.1806 − 0.1665) / 0.3468
        = 0.2828 / 0.3468
        ≈ 0.8155
θ₃ = ± arccos(0.8155) ≈ ± 0.6162 rad
```

For θ₃ = +0.6162 (**elbow-up**):

```
ψ = atan2(0.548, 0.5741) ≈ 0.7626 rad
β = arcsin(0.4080 · sin(0.6162) / 0.7937) = arcsin(0.4080 · 0.5775 / 0.7937)
  ≈ arcsin(0.2969) ≈ 0.3016 rad
θ₂ = 0.7626 − 0.3016 = 0.4610 rad
```

For θ₃ = −0.6162 (**elbow-down**):

```
θ₂ = 0.7626 + 0.3016 = 1.0642 rad
```

### Step 5 — Solve θ₄, θ₆

For each (θ₁, θ₂, θ₃, θ₅), compute `R₀₃` and extract θ₄, θ₆. The Z-Y-Z structure holds for FR5 (J4 ⊥ J5 ⊥ J6, J4 ∥ J6).

For example, with θ₁ = 0.3646, θ₂ = 0.4610, θ₃ = 0.6162, θ₅ = +1.9435:

```
R₀₃ = R(a₁, 0.3646) · R(a₂, 0.4610) · R(a₃, 0.6162)
R_wrist = R₀₃ᵀ · R_flange
θ₄ = atan2(R_wrist[1,2] / sin(1.9435), R_wrist[0,2] / sin(1.9435))
θ₆ = atan2(R_wrist[2,1] / sin(1.9435), −R_wrist[2,0] / sin(1.9435))
```

Numerical values depend on the axis directions from the FR5 URDF.

### Step 6 — Assemble

Up to 8 solutions (2 shoulder × 2 elbow × 2 wrist). Filter by FR5 limits:

- J1: ±3.0543 rad
- J2: [−4.6251, 1.4835] rad
- J3: ±2.8274 rad
- J4: [−4.6251, 1.4835] rad
- J5: ±3.0543 rad
- J6: ±3.0543 rad

Both shoulder branches and both elbow branches are likely within limits for this target. Both wrist branches are valid (θ₅ is not near 0 or π, so no singularity). Up to 8 valid solutions.

### What this example shows

1. **The `d₆ = 0` case is a real case, not a corner to avoid.** FR5 exercises it. The solver has a branch for it, and the branch is exact.
2. **θ₅ can be determined from orientation when it cannot be determined from position.** The two solutions still exist; they come from the `± arccos` of the orientation equation.
3. **The offset-wrist arm-plane construction (`φ ± α` for shoulder) is used, not the spherical construction (`{φ, φ + π}`).** FR5's `wrist_offset = 0.102 m` is nonzero.
4. **All eight branches are reachable for a typical target.** The limits on the FR5 are wide enough that most branches survive.

---

## 5. What the Examples Do Not Show

The examples use simple targets to keep the numbers clean. Real targets have:

- Larger lateral offsets, making `o_flange` far from `σ · wrist_offset` and `cos(θ₅)` far from 0 or ±1.
- Configurations near singularities (θ₅ ≈ 0 or π), where θ₄ and θ₆ are not independent.
- Targets near the reachability boundary (`r ≈ wrist_offset` or `D ≈ |a₂ ± a₃|`), where the solver must decide between reporting a solution and reporting a reach failure.

The algorithm handles all of these. The examples show the structure; the test suite exercises the boundaries.

---

# Appendix A: DH Comparison

## A.1 What DH Encodes

The Denavit–Hartenberg convention describes the relationship between two consecutive joint axes with four numbers:

| DH symbol | Meaning |
|---|---|
| `d` | offset along the previous joint's axis |
| `a` | distance along the common perpendicular between axes |
| `α` | twist angle between the axes |
| `θ` | joint angle (the variable) |

For a serial arm, the DH table is a list of these four numbers per joint. The forward kinematics is the product of the DH transformation matrices.

## A.2 How the Four Geometric Quantities Relate to DH

| Hatch quantity | DH analog | Relationship |
|---|---|---|
| `axis_gap(i)` | `a` (link length) | Perpendicular distance between axes i and i+1. Same value, different name. |
| `axis_twist(i)` | `α` (link twist) | Angle between axes i and i+1. Same value, different name. |
| `axis_shift(i)` | `d` (link offset) | Distance along axis i to the common perpendicular with axis i+1. Same value for joints 1, 2, 3, 5, 6. **Different for joint 4** (see A.4). |
| `joint_angle(i)` | `θ` (joint variable) | The joint's rotation. Same value. |

For most joints, the mapping is a renaming. The four quantities Hatch uses are the same physical information DH encodes.

## A.3 Why Hatch Does Not Use DH

Three reasons:

**1. DH is lossy for the general case.** The DH convention assumes the joint frames are placed in a canonical way — the Z axis along the joint axis, the X axis along the common perpendicular to the next axis. This placement is possible for any serial arm, but it is not unique, and it becomes ambiguous when two consecutive axes are parallel (the common perpendicular is not unique).

**2. URDF gives more information.** A URDF joint origin is a full 6-DOF transform. DH compresses that transform into four scalars by assuming the frame placement. The compression loses information that the solver may need.

**3. DH requires a convention choice that the solver does not need.** The DH parameters depend on where the frames are placed. Hatch works directly with the joint axes as lines in space, so it does not need to choose a frame placement.

## A.4 The Joint 4 Offset: DH `d₄` vs. Geometric `wrist_offset`

The most important difference between DH and Hatch's geometric quantities is at **joint 4**.

In the standard UR DH table, `d₄` is labelled "the wrist offset." For UR10, `d₄ = 0.163941 m`. But the extraction code reports `wrist_offset = 0.1157 m`.

Why the difference? **Frame convention.** The DH `d₄` is measured along the J3 axis (in the DH frame placement), not along the J4 axis. The geometric `wrist_offset` is the **perpendicular distance from the J4 axis to P5**, which is a different quantity.

The solver uses the **geometric** value (0.1157 m for UR10), because the tangency condition in Part III is about the perpendicular distance from J4's axis to P5, not about a distance along J3's axis.

**This is why Hatch works from geometry, not DH.** The DH `d₄` is correct for the DH frames, but it is not the quantity the tangency condition needs. The solver would produce wrong answers if it used the DH value. It uses the geometric value, computed from the axes as lines in space.

## A.5 Summary

For the reader who knows DH: the four geometric quantities are the same physical information, renamed. For most joints, the values match. At joint 4, they differ because of the frame convention. Hatch uses the geometric value because that is what the tangency condition requires. The DH value is correct for DH frames but wrong for this solver.

---

# Appendix B: What Hatch Refuses to Do

## B.1 The Supported Class

Hatch's analytical IK solver applies to **arm-plane 6R arms with intersecting wrist axes**. Precisely:

1. **J2 and J3 are parallel** (`axis_twist(1) ≈ 0` or `≈ π`). This is what makes the upper arm and forearm move in a single plane — the arm plane. Without it, there is no arm plane, and the tangency construction fails.

2. **J5 and J6 intersect** (`distance between axes[4] and axes[5] < tol`). This is what gives the wrist a well-defined center P5. Without it, P5 does not exist, and the wrist cannot be decoupled from the arm.

3. **The arm has exactly 6 revolute joints.** Not 5, not 7. The counting argument (6 DOF = 6 joints) is what makes the problem solvable analytically.

Robots in this class: UR10, UR5, FR5, Elfin E15 Pro, and most industrial 6-axis arms. The class covers both spherical wrists (`wrist_offset = 0`) and offset wrists (`wrist_offset ≠ 0`).

## B.2 The Refusal Policy

If a robot violates any of the three conditions, Hatch **refuses loudly**. It does not:

- Approximate the geometry to fit the solver.
- Fall back to a numerical solver silently.
- Return a solution that is "close" but not exact.

The refusal is a **RuntimeError** with a message that states the violation:

```
Unsupported arm geometry: J2 and J3 are not parallel
(axis_twist(1) = 0.523 rad, expected 0 or π).
The analytical solver requires an arm-plane 6R arm.
```

The message names the violated condition and the measured value. The caller knows exactly what is wrong and can decide what to do — use a different solver, refuse the robot, or investigate the URDF.

## B.3 Why Refuse Instead of Approximate

**Because a plausible-looking wrong answer is worse than no answer.**

If the solver approximated a non-arm-plane robot, it would produce joint angles that place the flange near the target but not exactly at it. The error might be small for some poses and large for others. The caller would have no way to know which. In a safety-critical context — which is the context Hatch is designed for — that is unacceptable.

The refusal is the safe behavior. It says: "This robot is not in the class I can solve. I will not guess."

## B.4 What Is Not in the Class

**Robots with skew J2 and J3.** Some painting robots and welding robots use non-parallel shoulder-elbow axes for cable routing or structural reasons.

**Robots with non-intersecting J5 and J6.** Some collaborative robots and some 7-DOF arms have wrist geometries where J5 and J6 do not share a point.

**7-DOF arms.** Redundant arms have more than 6 joints. Hatch does not support them in v1.0.0.

**Parallel robots, delta robots, SCARA arms.** Different kinematic structures entirely.

For all of these, Hatch refuses. The word is **unified**, not **universal**. Unified across the supported class — spherical and offset wrists, UR and FR and Elfin — not universal across all robots.

## B.5 What Happens to the Caller

When the solver refuses, the caller has options:

1. **Use a different solver.** A numerical IK solver can handle a wider class of robots, at the cost of speed and determinism. Hatch does not ship one, but the interface allows one to be attached.

2. **Refuse the robot.** If the application requires the analytical solver, the robot must be in the supported class. The refusal is a signal that the robot is not suitable.

3. **Investigate the URDF.** Sometimes the violation is a URDF error, not a real geometric feature. The message names the measured value, so the caller can check whether the URDF is correct.

The refusal is not a dead end. It is a clear signal about what the robot is and what the solver can do.
