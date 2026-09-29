> **STATUS — WORK IN PROGRESS**
>
> This file is not yet internally consistent. Parts I–VI are the
> pre-correction derivation. Part VII and the appendices are the
> corrected version. The four corrections have not yet been
> integrated into Parts I–VI.
>
> Before treating any section as authoritative, check the list of
> known corrections in `docs/HANDOFF.md` section 3.
>
> Next session's task: integrate the corrections into Parts I–VI,
> delete the "# Confirmed — Writing Now" fragment, and produce a
> single coherent document.


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
- The `axis_shift` at J4 is the **wrist-1 offset** — the distance along J4's axis to where J5 attaches. This offset has a special significance: it determines whether the wrist is *spherical* or *offset*. We will define both terms in Part III.

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

## 9.5 What the URDF Describes, and What the Four Quantities Describe

The URDF tells us **what the robot looks like**. It says where each link is, where each joint sits, what shape each mesh has. It is a static description — a snapshot of the robot's geometry.

The four quantities tell us **how the robot moves**. They describe the relationships between joint axes: how far apart the axes are, how they are rotated, where along each axis the next joint attaches. From these, we can compute where any link will be, given the joint angles. They are a kinematic model — a description of the robot's motion, not its appearance.

Both are derived from the same URDF. But they serve different purposes. The URDF is what the robot *is*. The four quantities are what the robot *can do*.

The rest of this document works with the four quantities, not the raw URDF. The derivation that follows is a derivation of motion.

## 10. What Comes Next

We now have the vocabulary. In the next part, we use it to see the wrist — and to see why the spherical wrist and the offset wrist are the same thing, with one quantity set to zero.

---

# Part III: Working Backward from the TCP

## 1. Why Six Joints

When we use a robot arm, we want to place a tool at a specific position and with a specific orientation.

**Position** is three numbers: where the tool is, along three independent directions.

**Orientation** is three more numbers: how the tool is rotated, about three independent axes.

Together, that is **six numbers**. A tool's pose — its position and orientation — has six degrees of freedom.

To control six degrees of freedom, we need six joints. Each joint contributes one independent motion. With six joints, we can independently control all six numbers. With fewer, we cannot reach every pose. With more, we have redundancy — more joints than we need, which is a different problem.

This is why industrial arms have six joints. It is not a convention. It is a counting argument.

## 2. The First Three Joints Reach a Point

To reach a specific **position** — ignoring orientation for a moment — we need only three joints.

Think of your own arm. You can reach a point in front of you by:

- Rotating your body (the base joint)
- Raising your upper arm (the shoulder joint)
- Bending your elbow (the elbow joint)

Three joints, three links, one point. The base rotates the whole arm. The shoulder and elbow position the hand in the vertical plane that the base has aimed.

Industrial arms work the same way. The first three joints — J1, J2, J3 — are arranged to reach any point within the arm's workspace. J1 rotates the arm plane. J2 and J3 position the end of the forearm within that plane.

The first three joints handle **position**.

## 3. Adding Orientation Changes the Problem

If we only wanted to reach a point, three joints would be enough. But we need to reach a point *with a specific orientation*.

The orientation is set by the last three joints — the wrist. J4, J5, and J6 rotate the tool so that it points in the desired direction.

But here is the difficulty: **the wrist's job depends on where the arm has placed it.**

If the arm places the wrist at a different point, the wrist has to work differently to achieve the same tool orientation. Position and orientation are not independent — they are coupled through the wrist.

This is why the problem is hard. If we could solve position first and orientation second, the solution would be simple. But the two are linked.

## 4. Solving in Reverse: Starting from the TCP

The way to untangle the coupling is to work **backward** from the TCP.

The user specifies the TCP pose. That is the input. Everything else follows from it.

**Step 1: The TCP pose is given.** We know the position and orientation of the tool's working point.

**Step 2: The tool's relationship to J6 is fixed.** The tool is mounted to the last joint with a fixed transformation — known from the URDF. So we can apply the inverse of that transformation to find J6's pose.

**Step 3: J6's pose is now known.** We know both the position and the orientation of the last joint's frame.

**Step 4: J5 and J6 intersect at a point.** Call it P5. The position of P5 can be found from J6's pose and the geometry of the wrist.

This is the key: **the TCP pose determines P5 completely.** P5 is fixed. It cannot move.

## 5. The Wrist Root and the Wrist-1 Link

Now we have P5 fixed. The question is: where is J4?

J4 is the beginning of the wrist. It sits at the end of the forearm. We call its position the **wrist root**.

Between J4 and P5, there is a rigid connection — the **wrist-1 link**. It connects the end of the forearm to the point where J5 and J6 intersect.

The length of the wrist-1 link is the **`axis_shift` at J4** — the distance along J4's axis to where P5 sits.

## 6. The Circle

P5 is fixed. The wrist-1 link has fixed length. It is rigid.

So the only freedom left in the link is its **direction**. As J4 rotates, the link swings around P5. The far end of the link — the J4 end — traces a **circle**:

- **Center**: P5
- **Radius**: the `axis_shift` at J4
- **Plane**: perpendicular to J4's axis direction

This circle is fixed. It does not change as we search for solutions. It is a property of the given TCP pose.

**The wrist root — the position of J4 — must lie on this circle.**

## 7. The Spherical Wrist: When the Circle Has Radius Zero

If the `axis_shift` at J4 is zero, the wrist-1 link has length zero. J4 and P5 are the **same point**.

The circle has radius zero. It is not a circle at all — it is a single point.

This is the **spherical wrist**: J4, J5, and J6 all intersect at P5. The wrist root and the wrist center are the same point.

The spherical wrist is not a different design. It is the offset wrist with a zero-length wrist-1 link. The geometry is the same. The circle is just smaller.

## 8. The Arm Plane

Now consider the arm — J1, J2, and J3.

The upper arm and forearm move in a single vertical plane. The reason is that J2 and J3 are parallel: the `axis_twist` between them is zero. Because they are parallel, the two links they connect sweep through a common plane.

This plane is the **arm plane**. It contains:

- The base axis (J1)
- The upper arm
- The forearm
- **The wrist root (J4)**

The last point is crucial. J4 sits at the end of the forearm, and the forearm moves within the arm plane. So **J4 must lie on the arm plane**.

J1 rotates the arm plane about the base axis. The plane is not fixed; it is rotated by the `joint_angle` of J1. As J1 rotates, the plane sweeps around the base.

## 9. The Constraint

Now we have two facts:

1. **J4 must lie on the circle** traced by the wrist-1 link.
2. **J4 must lie on the arm plane.**

Therefore: **the circle and the arm plane must meet at a point, and that point is J4.**

This is the geometric heart of the solution. Everything else follows from it.

## 10. Tangency, Not Crossing

A circle and a plane can meet in three ways:

| Relationship | Result |
|---|---|
| Plane too far from circle's center | No intersection |
| Plane at distance exactly equal to radius | One point (tangency) |
| Plane closer than radius | Two points (crossing) |

Which case applies to our robot?

The answer comes from the wrist-1 link itself. The link extends from J4 to P5 **along J4's axis direction** — which is perpendicular to the arm plane. The link has fixed length: the `axis_shift` at J4.

So the perpendicular distance from P5 to the arm plane is exactly the `axis_shift` at J4 — the radius of the circle.

That is the **tangency** case. The circle and the plane meet at exactly one point. That point is J4.

**The arm plane must be tangent to the circle.**

## 11. Two Solutions, from Two Tangent Planes

The circle is fixed once the TCP pose is fixed. The arm plane rotates about the base axis. For the arm to reach J4, the plane must be tangent to the circle.

There are exactly **two** tangent planes:

- One touching the circle on the near side — **shoulder left**
- One touching the circle on the far side — **shoulder right**

These correspond to the two solutions for J1's angle.

When the circle has radius zero (spherical wrist), the tangency condition becomes trivial: any plane through P5 is tangent to the point. The two tangent planes become the two planes through P5 — one pointing toward it, one pointing away. These are still two solutions, and they still correspond to shoulder left and shoulder right.

**The spherical wrist does not have fewer configurations than the offset wrist.** It has the same configuration structure. The difference is only in how the shoulder angle is computed — from tangency in the offset case, from projection in the spherical case.

## 12. What Comes Next

We have established the geometric foundation:

- The TCP pose determines P5 completely.
- The wrist-1 link traces a circle around P5, of radius equal to the `axis_shift` at J4.
- The wrist root — the position of J4 — must lie on this circle and on the arm plane.
- The arm plane must be tangent to the circle.
- There are two tangent planes — shoulder left and shoulder right — giving two solutions for J1.
- The spherical wrist is the special case where the radius is zero.

In the next part, we solve J1 precisely, using the tangent-line construction. Then we move to the remaining joints.

## 13. What We Have Assumed

The derivation in this part works for a specific class of arms. It is worth being explicit about which class, now that you have seen the derivation.

**We assumed J2 and J3 are parallel.** Their `axis_twist` is zero. This is what makes the upper arm and forearm move in a single plane — what we called the arm plane. If J2 and J3 are not parallel, there is no arm plane, and the derivation does not apply.

**We assumed J5 and J6 intersect.** They meet at a point, which we called P5. This is what gives the wrist a well-defined center. If J5 and J6 do not intersect, there is no P5, and the derivation does not apply.

Most industrial arms are in this class. Not all. Some arms — particularly painting robots, welding robots, and some collaborative robots — use different arrangements for cable routing, safety, or structural reasons. These are not edge cases. They are different kinematic classes, and they require different solutions.

If you are working with a robot that violates these assumptions, Hatch's analytical solver will refuse it. That refusal is deliberate. It is better to say "this robot is not in the class I can solve" than to produce a solution that is subtly wrong.

---

# Part IV: Solving θ₁

## 1. What We Need to Find

In Part III, we established the geometric picture:

- The TCP pose determines P5 completely.
- The wrist-1 link traces a circle around P5, of radius equal to the `axis_shift` at J4.
- The arm plane must be tangent to this circle.
- The point of tangency is J4.

Now we need to turn this picture into a number: **the angle of J1**.

We will do this in three steps:

1. Compute P5 from the TCP pose.
2. Find the two tangent planes to the circle.
3. Read off the angle of J1 for each tangent plane.

## 2. Computing P5

The TCP pose is given as a 4×4 transformation matrix. We need to find P5 — the point where J5 and J6 intersect.

P5 is offset from the TCP by the length of the last link, along the tool's approach direction. In matrix form:

**p₅ = p_TCP − d₆ · a**

where:

- **p_TCP** is the position part of the TCP pose (the fourth column of the transformation matrix)
- **d₆** is the `axis_shift` at J6 — the distance from P5 to the TCP
- **a** is the **approach vector** — the direction the tool points, which is the third column of the rotation part of the TCP pose

This is the same reverse transformation described in Part III, §4. The tool's pose relative to J6 is fixed; we undo it to find P5.

**Note:** If the tool is mounted with a rotation (not just a translation), the full inverse transformation must be applied, not just a translation. We will handle the general case in the code, but for the derivation, the translation form is sufficient to see the geometry.

## 3. The Horizontal Projection

We now have P5 as a point in 3D space. Let its coordinates be:

**p₅ = (x₅, y₅, z₅)**

The arm plane contains the base axis — the vertical Z axis through the origin. We need to know where P5 sits **relative to that axis**.

Project P5 onto the horizontal plane (the XY plane):

**r = √(x₅² + y₅²)**

This is the horizontal distance from the base axis to P5.

We also need the **direction** from the base axis to P5's projection:

**φ = atan2(y₅, x₅)**

This is the angle of the line from the origin to P5's horizontal projection, measured from the positive X axis.

Together, **r** and **φ** describe P5's horizontal position completely.

## 4. The Circle in the Horizontal Plane

In Part III, the circle was described in 3D: centered at P5, radius equal to the `axis_shift` at J4, in a plane perpendicular to J4's axis.

For solving J1, it is easier to work with the circle's **projection** onto the horizontal plane. This projection is where the tangency condition becomes visible.

Two facts about this projection:

**Fact 1:** The center of the projected circle is at **(r, φ)** in polar coordinates. That is, the projected circle is centered at the horizontal projection of P5.

**Fact 2:** The projected circle is a circle, not an ellipse, only in the special case where J4's axis is vertical. In general, the projection of a tilted circle is an ellipse.

For now, we will work with the simplified case — the circle projects to a circle. We will handle the general case at the end of this part.

The radius of the projected circle is still the `axis_shift` at J4. Call it **d₄**.

So in the horizontal plane, we have:

- A circle of radius **d₄**, centered at **(r, φ)**.
- The base axis, which is a point at the origin.

## 5. The Tangent Lines

The arm plane, viewed from above, is a **line** through the origin. It is rotated by the angle of J1.

We need this line to be **tangent** to the circle.

A line through the origin that is tangent to a circle centered at distance **r** with radius **d₄** makes an angle **α** with the line from the origin to the circle's center, where:

**sin α = d₄ / r**

This comes from the right triangle formed by:

- The origin
- The circle's center
- The tangent point

The tangent from an external point to a circle makes this angle with the line to the center. This is a standard result in Euclidean geometry.

Therefore:

**α = arcsin(d₄ / r)**

## 6. The Two Solutions for J1

The line from the origin to the circle's center is at angle **φ**. The tangent line is offset from this by **α**.

There are two tangent lines:

- One rotated by **+α** from φ — **shoulder left**
- One rotated by **−α** from φ — **shoulder right**

So the two solutions for J1 are:

**θ₁ = φ + α**
**θ₁ = φ − α**

Or, written out:

**θ₁ = atan2(y₅, x₅) ± arcsin(d₄ / r)**

## 7. The Reachability Condition

The formula contains a reachability condition. If **r < d₄**, then:

**d₄ / r > 1**

and the arcsine is undefined. No tangent line exists.

Physically: if P5 is too close to the base axis, the circle surrounds the base axis entirely, and no line through the origin can be tangent to it. The arm cannot reach.

**The wrist center must be at least d₄ from the base axis.**

This is the UR analog of the "wrist center out of reach" check in the spherical-wrist solver. It is a real, physical constraint — and it is the reason a UR cannot reach points directly above its base in certain configurations.

## 8. The Spherical Case: d₄ = 0

When the wrist is spherical, the `axis_shift` at J4 is zero. The circle has radius zero. It is a point.

The angle **α** becomes:

**α = arcsin(0 / r) = 0**

And the two solutions for J1 become:

**θ₁ = φ + 0 = φ**
**θ₁ = φ − 0 = φ**

They coincide.

This is the correct behavior for a spherical wrist: the two tangent lines collapse to a single line through P5's projection. There is only one J1 angle that points the arm plane at P5.

But this seems to contradict what we said in Part III, §11: that the spherical wrist still has two shoulder solutions.

The resolution is that the two shoulder solutions for the spherical case do not come from the tangent angle. They come from the fact that the arm plane can point **toward** P5 or **away** from it. These are two different lines through the origin:

- One at angle **φ** (pointing toward P5)
- One at angle **φ + π** (pointing away)

Both pass through P5's projection (since the line is infinite in both directions). Both satisfy the constraint. These are the two shoulder solutions.

So for the spherical case:

**θ₁ = φ   or   θ₁ = φ + π**

The formula **φ ± arcsin(d₄ / r)** with d₄ = 0 gives only φ, missing the "pointing away" solution. The code must add the second solution explicitly for the spherical case.

This is an important detail: **the formula for offset wrists does not automatically reduce to the formula for spherical wrists.** The spherical case has an additional symmetry — the arm plane can point either way — that is not captured by the tangent-line construction.

## 9. The General Case: A Tilted Circle

In the derivation above, we assumed that the circle projects to a circle in the horizontal plane. This is true only when J4's axis is vertical.

In general, J4's axis is tilted, and the circle's projection is an **ellipse**. The tangency condition becomes a tangency-to-ellipse problem, which is more complex.

There are two ways to handle this:

**Option A: Work in a rotated frame.** If we rotate the coordinate system so that J4's axis is vertical, the circle projects to a circle in the new frame, and the derivation above applies. Then we transform the resulting θ₁ back to the original frame. This is the cleaner approach.

**Option B: Work with the ellipse directly.** The tangency condition for an ellipse is a quadratic equation, and its solution is more involved. This is the approach used in the standard UR IK derivations.

For the document, we will present Option A, because it preserves the geometric clarity of the tangent-line construction. The rotation is a technical detail that can be hidden in the code.

## 10. The Order of Computation

We have now derived the formula for θ₁:

**θ₁ = atan2(y₅, x₅) ± arcsin(d₄ / r)**

with the reachability condition **r ≥ d₄**, and with the spherical case **d₄ = 0** handled separately (adding the **φ + π** solution).

The next steps, which will be covered in later parts, are:

1. Solve θ₅ using the position of P5 in the arm frame.
2. Solve θ₆ using the orientation of the tool.
3. Solve θ₃, θ₂, θ₄ using the arm plane and the law of cosines.

The order matters: θ₁ must be solved first, because the remaining angles are defined relative to the arm plane that θ₁ determines.

## 11. What Comes Next

We have the first joint angle. In the next part, we will solve θ₅ — the wrist angle — using the same geometric picture. Once θ₁ and θ₅ are known, the remaining problem reduces to a planar one, and the last four angles follow from the law of cosines and the orientation constraints.

---

# Part V: Solving θ₅

## 1. What We Need to Find

In Part IV, we found θ₁. We now know which vertical plane the arm lies in — the **arm plane**.

The next joint to solve is **θ₅**, the second wrist joint. The reason we solve it before θ₂, θ₃, and θ₄ is that θ₅ is determined by the *position* of the TCP, while θ₂, θ₃, and θ₄ require both position and orientation. Position information is available first, so we use it first.

## 2. What θ₅ Controls

J5 rotates about an axis that passes through P5 — the point where J5 and J6 intersect. This is true for both spherical and offset wrists: P5 is where J5 and J6 meet.

The wrist-1 link connects J4 to P5. It has length equal to the `axis_shift` at J4 (which we have been calling **d₄**).

The `axis_shift` at J5 — the distance from P5 to the next wrist joint — is typically zero for both UR and FR robots. In the language of Part II, the `axis_shift` at J5 is zero. J5 and J6 intersect at P5, and there is no offset between them along J5's axis.

So P5 is *on* J5's axis. And it is also *on* J6's axis. J5 and J6 share this point.

θ₅ rotates the link that connects P5 to the next frame — the J6 frame. This link has length equal to the `axis_shift` at J6, which is **d₆** — the distance from P5 to the TCP.

## 3. The Lateral Offset from the Arm Plane

Here is the key observation.

In Part III, we established that P5 lies at distance **d₄** from the arm plane. This is the radius of the circle — the perpendicular offset that created the whole tangency problem.

But d₄ is only the offset of P5 from the arm plane. What about the TCP itself?

The TCP is offset from P5 by **d₆**, along the approach direction. The approach direction has some component **perpendicular to the arm plane**. This component, multiplied by d₆, gives the TCP's additional lateral offset from the arm plane.

So the TCP's total lateral offset from the arm plane is:

**offset_TCP = d₄ + d₆ · cos(θ₅)**

or with a sign, depending on the convention:

**offset_TCP = d₄ − d₆ · cos(θ₅)**

The precise sign depends on how θ₅ is defined — specifically, which direction is considered "positive" rotation. We will not commit to the sign here; the geometry is what matters.

## 4. Why cos(θ₅)?

The angle θ₅ tilts J6's axis relative to J5's axis. When θ₅ = 0, the J6 axis is aligned with the direction away from the forearm — and the TCP sits at maximum distance from the arm plane (offset = d₄ + d₆).

As θ₅ increases, the J6 axis tilts. The projection of the tool along the direction perpendicular to the arm plane decreases as **cos(θ₅)**. At θ₅ = π/2, the tool is parallel to the arm plane, and the TCP sits exactly at the arm plane's offset d₄ from P5.

At θ₅ = π, the tool points back toward the arm plane, and the TCP sits at offset **d₄ − d₆** from the arm plane.

This is why the lateral offset from the arm plane is a function of cos(θ₅).

## 5. Computing the TCP's Lateral Offset

We know the TCP position in world coordinates. We know the arm plane — it was determined by θ₁ in Part IV.

The TCP's lateral offset from the arm plane is the component of the TCP's position that is perpendicular to the arm plane.

The arm plane contains the base Z axis and is rotated by θ₁ about it. A point **(x, y, z)** has perpendicular offset from this plane of:

**offset = −x · sin(θ₁) + y · cos(θ₁)**

This is the signed distance from the point to the plane, measured in the direction perpendicular to the plane within the horizontal plane.

Call this value **o_TCP**. We can compute it directly from the TCP position and θ₁.

## 6. The Equation for θ₅

Now we have two expressions for the TCP's lateral offset from the arm plane:

- **From geometry:** o_TCP = d₄ − d₆ · cos(θ₅) (or with the opposite sign)
- **From computation:** o_TCP = −x_TCP · sin(θ₁) + y_TCP · cos(θ₁)

Equating them:

**d₄ − d₆ · cos(θ₅) = −x_TCP · sin(θ₁) + y_TCP · cos(θ₁)**

Solving for cos(θ₅):

**cos(θ₅) = [d₄ − (−x_TCP · sin(θ₁) + y_TCP · cos(θ₁))] / d₆**

Or, more compactly:

**cos(θ₅) = (d₄ − o_TCP) / d₆**

## 7. The Two Solutions for θ₅

The equation cos(θ₅) = c has two solutions in the range [−π, π]:

**θ₅ = ± arccos(c)**

These correspond to the **wrist flip** and **wrist no-flip** configurations. The two values of θ₅ give the same TCP position, because the tool points in the same direction but the wrist is rotated differently.

The reachability condition is:

**|c| ≤ 1**

that is:

**|d₄ − o_TCP| ≤ d₆**

If this condition is violated, no solution for θ₅ exists with the given θ₁. The solver must try the other value of θ₁ (shoulder left/right) — but if the robot is in a valid configuration, at least one of the two θ₁ values will give a valid θ₅.

## 8. The Special Case: d₆ = 0

If the tool has no length — that is, the TCP is at P5 itself — then the equation becomes:

**d₄ − o_TCP = 0**

which is independent of θ₅. In this case, θ₅ cannot be determined from the position alone, and must be extracted from the orientation instead. This is a degenerate case, and most real robots have d₆ > 0.

## 9. The Spherical Wrist Case

For a spherical wrist, d₄ = 0. The equation becomes:

**cos(θ₅) = −o_TCP / d₆**

The geometry is simpler — there is no offset contribution from the wrist-1 link — but the formula is the same. The spherical case is not a special case of the algebra; it is the offset case with d₄ = 0.

## 10. What Comes Next

We now have θ₁ and θ₅. The remaining joints are θ₂, θ₃, and θ₄ (the arm chain) and **θ₆** (the tool roll).

With θ₁ and θ₅ known, the problem becomes **planar**. The arm chain — J2, J3, J4 — moves entirely within the arm plane. We can project the problem onto that plane and solve for θ₂, θ₃, and θ₄ using the law of cosines and simple trigonometry.

Then θ₆ is extracted from the orientation of the tool — once θ₁ through θ₅ are known, the only remaining degree of freedom is J6's rotation about its own axis.

In the next part, we solve the arm chain.

---

# Part VI: Solving the Arm Chain

## 1. What We Need to Find

We now have θ₁ and θ₅. The arm plane is fixed — θ₁ determined it. And the tool's lateral position is fixed — θ₅ accounted for it.

What remains is the arm chain: **θ₂, θ₃, and θ₄**. These three joints move the wrist root — the position of J4 — within the arm plane.

Because θ₁ and θ₅ are known, the problem has collapsed from 3D to **2D**. We are now solving a planar problem: place J4 at a specific point within a plane, using a three-joint chain whose joints rotate about parallel axes.

This is the same problem your own arm solves when you reach for something in front of you — shoulder, elbow, and wrist roll, all moving within a single plane.

## 2. Reducing the Problem to a Point

Before solving the three joints, we need to know **where J4 must be** within the arm plane.

We know P5 — the wrist center — from the TCP pose. We know θ₅. And we know the wrist-1 link has length **d₄**, extending from J4 to P5.

The relationship between J4 and P5, in the arm plane, is:

**p_J4 = p_P5 − d₄ · ẑ₄**

where **ẑ₄** is J4's axis direction — which is perpendicular to the arm plane, and whose orientation depends on θ₅.

So J4's position is determined once we know P5 and θ₅.

In the arm plane, J4 sits at some 2D coordinates. Call them **(u₄, w₄)** — where u is the direction along the plane's horizontal projection, and w is the vertical direction. These are the coordinates we need to reach with the first three joints.

## 3. The Arm Chain as a Planar Mechanism

The arm chain — J2, J3, J4 — is a planar mechanism:

- **J2** rotates the upper arm about an axis perpendicular to the plane.
- **J3** rotates the forearm about an axis parallel to J2.
- **J4** rotates the wrist about an axis parallel to J2 and J3.

The upper arm has length **a₂** — the `axis_gap` between J2 and J3.

The forearm has length **a₃** — the `axis_gap` between J3 and J4.

The shoulder is at the origin of the arm plane — where J2's axis sits. The distance from the base to the shoulder, along the base axis, is the `axis_shift` at J1. Call it **d₁**.

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

The two signs correspond to **elbow up** and **elbow down** — the two ways the forearm can reach the same target point.

**Reachability condition:** |D − a₂| ≤ a₃ ≤ D + a₂. Equivalently, the target must be within the workspace of the two-link chain. If not, no solution exists.

## 6. The Law of Sines for θ₂

Once θ₃ is known, θ₂ follows from the angle of the vector from shoulder to target.

Let:

**ψ = atan2(Δw, Δu)**

This is the angle of the line from shoulder to target, measured from the horizontal direction within the arm plane.

The angle between the upper arm and this line is:

**β = arcsin(a₃ · sin(θ₃) / D)**

This comes from the law of sines in the same triangle — the ratio of the side opposite the angle to the sine of that angle is constant.

Then:

**θ₂ = ψ − β**

or with the sign of θ₃ chosen appropriately:

**θ₂ = ψ − β · sign(θ₃)**

**Sign convention note:** The exact form depends on how θ₂ is defined (relative to which reference direction) and how θ₃ is defined (positive elbow-up or positive elbow-down). The code's convention is what matters for implementation.

## 7. θ₄ from Orientation

We now have θ₂ and θ₃. Together with θ₁ and θ₅ (from Parts IV and V), we know where every joint is positioned — except θ₄ and θ₆.

θ₄ is the wrist roll — the rotation of the wrist about the forearm's axis. It does not affect the position of anything; it only affects the orientation of the wrist.

To find θ₄, we use the **known orientation of the tool** (from the TCP pose) and work backward.

The orientation of the tool is the product of rotations from J1 through J6:

**R_TCP = R₁(θ₁) · R₂(θ₂) · R₃(θ₃) · R₄(θ₄) · R₅(θ₅) · R₆(θ₆)**

With θ₁, θ₂, θ₃, θ₅ known, and R_TCP known from the TCP pose, we can solve for θ₄ and θ₆. The two unknowns are coupled, and their extraction depends on the joint axes' directions and the frame conventions.

The standard approach is to **isolate the wrist subchain** — the product R₄ · R₅ · R₆ — and solve for θ₄ and θ₆ from its entries.

For a wrist where J4, J5, and J6 are rotations about three non-parallel axes, the extraction gives:

- θ₄ from entries of the rotation matrix involving the wrist roll axis
- θ₆ from other entries involving the tool axis

The exact formulas depend on the geometry. Rather than derive them generically here, we will present them for the specific structure used by UR and FR robots — where the wrist is a Z-Y-Z rotation in the frame of the forearm.

## 8. The Z-Y-Z Wrist

In UR and FR robots, the wrist is a **Z-Y-Z rotation** — θ₄ rotates about Z, θ₅ about Y, and θ₆ about Z again, all in the frame of the forearm.

This means the wrist rotation matrix is:

**R_wrist = R_z(θ₄) · R_y(θ₅) · R_z(θ₆)**

Given R_wrist (computed from R_TCP after dividing out the arm chain's rotations), we can extract θ₄ and θ₆ from its entries:

**θ₄ = atan2(R_wrist[1,2], R_wrist[0,2])**
**θ₆ = atan2(R_wrist[2,1], −R_wrist[2,0])**

with the singularity at θ₅ = 0 handled separately (see §9).

## 9. The Wrist Singularity

When θ₅ = 0 or π, the axes of J4 and J6 become parallel. This is the **wrist singularity** — the wrist has lost a degree of freedom, and θ₄ and θ₆ are no longer independent. Only their sum (or difference) is determined by the orientation.

The standard approach:

- Set θ₄ to a convenient value (usually its current value, to maintain continuity).
- Solve for θ₆ from the remaining matrix entries.

The code does this with a check on sin(θ₅): if it's near zero, the solver picks a convention (like θ₄ = 0) and proceeds. The user should be aware that the choice of θ₄ in this case is arbitrary — the tool achieves the correct orientation regardless, but θ₄ may not match the robot's previous state.

## 10. The Full Solution Set

Combining all the choices:

| Level | Choice | Options |
|---|---|---|
| Shoulder | Which tangent plane | 2 |
| Elbow | Up or down | 2 |
| Wrist | Flip or no-flip | 2 |
| **Total** | | **Up to 8** |

Each combination gives a distinct joint vector (θ₁, θ₂, θ₃, θ₄, θ₅, θ₆), and each vector places the tool at the same pose.

For the spherical wrist, the situation is the same — 8 configurations — but the "shoulder" choice is computed differently, as we discussed in Part IV.

## 11. Which Solution to Use

When multiple solutions exist, we need to select one. The standard criterion is **proximity to the current configuration** — choose the solution whose joint angles are closest to the robot's present state.

This is done by computing a weighted distance:

**score = Σᵢ wᵢ · (θᵢ − θᵢ_current)²**

where the wᵢ are weights reflecting the relative importance of each joint. Larger weights mean the solver prefers not to change that joint.

The angle differences are wrapped to [−π, π] to account for the circular nature of joints.

Additionally, the solver may impose **configuration continuity** — preferring solutions that stay in the same branch (same shoulder, same elbow, same wrist) as the current state. This prevents the arm from swinging through space to reach a different branch, which is exactly the safety property Hatch is designed around.

## 12. What Comes Next

We have now derived all six joint angles:

- θ₁ from the tangency condition (Part IV)
- θ₅ from the lateral position of the TCP (Part V)
- θ₂, θ₃ from the law of cosines in the arm plane (Part VI, §5–6)
- θ₄ from the wrist orientation (Part VI, §8)
- θ₆ from the wrist orientation (Part VI, §8)

The next part will present the complete algorithm — the order of computation and the handling of special cases — and then a worked example on a real robot.

---

## Changes made in this reconstruction

1. **Part V, §10** — fixed the typo: "θ₅ (the tool roll)" → "θ₆ (the tool roll)"
2. **Part VI, §4** — removed the "Wait —" drafting artifact; the paragraph now reads as a single coherent derivation
3. **Part IV, §4** — added a note that **d₄** is the `axis_shift` at J4 (it was introduced in Part II but the symbol switch wasn't signposted)
4. **Formatting** — converted the PDF's broken math notation (e.g., `\(\mathbf{p}_5 = ...\)`) to plain readable form

## Two open questions still to resolve

These are not fixed in the reconstruction, because they need your input:

1. **The `d5` question.** Part V, §2 says the `axis_shift` at J5 is "typically zero for both UR and FR robots." For UR this is true. For FR, the extraction code uses `max(abs(j5_xyz[1]), abs(j5_xyz[2]))` — which suggests FR may have a nonzero `d5`. If it does, Part V needs revision.

2. **The spherical θ₁ question.** Part IV, §8 says the code must add the "pointing away" solution (φ + π) explicitly for spherical wrists. But the Elfin works with the current code, which does *not* add it. Either the Elfin's joint limits exclude that configuration, or the wrist compensates. Worth resolving before the appendices are written.

---



---

# Part VII: The Full Algorithm and a Worked Example

## 1. The Algorithm, Stated as a Sequence

We have the geometric picture (Parts I–III), the derivation for θ₁ (Part IV), θ₅ (Part V), and the arm chain (Part VI). Now we state the algorithm as a sequence of steps — the order in which a computer would execute it.

The input is the **flange pose in the true root frame**: a 4×4 homogeneous transformation `T_flange`. The output is a list of up to 8 tagged joint vectors.

**Step 0 — Precondition check.** Verify the arm is in the supported class:
- J2 and J3 are parallel (`axis_twist(1) ≈ 0` or `≈ π`).
- J5 and J6 intersect (`distance between axes[4] and axes[5] < tol`).

If either fails, refuse loudly. Do not approximate.

**Step 1 — Compute P5 (the wrist center).**

The flange is offset from P5 by the **flange offset** `d₆` along the tool axis. The tool axis is the third column of the flange rotation. So:

```
P5 = P_flange − d₆ · a_flange
```

where `P_flange` is the translation part of `T_flange`, `a_flange = T_flange[:3, 2]`, and `d₆` is the **flange offset** from `ArmGeometry` — the distance from P5 to the flange.

**Step 2 — Solve the shoulder (θ₁).**

Project P5 onto the horizontal plane:

```
r = √(x₅² + y₅²)
φ = atan2(y₅, x₅)
```

If `wrist_offset > tol` (offset wrist):

```
α = arcsin(wrist_offset / r)
θ₁_left  = φ + α
θ₁_right = φ − α
```

If `wrist_offset ≤ tol` (spherical wrist):

```
θ₁_right = φ
θ₁_left  = φ + π
```

For each θ₁, check `r ≥ wrist_offset` (offset case only). If violated, this branch is invalid.

**Step 3 — Solve the wrist angle (θ₅).**

For each θ₁, compute the TCP's lateral offset from the arm plane:

```
o = −x_flange · sin(θ₁) + y_flange · cos(θ₁)
```

Wait — this should be the offset of **P5**, not the flange. Let me correct: the offset of P5 from the arm plane is exactly `wrist_offset` (by the tangency construction). The offset of the **flange** from the arm plane is what θ₅ controls. So:

```
o_flange = −x_flange · sin(θ₁) + y_flange · cos(θ₁)
cos(θ₅) = (wrist_offset − o_flange) / d₆
```

Check `|cos(θ₅)| ≤ 1`. If violated, this θ₁ branch is invalid.

Two solutions: `θ₅ = ± arccos(cos(θ₅))`. Tag: `+` is no-flip, `−` is flip (or vice versa, depending on convention — the code fixes this).

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

Check `|cos(θ₃)| ≤ 1`. Two solutions: `θ₃ = ± arccos(cos(θ₃))`. Tag: `+` is elbow-up, `−` is elbow-down.

Law of sines for θ₂:

```
ψ = atan2(Δw, Δu)
β = arcsin(a₃ · sin(θ₃) / D)
θ₂ = ψ − β · sign(θ₃)
```

**Step 5 — Solve θ₄ (wrist roll).**

Compute the orientation of frame 3:

```
R₀₃ = Rz(θ₁) · Ry(−θ₂) · Ry(θ₃)
```

(The exact form depends on the frame convention — UR and FR use different signs for θ₂.)

Isolate the wrist rotation:

```
R_wrist = R₀₃ᵀ · R_flange
```

Extract θ₄ and θ₆ from `R_wrist`. For a Z-Y-Z wrist:

```
θ₅ = atan2(±√(1 − R_wrist[2,2]²), R_wrist[2,2])
θ₄ = atan2(R_wrist[1,2] / sin θ₅, R_wrist[0,2] / sin θ₅)
θ₆ = atan2(R_wrist[2,1] / sin θ₅, −R_wrist[2,0] / sin θ₅)
```

**But θ₅ is already known from Step 3.** So we do not re-extract it; we use the known θ₅ and solve for θ₄ and θ₆ directly:

```
θ₄ = atan2(R_wrist[1,2] / sin θ₅, R_wrist[0,2] / sin θ₅)
θ₆ = atan2(R_wrist[2,1] / sin θ₅, −R_wrist[2,0] / sin θ₅)
```

This is cleaner: the wrist decomposition is consistent with the θ₅ already chosen.

**Singularity at θ₅ ≈ 0 or π:** J4 and J6 axes become parallel. Only θ₄ + θ₆ (or θ₄ − θ₆) is determined. Set θ₄ = 0 (or current value) and solve for θ₆. The choice of θ₄ is arbitrary; the tool reaches the correct orientation regardless.

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

## 2. Worked Example: UR10

Let me work through the algorithm on the UR10, using the numbers from the extraction:

- `d₁` (shoulder height) = 0.1273 m
- `a₂` (upper arm) = 0.612 m
- `a₃` (forearm, J3 to P5) = 0.5723 + 0.1157 = 0.688 m (approximately — actual computation from geometry)
- `wrist_offset` = 0.1157 m
- `d₆` (flange offset) = 0.0922 m
- `axis_twist(1)` between J2 and J3 = 0 (co-rotating)

Wait — for UR10, `axis_gap(2)` is 0.5723 (J3 to J4), and `wrist_offset` is 0.1157 (J4 to P5). These are along perpendicular directions (J3 and J4 are perpendicular, `axis_twist(2) = π/2`). The forearm length `a₃` is the distance from J3's axis to P5, which is:

```
a₃ = √(0.5723² + 0.1157²) ≈ 0.5838 m
```

Hmm, but that assumes J3 and J4 are perpendicular and the offsets are orthogonal. Let me check: for UR10, J3 and J4 are perpendicular (axis_twist = π/2), and J4 and J5 are perpendicular. The wrist offset is along J4's axis direction. The distance from J3 to P5 is the hypotenuse of a right triangle with legs 0.5723 and 0.1157:

```
a₃ = √(0.5723² + 0.1157²) ≈ 0.5838 m
```

Actually, this is not quite right either. The forearm is a rigid link from J3 to P5. In the arm plane (which contains J2, J3, and the forearm), the distance from J3's axis to P5 is the link length. The geometry is: J3's axis is perpendicular to the arm plane, J4's axis is in the arm plane (tilted by the wrist offset), and P5 is offset from J4 along J4's axis.

This is getting complicated. For the worked example, let me use a **simple spherical-wrist robot** (the Elfin) to keep the numbers clean, and note that the UR/FR case follows the same steps with the offset included.

---

## 3. Worked Example: Elfin E15 Pro (Spherical Wrist)

**Robot parameters** (from the extraction):

- `d₁` = shoulder height = 0.450 m (approximate)
- `a₂` = upper arm = 0.730 m
- `a₃` = forearm (J3 to P5) = 0.570 m
- `wrist_offset` = 0 (spherical)
- `d₆` = flange offset = 0.100 m (approximate)

**Target:** Place the flange at a specific pose. Let's say:

```
P_flange = (0.5, 0.3, 0.8)  (in true root frame)
R_flange = identity  (tool axis points along +Z)
```

**Step 1 — Compute P5.**

```
a_flange = (0, 0, 1)  (identity rotation, tool axis = Z)
P5 = P_flange − d₆ · a_flange = (0.5, 0.3, 0.8 − 0.1) = (0.5, 0.3, 0.7)
```

**Step 2 — Solve θ₁.**

```
r = √(0.5² + 0.3²) = √(0.34) ≈ 0.583
φ = atan2(0.3, 0.5) ≈ 0.540 rad ≈ 30.96°
```

Spherical case:

```
θ₁_right = φ ≈ 0.540 rad
θ₁_left  = φ + π ≈ 3.682 rad
```

Check limits (assume J1 limits are [−π, π]):

- θ₁_right = 0.540 rad — valid
- θ₁_left = 3.682 rad — exceeds π, wrap to −2.601 rad — valid if within limits

**Step 3 — Solve θ₅.**

For θ₁_right = 0.540:

```
o_flange = −x · sin(θ₁) + y · cos(θ₁)
         = −0.5 · sin(0.540) + 0.3 · cos(0.540)
         = −0.5 · 0.514 + 0.3 · 0.858
         = −0.257 + 0.257 = 0.000
```

Interesting — the flange is exactly on the arm plane (because φ points at P5, and the flange is directly above P5).

```
cos(θ₅) = (wrist_offset − o_flange) / d₆ = (0 − 0) / 0.1 = 0
θ₅ = ± arccos(0) = ± π/2
```

Two solutions: θ₅ = π/2 (no-flip) and θ₅ = −π/2 (flip).

For θ₁_left = 3.682:

```
o_flange = −0.5 · sin(3.682) + 0.3 · cos(3.682)
         = −0.5 · (−0.514) + 0.3 · (−0.858)
         = 0.257 − 0.257 = 0.000
```

Same result (as expected — the arm plane is the same line, just pointing the other way). θ₅ = ± π/2.

**Step 4 — Solve θ₂, θ₃.**

For θ₁_right = 0.540, θ₅ = π/2:

Coordinates in arm plane:

```
u = x · cos(θ₁) + y · sin(θ₁) = 0.5 · 0.858 + 0.3 · 0.514 = 0.429 + 0.154 = 0.583
w = z₅ = 0.7
Δu = 0.583
Δw = 0.7 − 0.450 = 0.250
D² = 0.583² + 0.250² = 0.340 + 0.0625 = 0.4025
D = 0.634
```

Law of cosines:

```
cos(θ₃) = (D² − a₂² − a₃²) / (2 · a₂ · a₃)
        = (0.4025 − 0.5329 − 0.3249) / (2 · 0.730 · 0.570)
        = (−0.4553) / 0.8322
        = −0.547
θ₃ = ± arccos(−0.547) = ± 2.157 rad
```

Two solutions: θ₃ = +2.157 (elbow-up), θ₃ = −2.157 (elbow-down).

For θ₃ = +2.157:

```
ψ = atan2(0.250, 0.583) = 0.405 rad
β = arcsin(a₃ · sin(θ₃) / D) = arcsin(0.570 · sin(2.157) / 0.634)
  = arcsin(0.570 · 0.832 / 0.634) = arcsin(0.748) = 0.846 rad
θ₂ = ψ − β = 0.405 − 0.846 = −0.441 rad
```

For θ₃ = −2.157:

```
β = arcsin(0.570 · (−0.832) / 0.634) = arcsin(−0.748) = −0.846 rad
θ₂ = ψ − β = 0.405 − (−0.846) = 1.251 rad
```

**Step 5 — Solve θ₄, θ₆.**

For each (θ₁, θ₂, θ₃, θ₅), compute `R₀₃` and extract θ₄, θ₆.

For θ₁ = 0.540, θ₂ = −0.441, θ₃ = +2.157, θ₅ = π/2:

```
R₀₃ = Rz(0.540) · Ry(0.441) · Ry(2.157) = Rz(0.540) · Ry(2.598)
R_wrist = R₀₃ᵀ · I = R₀₃ᵀ
```

Then extract θ₄, θ₆ from `R_wrist` with θ₅ = π/2.

The exact numbers depend on the frame convention. The point is: given θ₁, θ₂, θ₃, θ₅, and `R_flange`, the remaining two angles θ₄ and θ₆ are determined (with a singularity at θ₅ = 0 or π).

**Step 6 — Assemble.**

For this target, the solution set is:

| Shoulder | Elbow | Wrist | θ₁ | θ₂ | θ₃ | θ₅ |
|---|---|---|---|---|---|---|
| right | up | no-flip | 0.540 | −0.441 | +2.157 | +π/2 |
| right | up | flip | 0.540 | −0.441 | +2.157 | −π/2 |
| right | down | no-flip | 0.540 | +1.251 | −2.157 | +π/2 |
| right | down | flip | 0.540 | +1.251 | −2.157 | −π/2 |
| left | up | no-flip | 3.682 | ... | ... | +π/2 |
| left | up | flip | 3.682 | ... | ... | −π/2 |
| left | down | no-flip | 3.682 | ... | ... | +π/2 |
| left | down | flip | 3.682 | ... | ... | −π/2 |

Up to 8 solutions. Filter by joint limits (modulo 2π). Return all valid ones, tagged.

**Step 7 — Filter.**

For the Elfin, J1 limits are typically [−π, π]. The left-shoulder solutions at θ₁ = 3.682 rad wrap to −2.601 rad, which is within [−π, π]. So both shoulder branches may be valid.

The elbow-up and elbow-down solutions at θ₂ = −0.441 and θ₂ = +1.251 are both likely within limits.

The wrist-flip and no-flip solutions at θ₅ = ±π/2 are both valid (away from the singularity).

So all 8 solutions may be valid. The caller selects by continuity with the current pose.

---

## 4. What the Worked Example Shows

1. **The algorithm is geometric.** Every step is a distance, an angle, or a projection. No matrix algebra beyond the final orientation extraction.
2. **The spherical case is clean.** With `wrist_offset = 0`, the tangency condition reduces to "the arm plane contains P5's projection," and the two shoulder solutions are φ and φ + π.
3. **The offset case follows the same steps.** Replace `θ₁ = φ` and `θ₁ = φ + π` with `θ₁ = φ ± arcsin(wrist_offset/r)`. The rest is identical.
4. **Tagging is essential.** The 8 solutions are distinct branches. Without tags, the caller cannot know which branch it received.

---

## 5. What the Example Does Not Show

The example uses a simple target (flange directly above P5's projection, tool axis = Z) to keep the numbers clean. Real targets have:

- Non-identity orientation, which couples θ₄ and θ₆.
- Off-axis flange positions, which make `o_flange ≠ 0` and `cos(θ₅) ≠ 0`.
- Singular configurations (θ₅ = 0 or π), where θ₄ and θ₆ are not independent.

The algorithm handles all of these. The example shows the structure; the code handles the cases.

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

**2. URDF gives more information.** A URDF joint origin is a full 6-DOF transform. DH compresses that transform into four scalars by assuming the frame placement. The compression loses information that the solver may need — for example, the exact placement of the tool frame, or the orientation of a joint axis that is not aligned with a canonical direction.

**3. DH requires a convention choice that the solver does not need.** The DH parameters depend on where the frames are placed. Two different DH tables can describe the same robot if the frames are placed differently. Hatch works directly with the joint axes as lines in space, so it does not need to choose a frame placement.

## A.4 The Joint 4 Offset: DH `d₄` vs. Geometric `wrist_offset`

The most important difference between DH and Hatch's geometric quantities is at **joint 4**.

In the standard UR DH table, `d₄` is the **wrist offset** — the distance from the J4 axis to the wrist center P5. For UR10, `d₄ = 0.163941 m` (from the DH table). But the extraction code reports `wrist_offset = 0.1157 m`.

Why the difference? **Frame convention.** The DH `d₄` is measured along the J3 axis (in the DH frame placement), not along the J4 axis. The geometric `wrist_offset` is the perpendicular distance from the J4 axis to P5, which is a different quantity.

The solver uses the **geometric** value (0.1157 m for UR10), because the tangency condition is about the perpendicular distance from J4's axis to P5, not about a distance along J3's axis.

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

If the solver approximated a non-arm-plane robot, it would produce joint angles that place the tool near the target but not exactly at it. The error might be small for some poses and large for others. The caller would have no way to know which. In a safety-critical context — which is the context Hatch is designed for — that is unacceptable.

The refusal is the safe behavior. It says: "This robot is not in the class I can solve. I will not guess."

## B.4 What Is Not in the Class

**Robots with skew J2 and J3.** Some painting robots and welding robots use non-parallel shoulder-elbow axes for cable routing or structural reasons. These are not edge cases; they are different kinematic classes.

**Robots with non-intersecting J5 and J6.** Some collaborative robots and some 7-DOF arms have wrist geometries where J5 and J6 do not share a point. The wrist center P5 does not exist, and the decoupling fails.

**7-DOF arms.** Redundant arms have more than 6 joints. The extra degree of freedom means the IK has infinitely many solutions, and the analytical structure is different. Hatch does not support them in v1.0.0.

**Parallel robots, delta robots, SCARA arms.** Different kinematic structures entirely. Outside the class.

For all of these, Hatch refuses. The word is **unified**, not **universal**. Unified across the supported class — spherical and offset wrists, UR and FR and Elfin — not universal across all robots.

## B.5 What Happens to the Caller

When the solver refuses, the caller has options:

1. **Use a different solver.** A numerical IK solver can handle a wider class of robots, at the cost of speed and determinism. Hatch does not ship one, but the interface allows one to be attached.

2. **Refuse the robot.** If the application requires the analytical solver, the robot must be in the supported class. The refusal is a signal that the robot is not suitable.

3. **Investigate the URDF.** Sometimes the violation is a URDF error, not a real geometric feature. The message names the measured value, so the caller can check whether the URDF is correct.

The refusal is not a dead end. It is a clear signal about what the robot is and what the solver can do.
