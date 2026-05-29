## Part Six: Cartesian Control and Inverse Kinematics

Joint control is direct: move joint 3 to 0.8 radians, and the elbow bends. Cartesian control is indirect: put the TCP at (0.5, 0.2, 0.8) with the torch pointing down. The user specifies where the tool should be. The platform must figure out what joint angles will put it there.

This is **inverse kinematics** — the hard direction. Forward kinematics (joint angles → TCP pose) always has exactly one answer. Inverse kinematics (TCP pose → joint angles) can have zero, one, or many answers. A pose might be unreachable. Or it might be reachable in eight different ways — shoulder left or right, elbow up or down, wrist flipped or not.

### The Decoupling: Position and Orientation

A six-joint robot with a spherical wrist has a crucial property: the last three joints control orientation without affecting position. Their axes intersect at a single point — the wrist center. This means the problem splits cleanly:

1. **Position the wrist center** using joints 1, 2, and 3. The wrist center is a point in space — three coordinates, three joints. This is a 3-DOF positioning problem solvable with trigonometry.

2. **Orient the tool** using joints 4, 5, and 6. Once the wrist center is placed, the remaining rotation from the wrist frame to the target TCP orientation is a pure orientation problem.

This decoupling is the foundation of analytical inverse kinematics. It transforms one hard six-dimensional problem into two manageable three-dimensional problems.

### The Wrist Center

The wrist center is not the TCP. The TCP is the tip of the tool — the point that touches the workpiece. The wrist center is the intersection of the last three joint axes, somewhere inside the wrist mechanism. For a naked robot with no tool, the TCP might be the mounting flange. For a robot with a welding torch, the TCP is the tip of the wire, offset from the wrist center by the torch length.

Given a desired TCP pose (position and orientation), the wrist center is found by walking backward along the tool's approach direction by the tool length:

```
wrist_center = tcp_position - tool_length × approach_vector
```

Once the wrist center is known, the arm must position it. This is where the geometry begins.

### The Arm Triangle

Joints 1, 2, and 3 form a planar mechanism. Joint 1 rotates the entire arm about the base's vertical axis. Joints 2 and 3 move within that vertical plane — the shoulder tilts, the elbow bends. The wrist center must lie somewhere in that plane.

From a top-down view, Joint 1 is simply the horizontal angle to the wrist center. Once the arm plane is aimed, the problem collapses to two dimensions: a triangle formed by the shoulder, the elbow, and the wrist center.

The sides of this triangle are the upper arm length, the forearm length, and the straight-line distance from shoulder to wrist center. Given all three sides, the Law of Cosines gives the elbow angle. The shoulder angle follows from the geometry of the triangle within the plane.

This yields four solutions: shoulder left or right (two choices for Joint 1), and elbow up or down (two choices for Joint 3). Each solution places the wrist center at exactly the same point.

### The Wrist Orientation

With the wrist center positioned, the remaining task is to orient the tool. The forearm arrives with a particular orientation determined by joints 1-3. The wrist joints must rotate from that orientation to the desired TCP orientation.

This is a pure rotation problem on a spherical joint. The target rotation is decomposed into three wrist joint angles. For a standard spherical wrist with orthogonal axes, this decomposition is straightforward. For wrists with non-orthogonal axes or compound rotations at the joint origins, the decomposition may require numerical methods.

Hatch uses the robot's kinematic model to solve this. Rather than deriving analytical formulas for every possible wrist geometry, it asks the model directly: "If I change joint 4 by a small amount, how does the TCP orientation change?" This numerical Jacobian approach works for any wrist geometry, at the cost of a few iterations.

### The Two Solver Strategy

Hatch uses different IK strategies for different robots, selected automatically:

- **UR robots (offset wrist):** An analytical solver using D-H parameters derived from the UR's known geometry. Fast, exact, and produces all valid solutions.

- **Spherical wrist robots (Elfin, KUKA, ABB):** A numerical solver that queries the kinematic model directly. No D-H parameters needed. Works for any spherical wrist regardless of joint origin conventions.

- **Hybrid (future):** Geometric arm solution with numerical wrist refinement. The arm is solved analytically (fast, all solutions found). The wrist is refined numerically (works for any geometry).

The dispatcher detects the wrist type from the URDF — if the last three joints have zero X offset, the wrist is spherical and the numerical solver is used. Otherwise, the analytical UR solver is used.

### The Cartesian Control Flow

The flow for Cartesian control mirrors joint control, with one extra step:

1. User moves a Cartesian slider
2. `CartesianControlPanel` updates the target pose and publishes `CARTESIAN_COMMAND`
3. `CommandHandler` routes to the active robot's `move_pose()` method
4. The robot solves IK to find joint angles for the target pose
5. The robot moves to those joint angles
6. The robot publishes `ROBOT_STATE` with its new position
7. `StateHandler` updates the model and registry
8. The display updates

The key difference from joint control: Step 4 involves solving IK. In Simulate mode, the simulated robot runs the IK solver locally. In Real mode, the real robot uses its controller's built-in IK (for UR robots) or Hatch's numerical solver (for other robots, when using `SIMULATE_REAL_IK` mode).

The Cartesian panel's sliders set the target pose. The "Current TCP" display shows the actual TCP pose from the kinematic model. In Simulate mode, these should match exactly. In Real mode, the actual pose may differ slightly due to the robot still being in motion or due to calibration differences.

### The IK Appendix

For readers who want to understand the mathematics behind inverse kinematics — the Law of Cosines, the arm triangle, the wrist decomposition, the rotation matrix extraction — see Appendix B: A Geometric Guide to Inverse Kinematics. That appendix derives every formula from first principles, with diagrams and worked examples. It is the long answer to the question: "How does the robot know what angles to use?"

