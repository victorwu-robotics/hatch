## Part Three: Placing Things in Space

The URDF describes the robot. But a description is not a position. A link is a rigid body with a shape, but it has no location until something computes where it sits at this moment, with these joint angles. The visualizer needs to know where to draw every piece. The user needs to query the distance between the TCP and the workpiece. The IK solver needs to know where the wrist center is relative to the base.

So Hatch needed two things: a **kinematic model** that computes positions from joint angles, and a **transform registry** that stores those positions and keeps them up to date as the robot moves.

### The Actor and the Space

VTK, the visualization library Hatch uses, thinks in terms of **actors**. An actor is a thing that appears on screen — a link, a tool, a sensor housing. Each actor has a mesh (its shape) and a transform (its position and orientation in the 3D scene). To place an actor correctly, you set its transform to the world position of the link it represents.

But the robot has many links. They move. Their positions change every time a joint angle changes. Keeping track of every link's world position — and updating every actor when something moves — is the central spatial problem Hatch must solve.

### The Kinematic Model: Computing Where Things Are

The kinematic model answers one question: given all six joint angles, where is every link?

It starts from the URDF. Each joint has an origin — a fixed offset from its parent link. A revolute joint adds a rotation about its axis. A prismatic joint adds a translation. The model walks the parent-child chain from the base to the tip, multiplying transforms:

```
T_world_to_link = T_world_to_parent @ T_parent_to_child @ T_joint_motion
```

At zero angles, every link has a known world position. Change joint 2 by 0.1 radians, and the model recomputes. Every link from joint 2 outward gets a new world transform. The computation is pure trigonometry — no iteration, no approximation. Forward kinematics is the easy direction.

The kinematic model also detects the **true root** — the parent of the first moving joint. Some URDFs place fixed joints between the world and the first moving joint (a pedestal, a mounting plate, a 180-degree rotation hack). The model walks backward through those fixed joints to find where the kinematic chain actually begins. This matters for IK, where using the wrong root produces systematically wrong solutions.

### The Transform Registry: Remembering Where Things Are

Computing transforms is one thing. Keeping them organized is another. A six-joint robot with sensors and tools has dozens of frames — the base, six links, a camera, a scanner, a torch holder, optical reference frames. Each frame has a parent-child relationship. Each frame's world position depends on its parent's world position.

The transform registry stores every frame in a tree:

```
world
  └── base_link
        └── shoulder_link
              └── upper_arm_link
                    └── forearm_link
                          └── wrist_1_link
                                └── wrist_2_link
                                      └── wrist_3_link
                                            ├── torch
                                            ├── camera_link
                                            │     └── camera_depth_frame
                                            └── scanner_optical_frame
                                                  └── scanner_frame
```

Each frame stores its transform **relative to its parent** — not relative to the world. This is the key insight. When joint 2 rotates, only `shoulder_link` and its descendants change. The registry invalidates the cached world transforms for those frames. Everything else stays cached. When something queries the world position of the scanner, the registry walks up the tree — `scanner_frame` → `scanner_optical_frame` → `wrist_3_link` → ... → `world` — multiplying parent-relative transforms along the way. The result is cached until the next invalidation.

This is lazy evaluation. If nobody asks for a frame's world position, it is never computed. If a frame hasn't moved since the last query, the cached answer is returned instantly. The registry is event-driven: it only recomputes when something changes, and only for the frames that are actually needed.

This is Principle #5: **Space = TransformRegistry.** All relative poses in one place. Lazy evaluation. Cache invalidation on change. No polling. No periodic recomputation.

### The Single Owner of Updates

The kinematic model computes transforms. The transform registry stores them. But who decides when to update? If multiple components call `update_frame()` at different times, the registry's cache could reflect inconsistent states.

Hatch has exactly one component that updates the model and registry in response to robot motion: the **StateHandler**. It subscribes to `ROBOT_STATE` events. When a new state arrives, it updates the kinematic model with the new joint angles, then updates every frame in the registry with the model's new link transforms. No other component touches the model or the registry during operation.

The initial registration happens once, when the robot is loaded. The runtime updates happen only through StateHandler. This single-owner pattern prevents the duplicate updates and inconsistent states that plague systems where every component can modify the shared spatial data.

