Architecture Note: Fixed Chain Tail
The robot only moves between the first and last moving joints. The kinematic chain is bounded by two points:

Before the first moving joint: Fixed joints connect the world to the true kinematic base. These links (pedestals, mounting plates, base frames) never move.

After the last moving joint: Fixed joints connect the last moving link to sensors, tools, flanges, and end-effectors. These links move as a rigid assembly with the last moving link's child.

KinematicModel._find_true_root() walks backward through fixed joints to find the true base. It also walks forward through fixed joints to find the naked tool mount point (tool_mount_link).

StateHandler._build_arm_chain_links() includes all fixed children of arm chain links, so sensors and tools are updated when the arm moves. The TransformRegistry tracks each frame individually. A future optimization would combine all post-wrist links into a single compound VTK actor, since they move as one rigid body.

TCP Switching Specification
Multiple TCP Support (Planned)
A robot wrist often carries multiple attachments: a welding torch, a camera, a laser scanner. Each has a working point that can serve as the TCP. Hatch will support switching between them.

Data model: Every fixed link after the last moving joint is a potential TCP. KinematicModel already walks the fixed chain to set tool_mount_link. This will be extended to expose the full list of available endpoints.

UI: A dropdown in the Cartesian Control panel lists all available TCPs with their offset distance from the wrist center. Selecting a different TCP updates tool_mount_link, and the IK solver automatically targets the new link.

Display: The active TCP is highlighted in green in the Joint Frame panel. Other available endpoints are listed but not highlighted. The magenta TCP frame always shows the currently active tool mount point.

Implementation path:

KinematicModel builds a list of all fixed endpoints during URDF parsing.

KinematicModel.set_active_tcp(link_name) changes tool_mount_link.

IK solver, displays, and panels reference tool_mount_link — they follow automatically.

CartesianControlPanel adds a TCP selector dropdown populated from the endpoint list.

This feature is planned but not yet implemented. Currently, Hatch uses the naked tool mount point (last fixed link after the last moving joint) as the TCP.