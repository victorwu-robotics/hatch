## Part Nine: The URDF Preprocessor and Scene Composition

A robot does not exist in isolation. It sits on a table. A camera is bolted to its wrist. A laser scanner peers down at the workpiece. A welding torch extends from the flange. All of these things must be described and positioned relative to each other. The URDF is the single source of truth for the entire scene.

### Why a Preprocessor?

The URDF format is verbose. A robot with six links, six joints, three sensors, and a tool can produce a file thousands of lines long. Much of that content is repeated — the same sensor appears on multiple robots, the same tool is used across projects, the same material colors are applied to every link.

Xacro is a macro language for URDF. It lets you define a component once and reuse it. A laser scanner is defined in its own file, with its own meshes, its own frames, its own parameters. The main scene file includes it with a single line and positions it with a single joint. If the scanner's mesh is updated, every scene that includes it gets the update automatically.

Hatch includes its own xacro preprocessor. It is not the full ROS xacro — it does not need to be. It supports the five features that matter for scene composition:

- `<xacro:include>` — include another file
- `<xacro:property>` — define a variable
- `${variable}` — use a variable
- `<xacro:macro>` — define a reusable block
- `<xacro:macro_name>` — instantiate a macro

The preprocessor also handles `$(find package_name)` — the ROS convention for locating packages in a search path. This means URDF files from ROS-Industrial, robot manufacturers, and the community work in Hatch without modification.

The user loads a `.xacro` file. The preprocessor resolves includes, substitutes variables, expands macros, and produces a clean URDF. The `KinematicModel` parses that URDF. The user never sees the intermediate file. The preprocessing is invisible.

### The package:// Resolution

URDF files reference mesh files using URIs. The most portable URI scheme is `package://`:

```xml
<mesh filename="package://ur_description/meshes/base_link.stl"/>
```

This means: "find a directory named `ur_description` in one of the search paths, then look for `meshes/base_link.stl` inside it." The search paths are configured when Hatch starts and include the user's `~/hatch/assets/` directory and its subdirectories.

The convention is ROS-standard:

```
~/hatch/assets/
├── robots/
│   └── ur_description/
│       ├── urdf/
│       └── meshes/
├── sensors/
│   ├── keyence_experimental/
│   └── orbbec_camera/
├── tools/
│   └── welding_torch/
└── scenes/
    └── my_workcell/
        └── urdf/
            └── scene.xacro    ← The file the user loads
```

A user downloading a robot package from ROS-Industrial places it in `~/hatch/assets/robots/`. The `package://` references in the URDF resolve automatically. No path editing. No file searching. Just place the package and load the scene.

### The Scene URDF

The scene file is the top-level URDF. It includes all components and positions them with fixed joints:

```xml
<robot name="my_workcell">
  <!-- The table -->
  <link name="table">
    <visual>
      <mesh filename="package://my_workcell/meshes/table.stl"/>
    </visual>
  </link>
  <joint name="table_to_world" type="fixed">
    <parent link="world"/>
    <child link="table"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- The robot, included from its own package -->
  <xacro:include filename="$(find ur_description)/urdf/ur10.urdf"/>
  <joint name="robot_to_table" type="fixed">
    <parent link="table"/>
    <child link="base_link"/>
    <origin xyz="0.5 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- The scanner, mounted to the wrist -->
  <xacro:include filename="$(find keyence_experimental)/urdf/lj_v7200_macro.xacro"/>
  <xacro:lj_v7200 prefix=""/>
  <joint name="scanner_to_wrist" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="lj_v7200_optical_frame"/>
    <origin xyz="0.062 -0.105 0.3667" rpy="0 ${pi} ${-pi/2}"/>
  </joint>
</robot>
```

Everything is in the URDF. No separate world file. No launch file. No external configuration for where things are placed. Fixed joints from `world` position every object. One format. One parser. One truth.

This is Principle #4: **Everything in URDF.**

### The Fixed Chain Tail

The URDF's kinematic chain has a beginning and an end. Before the first moving joint, fixed joints connect the world to the true base — a pedestal, a mounting plate, a table. After the last moving joint, fixed joints connect the wrist to tools, sensors, and flanges.

Hatch detects the true base by walking backward through fixed joints from the first moving joint. It detects the TCP by walking forward through fixed joints from the last moving joint. The kinematic model's `_find_true_root()` method follows both chains, ensuring that the IK solver targets the actual tool tip, not the wrist center, and that the base frame is where the robot meets the world, not where the first motor sits.

This automatic detection means the user doesn't need to configure base frames or TCP offsets. The URDF already contains this information. Hatch reads it.

