## 孵 (Hatch) 架构文献

## 一个推导出来的架构

> *「我需要控制一个机械臂。为此，什么必须存在？」*

本文献不描述一个预先设计的系统。
它追溯一条需求之链——每一个需求催生下一个——直到一个完整的平台浮现。

孵中的每一个组件，皆因某个需求要求它存在。
每一条原则，皆是被发现的，而非被颁布的。
这就是**推导出来的架构**的含义。

---

## 第一部分：场景

### 需求：我必须描述我的机械臂及其世界

在任何东西运动之前，在任何东西被可视化之前，在任何控制面板存在之前——必须先有对场景中存在之物的描述。

一个机械臂不是一件东西。它是由关节连接起来的一串连杆。
它携带工具。它坐在桌子或移动底座上。它的腕部安装着传感器。
所有这些物体在空间中彼此关联。

我们需要一种格式，能够描述：
- 连杆（具有几何形状的刚体）
- 关节（连杆之间的连接，有些是固定的，有些是运动的）
- 它们彼此之间的相对位置
- 它们的外观（网格）

URDF 格式就是为此而生的。它是机器人社区描述机械臂的标准。
我们采用它——不是因为 ROS 使用它，而是因为它解决了这个需求。

### 原则：一切皆在 URDF

所有组件——机械臂、传感器、工具、夹具、桌子、移动底盘——皆由 URDF 描述。
URDF 是整个场景的唯一真实来源。
没有单独的世界文件，没有启动文件，没有外部的位置配置。
`world` 的固定关节定位每一个物体。
一种格式，一个解析器，一个真相。

### 需求：我的场景有许多来自不同来源的部件

一个机械臂来自一家制造商。一台激光扫描器来自另一家。一个移动底盘来自第三家。
每一个都有自己的网格、自己的坐标系、自己的 URDF 定义。
我需要将它们组合成一个场景。

Xacro 格式提供了组合能力：包含文件、定义变量、实例化宏。
我们不需要全部的 xacro——只需要足够从模块化部件组装一个场景即可。

### 组件：URDFPreprocessor

```
用户提供 .urdf 或 .xacro 文件
    ↓
URDFPreprocessor 解析 <xacro:include> 文件
    ↓
解析 package:// 路径以找到网格文件
    ↓
替换 ${变量}
    ↓
展开 <xacro:macro> 调用
    ↓
输出纯 URDF XML
    ↓
KinematicModel 解析它
```

预处理器对用户不可见。他们加载任何 `.urdf` 或 `.xacro` 文件，孵会自行处理其余部分。没有单独的构建步骤。没有外部工具。

### 原则：个体先于群体

一群机械臂的可靠程度，取决于每一个个体。
孵专注于**一个机械臂、一个会话**。
多臂协作是组合，而非核心。

### 需求：我的场景需要可移植地引用网格文件

URDF 引用网格文件用于可视化。这些文件与 URDF 一起存放在包结构中。
路径必须在任何机器上都能工作，而不仅仅在作者的机器上。

`package://` URI 方案解决了这个问题：`package://包名/相对路径`。
孵在配置的目录中搜索 `包名`，并在其中跟随相对路径。

用户按照 ROS 包惯例组织文件：

```
~/hatch/assets/
├── scenes/my_scene/urdf/scene.urdf    ← 在孵中加载的文件
├── robots/ur10/urdf/ur10.urdf         ← 被 scene.urdf 包含
├── robots/ur10/meshes/base_link.stl   ← 被 ur10.urdf 引用
├── sensors/keyence/...                ← 被 scene.urdf 包含
└── ugv/bunker/...                     ← 被 scene.urdf 包含
```

---

## 第二部分：理解机械臂

### 需求：我必须知道每个东西在哪里

URDF 描述连杆和关节。但关节会运动。在任何时刻，我需要知道：
TCP 相对于世界在哪里？相机相对于机械臂基座在哪里？
link_3 相对于 link_1 在哪里？

这就是变换问题。每个机器人框架都必须解决它。

### 组件：KinematicModel

```
URDF 文件
    ↓
KinematicModel 解析连杆和关节
    ↓
检测真正的运动学根
    ↓
在状态变化时计算正向运动学
    ↓
为世界坐标系中的每个连杆提供变换
```

模型是纯粹的数据。无可视化。无控制逻辑。
它回答一个问题：给定关节角，每个东西在哪里？

### 真正的运动学根

并非所有 URDF 都使用 `base_link` 作为运动学根。Universal Robots 在 `base_link` 和第一个运动关节之间插入了一个带有 180° 固定关节的 `base_inertia` 连杆。使用 `base_link` 进行运动学计算会产生错误的结果。

`KinematicModel` 自动检测真正的根：找到第一个类型为 `revolute`、`continuous` 或 `prismatic` 的关节。其父连杆即为运动学根。所有正向运动学都从此帧计算。

### 需求：变换必须高效计算

一个有六个关节的机械臂，加上固定偏移、传感器支架和工具帧，有几十个连杆。
在每次查询时重新计算所有变换是浪费。在定时器上重新计算是轮询。

### 原则：空间即 TransformRegistry

所有相对位姿集中一处。惰性求值——仅在需要时计算变换。
变更时缓存失效。回调通知相关方变换已更改。

### 组件：TransformRegistry

```
KinematicModel 计算世界帧中的连杆变换
    ↓
StateHandler 转换为相对于父连杆的变换
    ↓
TransformRegistry 以惰性缓存存储它们
    ↓
查询时：通过沿树向上遍历计算世界变换
    ↓
更新时：使该帧及其所有后代的缓存失效
    ↓
回调通知 KinematicDisplay 重新渲染
```

### 需求：场景必须变得可见

我有一个运动学模型。我有变换。我有网格文件。我需要以 3D 形式看到机械臂——其连杆处于当前位置，随着关节变化而移动。

### 原则：可视化器乃窥心之镜

可视化是一项阅读服务，而非控制服务。
3D 视图是窥视机械臂内部状态的窗口——而非一个独立的仿真。

### 组件：KinematicDisplay + VisualizerEngine

```
KinematicModel 提供连杆变换和网格路径
    ↓
MeshLoader 将网格文件加载到 VTK PolyData
    ↓
KinematicDisplay 为每个连杆创建 VTK actor
    ↓
TransformRegistry 回调更新 actor 位置
    ↓
VisualizerEngine 在脏时以 60Hz 渲染
```

显示器不控制任何东西。它观察 TransformRegistry 并反映它所看到的。
引擎的渲染循环在没有任何东西移动时睡眠。

---

## 第三部分：让机械臂运动

### 需求：我必须命令机械臂运动

看到机械臂还不够。我需要移动它的关节。我需要将它的 TCP 移动到特定位置。
无论机械臂是真实硬件还是仿真，我都需要能够做到这一点。

### 组件：RobotInterface、SimulatedRobot、RealRobot

```
CommandHandler 接收命令
    ↓
路由到活动的机械臂（SimulatedRobot 或 RealRobot）
    ↓
机械臂执行命令：
    SimulatedRobot：本地求解 IK，更新内部状态
    RealRobot：通过 RTDE 发送命令到硬件
    ↓
机械臂发布 ROBOT_STATE 及新的关节位置
```

### 原则：运动即模型

轨迹、命令、目标皆是数据，而非副作用。
运动可以被预测、监控和回放。

### 原则：纯 Python

除 VTK 绑定外，无 C++ 扩展。快速开发。安全内存管理。
Qt 仅允许在 UI 层和硬件驱动信号桥接中使用——永远不在核心服务中。

### 需求：模型必须与机械臂保持同步

当机械臂运动时，运动学模型必须更新。变换注册表必须更新。显示器必须更新。
这必须在每次状态变化时恰好发生一次，而不是从不同路径多次发生。

### 组件：StateHandler

```
机械臂发布 ROBOT_STATE
    ↓
StateHandler 接收它（唯一修改状态的订阅者）
    ↓
用新的关节位置更新 KinematicModel
    ↓
KinematicModel 重新计算正向运动学
    ↓
StateHandler 用新的变换更新 TransformRegistry
    ↓
TransformRegistry 回调触发
    ↓
KinematicDisplay 设置 _needs_render = True
    ↓
VisualizerEngine 在下一个定时器滴答时渲染
```

状态更新的唯一所有者。没有重复注册。没有遗漏的更新。没有竞态条件。

---

## 第四部分：通信

### 需求：组件之间必须在不知道彼此的情况下通信

关节控制面板不应导入机械臂驱动。机械臂驱动不应导入 3D 显示器。
组件必须在没有耦合的情况下通信。

### 原则：事件驱动，不轮询

组件通过事件通信。没有 `while` 循环等待数据。没有忙等待。没有周期性检查。

### 原则：时间即 StateChannel

所有事件集中一处。发布/订阅，带历史记录。时间戳保留顺序。解耦通信。

### 组件：StateChannel

```
JointControlPanel 发布 JOINT_COMMAND
    ↓
StateChannel 分发给所有订阅者
    ↓
CommandHandler 接收，路由到活动机械臂
    ↓
机械臂发布 ROBOT_STATE
    ↓
StateChannel 分发给所有订阅者
    ↓
StateHandler 更新模型（一个订阅者）
    ↓
JointControlPanel 更新滑块（另一个订阅者）
    ↓
CartesianControlPanel 更新显示（另一个订阅者）
    ↓
KinematicDisplay 通过 TransformRegistry 回调更新（间接）
```

### 原则：单进程，单内存空间

组件之间无需序列化。直接数据访问。无网络开销。无分布式复杂性。
事件携带 Python 对象——无消息定义，无代码生成，无序列化。

---

## 第五部分：用户界面

### 需求：我需要控件来与机械臂交互

关节需要滑块。笛卡尔运动需要位置控件。
连接到硬件需要 IP 输入和状态显示。
这些是呈现关注点——它们不应包含业务逻辑。

### 原则：UI 与服务分离

UI 组件发布事件。它们不直接调用管理器（用户主动发起的命令除外）。
它们不持有业务逻辑。它们不更新模型或注册表。它们是纯粹的呈现。

### 组件：UI 面板

```
JointControlPanel：
    用户拖动滑块 → 发布 JOINT_COMMAND
    订阅 ROBOT_STATE → 更新滑块位置

CartesianControlPanel：
    用户拖动滑块 → 发布 CARTESIAN_COMMAND
    订阅 ROBOT_STATE → 更新当前 TCP 显示

RobotConnectionPanel：
    用户点击连接 → 调用 RobotManager.connect_robot()
    订阅 CONNECTION_ESTABLISHED → 显示绿色状态
    订阅 CONNECTION_LOST → 显示红色状态
    订阅 MODE_SWITCHED → 更新模式显示
```

### 原则：一个机械臂，一个会话

平台一次管理一个机械臂。要使用不同的机械臂，重新启动应用程序。
清晰的边界。无需复杂的清理。

### 需求：所有东西必须连接在一起

服务、显示器、UI 面板——它们需要被创建和连接。
一个地方必须拥有这个责任，而不自己做这项工作。

### 组件：MainWindow

```
MainWindow.__init__:
    创建 TransformRegistry
    创建 StateChannel
    创建 VisualizerEngine
    创建 MeshLoader
    创建 RobotManager
    创建 SimulatedRobot、RealRobot
    创建 CommandHandler
    创建 CameraManager
    创建 UIBuilder
    订阅 ROBOT_LOADED → 创建 MotionContainer
    订阅 ERROR_OCCURRED → 显示对话框
```

`MainWindow` 编排。它不更新模型、修改变换或处理命令。
它创建组件并连接它们。然后退后一步。

---

## 第六部分：渲染循环

### 需求：3D 视图必须在不轮询的情况下流畅更新

当机械臂运动时，显示器必须重新渲染。当没有任何东西运动时，CPU 必须睡眠。
一个以固定间隔运行并重新计算所有东西的渲染循环是浪费的。

### 组件：VisualizerEngine 渲染定时器

```
QTimer 以 60Hz 触发
    ↓
检查每个显示器的 _needs_render 标志
    ↓
如果没有显示器需要渲染：立即返回（CPU 睡眠）
    ↓
如果有任何显示器需要渲染：调用 Render()
    ↓
清除所有 _needs_render 标志
```

这不是轮询。定时器检查一个布尔值——每个显示器一次内存读取。
标志仅由 TransformRegistry 回调设置，而回调仅在变换实际变化时触发。
当机械臂静止时，没有任何东西发生。CPU 在定时器滴答之间进入低功耗状态。

---

## 第七部分：扩展点

### 需求：我必须添加自己的功能

平台无法预见每一个用例。用户需要添加日志、安全监控、自定义控制策略、传感器处理。
用于构建内置面板的相同 API 必须对扩展可用。

### 公共 API

| API | 用途 |
|-----|------|
| `StateChannel.subscribe()` | 响应机械臂状态、连接事件、错误 |
| `StateChannel.publish()` | 发送命令、报告检测、触发动作 |
| `TransformRegistry.get_transform()` | 查询空间关系 |
| `TransformRegistry.register_callback()` | 响应变换变化 |
| `EventType` 枚举 | 系统理解的所有事件 |

扩展遵循与内置组件相同的原则：观察而非控制；发布而非调用；事后清理。

### 动态对象（未来）

`TransformRegistry` 支持 `FrameStatus.DYNAMIC`——在操作期间变换发生变化的帧。
目前这服务于机械臂关节。在未来的版本中，它也将服务于来自相机和传感器的运行时发现的对象。

这是一个设计好的扩展点，而非当前能力。`FrameStatus.DYNAMIC` 值和回调系统已经存在。感知管道尚不存在。

---

## 第八部分：局限

孵是一个诚实的平台。以下领域尚未解决。

### 机械臂拓扑

孵假定一个串联运动链。臂链检测、逆运动学求解器和变换注册都期望从基座到工具的单一连杆序列。并行机械臂、分支链和闭合环在当前版本中不受支持。

### 错误处理

错误作为 `ERROR_OCCURRED` 事件发布并在对话框中显示。没有错误严重性分类，没有结构化的恢复路径，一些驱动级别的错误被静默捕获。用户在开发期间应监控控制台输出。

### 配置管理

如 RTDE 频率、网格大小和渲染 FPS 等值被硬编码为默认值。没有配置文件，没有会话之间的持久化，不修改源代码就无法覆盖默认值。配置管理将在常规使用中痛点显现时加以解决。

### 传感器标定

相机外参假定与 URDF 匹配。没有手眼标定，没有 TCP 标定，没有用标定值覆盖 URDF 变换的机制。对于需要精确空间精度的应用，请在加载前预标定你的 URDF 变换。

### 测试

不存在自动化测试。`TransformRegistry`、`StateChannel` 和 `KinematicModel` 特别适合测试——它们具有清晰的输入和输出，没有外部依赖。测试将在平台稳定后添加。

---

## 十大原则（总结）

| 原则 | 发现来源 |
|------|----------|
| 个体先于群体 | 需求：一个机械臂，一个会话 |
| 单进程，单内存空间 | 需求：无序列化开销 |
| 事件驱动，不轮询 | 需求：解耦通信 |
| 可视化器乃窥心之镜 | 需求：看到机械臂的真实状态 |
| 一切皆在 URDF | 需求：描述场景 |
| 空间即 TransformRegistry | 需求：知道每个东西在哪里 |
| 时间即 StateChannel | 需求：组件必须通信 |
| 运动即模型 | 需求：命令作为数据 |
| 纯 Python | 需求：快速开发 |
| UI 与服务分离 | 需求：无耦合的控件 |
| 一个机械臂，一个会话 | 需求：清晰的边界 |

每一个原则都不是被选择的。它是由推导过程中出现的需求所要求的。
这就是架构诚实的原因。

---

## 完整的事件流

```
用户移动滑块
    ↓
UI 面板发布 COMMAND 事件
    ↓
StateChannel 分发事件
    ↓
CommandHandler 接收，路由到活动机械臂
    ↓
机械臂执行命令，发布 ROBOT_STATE
    ↓
StateHandler 接收 ROBOT_STATE（唯一所有者）
    ↓
StateHandler 更新 KinematicModel → 重算 FK
    ↓
StateHandler 更新 TransformRegistry → 新的连杆变换
    ↓
TransformRegistry 通知回调
    ↓
KinematicDisplay 设置 _needs_render = True
    ↓
VisualizerEngine 60Hz 定时器检查标志 → 脏时渲染
    ↓
ROBOT_STATE 也被 UI 面板接收
    ↓
UI 面板更新滑块位置和数值标签
```

**UI 和模型之间没有直接调用。没有轮询。单一渲染路径。状态更新的唯一所有者。**

---

## 目录结构

```
hatch/
├── core/
│   ├── urdf_preprocessor.py    # 从 .xacro 文件组合场景
│   ├── mesh_loader.py           # 纯网格加载服务
│   ├── robot_manager.py         # 机械臂生命周期（无 Qt）
│   ├── command_handler.py       # 命令路由
│   ├── state_handler.py         # 模型 + 注册表更新的唯一所有者
│   ├── mode.py                  # 模式枚举
│   ├── kinematics/
│   │   ├── kinematic_model.py   # URDF 解析、FK、真根检测
│   │   ├── ik_solver.py         # IK 求解器封装
│   │   └── ur_ik_solver.py      # UR 机械臂参数化解析 IK
│   └── world_state/
│       ├── transform_registry.py
│       ├── state_channel.py
│       └── event_types.py
│
├── drivers/
│   └── robot_arm/
│       ├── robot_interface.py   # 纯 ABC（无 Qt）
│       ├── base_robot_arm.py    # 驱动内部 ABC（无 Qt）
│       ├── simulated_robot.py   # 纯 Python 仿真
│       ├── real_robot.py        # 硬件桥接（Qt 用于信号处理）
│       └── ur_rtde_bridge.py    # RTDE 驱动
│
├── displays/
│   └── kinematic_display.py     # VTK 可视化
│
├── viz/
│   └── visualizer_engine.py     # VTK 渲染窗口、网格、相机
│
├── ui/
│   ├── main_window.py           # 应用程序入口点
│   ├── ui_builder.py            # 菜单和停靠构建
│   ├── menus/
│   └── panels/
│
├── assets/
│   ├── scenes/                  # 场景定义 URDF 文件
│   ├── robots/                  # 机械臂 URDF 和网格
│   ├── sensors/                 # 传感器 URDF 和网格
│   ├── ugv/                     # 移动底座 URDF 和网格
│   └── tools/                   # 末端执行器 URDF 和网格
│
└── docs/
    ├── architecture.md           # 本文献
    └── user_guide.md             # 入门指南
```

---

## 结尾

> *「一个平台不由它能做什么来定义。它由它不会做什么——以及为什么——来定义。」*

孵做好一件事：**让一个机械臂的心灵透明、高效、易于编程。**

从这一基础出发，一切皆可成长。

---

*文献版本 3.0*
*孵 (Hatch) 架构基础*
*以需求链重构。用户需求驱动组件创建。每一条原则对应对催生它的需求。*
