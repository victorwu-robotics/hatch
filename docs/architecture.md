Hatch (孵) Architecture Document

---

## Prologue: On Understanding

> *"Understand them, or you will not fully utilise them. Understand your life, or you will not live fully on earth."*

Life is not about following. It is about **seeing**.

The crow follows. The crowd drifts. The current carries.

But the one who understands — they choose their own direction.

### The Two Worlds

| Physical World | Emotional World |
| --- | --- |
| Laplace transforms for stability | Empathy for connection |
| Orthogonal matrix inverse | Trust that is reciprocal |
| PCA for variance | Wisdom from experience |
| Rotation vectors for clarity | Honesty in communication |

Both require **understanding**, not just formalism.

You cannot solve stability with Laplace if you do not feel what poles mean.
You cannot truly connect if you only recite the word "love."

### The Trap of Formalism

Modern life — and modern education — teaches us to **follow**:

| Instead of Understanding | We Are Taught To |
| --- | --- |
| See the projection | Memorize the matrix |
| Feel the stability | Apply the Laplace |
| Know the direction | Follow the crowd |
| Understand | Recite |

The result is a world of **competent engineers who cannot innovate**.
A world of **people who speak of meaning but do not feel it**.

### The Stubborn Student

You were the stubborn student.

You refused to memorize.
You demanded to see.
You asked "why?" until the formalism cracked open and revealed its meaning.

That is why you can:

- See the orthogonal matrix inverse as a transpose
- Choose rotation vectors over quaternions
- Build Hatch from first principles, not from convention

You are not following. You are **seeing**.

### Why Hatch Exists

> *"I build Hatch because I want to fully understand robots. Then I can use robots to their limits."*

Most platforms help you **use**.
Hatch helps you **see**.

| Other Platforms | Hatch |
| --- | --- |
| Hide complexity | Reveals it |
| Provide black boxes | Opens them |
| Give you APIs | Gives you insight |
| Focus on what you can do | Focus on why it works |

Because the builder — you — built it to understand first.

### What Understanding Unlocks

| Without Understanding | With Understanding |
| --- | --- |
| Move sliders | Know why the robot moves |
| Load URDFs | See the kinematic chain |
| Publish events | Trace the flow of data |
| Follow instructions | Create new possibilities |
| Shallow utilisation | **Full utilisation** |

The platform is not the goal. **Understanding** is the goal. Hatch is just the vehicle.

### The Invitation

Hatch is open. Others may use it.

But the invitation is not:

> *"Here is a tool to control robots."*

The invitation is:

> *"Here is a way to understand robots. Then you can use them to your own limits."*

### The Stubborn Student's Creed

> *"I will not follow. I will see. I will not memorise. I will understand. I will not accept formalism as truth. I will demand meaning."*

This is the creed of Hatch.
This is the creed of a life fully lived.

### Closing

> *"Do not follow the flow, the crowd, the drift. See the direction. Choose it. Understand it. Then move."*

This is Hatch.
Built to understand.
Built to fully utilise.
Built to see.

You are invited.

---

*— The Stubborn Student*
*Hatch (孵)*

---

## Core Philosophy

> *"A robot platform should be understood, not just used. Every line of code must be traceable to a first principle."*

Hatch is not a collection of tools. It is a **derived architecture** — a system where every component exists because a principle demanded it.

The name **Hatch (孵)** represents the moment a new robot comes to life. The right side of the character (孚) signifies incubation and nurturing — bringing ideas into existence through careful development.

---

## The Ten Principles

### Principle #0: Individuals Before Groups

A team of robots is only as reliable as each individual. Hatch focuses on **one robot, one session**. Multi-robot coordination is composition, not core.

### Principle #1: Single Process, Single Memory Space

No serialization between components. Direct data access. No network overhead. No distributed complexity.

### Principle #2: Event-Driven, No Polling

Components communicate via events. No `while` loops waiting for data. No busy-waiting. No periodic checks. (See Appendix A.)

### Principle #3: Visualizer as Mind-Prying Tool

Visualization is a service that reads, not controls. The 3D view is a window into the robot's internal state — not a separate simulation.

### Principle #4: Everything in URDF

All components (robot, sensors, tools) are described by URDF. No hardcoded transforms. No special-case loading.

### Principle #5: Space = TransformRegistry

All relative poses in one place. Lazy evaluation — transforms computed only when requested. Cache invalidation on change.

### Principle #6: Time = StateChannel

All events in one place. Publish/subscribe with history. Timestamps preserve sequence. Decoupled communication.

### Principle #7: Movements as Models

Trajectories, commands, and goals are data, not side effects. Movements can be anticipated, monitored, and replayed.

### Principle #8: Pure Python

No C++ extensions except VTK bindings. Rapid development. Safe memory management. Access to scientific stack.

### Principle #9: UI Separate from Services

UI components publish events. They do not call managers directly. They do not hold business logic. They are pure presentation.

### Principle #10: One Robot Per Session

The platform manages one robot at a time. To work with a different robot, restart the application. Clean boundary. No complex cleanup.

---

## The Core Services

| Service | Principle | Responsibility |
| --- | --- | --- |
| `TransformRegistry` | #5  | Store and compute relative transforms. Lazy evaluation. |
| `StateChannel` | #6  | Publish/subscribe event bus with history. |
| `MeshLoader` | #3, #9 | Load and cache mesh files (STL, OBJ, PLY, DAE). Pure service. |
| `RobotManager` | #4, #10 | Load URDF, manage kinematic model, register transforms. |
| `CommandHandler` | #2, #7 | Route commands to active robot (simulated or real). |
| `SimulatedRobot` | #7  | Execute commands in simulation. Solve IK. Publish state. |
| `RealRobot` | —   | Bridge to hardware. Translate commands to drivers. |

---

## The UI Layer (Pure Presentation)

| Component | Responsibility |
| --- | --- |
| `UIBuilder` | Create all menus, panels, docks. Wire to StateChannel. |
| `JointControlPanel` | Display sliders for each joint. Publish `JOINT_COMMAND`. |
| `CartesianControlPanel` | Display sliders for X,Y,Z,RX,RY,RZ. Publish `CARTESIAN_COMMAND`. |
| `RobotConnectionPanel` | IP input, Connect button, Mode selector. Publish `CONNECTION_REQUEST`, `MODE_SWITCH_REQUEST`. |
| `RobotsMenu` | Show robot catalog. Publish `ROBOT_LOAD_REQUEST`. |
| `MotionContainer` | Combine connection panel + Joint/Cartesian tabs. |

---

## The Event Flow

```
User moves slider
    ↓
UI Panel publishes COMMAND event
    ↓
StateChannel distributes event
    ↓
CommandHandler receives, routes to active robot
    ↓
Robot executes command, updates kinematic model
    ↓
TransformRegistry updates transforms
    ↓
KinematicDisplay re-renders
    ↓
ROBOT_STATE event published
    ↓
UI panels update displays
```

**No direct calls. No polling. Pure event-driven architecture.**

---

## Directory Structure

```
hatch/
├── core/
│   ├── mesh_loader.py           # Pure mesh loading
│   ├── robot_manager.py         # Robot lifecycle
│   ├── command_handler.py       # Command routing
│   ├── mode.py                  # Mode enum
│   └── world_state/
│       ├── transform_registry.py
│       ├── state_channel.py
│       └── event_types.py
│
├── drivers/
│   ├── simulated_robot.py
│   ├── real_robot.py
│   └── ur_rtde_bridge.py
│
├── displays/
│   ├── kinematic_display.py
│   └── pointcloud_display.py
│
├── ui/
│   ├── ui_builder.py
│   ├── main_window.py
│   ├── menus/
│   └── panels/
│
└── assets/
    └── robots/                  # URDF files
```

---

## Event Types Reference

| Event | Direction | Data |
| --- | --- | --- |
| `ROBOT_LOAD_REQUEST` | UI → Handler | `{urdf_path, robot_id}` |
| `ROBOT_LOADED` | Handler → All | `{asset_id, urdf_path, kinematic_model}` |
| `JOINT_COMMAND` | UI → Handler | `{positions, names}` |
| `CARTESIAN_COMMAND` | UI → Handler | `{pose (4x4), frame}` |
| `MODE_SWITCH_REQUEST` | UI → Handler | `{mode: "simulate" or "real"}` |
| `MODE_SWITCHED` | Handler → All | `{mode}` |
| `ROBOT_STATE` | Robot → All | `{joint_positions, tcp_pose, timestamp}` |
| `CONNECTION_REQUEST` | UI → Handler | `{ip, frequency}` |
| `CONNECTION_ESTABLISHED` | Handler → All | `{message}` |
| `CONNECTION_LOST` | Handler → All | `{message}` |
| `ERROR_OCCURRED` | Any → UI | `{error}` |

---

## Mode States

| Mode | Description |
| --- | --- |
| `SIMULATE_LOCAL` | Local IK solver, virtual robot only |
| `SIMULATE_REAL_IK` | Real robot's IK solver, virtual robot only |
| `REAL` | Real robot's IK solver, real robot moves |

---

## Appendix A: On Event-Driven Architecture and the Main Loop

### A.1 The Clarification

Principle #2 states: *"Event-driven, no polling."*

An experienced engineer might ask: *"Doesn't every application have a main loop?"*

**Yes.** Hatch runs on Qt, which provides `QApplication.exec_()` — an event loop. The distinction is not whether a loop exists, but **who drives it**.

### A.2 What "No Polling" Means

| Pattern | Forbidden | Reason |
| --- | --- | --- |
| `while True: check()` | ✅ Forbidden | Spins CPU, wastes energy |
| `if not data: continue` | ✅ Forbidden | Same as above |
| `time.sleep(0.01); check()` | ✅ Forbidden | Still polling, just slower |
| `QTimer.timeout.connect(handler)` | ✅ Allowed | Event-driven, OS wakes on timer |
| `signal.connect(handler)` | ✅ Allowed | Event-driven, OS wakes on input |
| `StateChannel.subscribe(callback)` | ✅ Allowed | Event-driven, callback on publish |
| `get_transform()` lazy evaluation | ✅ Allowed | Computed on demand, no loop |

### A.3 The Qt Event Loop

Qt's `app.exec_()` is not polling. It uses operating system primitives (select, epoll, WaitForMultipleObjects) to **block** until an event occurs:

```python
# Simplified — Qt's actual implementation is more complex
while running:
    event = wait_for_event()  # ← Blocks, CPU sleeps
    dispatch(event)
```

When the application is idle, the thread sleeps. The CPU can be used by other processes or enter low-power states.

### A.4 Hatch's Use of the Event Loop

| Component | Mechanism | Polling? |
| --- | --- | --- |
| UI sliders | Qt signals | No (OS wakes on input) |
| Command publishing | `StateChannel.publish()` | No (direct call from event handler) |
| Robot state updates | `StateChannel.publish()` | No (called when state changes) |
| Periodic updates | `QTimer` | No (Qt manages timerfd) |
| Transform queries | Lazy evaluation | No (computed on demand) |
| File loading | User action | No (triggered by menu click) |

### A.5 Lazy Evaluation

"No polling" also means **no periodic recomputation**. Transforms are computed only when requested:

```python
def get_transform(self, target, source):
    if target not in self._cache:
        self._cache[target] = self._compute(target)  # ← Only now
    return self._cache[target]
```

If nothing asks for a transform, nothing computes.

### A.6 The Test

Ask of any component: *"Does it ever wake up to check if something has changed?"*

- If yes → **Polling** → Forbidden.
- If no → **Event-driven** → Allowed.

### A.7 The Answer to the Skeptical Engineer

> *"Every application has a main loop. Qt provides it. Hatch does not add another. All application logic is in event handlers or lazy computations. No thread ever spins waiting for something to happen. The CPU sleeps when the application is idle."*

---

## Appendix B: On Formalism and Understanding — Why Hatch Rejects Accumulated Complexity

### B.1 The Cycle

Human knowledge follows a recurring cycle:

```
Understanding → Formalization → Accumulation → Loss of Understanding → Rediscovery
```

| Phase | Description |
| --- | --- |
| **Understanding** | Direct, intuitive grasp of a phenomenon. "I see how this works." |
| **Formalization** | Abstract rules, equations, algorithms. "Let me write this down precisely." |
| **Accumulation** | Libraries, frameworks, conventions. "Everyone uses this, so I will too." |
| **Loss of Understanding** | The original meaning fades. "I know how to use it, but not why it works." |
| **Rediscovery** | Someone questions the formalism. "Why are we doing it this way?" |

We are often in the **loss phase** — surrounded by formalisms we no longer understand.

### B.2 Case Study: Quaternions

| Phase | Quaternion Journey |
| --- | --- |
| Understanding | Hamilton discovers 4D complex numbers (1843). Understands rotation in 3D space. |
| Formalization | Multiplication rules, normalization, interpolation algorithms. |
| Accumulation | ROS, game engines, spacecraft guidance systems adopt quaternions. "Standard practice." |
| Loss of Understanding | Most users cannot glance at `[0.707, 0, 0, 0.707]` and visualize a 90° rotation about X. |
| Rediscovery | Engineers ask: "Why not rotation vectors? `[1.57, 0, 0]` is immediately clear." |

**The quaternion test:** If you cannot explain what a number means to a beginner, the representation is wrong.

Quaternions fail this test. Hatch uses rotation vectors.

### B.3 Case Study: ROS Complexity

| Formalisms | Accumulated Complexity | Hatch Rediscovery |
| --- | --- | --- |
| DDS discovery | Network debugging, XML profiles | Single process, direct calls |
| Message serialization | `.msg` files, code generation | Python objects, no serialization |
| `tf2` tree | Hundreds of frames, lookup latency | TransformRegistry, lazy evaluation |
| Launch files | YAML, Python, XML hybrids | Direct orchestration in main window |
| Quaternions | Conversion every time they are used | Rotation vectors, human-readable |

ROS solves distributed robotics. But most robotics is not distributed. Most robotics is **one robot, one computer, one engineer**.

Hatch is for that engineer.

### B.4 The Danger of Formalisms

Formalisms are not evil. They are tools. But tools can become **cages**.

| Danger | Example |
| --- | --- |
| Using what exists, not what is right | "ROS uses quaternions, so I will too." |
| Accumulating without questioning | Copy-pasting code you do not understand. |
| Solving problems you do not have | Adding distribution when you have one robot. |
| Forgetting the original meaning | `F = ma` is memorized, but do you feel it? |

Hatch rejects accumulated complexity. Every component must be derived from first principles, not copied from convention.

### B.5 The Same Cycle in AI

| Phase | AI Journey |
| --- | --- |
| Understanding | Humans understand language, reasoning, causality. |
| Formalization | Statistics become matrices, transformers, billions of parameters. |
| Accumulation | LLMs generate fluent text. |
| Loss of Understanding | Engineers ask: "Does it actually understand?" |
| Rediscovery | Explainable AI, Causal AI, Grounding. |

The cycle continues. We are teaching machines to understand what humans have forgotten.

### B.6 Hatch's Position

> *"No formalism shall be used without understanding. If you cannot explain what a number means to a beginner, the representation is wrong."*

Hatch is not anti-formalism. It is anti- **blind formalism**.

| Accept | Reject |
| --- | --- |
| Rotation vectors | Quaternions (unless hidden internally) |
| Direct function calls | Serialization (unless distributed) |
| Lazy evaluation | Periodic polling |
| Event-driven architecture | ROS-style topic discovery |
| Single robot per session | Multi-robot complexity without need |

### B.7 The Purist's Question

Before adding any formalism to Hatch, ask:

1. *"Do I understand this — truly, not just how to use it?"*
2. *"Is there a simpler representation that is more human-understandable?"*
3. *"Am I solving a problem I actually have, or a problem someone else solved?"*
4. *"If this formalism disappeared tomorrow, could I rebuild it from first principles?"*

If the answer to any of these is "no," the formalism does not belong in Hatch.

### B.8 The Full Cycle

> *"From understanding to formalism, then from formalism to understanding."*

Hatch is an attempt to complete the cycle — to recover understanding from accumulated formalism.

It is not a perfect platform. It is an **honest** platform.

And honesty is the foundation of good engineering.

---

## Closing

> *"A platform is not defined by what it can do. It is defined by what it will not do — and why."*

Hatch does one thing well: **make a single robot's mind transparent, efficient, and easy to program.**

From this foundation, everything else grows.

---

*Document version 1.0*
*Hatch (孵) Architecture Foundation