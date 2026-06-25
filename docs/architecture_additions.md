## Data Flow Diagram

The event flow (user → UI → command → robot → state → display) shows *control*.
The data flow shows *state* — how information moves through the system as the robot moves.

![Hatch (孵) 🐣 Data Flow](images/hatch_data_flow_diagram.png)

**Key invariant:** `StateHandler` is the *only* component that writes to `KinematicModel` and `TransformRegistry` after initial load. All other components read.

**Key performance property:** When the robot is stationary, the render loop checks one boolean per display and returns. No VTK operations. No transform recomputation. CPU usage approaches zero.

---

## Mode State Machine

```
                    ┌─────────────────┐
                    │  SIMULATE_LOCAL │
                    │  (local IK,     │
                    │   virtual only) │
                    └──┬───────────┬──┘
                       │           ▴    
                    connect        │
                (real robot)   disconnect
                       │           │
                       ▼           │
                    ┌──┴───────────┴──┐
                    │ SIMULATE_REAL_IK│
                    │ (real IK,       │
                    │  virtual only)  │
                    └────────┬────────┘
                             │
                    switch to REAL
                             │
                             ▼
                    ┌─────────────────┐
                    │      REAL       │
                    │ (real IK,       │
                    │  real movement) │
                    └─────────────────┘
                             │
                    switch to SIMULATE
                             │
                             └────► sync virtual robot to real position
                                   (prevents jump when returning to sim)
```

**Why three modes?**

| Mode | Use Case | Safety |
|------|----------|--------|
| `SIMULATE_LOCAL` | Test URDF, verify kinematics, develop without hardware | Cannot move real robot |
| `SIMULATE_REAL_IK` | Validate IK accuracy against real controller before moving | Uses real solver but no motion |
| `REAL` | Full operation | Hardware moves — use with caution |

---
