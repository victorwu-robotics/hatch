# Non-Polling RTDE Robot Driver for Hatch

## Principles

This driver follows Hatch's Principle #2: **Event-Driven, No Polling**.

- No continuous 125Hz state reading loop
- No periodic checking of robot state
- State is published **only when it changes** (after commands)

---

## How UR RTDE Supports Non-Polling

The `ur_rtde` library provides **blocking move commands**:

| Command | Behavior | Returns When |
|---------|----------|--------------|
| `moveJ(q, speed, acc)` | Blocks until robot reaches target | Movement complete |
| `moveL(pose, speed, acc)` | Blocks until robot reaches target | Movement complete |

When the function returns, the robot is **guaranteed** to be at the target position.

---

## The Command-Response Flow

```
User action (slider moved)
    ↓
JOINT_COMMAND published
    ↓
RealRobot.move_joints() called
    ↓
rtde_c.moveJ() ← BLOCKS (waits for movement)
    ↓
(robot moves)
    ↓
moveJ() returns (movement complete)
    ↓
rtde_r.getActualQ() reads final position
    ↓
ROBOT_STATE published
    ↓
Virtual robot updates
    ↓
(Optional) UI updates
```

**No polling. No separate thread. No 125Hz loop.**

---

## Implementation: RealRobot.move_joints()

```python
def move_joints(self, positions: List[float]) -> bool:
    """
    Move robot to target joint positions.
    
    Blocks until movement completes, then publishes ROBOT_STATE.
    """
    if not self.is_connected:
        return False
    
    try:
        # Send command and wait for completion (blocking)
        success = self._rtde_c.moveJ(
            positions,
            speed=0.5,
            acceleration=0.5
        )
        
        if not success:
            return False
        
        # Movement complete — read final state
        final_q = self._rtde_r.getActualQ()
        final_tcp = self._rtde_r.getActualTCPPose()
        
        # Publish state event
        self._channel.publish(
            EventType.ROBOT_STATE,
            data={
                'joint_positions': final_q,
                'tcp_pose': final_tcp,
                'timestamp': time.time()
            },
            source="real_robot"
        )
        
        return True
        
    except Exception as e:
        self._channel.publish(
            EventType.ROBOT_ERROR,
            data={'error': str(e)},
            source="real_robot"
        )
        return False
```

---

## For Cartesian Control (moveL)

Same pattern, different command:

```python
def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
    """Move robot to target Cartesian pose (blocking)."""
    if not self.is_connected:
        return False
    
    pose_list = self._transform_to_pose_list(pose)
    
    try:
        success = self._rtde_c.moveL(
            pose_list,
            speed=0.5,
            acceleration=0.5
        )
        
        if not success:
            return False
        
        # Movement complete — read final state
        final_q = self._rtde_r.getActualQ()
        final_tcp = self._rtde_r.getActualTCPPose()
        
        self._channel.publish(
            EventType.ROBOT_STATE,
            data={
                'joint_positions': final_q,
                'tcp_pose': final_tcp,
                'timestamp': time.time()
            },
            source="real_robot"
        )
        
        return True
        
    except Exception as e:
        self._channel.publish(
            EventType.ROBOT_ERROR,
            data={'error': str(e)},
            source="real_robot"
        )
        return False
```

---

## What About Continuous Command Streams?

For applications requiring smooth, uninterrupted motion (e.g., path following):

| Scenario | Approach |
|----------|----------|
| Single command (jogging) | Blocking mode |
| Multiple commands (trajectory) | Async mode for all but last |
| Final command | Blocking mode to confirm completion |

```python
# Example: Streaming multiple poses
for pose in poses[:-1]:
    rtde_c.moveL(pose, speed, acc, async=True)

# Final pose — block to confirm completion
rtde_c.moveL(poses[-1], speed, acc)

# Now read final state
final_q = rtde_r.getActualQ()
```

---

## Comparison: Polling vs. Non-Polling

| Aspect | Polling (125Hz) | Non-Polling (Command-Response) |
|--------|-----------------|-------------------------------|
| CPU usage | High (continuous) | Low (only on command) |
| Threads | Separate thread needed | Same thread |
| State freshness | Continuous | After each command |
| Detects external changes | ✅ Yes | ❌ No (assumes Hatch is sole controller) |
| Aligns with Principle #2 | ❌ No | ✅ Yes |

---

## Assumptions and Limitations

This non-polling approach assumes:

| Assumption | Implication |
|------------|-------------|
| Hatch is the sole command source | No external changes to monitor |
| Safety is handled by hardware | E-stop, collision detection, limits |
| Manual intervention is out of scope | Operator responsible for inconsistencies |

These assumptions align with Hatch's philosophy and scope.

---

## Summary

| Component | Behavior |
|-----------|----------|
| `RealRobot.move_joints()` | Blocking command → publish state on completion |
| `RealRobot.move_pose()` | Blocking command → publish state on completion |
| RTDE receive interface | Used only after commands (not continuous) |
| `ROBOT_STATE` event | Published once per command (when movement completes) |

**No polling. No separate thread. True event-driven control.**
 🐣