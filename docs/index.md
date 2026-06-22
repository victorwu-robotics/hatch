---
title: Hatch (孵) — Robotics Platform
---

# Hatch (孵)

![Architecture Diagram](images/hatch_top_level_architecture.png)

**A single-process, event-driven robotics platform for one robot arm.**

---

## For the Young Engineer

If you are about to use a robot arm for the first time, you are where I was. You have the hardware. You have the task. You do not yet have the mental model.

This is normal. This is correct. The mental model is not trivial.

Hatch is my attempt to give you what I did not have: a transparent space where every concept is visible, every movement is inspectable, and every decision is yours.

- **Move one joint at a time.** See the pose change. That is *joint space*.
- **Move the tool in XYZ.** See the joints solve. That is *Cartesian space* and *Inverse Kinematics*.
- **See the transform tree update in real time.** That is *knowing where everything is*.
- **See every configuration, every limit, every possibility.** That is *understanding before trusting*.

Use Hatch to learn. Then use whatever framework fits your production needs. The understanding you build here will serve you in any system.

[Hatch is pre-flight. The flight is yours.](#philosophy)

---

## Try Hatch in 5 Minutes

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window
```

File → Load URDF → Select your robot → Move a slider → See it move.

Full Getting Started Guide →
What Hatch Is
Table
Principle	What It Means
Single Process	One Python process. One memory space. No distributed systems.
Event-Driven	No polling. The timer is the only driver.
Everything in URDF	The scene description is the single source of truth.
Visualizer as Mind-Prying Tool	See every transform, every state, every decision.
UI Separate from Services	Panels publish events. They do not control.
Full Architecture →
Documentation Paths
Table
I want to...	Start here
Try Hatch now	Getting Started (30 min)
Understand the philosophy	Philosophy (10 min summary, 2 hr deep)
Understand the architecture	Architecture (diagram + walkthrough)
Connect real hardware	Integrating Hardware
Extend Hatch	API Reference
Read the story	Technical Notes
Principles
#0 Individuals Before Groups | #1 Single Process | #2 Event-Driven | #3 Visualizer as Mind-Prying Tool | #4 Everything in URDF | #5 Space = TransformRegistry | #6 Time = StateChannel | #7 Movements as Models | #8 Pure Python | #9 UI Separate from Services | #10 One Robot Per Session
Full Principles →
plain

---

## **Where "For the Young Engineer" Goes**

| Location | Purpose |
|----------|---------|
| **`index.md`** (top) | First thing visitors see — sets the tone |
| **`philosophy.md`** (top, before Origin Story) | Deeper context for readers who want the full story |
| **`getting-started.md`** (introduction) | Reminds learners why they are here |

---

Should I proceed with **Recommendation 2: Create `getting-started.md`**? Or would you like to refine the `index.md` draft first?