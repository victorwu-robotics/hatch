# Sensing Funnel — Design Note

**Status: PROPOSAL. Semantics argued with the maintainer; no code written yet.**
Drafted against `main @ ecc6a80`. Nothing here is implemented.

Scope: how a measurement becomes a path. The contracts between sensing, geometry
and motion, and the invariants that make sensors interchangeable. **Not** a motion
document — no feedrates, no interpolation, no execution. Those consume what is
defined here.

Every claim below is labelled **measured** (executed against this repo or a cited
specification) or **derived** (calculated, assumptions stated). Nothing is asserted
because it sounds right.

---

## 1. Why measurement is mandatory

Typed dimensions and CAD describe a part **in the part's own frame**. Without
locating that frame relative to the robot base, the data is unusable no matter how
accurate it is. On a construction site there is no fixture, so localisation cannot
be assumed.

This is why "all paths must come from vision" feels forced. But *vision* conflates
two different measurements:

- **recognising what a thing is** from its appearance — photometric, dominated by
  ambient light;
- **measuring where a surface is** — geometric, and largely ambient-immune if the
  optical chain is designed for it.

Seam finding is the second. The goal is not a picture of the workpiece; it is the
intersection line of two surfaces.

---

## 2. What already exists (measured)

| fact | where |
|---|---|
| The maintainer's sensing pipeline works — in ROS. Hatch is a re-foundation, not a research project. | stated by the maintainer |
| Scanner acquisition exists; `request_profile()` **triggers** one capture on demand | `drivers/camera/keyence_scanner.py` |
| `capture_profiles(count, interval)` spaces captures by **wall clock only** | same file |
| Profiles are returned as `(points, colors)` in the **scanner frame**, with **no robot pose and no timestamp** | same file |
| Camera drivers likewise return raw arrays | `realsense_capture.py`, `orbbec_capture.py` |
| `transform_registry.py` contains **no** time, stamp, buffer, history or interpolation | grep, zero hits |
| `StateChannel` events **do** carry a timestamp | `core/world_state/state_channel.py` |
| Tool transform is implemented and honoured: `inv(T_root_world) @ T_tcp_world @ inv(T_tool)` | `core/kinematics/ik_solver.py` |
| …and tested with a bent torch, and with the arm on a pedestal | `test_ik_solver.py` |
| `set_tool_transform` is called **only** by that test; default is identity | grep |
| **No** path, trajectory, waypoint or playback type exists anywhere | grep, zero hits |
| `scipy 1.15.3` and `trimesh 4.12.0` are already pinned | `requirements.txt` |

So the geometry kernel is ready and the sensing *contracts* are absent. Today the
interface is the Keyence's output format. That is the coupling this note removes.

---

## 3. The funnel: three phases, three accuracy budgets

| phase | arc | scene | field of view | accuracy needed |
|---|---|---|---|---|
| **A** — locate the member and groove direction | off | static | ~1 m | **~10 mm** |
| **B** — measure the groove | off | static | ~40 mm | **~0.2 mm** |
| **C** — weld, and hold the torch on the seam | **on** | dynamic | ~40 mm | ~0.5 mm |

The budgets are forced by optics, not chosen. A profiler with a 25–39 mm field
cannot search a metre-long member; it must be *placed* on the groove first. And
placing a 40 mm field needs only ~10 mm — a **50× looser** budget than measuring
the groove.

**The coarse stage is optically easy precisely because it is allowed to be wrong.**
Ambient light destroys fine photometric detail long before it destroys "where is
the large steel member and which way does its edge run".

---

## 4. Phase A — coarse localisation

### 4.1 What the sensors are (measured, cited)

- D415: baseline **55 mm**, HFOV **65°**. D435: baseline **50 mm**, HFOV **90°**
  ([Intel tuning guide](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)).
- D455: baseline **95 mm**, global shutter
  ([comparison](https://physicalailab.io/en/realsense-d455-vs-d435i-comparison/)).
- Depth error **scales as the square of distance** (Intel, same page).
- Intel's own performance verification is a **textured** wall (same page).

### 4.2 Derived depth noise

`RMS ≈ Z² · subpixel / (f · B)`, subpixel 0.08, 848 px. **Derived**, not measured:

| standoff | D435 (f=424 px, B=50 mm) | D455 (f=447 px, B=95 mm) |
|---|---|---|
| 0.7 m | 1.85 mm | 0.92 mm |
| 1.0 m | 3.77 mm | 1.88 mm |
| 1.5 m | 8.49 mm | 4.24 mm |
| 2.0 m | 15.09 mm | 7.54 mm |

Against a ~10 mm budget: usable **inside ~1.5 m**, and the longer baseline roughly
halves the error for free. On a *textured* surface.

### 4.3 The failure mode is not brightness

The D400 is passive stereo with an *optional* IR projector. Intel states these
cameras "tend to perform even better in bright light" (same page) — true, because
more ambient light improves imager SNR. Two things break anyway:

1. **The projector is washed out in sun**, so depth depends entirely on the
   surface's own texture. Mill-scale steel plate is largely textureless and
   specular: nothing to match.
2. **Specular glint** saturates locally regardless of average illuminance.

### 4.4 The insight that rescues the coarse stage

**Per-point accuracy is not required. Plane parameters are.** The groove comes
from intersecting two large planes; fitting a plane averages random noise by ~√N.
**Derived**, for 4 mm per-point RMS, unbiased:

| points | error on the plane offset |
|---|---|
| 100 | 0.4000 mm |
| 1 000 | 0.1265 mm |
| 10 000 | 0.0400 mm |

What defeats this is **bias**, not noise: glint, mixed pixels at edges. Hence four
countermeasures, all cheap:

- a **linear polariser** — recommended by Intel's own tracker for glare off
  reflective surfaces ([issue 10229](https://github.com/realsenseai/librealsense/issues/10229));
- the **"High Accuracy" preset** — Intel: *"false depth, aka Hallucinations, are
  much worse than no depth"*. This is Principle-level agreement: prefer holes to lies;
- **get close** — error ∝ Z²;
- **use RGB edges for the groove direction**, depth for the planes. A groove is a
  strong intensity edge even where depth is empty.

HDR is the wrong axis. It addresses dynamic range *within* a frame; the problem is
signal-to-background photon ratio, and background shot noise grows as √N_bg. A
deeper well does not recover a line buried in background photons.

---

## 5. Phase B — groove measurement

### 5.1 The instrument (measured, cited)

LJ-V7080: **405 nm**, **4.8 mW**, height ±23 mm, width **25–39 mm**, standoff
**32–80 mm**
([datasheet](https://www.scribd.com/document/604328210/LJ-V7080-Datasheet)).

### 5.2 The ambient rating is in the wrong units

LJ-V7000 is rated *"Ambient light: incandescent lamp, 10,000 lux max"*
([Keyence specs](https://www.keyence.eu/products/measure/laser-2d/lj-v/specs/)).
Lux is photopic-weighted. A ~2800 K tungsten lamp emits very little at 405 nm;
daylight emits a great deal. **The rating therefore cannot be transferred to
sunlight** — it overstates daylight immunity. *(Derived.)*

### 5.3 Why a few milliwatts is not drowned

**Derived**, assumptions stated: a 30 mm line 0.07 mm wide is 2.10 mm², so
4.8 mW gives **2286 W/m²** at the surface. Direct sun is ~1.5 W/m²/nm at 405 nm;
through a **10 nm** bandpass that is **15 W/m²**. Ratio **152 : 1** in favour of the
line, before any temporal gating.

Blue is chosen deliberately: solar spectral irradiance is lower at 405 nm than in
red/NIR, and hot steel's thermal emission is negligible there.

The ambient problem is therefore solved by **excluding background photons**, not
by tolerating them: spectral filtering, pulsed illumination with short exposure,
imaging only the line plane, and laser on/off differencing in software. Purpose-built
seam sensors are advertised as working "in all lighting"
([Yaskawa comparison](https://www.motoman.com/en-us/about/blog/pros-and-cons-of-robotic-seam-finding-and-seam-tra))
for exactly this reason.

**Consequence for this project:** the LJ-V is a factory instrument. Outdoor Phase B
needs a filtered seam sensor or an equivalent optical chain. Software does not fix
this, and Hatch must not pretend it does.

---

## 6. Phase C — the arc, and how to avoid it

The arc is the extreme optical regime: broadband, intense, moving. HDR fails here,
not during the search.

**The proposal avoids the regime entirely.** Thick-plate work is multi-pass, and
**between passes the arc is off**:

> measure groove → plan pass → weld pass → stop → **re-measure** → plan next pass

Every measurement happens on a static scene in benign light. Distortion between
passes — the thing a tracking loop fights — is handled by re-measurement instead.
This is not a compromise; it is exploiting when the physics is easiest.

Real-time through-arc tracking remains a later option, and it is light-free
(needs weaving and thicker material — Yaskawa, same page).

---

## 7. The contracts: sensor-independent, or nothing is swappable

Two boundaries, described in terms no instrument owns.

**Phase A output — `Localisation`**
- `T_base_part` (4×4)
- confidence
- provenance: sensor identity, standoff, exposure, capture index

**Phase B output — `GrooveSection`**
- two surface planes, or a cross-section polyline
- gap, angle, depth
- expressed in a groove frame, **arc-length parameterised**
- confidence

Not point clouds. Not profiles. Those are artefacts of a particular instrument.
With these boundaries, D415/D435/D455/LiDAR are interchangeable behind the first,
and Keyence/Scansonic/Micro-Epsilon behind the second. Motion consumes **geometry**,
never a profile.

---

## 8. The capture record, and the invariant that replaces tf

**No timestamps. No tf.** Principle #1: single process, single memory space.
Profiles arrive in sequence and are consumed in the same sequence. There is no
transport latency, no serialization, no cross-process clock to reconcile.

What makes that correct is not the absence of a clock but the presence of the pose:

> **A capture record is `(points, q_at_capture, T_flange_optical)`.**

The joint vector is read **synchronously in the same call** that triggers the
capture. The record is then self-describing: nothing needs looking up later, so
there is nothing to interpolate, extrapolate or guess — and the on-disk recording
reproduces the geometry with no ambient state.

**The invariant, enforced rather than assumed:**

> A measurement is valid only when taken with the arm **settled** — not
> "command finished". Capture refuses if motion is in flight.

Why it must be enforced. **Derived** displacement if a profile is composed with a
stale flange pose:

| scan speed | 40 ms stale | 100 ms stale |
|---|---|---|
| 50 mm/s | 2.00 mm | 5.00 mm |
| 100 mm/s | 4.00 mm | 10.00 mm |

Against a 0.2 mm budget that is 10–50× over, and it looks *plausible* — which is
what makes it dangerous. With the arm settled the term is zero, and device latency
becomes irrelevant.

**Stop-and-scan is affordable.** *(Derived: ~30 mm usable field, 50% overlap →
~67 stops per metre; ~0.4 s settle-and-capture plus short moves ≈ 40 s per scan,
~80 s per pass with before-and-after.)* Against a site weld lasting minutes, that
is noise. If fly-scan is ever wanted, the **invariant** is revisited — not the
architecture, and not the record format.

---

## 9. One localisation maths, three jobs

Tool calibration, scanner hand-eye, and touch-based part localisation are the
**same linear problem**. For each touch-up pose `i`:

```
R_i · t + p_i = P        →        [R_i  −I] · [t; P] = −p_i
```

Stack and solve by least squares. `t` is the offset (torch tip, scanner origin,
wire tip), `P` the fixed point (pivot, or workpiece corner). **No IK, no iteration,
no numerical solver** — it does not touch the refusal of numerical IK.

**Measured** (`measure_tool_and_playback.py`, UR10, true tool
`[31, −47, 183]` mm, 0.2 mm Gaussian noise on each position):

| poses | tool error | pivot error |
|---|---|---|
| 3 | 0.4460 mm | 0.6071 mm |
| 4 | **0.1828 mm** | 0.1636 mm |
| 6 | 0.1669 mm | 0.1144 mm |

Noise-free: exact in all three cases. This is why four-point touch-up is industrial
practice, and it is a `numpy.linalg.lstsq` call.

**A measured correction worth recording:** a 1° error in the *tool's own rotation*
moves the TCP origin by **0.000000 mm** and rotates the TCP frame by **1.000000°**.
Because `t` is expressed in flange coordinates, `R_tool` cannot move the origin.
The familiar `|t|·sin(δθ)` figure applies to an error in the **robot's** orientation
acting on the offset, not to the tool angle. Torch angle still matters for the
process; it is not a position error.

**Touch sensing is light-free** and needs no hardware on the arm — the weld supply
provides the low-voltage circuit; it locates most lap and fillet joints, is slower
than optics, and cannot find square butt joints (Yaskawa, same page). On site,
"slower" is nearly free.

---

## 10. Process independence

The funnel is not a welding pipeline. What changes per process is the last column:

| process | fine geometry needed | tolerance | fine stage |
|---|---|---|---|
| welding | groove gap, angle, depth → pass plan | ~0.2–0.5 mm | laser, essential |
| painting | surface, standoff, overlap, coverage | ~5 mm | **coarse may suffice** |
| drilling | hole position + surface normal | ~0.5 mm | normal, not groove |

Three consequences:

1. **Stages must be skippable.** A hardcoded three-stage funnel is a welding
   appliance, not a platform.
2. **`Process` sits beside `CartesianPath`.** Geometry and intent are separate data.
3. **Validation tolerances come from `Process`.** A painting path 3 mm off is fine;
   a weld 3 mm off is a reject. Same path type, different acceptance.

Drilling caveat, recorded so it is not discovered late: the binding constraint is
**stiffness**, not sensing. An arm deflects under drill thrust. The feed axis belongs
in the tool, so the robot only positions — which makes it an actuated tool, and a
tool transform question again.

---

## 11. Refusal, and confidence as a first-class output

Every stage emits a confidence, and **a path is refused when the measurement is not
trustworthy**. Not a warning appended to a path that gets executed anyway.

This is the constitution applied to perception. Most systems return a seam whether
or not they found one; Intel's own guidance says hallucinations are worse than no
depth. Hatch's equivalent: *no seam is a real answer; a fabricated seam is not.*

Refusal must carry a reason: fill rate too low, plane fit residual too high, glint
saturation, groove not in field, arm not settled, localisation residual too large.

---

## 12. Verification plan

**The working ROS system is the oracle.** Record there, replay in Hatch, compare
numerically: same hand-eye composition, same fitted planes, same groove, same path.
Acceptance becomes a number rather than a confidence, and a proven system can be
ported without quietly regressing it. The capture record of §8 makes this straight-
forward — the pose is already in the record, so there are no tf semantics to
reproduce.

**The plate experiment**, which settles Phase A by data instead of argument:
plane-fit RMS and fill rate on real mill-scale plate, in sun and in shade, at
0.7 / 1.0 / 1.5 m, with and without a polariser, on D415 / D435 / D455. All three
instruments are already owned.

---

## 13. Open decisions

1. **Is the robot clamped to the member (crawler) or beside it?** If clamped, the
   part pose is fixed by construction and Phase A shrinks to a local search on a
   known section. This is the largest single lever on the whole design.
2. **Which groove types and plate thicknesses first?** Fillet and lap on sheet are
   tractable from a profile (piecewise-linear fit, breakpoints, plane intersection).
   General seam extraction is not.
3. **Phase B instrument outdoors** — filtered seam sensor, or a shrouded LJ-V?
   A hardware decision, not a software one.
4. **Is a weld supply's touch-sense / arc signal available?** If yes, the light-free
   route is open for localisation and cross-checking.
5. **Where does `Localisation` live** — a URDF override, a calibration record, or
   both? Philosophy B.3 currently states there is no override mechanism; the tool
   transform *is* one, so this needs reconciling.

---

## 14. Compliance

- **#1 Single Process, Single Memory Space** — §8: sequencing and a bound pose
  replace temporal bookkeeping. No tf, no timestamps.
- **#3 Visualizer as Mind-Prying Tool** — the measured geometry and the fitted
  groove are drawn from the same arrays the geometry code uses, so the operator
  sees what was computed, not a decimated stream.
- **#4 Everything in URDF** — sensor mounts come from the URDF; calibration refines
  the optical-frame transform, not the scene description.
- **#7 Movements as Models** — `Localisation`, `GrooveSection`, `Process` are data.
- **#8 Pure Python** — the extraction needs `numpy` and `scipy`, both already pinned.
- **No numerical IK** — §9 is linear least squares, not an IK solver.
- **No quaternions in the public API** — poses are 4×4 internally, rotation vectors
  in every readout and record.
- **Refuse loudly** — §11.

---

*Nothing in this note is implemented. It is the contract to agree before the
sensing code is ported, so that the instruments remain replaceable.*
