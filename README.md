# CollisionBenchmarkController

An [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/) FSM controller for benchmarking collision detection algorithms on a 7-DOF robotic arm (Kinova Gen3). The controller drives the robot through a set of repeatable motion scenarios while continuously polling a collision-detection datastore flag. Upon detection, it immediately halts the robot and transitions to a structured braking state.

---

## Overview

The controller is built around a **hub-and-spoke FSM**: a central `Initial` state returns the robot to a known home posture between every benchmark run, and four independent motion states each exercise a different trajectory class. Any motion state can be interrupted mid-run by a collision detection signal, which triggers the `ReactionSimple` braking state before the hub is re-entered.

**Benchmark scenarios**

| Scenario | State | Trajectory type |
|---|---|---|
| Forward reach | `Forward` | End-effector oscillation along the X axis |
| Last-joints rotation | `LastJointsRotation` | Joint-space cycling through Up / Down / Right / Left postures |
| First-joint sweep | `FirstJointRotation` | Trapezoidal velocity profile on joint 1, back-and-forth |
| Pick and place | `PickPlace` | End-effector rectangular loop in the YZ plane |

## Dependencies

- **mc_rtc** (FSM backend: TVM)
- **Kinova Gen3** robot module (or any 7-DOF arm with matching joint names)

The following mc_rtc **plugins** must be available at runtime (configure which set applies in the YAML):

```yaml
# Simulation
Plugins: [TorqueResidual, EnergyResidual, CurrentResidual,
          CollisionDetectionZurlo, CollisionDetectionJerk,
          CollisionDetectionMomentumError, CollisionDetectionPiSliding,
          SuperTwisting, CollisionDetectionForceTorqueSensor]

# Real robot (add ROS sensor plugins)
Plugins: [RosImuSensor, RosForceSensor, TorqueResidual, ...]
```

The CMake install step places compiled state libraries under the paths referenced by `StatesLibraries` in the YAML.

---

## FSM design

### Transition table

```
Initial  ──[Forward]──────────────────────────► Forward
                                                    │ OK
                                               ReactionSimple ──[OK/Auto]──► Initial

Initial  ──[LastJointsRotation]───────────────► LastJointsRotation
                                                    │ OK
                                               ReactionSimple ──[OK/Auto]──► Initial

Initial  ──[FirstJointRotation]──────────────► FirstJointRotation_InitialPose
                                                    │ OK/Auto
                                               FirstJointRotation
                                                    │ OK
                                               ReactionSimple ──[OK/Auto]──► Initial

Initial  ──[PickPlace]───────────────────────► PickPlace_InitialPose
                                                    │ OK/Auto
                                               PickPlace
                                                    │ OK
                                               ReactionSimple ──[OK/Auto]──► Initial
```

The FSM is **not managed** (`Managed: false`) and runs **step-by-step** (`StepByStep: true`), meaning an external tool or operator must trigger each outgoing transition from `Initial` by name. All downstream transitions are `Auto` (fired immediately on output).

### Collision interrupt

Every motion state checks the datastore flag `"Obstacle detected"` at the top of its `run()` loop. When `true`:

1. The posture task is reset to the current configuration with high stiffness (500).
2. The EE task is removed from the solver (where applicable).
3. The state outputs `"OK"`, triggering the auto-transition to `ReactionSimple`.

`Initial` resets the flag to `false` on re-entry, clearing it for the next run.

---

## States

### `Initial`

**Purpose**: safe home position, flag reset hub between runs.

- Sets posture task stiffness to 1, damping to 2, targets `postureHome`.
- Removes the EE task from the solver (so a previously running Forward or PickPlace state leaves a clean slate).
- Clears `"Obstacle detected"` in the datastore on every `run()` tick while the flag is set.
- Logs a success message when the home posture error drops below 0.05 rad.
- Never auto-exits: awaits an explicit external transition trigger.

### `Forward`

**Purpose**: end-effector oscillation along the X axis (home ↔ forward).

- Control mode set to `"Position"` in the datastore.
- Posture task kept with very low stiffness (0.5) and weight (0.1) as a background regulariser.
- EE position task stiffness 400 for the outward stroke, 10 for the return.
- Toggles between `taskPosHome` and `taskPosForward` every time position error < 0.01 m.
- Interrupted immediately on `"Obstacle detected"`.
- Teardown removes the EE task from the solver.

### `LastJointsRotation`

**Purpose**: joint-space benchmark exercising the wrist (joints 5–6) and shoulder (joint 2) in four cardinal postures.

The state cycles through a home waypoint between every posture change:

```
Home → Up → Home → Down → Home → Right → Home → Left → Home → Up → …
```

Posture switches are gated on `eval().norm() < 0.1 rad`. All motion uses posture task stiffness 400. Interrupted immediately on collision detection.

### `FirstJointRotation_InitialPose`

**Purpose**: move the robot to a pre-defined neutral pose before the joint-1 sweep begins.

- Targets `postureFirstJointRotation` with stiffness 400.
- Outputs `"OK"` (auto-transition) once `eval().norm() < 0.05 rad`.

### `FirstJointRotation`

**Purpose**: continuous back-and-forth sweep of joint 1 from 0 to 1.57 rad using a trapezoidal velocity profile.

The velocity profile is computed analytically in `computeTrapezoidVelocity()`:

```
Phase 1 — acceleration:   0 → vel_max  over tf_acc
Phase 2 — constant speed: vel_max      over tf_const
Phase 3 — deceleration:   vel_max → 0  over tf_acc

Total time = Δq / vel_max / (1 − accel_ratio)
```

Parameters (header defaults):

| Parameter | Default | Description |
|---|---|---|
| `vel_max_` | 0.3 rad/s | Peak joint velocity |
| `accel_ratio_` | 0.2 | Fraction of total time spent accelerating |

The direction flag `sign_` flips at each end of the stroke, producing continuous oscillation. Both `refVel` and `refAccel` are fed to the posture task on every tick for feed-forward control. Interrupted immediately on collision detection.

### `PickPlace_InitialPose`

**Purpose**: move the EE to the starting corner (`taskPosPickPlaceDownRight`) before the pick-and-place loop begins.

- EE position stiffness 400, weight 10 000; posture task used as low-weight background (stiffness 0.5, weight 0.1).
- Outputs `"OK"` (auto-transition) once position error < 0.01 m.

### `PickPlace`

**Purpose**: rectangular end-effector loop in the YZ plane, simulating a pick-and-place cycle.

Corner traversal order:

```
DownRight (DR) → UpRight (UR) → UpLeft (UL) → DownLeft (DL)
                                                     │
UpLeft (LU) ← UpRight (RU) ←────────────────────────┘
```

A six-state enum drives the transitions; corners switch on position error < 0.01 m. The orientation task is not used in this state (set separately on the EE task before entry). Interrupted immediately on collision detection.

### `ReactionSimple`

**Purpose**: structured braking — hold joint positions until all joints come to a standstill, then signal completion.

Algorithm:

1. Reset posture task to the current configuration; stiffness and damping both set to 500.
2. On each tick, check every joint's velocity via `robot.alpha()`. A joint is considered "stopped" when `|velocity| ≤ 0.001 rad/s`.
3. Once **all** joints are stopped, start a `stop_time_` (default 0.5 s) counter.
4. If any joint is still moving, reset the counter and re-apply the high-stiffness hold.
5. After the counter elapses, output `"OK"` → auto-transition back to `Initial`.

---

## Datastore interface

| Key | Type | Direction | Description |
|---|---|---|---|
| `"Obstacle detected"` | `bool` | Plugins → states | Set to `true` by any collision-detection plugin |
| `"getPostureTask"` | callable | Controller → plugins | Returns `PostureTaskPtr` |

Collision detection plugins write to `"Obstacle detected"`. All motion states read this key on every tick. `Initial` clears it on re-entry.

## Running

### Simulation

```bash
# Launch with the simulation plugin set
mc_mujoco --sync
```

### Real robot

Swap the `Plugins` list in the YAML (uncomment the real-robot line, comment the simulation line), then launch your robot interface as normal. The datastore key `"ControlMode"` is read by the hardware driver to switch between position and torque control.

Run the Bota driver to let the plugins have access to the Bota ros2 topics.

```bash
# Launch with the real robot plugin set
mc_kortex
```

## Extending

**Tuning the trapezoid sweep**: adjust `vel_max_` and `accel_ratio_` in `CollisionBenchmarkController_FirstJointRotation.h`. The total time and acceleration are recomputed automatically in `start()`.

**Changing collision sensitivity**: configure thresholds inside the relevant detection plugin (e.g. `CollisionDetectionJerk`, `CollisionDetectionZurlo`). The FSM only observes the binary `"Obstacle detected"` flag.