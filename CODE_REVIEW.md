# Code Review: FRC Team 6651 (AGE) — 2026 Season

**Reviewed at commit:** `418c1cf Day 1.5 Peoria`
**Date:** 2026-03-20

---

## Table of Contents

- [Critical Defects](#critical-defects)
- [Significant Defects](#significant-defects)
- [Controllability Issues](#controllability-issues)
- [Improvement Suggestions](#improvement-suggestions)

---

## Critical Defects

### C1. `seed_pigeon_with_vision` crashes when vision data is available

**File:** `subsystems/command_swerve_drivetrain.py:333-338`

**Problem:** Three compounding bugs in the seeding success path:

1. `best_pose.x()` — in WPILib Python, `Pose2d.X()` returns a float (capital X). `.x()` may not exist, and `.x().value` will certainly fail.
2. `best_pose` is reassigned from a `Pose2d` to a plain tuple, then `best_pose.rotation().degrees()` is called on that tuple, producing an `AttributeError`.
3. `self.reset_pose(best_pose)` expects a `Pose2d`, not a tuple.

The net effect: the robot crashes the moment a Limelight provides a valid MT1 pose.

**Current code:**
```python
if best_pose is not None:
    angle = 0 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else 180
    best_pose = (best_pose.x().value, best_pose.y().value, angle)
    self.reset_pose(best_pose)
    print(f"Pigeon SEEDED with MT1: {best_pose.rotation().degrees():.2f}°")
    self._PIGEON_SEEDED = True
```

**Proposed fix:**
```python
if best_pose is not None:
    angle = 0 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else 180
    seeded_pose = Pose2d(best_pose.X(), best_pose.Y(), Rotation2d.fromDegrees(angle))
    self.reset_pose(seeded_pose)
    print(f"Pigeon SEEDED with MT1: {seeded_pose.rotation().degrees():.2f}°")
    self._pigeon_seeded = True
```

> Note: This fix also incorporates the fix for C2 below (instance variable rename).

---

### C2. `_PIGEON_SEEDED` is a class variable shared across all instances

**File:** `subsystems/command_swerve_drivetrain.py:16`

**Problem:** `_PIGEON_SEEDED = False` is defined at class level. Once set to `True`, the back-button re-seed binding will silently do nothing because the check at line 317 returns early. Additionally, the back button calls `seed_pigeon_with_vision()` which has no way to bypass the flag.

**Current code (class level):**
```python
class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    _PIGEON_SEEDED = False
```

**Proposed fix — step 1:** Remove the class variable and initialize in `__init__` (after line 152):
```python
self._pigeon_seeded = False
```

**Proposed fix — step 2:** Add a `force` parameter to the seeding method:
```python
def seed_pigeon_with_vision(self, force=False):
    if self._pigeon_seeded and not force:
        return
    # ... rest of method using self._pigeon_seeded ...
```

**Proposed fix — step 3:** Update the back button binding in `robotcontainer.py:219-220`:
```python
self._joystick.back().onTrue(
    commands2.cmd.runOnce(lambda: self.drivetrain.seed_pigeon_with_vision(force=True))
)
```

**Proposed fix — step 4:** Update every reference from `self._PIGEON_SEEDED` to `self._pigeon_seeded` within the method body (lines 317 and 339).

---

### C3. Turret `HUB_POSITION` is never set to a real value

**File:** `subsystems/turret.py:30-39`

**Problem:** The constructor checks alliance color and sets `self.TEAM_COLOR` to a string, but `self.HUB_POSITION` stays at `Pose2d(0, 0, 0)` (field origin). If `aim_at_hub()` is ever called, the turret will always aim at the origin instead of the actual hub.

**Current code:**
```python
self.HUB_POSITION: Pose2d = Pose2d(0, 0, 0)

_team_color = DriverStation.getAlliance()
if _team_color == DriverStation.Alliance.kRed:
    self.TEAM_COLOR = "Red"
else:
    self.TEAM_COLOR = "Blue"
```

**Proposed fix:**
```python
self.TEAM_COLOR: str = ""
self.HUB_POSITION: Pose2d = Pose2d(0, 0, 0)

_team_color = DriverStation.getAlliance()
if _team_color == DriverStation.Alliance.kRed:
    self.TEAM_COLOR = "Red"
    self.HUB_POSITION = self.red_hub
else:
    self.TEAM_COLOR = "Blue"
    self.HUB_POSITION = self.blue_hub
```

> Note: Since `getAlliance()` may return `None` during early init, consider also re-checking alliance in `periodic()` and updating `HUB_POSITION` if it was initially set to the default. See also S7.

---

### C4. Turret `aim_at_hub` has a type error on subtraction

**File:** `subsystems/turret.py:154`

**Problem:** `self.HUB_POSITION` is a `Pose2d`, and `robot_pose.translation()` is a `Translation2d`. WPILib's `Pose2d` does not support direct subtraction with `Translation2d`. This will raise a `TypeError` at runtime.

**Current code:**
```python
target_vector = self.HUB_POSITION - robot_pose.translation()
```

**Proposed fix:**
```python
target_vector = self.HUB_POSITION.translation() - robot_pose.translation()
```

> Note: The same pattern exists in `aim_at_hub` only. `aim_to_position` (line 211) already correctly uses `.translation()` on both operands.

---

### C5. Duplicate `_drive_gains` definition — second silently shadows the first

**File:** `generated/Crimson_tuner_constants.py:57-75`

**Problem:** `_drive_gains` is defined twice at class level. The second definition (line 67) completely replaces the first (line 57). The active gains have `k_s=0`, `k_v=0`, `k_a=0` — meaning the drive motors have **no feedforward** in TORQUE_CURRENT_FOC mode. PID alone (kP=1.5) must overcome friction and maintain velocity, which will be sluggish and poorly damped.

**Current code:**
```python
_drive_gains = (  # First definition (line 57) — DEAD CODE
    configs.Slot0Configs()
    .with_k_p(5.06).with_k_s(3.26).with_k_v(2.19).with_k_a(0.04)
)

_drive_gains = (  # Second definition (line 67) — THIS IS WHAT RUNS
    configs.Slot0Configs()
    .with_k_p(1.5).with_k_s(0).with_k_v(0).with_k_a(0)
)
```

**Proposed fix:** If the second set of gains is the intended tuning from Peoria, delete the first definition entirely to avoid confusion:
```python
_drive_gains = (  # TORQUE_CURRENT_FOC — tuned at Peoria Day 1.5
    configs.Slot0Configs()
    .with_k_p(1.5)
    .with_k_i(0)
    .with_k_d(0.1)
    .with_k_s(0)
    .with_k_v(0)
    .with_k_a(0)
)
```

If the first set was the better-performing config, delete the second. Either way, there should be exactly one definition. Consider adding feedforward values back (`k_s` and `k_v` at minimum) for proper velocity tracking.

---

### C6. Steer gains: kP=300 with all feedforward zeroed out

**File:** `generated/Crimson_tuner_constants.py:36-45`

**Problem:** The steer motor gains were changed to kP=300 with kS=0, kV=0, kA=0. In TORQUE_CURRENT_FOC mode, kP=300 means 300 amps per rotation of error. Even 0.001 rotations of encoder noise produces 0.3A of corrective current. With no static friction feedforward (kS=0), the PID must also overcome carpet friction through error accumulation alone, which can cause steady-state oscillation.

**Current code:**
```python
_steer_gains = (
    configs.Slot0Configs()
    .with_k_p(300)
    .with_k_d(0.5)
    .with_k_s(0).with_k_v(0).with_k_a(0)
)
```

**Proposed fix:** Restore some feedforward to reduce the burden on the P term. A reasonable starting point based on the previously commented-out values:
```python
_steer_gains = (
    configs.Slot0Configs()
    .with_k_p(300)
    .with_k_i(0)
    .with_k_d(0.5)
    .with_k_s(1.0)    # Static friction compensation — tune on carpet
    .with_k_v(0.94)   # Velocity feedforward — from prior characterization
    .with_k_a(0.36)   # Acceleration feedforward
    .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
)
```

> Note: If kP=300 with zero FF was validated on the robot at Peoria and the modules aren't buzzing, it may be acceptable short-term. But monitor steer motor temperatures — the lack of feedforward forces the PID to work harder, generating more heat.

---

## Significant Defects

### S1. Indexer front/back motor configs are swapped

**File:** `subsystems/indexer.py:49-88`

**Problem:** The config objects are named after CAN IDs (`config50`, `config51`), but they are applied to the wrong motors. `self.front` (CAN 51) receives `config50`, and `self.back` (CAN 50) receives `config51`. Critically, the velocity conversion factors differ:

- `config50`: `velocityConversionFactor(1.0 / 60.0)` — converts RPM to RPS
- `config51`: `velocityConversionFactor(1.0)` — raw RPM (the `/ 60.0` is commented out)

This means the front and back motors interpret velocity setpoints in different units, causing unpredictable indexer behavior.

**Current code:**
```python
self.front = SparkMax(51, ...)  # CAN 51
self.back = SparkMax(50, ...)   # CAN 50
# ...
self.front.configure(config50, ...)  # Wrong! CAN 51 gets config meant for CAN 50
self.back.configure(config51, ...)   # Wrong! CAN 50 gets config meant for CAN 51
```

**Proposed fix — option A (swap the apply calls):**
```python
self.front.configure(config51, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
self.back.configure(config50, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
```

**Proposed fix — option B (rename configs for clarity, preferred):**
Rename `config50`/`config51` to `front_config`/`back_config` and apply accordingly. Also ensure both have consistent velocity conversion factors — both should either be RPM or RPS, not a mix:
```python
front_config = SparkMaxConfig()
front_config.encoder.velocityConversionFactor(1.0 / 60.0)  # RPM -> RPS
# ... rest of front-specific PID/FF ...

back_config = SparkMaxConfig()
back_config.encoder.velocityConversionFactor(1.0 / 60.0)   # RPM -> RPS (uncomment the /60)
# ... rest of back-specific PID/FF ...

self.front.configure(front_config, ...)
self.back.configure(back_config, ...)
```

---

### S2. Shooter tuning table has duplicate distances and non-monotonic ordering

**File:** `subsystems/shooter.py:120-129`

**Problem:** The tuning table has two entries at distance 68 with different values, and distance 53.5 appears after 68. Although `get_values_for_distance` sorts by distance, the duplicate at 68 means one data point is unreachable during interpolation (the interpolator will match the first 68 entry and skip the second). Additionally, the distances (44-163) are labeled as "meters" in the docstring but seem far too large for an FRC field (16.5m long) — they are likely inches.

**Current code:**
```python
self.tuning_table = [
    (44.0, 40.0, 0.0),
    (56.0, 43.0, 0.0),
    (68, 45, 0),       # distance 68, hood 0
    (68, 38, 0.05),    # distance 68 AGAIN, different values — unreachable
    (53.5, 35, 0.02),  # 53.5 comes after 68 — out of order
    (87, 39, .06),
    (125, 44, .07),
    (163, 46, .08)
]
```

**Proposed fix:** Remove the duplicate, sort the table, and clarify units in the comment:
```python
# Format: (Distance in INCHES, Flywheel RPS, Hood Rotations)
self.tuning_table = [
    (44.0, 40.0, 0.0),
    (53.5, 35.0, 0.02),
    (56.0, 43.0, 0.0),
    (68.0, 38.0, 0.05),   # Keep whichever 68 entry is correct
    (87.0, 39.0, 0.06),
    (125.0, 44.0, 0.07),
    (163.0, 46.0, 0.08),
]
```

> Note: The two entries at distance 68 have very different flywheel RPS (45 vs 38) and hood values (0 vs 0.05). Determine from testing which one produced better shots and keep that one.

---

### S3. Hood encoder seeding ignores the computed value

**File:** `subsystems/shooter.py:87-104`

**Problem:** The code reads the absolute encoder, computes a `seeded_value`, then ignores it entirely and hard-codes `self.hood.set_position(0)`. If the hood is not physically at its zero position on startup, the internal encoder will be wrong, causing the soft limits and position commands to be offset.

**Current code:**
```python
SENSOR_OFFSET = -0.391
raw_abs_val = self.abs_encoder.get_absolute_position().refresh().value
seeded_value = raw_abs_val - SENSOR_OFFSET
self.hood.set_position(0)  # Ignores seeded_value!
```

**Proposed fix:**
```python
SENSOR_OFFSET = -0.391
raw_abs_val = self.abs_encoder.get_absolute_position().refresh().value
seeded_value = raw_abs_val - SENSOR_OFFSET
self.hood.set_position(seeded_value)
```

> Note: Verify the seeding math is correct by comparing `seeded_value` to the expected hood position when powered on in a known physical state. The comment references "Turret Laps" which suggests this formula may have been copied from the turret code and not updated.

---

### S4. Telemetry distance publishers use wrong topic type

**File:** `telemetry.py:119-131`

**Problem:** The distance topics use `getDoubleArrayTopic` (for arrays of doubles), but `subscribe()`, `publish()`, and `set()` are called with scalar `0.0` values instead of lists. This will cause type errors at runtime.

**Current code:**
```python
self._distance_to_blue_topic = self._distance_table.getDoubleArrayTopic("Distance to blue hub")
self._distance_to_blue_sub = self._distance_to_blue_topic.subscribe(0.0)    # Should be [0.0]
self._distance_to_blue_pub = self._distance_to_blue_topic.publish(0.0)      # publish() doesn't take a default
self._distance_to_blue_pub.set(0.0)                                         # Should be [0.0]
```

**Proposed fix:** Since distance is a single scalar value, use `getDoubleTopic` instead:
```python
self._distance_to_blue_topic = self._distance_table.getDoubleTopic("Distance to blue hub")
self._distance_to_blue_sub = self._distance_to_blue_topic.subscribe(0.0)
self._distance_to_blue_pub = self._distance_to_blue_topic.publish()
self._distance_to_blue_pub.set(0.0)

self._distance_to_red_topic = self._distance_table.getDoubleTopic("Distance to red hub")
self._distance_to_red_sub = self._distance_to_red_topic.subscribe(0.0)
self._distance_to_red_pub = self._distance_to_red_topic.publish()
self._distance_to_red_pub.set(0.0)
```

---

### S5. Shooter/Indexer/Intake state variables are class-level, not instance-level

**Files:**
- `subsystems/shooter.py:18-19` — `last_rps = -1.0`, `last_hood_rot = -1.0`
- `subsystems/indexer.py:16-17` — `last_indexer_rps = -1.0`, `state_hopper_forward = True`
- `subsystems/intake.py:11-12` — `is_shoulder_braking = True`, `last_shoulder_position = False`

**Problem:** These mutable state variables are defined at the class level, making them shared across all instances. While FRC typically only creates one instance per subsystem, this is bad practice. More concretely, `Intake.last_shoulder_position` is initialized to `False` (a bool) but later compared against `float` values. In Python, `0 == False` is `True` and `0.0 == False` is `True`, which means the first call to `set_shoulder_position(0)` will appear to be "no change" and the command won't be sent.

**Proposed fix:** Move all mutable state into `__init__` as instance variables.

For `shooter.py`, add to `__init__`:
```python
self.last_rps = -1.0
self.last_hood_rot = -1.0
```

For `indexer.py`, add to `__init__`:
```python
self.last_indexer_rps = -1.0
self.state_hopper_forward = True
```

For `intake.py`, add to `__init__`:
```python
self.is_shoulder_braking = True
self.last_shoulder_position = -1.0  # Use a float sentinel, not False
```

Remove the corresponding class-level declarations from each file.

---

### S6. `intake.stop()` stops both the intake wheel AND the shoulder motor

**File:** `subsystems/intake.py:138-140`

**Problem:** The `stop()` method stops both the intake motor and the shoulder motor. This method is called from multiple button release handlers (right trigger `onFalse` at `robotcontainer.py:136`, left trigger `onFalse` at `robotcontainer.py:183`). If the driver is holding A (shoulder down) while releasing the left trigger, `stop()` will kill the shoulder position hold unexpectedly.

**Current code:**
```python
def stop(self):
    self.intake.set(ControlMode.PercentOutput, 0)
    self.shoulder.stopMotor()
```

**Proposed fix:** Separate the two stop functions:
```python
def stop_intake(self):
    """Stops only the intake roller motor."""
    self.intake.set(ControlMode.PercentOutput, 0)

def stop_shoulder(self):
    """Stops only the shoulder motor."""
    self.shoulder.stopMotor()

def stop(self):
    """Stops both intake and shoulder."""
    self.stop_intake()
    self.stop_shoulder()
```

Then update the button bindings in `robotcontainer.py` to call `stop_intake()` instead of `stop()` where only the roller should be stopped:

- Line 136: `lambda: self.intake.stop_intake()` (right trigger release — only stop roller)
- Line 183: `lambda: self.intake.stop_intake()` (left trigger release — only stop roller)

---

### S7. `team_color` and alliance-dependent values fetched too early

**Files:**
- `robotcontainer.py:48` — `self.team_color = DriverStation.getAlliance()`
- `subsystems/turret.py:33` — `_team_color = DriverStation.getAlliance()`

**Problem:** `DriverStation.getAlliance()` frequently returns `None` during robot initialization because the DriverStation connection hasn't been established yet. In `robotcontainer.py`, the value is stored but never refreshed. In `turret.py`, this means the turret defaults to "Blue" team regardless of actual alliance, and `HUB_POSITION` will be set incorrectly for red alliance robots.

**Proposed fix for `turret.py`:** Re-check alliance in `periodic()` until it's resolved:
```python
def __init__(self, _telemetry: Telemetry):
    super().__init__()
    self.telemetry = _telemetry
    self.TEAM_COLOR = "Blue"  # Default
    self.HUB_POSITION = self.blue_hub  # Default
    self._alliance_resolved = False
    self.location_state = 1
    # ... rest of init ...

def periodic(self):
    # Resolve alliance if not yet done
    if not self._alliance_resolved:
        alliance = DriverStation.getAlliance()
        if alliance is not None:
            if alliance == DriverStation.Alliance.kRed:
                self.TEAM_COLOR = "Red"
                self.HUB_POSITION = self.red_hub
            else:
                self.TEAM_COLOR = "Blue"
                self.HUB_POSITION = self.blue_hub
            self._alliance_resolved = True

    # ... rest of periodic ...
```

**Proposed fix for `robotcontainer.py:48`:** Either remove the line (it's unused elsewhere) or defer it similarly.

---

### S8. Duplicate Vision `imumode_set` publisher per camera

**File:** `subsystems/vision.py:22, 29`

**Problem:** Inside the `for name in limelight_names` loop, two publishers are created for the same `imumode_set` topic. The first is stored in `self.mode_publishers[name]` (line 22), while the second is assigned to local variable `mode_pub` (line 29) which gets garbage collected after the loop. This wastes resources and may cause NetworkTables conflicts.

**Current code:**
```python
for name in limelight_names:
    table = self.inst.getTable(name)
    # ...
    self.mode_publishers[name] = table.getIntegerTopic("imumode_set").publish()  # Stored
    # ...
    mode_pub = table.getIntegerTopic("imumode_set").publish()  # Duplicate, not stored
```

**Proposed fix:** Remove the duplicate `mode_pub` creation (delete line 29):
```python
for name in limelight_names:
    table = self.inst.getTable(name)
    self.subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])
    self.orientation_publishers[name] = table.getFloatArrayTopic("robot_orientation_set").publish()
    self.mode_publishers[name] = table.getIntegerTopic("imumode_set").publish()
    self.mt1_subscribers[name] = table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([])

self.force_imu_modes()
```

---

### S9. Commented-out turret auto-rotation uses `==` (comparison) instead of `=` (assignment)

**File:** `subsystems/turret.py:268, 278, 288, 294, 304, 314`

**Problem:** Throughout the commented-out auto-rotation logic, every `location_state` update uses `==` instead of `=`:
```python
self.location_state == 1   # Comparison — evaluates to True/False and discards result
```

When this code is eventually uncommented, `location_state` will never change from its initial value.

**Proposed fix:** When uncommenting this code, change all occurrences to:
```python
self.location_state = 1   # Assignment
```

---

### S10. Commented-out turret alliance check is always truthy

**File:** `subsystems/turret.py:254`

**Problem:** The commented-out code reads:
```python
# if DriverStation.Alliance.kBlue:
```
`DriverStation.Alliance.kBlue` is an enum member and is always truthy. This is a comparison against the enum value itself, not a comparison with the current alliance. When uncommented, this will always select `blue_hub`.

**Proposed fix:** When uncommenting, change to:
```python
if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
```

---

## Controllability Issues

### K1. Turret has no active control — driver cannot aim it

**File:** `robotcontainer.py:162-178`

**Problem:** All D-pad turret bindings are commented out. The auto-rotation logic in `turret.py:periodic()` is also commented out. The turret moves to position 0.75 on startup (`turret.py:124`) and stays there for the entire match. The driver has no way to aim the turret.

**Proposed fix:** Uncomment and update the D-pad bindings. Use `run` instead of `runOnce` so the turret continuously tracks while the button is held:
```python
self._joystick.povLeft().whileTrue(
    commands2.cmd.runOnce(lambda: self.turret.set_position(0.75), self.turret)
)
self._joystick.povRight().whileTrue(
    commands2.cmd.runOnce(lambda: self.turret.set_position(0.25), self.turret)
)
self._joystick.povUp().whileTrue(
    commands2.cmd.runOnce(lambda: self.turret.set_position(0.1), self.turret)
)
self._joystick.povDown().whileTrue(
    commands2.cmd.runOnce(lambda: self.turret.set_position(0.5), self.turret)
)
```

Alternatively, enable auto-rotation in `periodic()` after fixing the `==`/`=` and alliance check bugs (S9, S10, C3).

---

### K2. Left trigger intake command missing subsystem requirement

**File:** `robotcontainer.py:180-183`

**Problem:** The left trigger intake command does not pass `self.intake` as a subsystem requirement:
```python
commands2.cmd.run(
    lambda: self.intake.set_intake_dutyCycle(.4))  # No subsystem argument!
```

Without the subsystem requirement, the command scheduler won't enforce mutual exclusion. This command can run simultaneously with the right trigger intake command (which does require `self.intake`), sending conflicting duty cycle commands to the same motor.

Similarly, the `onFalse` handler at line 183 also lacks a subsystem requirement.

**Proposed fix:**
```python
self._joystick.leftTrigger().whileTrue(
    commands2.cmd.run(
        lambda: self.intake.set_intake_dutyCycle(.4), self.intake)
).onFalse(commands2.cmd.runOnce(
    lambda: self.intake.stop_intake(), self.intake))
```

> Note: Uses `stop_intake()` instead of `stop()` per the fix in S6.

---

### K3. Right trigger and right bumper `onFalse` handlers lack intake subsystem requirement

**File:** `robotcontainer.py:134-136, 142-144`

**Problem:** The `onFalse` cleanup commands for intake and shooter call `self.intake.stop()` and `self.shooter.stop()` without passing the respective subsystems. This means these cleanup commands can run concurrently with active commands on those subsystems.

**Proposed fix for right trigger `onFalse` (line 133-136):**
```python
).onFalse(
    commands2.cmd.runOnce(lambda: self.shooter.stop(), self.shooter).alongWith(
        commands2.cmd.runOnce(lambda: self.indexer.stop_all(), self.indexer)).alongWith(
            commands2.cmd.runOnce(lambda: self.intake.stop_intake(), self.intake))
)
```

---

### K4. Shoulder position commands have no `onFalse` handler

**File:** `robotcontainer.py:150-158`

**Problem:** When button A or Y is released, the `whileTrue` command ends but no `onFalse` handler is defined. The SparkMax will continue trying to hold its last commanded position (PID is still active from the last `setReference` call), but since no command is actively running, the behavior depends on the SparkMax firmware. This is fragile — if the command scheduler interrupts the subsystem, the shoulder could go limp.

**Proposed fix:** Since the shoulder is meant to hold position, the current behavior may be acceptable. However, to be explicit, either:

**Option A** — Use `runOnce` instead (set position once, let PID hold):
```python
self._joystick.a().onTrue(
    commands2.cmd.runOnce(lambda: self.intake.set_shoulder_position(0), self.intake)
)
self._joystick.y().onTrue(
    commands2.cmd.runOnce(lambda: self.intake.set_shoulder_position(0.3), self.intake)
)
```

**Option B** — Add an `onFalse` that commands a hold at current position (prevents unexpected drift):
```python
self._joystick.a().whileTrue(
    commands2.cmd.run(lambda: self.intake.set_shoulder_position(0), self.intake)
).onFalse(
    commands2.cmd.runOnce(lambda: self.intake.set_shoulder_position(
        self.intake.shoulder_encoder.getPosition()), self.intake)
)
```

---

### K5. No autonomous command configured

**File:** `robotcontainer.py:223-228`

**Problem:** `getAutonomousCommand()` returns a print statement. The robot does nothing during the 15-second autonomous period. PathPlanner is installed (`pyproject.toml`) and path files exist in `deploy/pathplanner/`, but no autonomous routine is wired up.

**Proposed fix:** At minimum, create a basic autonomous that drives forward or shoots from a known position. With PathPlanner already installed:
```python
def getAutonomousCommand(self) -> commands2.Command:
    # TODO: Wire up PathPlanner autos from deploy/pathplanner/
    return commands2.cmd.print_("No autonomous command configured")
```

This is noted here for awareness — the actual implementation depends on game strategy.

---

### K6. No joystick input curve — linear mapping at full speed

**File:** `robotcontainer.py:103-112`

**Problem:** Joystick values are mapped linearly to max speed (5.23 m/s). Small joystick movements near center cause disproportionately large velocity changes, making fine positioning difficult for the driver.

**Proposed fix:** Apply a squared or cubic input curve to give more control at low speeds while preserving full speed at maximum deflection:
```python
import math

def _apply_curve(self, value: float, exponent: float = 2.0) -> float:
    """Applies a power curve while preserving sign."""
    return math.copysign(abs(value) ** exponent, value)

# In configureButtonBindings:
self.drivetrain.setDefaultCommand(
    self.drivetrain.apply_request(
        lambda: (
            self._drive.with_velocity_x(
                -self._apply_curve(self._joystick.getLeftY()) * self._max_speed
            )
            .with_velocity_y(
                -self._apply_curve(self._joystick.getLeftX()) * self._max_speed
            )
            .with_rotational_rate(
                -self._apply_curve(self._joystick.getRightX()) * self._max_angular_rate
            )
        )
    )
)
```

---

### K7. Intake `set_intake_dutyCycle` negates the input

**File:** `subsystems/intake.py:136`

**Problem:** The duty cycle is negated before being sent to the motor:
```python
self.intake.set(ControlMode.PercentOutput, -DC)
```
When the button binding passes `0.4`, the motor actually receives `-0.4` (reverse). If this is compensating for motor wiring direction, it should use `self.intake.setInverted(True)` in the constructor instead, so the intent is clear and all callers get correct behavior.

**Proposed fix — option A (if motor wiring is reversed):** Set inversion in the constructor and remove the negation:
```python
# In __init__:
self.intake.setInverted(True)

# In set_intake_dutyCycle:
self.intake.set(ControlMode.PercentOutput, DC)  # Remove negation
```

**Proposed fix — option B (if current behavior is wrong):** Simply remove the negation:
```python
self.intake.set(ControlMode.PercentOutput, DC)
```

---

### K8. No hood stop or home command available to driver

**File:** `subsystems/shooter.py:140-142`

**Problem:** `shooter.stop()` only stops the flywheel. The hood motor is never explicitly stopped or homed — it continuously holds its last commanded Motion Magic position. There is no button binding to reset or home the hood. If the hood gets to an unexpected position, the driver has no recovery mechanism.

**Proposed fix:** Extend `stop()` to also neutral the hood, or add a separate hood stop:
```python
def stop(self):
    """Stops the flywheel and neutrals the hood."""
    self.flywheel.stopMotor()
    self.hood.stopMotor()
    self.last_rps = -1.0
    self.last_hood_rot = -1.0
```

---

## Improvement Suggestions

### I1. Centralize constants in `constants.py`

**File:** `constants.py` (currently empty)

CAN IDs, gear ratios, PID values, field coordinates, and physical limits are scattered across individual subsystem files. This makes tuning at competition difficult because you have to hunt through multiple files.

**Suggestion:** Move shared constants into `constants.py`:
```python
class CANIds:
    TURRET_MOTOR = 15
    TURRET_ENCODER = 5
    FLYWHEEL = 16
    HOOD = 17
    INTAKE = 18
    INDEXER_FRONT = 51
    INDEXER_BACK = 50
    HOPPER = 52
    SHOULDER = 53
    HOOD_ENCODER = 7

class FieldPositions:
    BLUE_HUB = Pose2d(0.0, 5.5, 0.0)
    RED_HUB = Pose2d(16.55, 8.05, 0.0)
    # ... corners ...
```

---

### I2. Remove debug print statements

**Files:**
- `robotcontainer.py:54-55` — `test = self.vision.get_mt1_pose()` / `print(test)`
- `subsystems/intake.py` — Previously had `print("attempting to set shoulder position")` (removed in latest commit but verify)

**Suggestion:** Remove leftover debug prints. For persistent debugging, use WPILib's `DataLogManager` or publish to NetworkTables instead of `print()`, which goes to the console log and is easy to miss.

---

### I3. Add voltage compensation to swerve drive

**File:** `robotcontainer.py:68-71`

The drivetrain uses `OPEN_LOOP_VOLTAGE` for drive motors. At full battery (13V), the robot will be ~8% faster than at 12V. This makes driver feel inconsistent across a match as the battery drains.

**Suggestion:** Either switch to closed-loop velocity control for more consistent speed, or configure voltage compensation on the drive TalonFXs.

---

### I4. Intake limit switch wiring verification needed

**File:** `subsystems/intake.py:53-63`

The limit switches are configured as `kNormallyClosed`. For NC switches, the default (unpressed) state is closed circuit. If the physical switches are actually NormallyOpen but configured as NormallyClosed, the shoulder will refuse to move because the SparkMax interprets the open circuit as "switch triggered."

**Suggestion:** Verify physical wiring matches the software config. Test by checking `self.shoulder.getReverseLimitSwitch().isPressed()` in periodic telemetry and comparing with the physical switch state.

---

### I5. Consider enabling soft limits on intake shoulder

**File:** `subsystems/intake.py:66-71`

Software soft limits for the shoulder are fully commented out. While limit switches provide physical protection, soft limits provide an additional safety layer and can decelerate the motor before hitting the physical switch.

**Suggestion:** Uncomment after verifying the correct limit values match your physical geometry:
```python
self.shoulder_config.softLimit.forwardSoftLimit(0.30)
self.shoulder_config.softLimit.forwardSoftLimitEnabled(True)
self.shoulder_config.softLimit.reverseSoftLimit(0.005)
self.shoulder_config.softLimit.reverseSoftLimitEnabled(True)
```

---

### I6. Unused control request objects in Shooter

**File:** `subsystems/shooter.py:107-116`

Several control request objects are created but never used:
- `self.fw_torque_request` (line 110) — `TorqueCurrentFOC`
- `self.torque_current_request` (line 112) — duplicate `TorqueCurrentFOC`
- `self.hood_torque_request` (line 113) — `MotionMagicTorqueCurrentFOC`
- `self.voltage_request` (line 116) — `VoltageOut`

**Suggestion:** Remove unused requests to reduce confusion about which control mode is active:
```python
# Keep only what's used:
self.fw_torque_request_vel = controls.VelocityTorqueCurrentFOC(0)
self.hood_position_request = controls.MotionMagicVoltage(0)
```

---

### I7. `SignalLogger.start()` called automatically but stopped on disable

**File:** `telemetry.py:16`, `robot.py:49-50`

`SignalLogger.start()` is called in the Telemetry constructor, but `disabledInit()` calls `SignalLogger.stop()`. The driver must manually press left stick to restart logging after every disable. This means logging is likely off during most of teleop if the robot was disabled between auto and teleop.

**Suggestion:** Either remove the `SignalLogger.stop()` from `disabledInit()`, or restart it automatically in `teleopInit()`:
```python
# In robot.py teleopInit:
def teleopInit(self) -> None:
    if self.autonomousCommand:
        self.autonomousCommand.cancel()
    SignalLogger.start()
```

---

### I8. Hood Motion Magic Expo parameters are zero

**File:** `subsystems/shooter.py:76-77`

```python
hood_cfg.motion_magic.motion_magic_expo_k_v = 0.0
hood_cfg.motion_magic.motion_magic_expo_k_a = 0.0
```

These parameters are only relevant if Motion Magic Expo mode is used. Since the hood uses `MotionMagicVoltage` (standard trapezoidal/S-curve profile), these values are currently harmless. However, if someone later switches to Expo mode, the zero values will cause problems.

**Suggestion:** Either remove these lines (let them default) or set them to reasonable values matching the standard Motion Magic characterization:
```python
# Remove these two lines — they have no effect on MotionMagicVoltage
```

---

### I9. Field2d pose publishing is commented out

**File:** `telemetry.py:168`

```python
#self._field_pub.set(pose_array)
```

The Field2d type is set but the actual pose is never published, so the field visualization in dashboards (Shuffleboard, Elastic, Glass) will show the robot at origin.

**Suggestion:** Uncomment:
```python
self._field_pub.set(pose_array)
```

---

## Summary

| ID | Severity | File | Description |
|----|----------|------|-------------|
| C1 | Critical | command_swerve_drivetrain.py:333 | `seed_pigeon_with_vision` crashes on success |
| C2 | Critical | command_swerve_drivetrain.py:16 | `_PIGEON_SEEDED` class variable prevents re-seeding |
| C3 | Critical | turret.py:31 | `HUB_POSITION` never set to actual hub |
| C4 | Critical | turret.py:154 | `aim_at_hub` type error on Pose2d subtraction |
| C5 | Critical | Crimson_tuner_constants.py:57-75 | Duplicate `_drive_gains` — first shadowed |
| C6 | Critical | Crimson_tuner_constants.py:36-45 | Steer kP=300 with zeroed feedforward |
| S1 | Significant | indexer.py:81-88 | Front/back motor configs swapped |
| S2 | Significant | shooter.py:120-129 | Tuning table duplicates and unit confusion |
| S3 | Significant | shooter.py:99-104 | Hood seeding ignores computed value |
| S4 | Significant | telemetry.py:119-131 | Distance topic type mismatch |
| S5 | Significant | shooter/indexer/intake | Mutable class variables instead of instance |
| S6 | Significant | intake.py:138-140 | `stop()` kills shoulder unexpectedly |
| S7 | Significant | robotcontainer.py:48, turret.py:33 | Alliance fetched too early |
| S8 | Significant | vision.py:22,29 | Duplicate imumode_set publisher |
| S9 | Significant | turret.py (commented) | `==` instead of `=` for location_state |
| S10 | Significant | turret.py:254 (commented) | Alliance check always truthy |
| K1 | Controllability | robotcontainer.py:162-178 | Turret has zero active control |
| K2 | Controllability | robotcontainer.py:180-183 | Left trigger missing subsystem |
| K3 | Controllability | robotcontainer.py:134-136 | onFalse handlers missing subsystem |
| K4 | Controllability | robotcontainer.py:150-158 | Shoulder no onFalse handler |
| K5 | Controllability | robotcontainer.py:223-228 | No autonomous command |
| K6 | Controllability | robotcontainer.py:103-112 | Linear joystick mapping |
| K7 | Controllability | intake.py:136 | Duty cycle negated |
| K8 | Controllability | shooter.py:140-142 | No hood stop for driver |
| I1 | Improvement | constants.py | Centralize constants |
| I2 | Improvement | robotcontainer.py:54-55 | Remove debug prints |
| I3 | Improvement | robotcontainer.py:68-71 | Add voltage compensation |
| I4 | Improvement | intake.py:53-63 | Verify limit switch wiring |
| I5 | Improvement | intake.py:66-71 | Enable shoulder soft limits |
| I6 | Improvement | shooter.py:107-116 | Remove unused control requests |
| I7 | Improvement | telemetry.py:16, robot.py:49 | SignalLogger lifecycle |
| I8 | Improvement | shooter.py:76-77 | Remove unused MM Expo params |
| I9 | Improvement | telemetry.py:168 | Uncomment Field2d publishing |
