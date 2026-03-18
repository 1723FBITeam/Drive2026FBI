# Tuning & Setup Guide

This document covers everything needed to get the robot's systems dialed in:
Limelight vision setup, turret PID tuning, shooter flywheel tuning, shot calibration,
zone-based targeting, and co-pilot offset tuning.

**Game: 2026 FRC REBUILT**

---

## Table of Contents

1. [Limelight Setup (New Install)](#1-limelight-setup-new-install)
2. [Turret PID Tuning](#2-turret-pid-tuning)
3. [Shooter Flywheel Tuning](#3-shooter-flywheel-tuning)
4. [Shot Calibration (Hood + Distance Tables)](#4-shot-calibration-hood--distance-tables)
5. [Vision Pose Estimation Tuning](#5-vision-pose-estimation-tuning)
6. [Velocity Compensation Tuning](#6-velocity-compensation-tuning)
7. [Zone-Based Smart Targeting](#7-zone-based-smart-targeting)
8. [Co-Pilot Offset Tuning](#8-co-pilot-offset-tuning)
9. [Quick Reference — What Lives Where](#9-quick-reference--what-lives-where)

---

## 1. Limelight Setup (New Install)

This section walks you through setting up a Limelight camera from scratch.
If you've never done this before, follow every step carefully — small mistakes
here cause big problems with auto-aiming later.


### 1.1 Physical Mounting

- Mount the Limelight **rigidly to the robot frame** — NOT on the turret.
- Position it where it has a clear view of field AprilTags (rear-facing or elevated is ideal).
- Avoid mounting near motors or areas with heavy vibration.
- Secure all cables — a loose Ethernet cable mid-match will kill your pose estimation.
- **If using Limelight 4 (LL4):** It MUST be mounted in **landscape orientation** (wider than tall) for the built-in IMU to work correctly. Portrait mounting will give wrong IMU readings.

### 1.2 Wiring

| Connection | Details |
|---|---|
| Power | 12V from PDP/PDH (use a dedicated slot, not shared with motors) |
| Ethernet | To the robot radio or a network switch on the robot |
| Default IP | `10.TE.AM.11` (e.g., team 1234 → `10.12.34.11`) |

**Tip:** Label the Ethernet cable so nobody accidentally unplugs it.

### 1.3 Web UI Configuration (Step by Step)

1. **Connect to the robot's WiFi** (or plug in via USB for initial setup)

2. **Open a browser** to `http://10.TE.AM.11:5801` (replace TE.AM with your team number)

3. **Settings tab:**
   - Set your team number
   - Set the Limelight name to `"limelight"` (must match exactly — this is the string used in our code in `Robot.java` and `LimelightHelpers.java`)

4. **3D tab — Camera Pose (THIS IS THE MOST IMPORTANT STEP):**
   - You need to tell the Limelight exactly where it is on the robot, relative to the robot's center.
   - Measure these values in **meters** (use a tape measure, be precise):
     - `LL Forward`: distance from robot center to camera, along the forward axis (positive = toward front of robot)
     - `LL Right`: distance from robot center to camera, along the right axis (positive = toward right side)
     - `LL Up`: height of camera lens from the ground
     - `LL Roll / Pitch / Yaw`: rotation of the camera in degrees (0/0/0 if mounted flat and facing straight)
   - Use the 3D viewer in the Limelight UI to verify the camera model looks correct on the robot
   - **Even 2cm of error here translates to aim error on the field.** Measure twice.

5. **AprilTag Pipeline:**
   - Create a new pipeline (or edit the default) and set it to "AprilTag" mode
   - Verify tag family is set to `36h11` (this is the standard for FRC)
   - **Upload the 2026 field map (.fmap file):**
     - Go to the [Limelight downloads page](https://limelightvision.io/pages/downloads) and download the 2026 REBUILT `.fmap` file
     - Upload it in the Limelight web UI under the AprilTag pipeline settings
     - This tells the Limelight where every AprilTag is on the 2026 field
   - If the official `.fmap` isn't available yet, you can generate one from the WPILib field layout JSON

6. **If using Limelight 4 (built-in IMU):**
   - Our code automatically handles IMU mode switching:
     - **While disabled:** Mode 1 (external heading seed) — the Pigeon IMU heading is sent to the Limelight to calibrate its internal IMU
     - **While enabled (auto/teleop):** Mode 4 (internal IMU + external assist) — best accuracy for MegaTag2
   - This is already coded in `Robot.java` → `disabledPeriodic()` and `teleopInit()`/`autonomousInit()`
   - **Important:** Let the robot sit disabled for a few seconds after power-on so the IMU can seed properly
   - The code calls `SetRobotOrientation()` every loop to keep the Limelight's heading in sync with the Pigeon


### 1.4 What is MegaTag2?

MegaTag2 is Limelight's latest pose estimation algorithm. Instead of calculating position from AprilTags alone, it combines:
- AprilTag detections (where tags are in the camera image)
- Robot heading from the IMU (Pigeon or Limelight's built-in IMU)

This makes it much more accurate than the original MegaTag, especially when you can only see one tag. Our code uses `getBotPoseEstimate_wpiBlue_MegaTag2()` to get the pose in WPILib blue-origin coordinates.

### 1.5 Verification Checklist

Do these checks BEFORE your first match:

- [ ] Deploy code and connect to robot
- [ ] Open Shuffleboard or AdvantageScope → check `DriveState/Pose`
- [ ] Place robot at a known position on the field (e.g., against the alliance wall)
- [ ] Pose should jump to a reasonable field position when AprilTags are visible
- [ ] Walk/drive robot around the field — pose should track smoothly without jumping
- [ ] If pose is mirrored (robot shows on wrong side of field) → re-check camera pose in Limelight UI (step 4)
- [ ] If pose is offset by a constant amount → re-measure camera position on robot
- [ ] Verify `Calibration` tab shows reasonable `Robot X`, `Robot Y`, `Dist to Hub` values
- [ ] Spin the robot in place — pose should stay roughly in the same spot (vision is rejected during fast spins, odometry handles it)
- [ ] Check the Limelight web UI (`10.TE.AM.11:5801`) — verify tags are being detected (you should see green outlines around tags in the camera feed)

### 1.6 Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| No pose data at all | Limelight not connected or wrong name | Check Ethernet, verify name is `"limelight"` in settings |
| Pose jumps around wildly | Camera pose (3D tab) is wrong | Re-measure camera position carefully |
| Pose is mirrored (wrong side of field) | Camera yaw is off by 180° | Flip the yaw value in Limelight 3D settings |
| Pose drifts over time | Vision std devs too high (not trusting vision enough) | Lower std devs in `Robot.java` (see Section 5) |
| Pose snaps/jumps when tags appear | Vision std devs too low (trusting vision too much) | Raise std devs in `Robot.java` |
| Tags not detected | Wrong pipeline, wrong .fmap, or camera blocked | Check pipeline is AprilTag mode, verify .fmap is uploaded |

---

## 2. Turret PID Tuning

The turret runs closed-loop position control on the TalonFX at 1kHz (1000 times per second) — way faster than anything we could do on the RoboRIO.
All gains are configured in `TurretSubsystem.java` constructor.

### 2.1 Current Gains

| Gain | Value | Purpose |
|------|-------|---------|
| KP   | 30.0  | Proportional — main correction force. High because turret gets stiffer at extremes. |
| KI   | 0.5   | Integral — builds up force over time to push through increasing friction/cable tension |
| KD   | 0.2   | Derivative — dampens oscillation (prevents overshooting) |
| KS   | 0.6   | Static friction feedforward — voltage to overcome stiction at the extremes of travel |
| KV   | 0.0   | Velocity feedforward — not needed for position control |

**Why are these gains so high?** The turret has cables and wires running through it that get tighter as it rotates further from center. The higher KP and KS give it enough force to push through that resistance. KI helps when the turret stalls against friction — it slowly builds up more force until it breaks through.


### 2.2 Turret Range and Soft Limits

The turret has an asymmetric range:
- **CCW (positive):** +420 degrees = +1.1667 mechanism rotations
- **CW (negative):** -245 degrees = -0.6806 mechanism rotations
- **Total travel:** 665 degrees

Hardware soft limits are enforced on the motor controller (not in code), so even if there's a bug, the turret can't over-rotate:
- Forward limit: `+1.1667` mechanism rotations
- Reverse limit: `-0.6806` mechanism rotations

To change these, edit `MAX_MECHANISM_ROTATIONS` and `MIN_MECHANISM_ROTATIONS` in `TurretSubsystem.java`.

### 2.3 Reset Behavior

Because the turret range exceeds 360°, any target angle has two equivalent positions. When the turret needs to wrap around 360° to reach a target (a "reset"), special behavior kicks in:

| Parameter | Value | What It Does |
|-----------|-------|-------------|
| RESET_THRESHOLD_ROTATIONS | 0.6 (216°) | Moves larger than this are considered resets |
| RESET_SPEED_FRACTION | 0.3 (30%) | Speed during reset (fraction of normal) |
| RESET_DONE_TOLERANCE | 0.02 (~7°) | How close to target before reset is "done" |

During a reset:
- The turret moves at 30% speed using duty cycle control (not PID)
- `isResetting()` returns true
- The shooter will NOT fire (AutoShootCommand checks this)
- The indexer stops to prevent accidental shots

**Tuning reset speed:** If the turret resets too slowly, increase `RESET_SPEED_FRACTION` (max ~0.5 before it gets jerky). If it slams at the end of a reset, decrease it.

### 2.4 Deadband

Motor output deadband is set to **4%** (`DutyCycleNeutralDeadband = 0.04`). This prevents the motor from buzzing/humming when it's very close to target but not perfectly on it. It was raised from 2% because the higher PID gains amplify small errors.

If the turret buzzes at rest, increase the deadband. If it doesn't hold position well enough, decrease it.

### 2.5 Tuning Procedure

1. Open Shuffleboard → `Calibration` tab:
   - `Turret Pos (mech rot)` — current position
   - `Turret Target (mech rot)` — where PID is trying to go
   - `Turret Error Deg` — error in degrees (should settle near 0)
   - `Turret Resetting` — true during a long wrap-around move

2. Start with KD = 0 and KS = 0. Set KP to a low value like 5.0.

3. Command the turret to aim at the hub (press Y or Start). Watch the error:
   - Slowly creeps to target → increase KP
   - Overshoots and oscillates → decrease KP or add KD
   - Oscillates once then settles → KP is close, add KD

4. Once KP is set:
   - Add KD starting at 0.05, increase until oscillation is damped
   - Too much KD = sluggish response and motor buzzing

5. Tune KS:
   - Command small position changes (a few degrees)
   - Turret hesitates before moving → increase KS
   - Turret lurches at start → decrease KS

6. If the turret stalls at extreme positions (near ±400°):
   - Add KI starting at 0.1, increase until it pushes through
   - Too much KI = overshoot after the turret breaks free from friction

### 2.6 What to Watch For

- **Oscillation around target**: KP too high or KD too low
- **Motor buzzing at rest**: KD too high or deadband too low
- **Turret doesn't reach target**: KP too low, or KS too low
- **Turret stalls at extreme angles**: KI too low (friction wins)
- **Turret slams into soft limits**: KP too high with no KD
- **Slow to respond**: KP too low — the turret can handle aggressive gains (11.515:1 gear ratio)


---

## 3. Shooter Flywheel Tuning

The flywheels use closed-loop velocity control on the TalonFX. The motor's built-in PID loop
runs at 1000Hz to maintain the target speed in Rotations Per Second (RPS).

### 3.1 Current Flywheel PID Gains

These are configured in `ShooterSubsystem.java` constructor:

| Gain | Value | Purpose |
|------|-------|---------|
| KP   | 0.1   | Proportional — corrects speed error. Increase if flywheels are slow to reach target. |
| KI   | 0.0   | Integral — not used. Add if flywheels consistently undershoot by a small amount. |
| KD   | 0.0   | Derivative — not used. Add if flywheels oscillate around target speed. |
| KS   | 0.1   | Static friction — voltage to start the motor spinning from rest. |
| KV   | 0.12  | Velocity feedforward — voltage per RPS to maintain speed. This is the main term. |

**Tip:** Run WPILib SysId characterization on the flywheels to find accurate KS and KV values
for your specific motors and gearing. The values above are starting points.

### 3.2 Flywheel Ready-to-Shoot Check

The `isReadyToShoot()` method returns true when both flywheels are within **5%** of the target RPS.
AutoShootCommand checks this before feeding a note. If shots are inconsistent, you can tighten
this tolerance (e.g., 3%) at the cost of slightly longer spin-up wait times.

### 3.3 Tuning Procedure

1. Set a target RPS using auto-aim (press Y or Start)
2. Watch `Shooter L Vel (RPS)` and `Shooter R Vel (RPS)` on the Calibration tab
3. Compare to `Shooter Target (RPS)` — they should match within 5%
4. If flywheels are slow to spin up → increase KP or KV
5. If flywheels overshoot then settle → add KD (start at 0.01)
6. If flywheels oscillate around target → decrease KP or add KD
7. If one flywheel is consistently faster than the other → check motor inversion and wiring

---

## 4. Shot Calibration (Hood + Distance Tables)

The shooter uses two interpolation tables that map distance-to-hub → hood position and flywheel RPS.
Between known data points, WPILib's `InterpolatingDoubleTreeMap` estimates the right value.

### 4.1 Current Calibration Data

These tables are in `ShooterSubsystem.java` constructor:

| Distance (m) | Hood Position | Flywheel RPS | ~RPM  |
|---------------|--------------|-------------|-------|
| 1.3           | 0.13         | 18.9        | 1134  |
| 2.1           | 0.21         | 24.6        | 1476  |
| 3.0           | 0.30         | 28.0        | 1680  |
| 3.8           | 0.38         | 32.5        | 1950  |
| 4.7           | 0.47         | 37.0        | 2220  |

Hood position: 0.0 = flat (low angle), 1.0 = max angle up.

### 4.2 How to Calibrate

1. Place the robot at a known distance from the hub
2. Press **Start** or **Y** to begin auto-shoot (sets hood + flywheels from the tables)
3. Check `Dist to Hub` on the **Calibration** tab in Shuffleboard
4. Use **A/B buttons** to manually nudge the hood up/down until shots land consistently
5. Press **D-pad Up** (driver controller) to log the shot data — it prints to the console and updates the dashboard:
   ```
   >>> SHOT DATA: dist=3.00m, hood=0.300, x=5.12, y=4.04 <<<
   ```
6. Repeat from at least 5 different distances spread across your expected shooting range
7. Update the `hoodTable` and `shooterRPSTable` in `ShooterSubsystem.java` with your new values

### 4.3 Tips

- Calibrate on a real field with real game pieces — practice balls fly differently
- Start close (1-2m) and work outward — close shots are easier to dial in
- Take 3-5 shots at each distance to confirm consistency before logging
- If shots curve left/right, that's a turret aim issue (see Section 2), not a hood/RPS issue
- The co-pilot can use D-pad up/down to nudge RPS during a match if conditions change (see Section 8)

---

## 5. Vision Pose Estimation Tuning

The robot fuses Limelight vision data with wheel odometry using a Kalman filter.
This happens in `Robot.java` → `updateVisionPoseEstimate()`.

### 5.1 Dual Limelight Setup

We run two Limelight 4 cameras for full-field AprilTag coverage:

| Camera | Hostname | IP Address | Mounting | IMU |
|--------|----------|------------|----------|-----|
| Front  | `limelight-front` | 10.17.23.12 | Landscape, facing forward | Works (landscape) |
| Back   | `limelight-back`  | 10.17.23.11 | Portrait (90° CW), facing rear | Not reliable (portrait) — MegaTag2 still works via Pigeon heading |

Both cameras independently estimate the robot's position, and both get fused into the
drivetrain's Kalman filter. The Pigeon IMU heading is sent to both cameras every loop
so MegaTag2 can work even when the camera's built-in IMU isn't reliable.

### 5.2 Camera Poses (configured in Limelight web UI)

These values tell each Limelight where it is on the robot, relative to robot center:

**Front camera:**
| Parameter | Value |
|-----------|-------|
| LL Forward | 0.127 m (5 in) |
| LL Right   | 0.254 m (10 in) |
| LL Up      | 0.4413 m (17.375 in) |
| LL Roll    | 0° |
| LL Pitch   | 0° |
| LL Yaw     | 0° |

**Back camera:**
| Parameter | Value |
|-----------|-------|
| LL Forward | -0.2286 m (9 in back) |
| LL Right   | 0.1175 m (4.625 in) |
| LL Up      | 0.5017 m (19.75 in) |
| LL Roll    | -90° (portrait, 90° CW) |
| LL Pitch   | 0° |
| LL Yaw     | 180° (facing rear) |

### 5.3 Standard Deviations (Trust Levels)

In `Robot.java`, the vision standard deviations control how much we trust vision vs. odometry:

```java
VecBuilder.fill(0.7, 0.7, 9999999)
//              X    Y    Rotation
```

- **X and Y = 0.7**: Moderate trust in vision position. Lower = trust vision more (snappier but jumpier). Higher = trust vision less (smoother but slower to correct drift).
- **Rotation = 9999999**: We completely ignore vision rotation — the Pigeon IMU is much more accurate for heading.

**Tuning guide:**
| Symptom | Adjustment |
|---------|-----------|
| Pose drifts over time (especially after driving far) | Lower X/Y std devs (e.g., 0.5) |
| Pose jumps/snaps when tags appear | Raise X/Y std devs (e.g., 1.0) |
| Pose is smooth but slightly wrong | Current values are good, check camera pose measurements |

### 5.4 Spin Rejection

Vision updates are rejected when the robot is spinning faster than **360°/sec**:
```java
boolean spinning = Math.abs(yawRateDps) > 360;
```
This prevents bad pose estimates during fast turns (the camera image is blurry and heading is changing rapidly). If you're getting bad estimates during moderate turns, lower this threshold (e.g., 270).

### 5.5 IMU Mode Switching

The code automatically switches Limelight IMU modes based on robot state:

| Robot State | IMU Mode | What It Does |
|-------------|----------|-------------|
| Disabled    | Mode 1   | Seeds the Limelight IMU with Pigeon heading (calibration) |
| Auto/Teleop | Mode 4   | Internal IMU + external Pigeon assist (best accuracy) |

**Important:** Let the robot sit disabled for a few seconds after power-on so the IMU can seed properly before enabling.

---

## 6. Velocity Compensation Tuning

When shooting on the move, the turret needs to aim slightly ahead of the target to compensate
for the robot's velocity. This is handled by `aimAtPoseCompensated()` in `TurretSubsystem.java`.

### 6.1 How It Works

The method predicts where the robot will be in **0.2 seconds** (the approximate note travel time)
and aims at where the target will appear from that future position:

```java
double latencySeconds = 0.2;
Translation2d futurePosition = robotPose.getTranslation().plus(
    new Translation2d(
        fieldSpeeds.vxMetersPerSecond * latencySeconds,
        fieldSpeeds.vyMetersPerSecond * latencySeconds));
```

### 6.2 Tuning the Lead Time

The `latencySeconds` value (currently 0.2) controls how far ahead the turret aims:

| Value | Effect |
|-------|--------|
| Too low (< 0.1) | Shots land behind the target when driving fast |
| Too high (> 0.3) | Shots land ahead of the target |
| Just right (~0.2) | Shots land on target while driving at moderate speed |

To tune:
1. Drive at a consistent speed parallel to the hub (~2 m/s)
2. Fire shots and observe where they land relative to the hub
3. If shots trail behind → increase `latencySeconds`
4. If shots lead too far → decrease `latencySeconds`

**Note:** Velocity compensation is currently used by `aimAtPoseCompensated()` but the default
turret command uses `aimAtPose()` (no compensation). To enable it, change the default command
in `RobotContainer.java` to call `aimAtPoseCompensated()` with the drivetrain's field-relative speeds.

---

## 7. Zone-Based Smart Targeting

The robot automatically changes its target based on where it is on the field.
This logic lives in `RobotContainer.java` → `getSmartTarget()`.

### 7.1 Field Zones (2026 REBUILT)

The field is 16.54m long, divided into three zones:

```
Blue Wall                                                    Red Wall
  |  Blue Alliance Zone  |     Neutral Zone      |  Red Alliance Zone  |
  0m                   4.03m                    12.51m               16.54m
                         ^                        ^
                      Trench                   Trench
                    (±1m buffer)             (±1m buffer)
```

### 7.2 Targeting Rules

| Robot Location | Target | Hood Behavior |
|---------------|--------|--------------|
| Own alliance zone (normal play) | Hub on our side | Auto-aim from distance tables |
| Near trench boundary (±1m) | N/A | Hood flattens to 0.0 to fit under trench (22.25in clearance) |
| Neutral zone / opponent's side | Closest corner target | Auto-aim from distance tables |

### 7.3 Target Positions (from Constants.java)

| Target | Blue Alliance | Red Alliance |
|--------|--------------|-------------|
| Hub | (4.625, 4.04) | (11.915, 4.04) |
| Corner Left | (3.5, 6.0) | (13.04, 6.0) |
| Corner Right | (3.5, 2.0) | (13.04, 2.0) |

Corner targets are used when the robot is on the opponent's side of the field — it aims at
whichever corner is closer to the robot's current Y position.

### 7.4 Tuning Zone Boundaries

All zone constants are in `Constants.java` → `FieldConstants`:

| Constant | Value | What It Controls |
|----------|-------|-----------------|
| `ALLIANCE_ZONE_DEPTH` | 4.03m | How deep each alliance zone extends from the wall |
| `TRENCH_BUFFER` | 1.0m | How far from the trench boundary the hood starts flattening |
| `FIELD_LENGTH_METERS` | 16.54m | Total field length (used to mirror red/blue) |

If the robot is flattening its hood too early or too late near the trench, adjust `TRENCH_BUFFER`.
If zone transitions feel wrong, verify `ALLIANCE_ZONE_DEPTH` matches the 2026 REBUILT game manual.

---

## 8. Co-Pilot Offset Tuning

The co-pilot (controller 2, USB port 1) can live-tune turret aim and shooter power during a match.
These offsets persist until manually reset and affect ALL auto-aim calculations.

### 8.1 Controls

| Input | Action | Increment |
|-------|--------|-----------|
| D-pad Left | Nudge turret aim CCW (left) | +0.5 degrees per press |
| D-pad Right | Nudge turret aim CW (right) | -0.5 degrees per press |
| D-pad Up | Increase flywheel power | +1 RPS (~60 RPM) per press |
| D-pad Down | Decrease flywheel power | -1 RPS (~60 RPM) per press |
| Back button | Reset BOTH offsets to zero | — |

Offsets accumulate — pressing D-pad Left 3 times shifts aim by 1.5 degrees.

### 8.2 When to Use

- **Turret aim offset:** If shots consistently land left or right of the hub, the co-pilot nudges
  the aim to compensate. This is faster than re-calibrating the camera pose mid-match.
- **Shooter power offset:** If shots are falling short or going long (due to battery voltage drop,
  game piece variation, or field conditions), the co-pilot nudges the power up or down.

### 8.3 Dashboard Monitoring

Current offsets are displayed on the **Calibration** tab in Shuffleboard:
- `Turret Aim Offset (deg)` — current aim offset in degrees
- `Shooter RPS Offset` — current power offset in RPS

Both values also print to the console when changed:
```
>>> Turret offset: 1.5 deg <<<
>>> Shooter RPS offset: +2.0 RPS <<<
```

### 8.4 Tips

- Start each match with offsets at zero (press Back on co-pilot controller during setup)
- Make small adjustments — one press at a time, then observe the next shot
- If you need more than ±3 degrees of aim offset, something else is wrong (check camera pose, turret zero)
- If you need more than ±5 RPS of power offset, check battery voltage and flywheel condition

---

## 9. Quick Reference — What Lives Where

A cheat sheet for finding and changing key values:

| What You Want to Change | File | Location |
|------------------------|------|----------|
| Turret PID gains (KP, KI, KD, KS) | `TurretSubsystem.java` | Constructor → `Slot0Configs` |
| Turret travel limits (degrees) | `TurretSubsystem.java` | `MAX_MECHANISM_ROTATIONS`, `MIN_MECHANISM_ROTATIONS` |
| Turret reset speed | `TurretSubsystem.java` | `RESET_SPEED_FRACTION` |
| Turret deadband | `TurretSubsystem.java` | Constructor → `DutyCycleNeutralDeadband` |
| Turret aim nudge size | `TurretSubsystem.java` | `AIM_NUDGE_DEGREES` |
| Flywheel PID gains | `ShooterSubsystem.java` | Constructor → `Slot0Configs` |
| Shot calibration tables (hood + RPS) | `ShooterSubsystem.java` | Constructor → `hoodTable` / `shooterRPSTable` |
| Flywheel ready tolerance | `ShooterSubsystem.java` | `isReadyToShoot()` → 0.05 (5%) |
| RPS nudge size | `ShooterSubsystem.java` | `RPS_NUDGE` |
| Hood servo limits | `Constants.java` | `ShootingConstants.HOOD_MIN` / `HOOD_MAX` |
| CAN IDs (all motors) | `Constants.java` | `DriveTrainConstants`, `ShootingConstants`, `IntakeConstants`, `ClimberConstants` |
| Field zone boundaries | `Constants.java` | `FieldConstants` |
| Hub and corner target positions | `Constants.java` | `FieldConstants` → `*_HUB_POSE`, `*_CORNER_*` |
| Trench buffer distance | `Constants.java` | `FieldConstants.TRENCH_BUFFER` |
| Vision std devs (trust level) | `Robot.java` | `updateVisionPoseEstimate()` → `VecBuilder.fill(...)` |
| Spin rejection threshold | `Robot.java` | `updateVisionPoseEstimate()` → `> 360` |
| Limelight names | `Robot.java` | `"limelight-front"`, `"limelight-back"` |
| IMU mode settings | `Robot.java` | `disabledPeriodic()`, `autonomousInit()`, `teleopInit()` |
| Velocity compensation lead time | `TurretSubsystem.java` | `calculateTargetWithCompensation()` → `latencySeconds` |
| Drive speed limits | `RobotContainer.java` | `MaxSpeed`, `MaxAngularRate` |
| Joystick curve | `RobotContainer.java` | `joystickCurve()` |
| Slew rate (acceleration smoothing) | `RobotContainer.java` | `SlewRateLimiter(3.0)` |
| Controller button bindings | `RobotContainer.java` | Constructor (look for `controller.*` and `copilot.*`) |
| Smart targeting logic | `RobotContainer.java` | `getSmartTarget()` |
| Auto routines | `src/main/deploy/pathplanner/autos/` | PathPlanner `.auto` files |
| Auto paths | `src/main/deploy/pathplanner/paths/` | PathPlanner `.path` files |
