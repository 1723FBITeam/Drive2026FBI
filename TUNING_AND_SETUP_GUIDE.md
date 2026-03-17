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