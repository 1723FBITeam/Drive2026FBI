# Tuning & Setup Guide

This document covers everything needed to get the robot's new systems dialed in:
Limelight vision setup, turret PID tuning, shooter flywheel tuning, and shot calibration.

---

## Table of Contents

1. [Limelight Setup (New Install)](#1-limelight-setup-new-install)
2. [Turret PID Tuning](#2-turret-pid-tuning)
3. [Shooter Flywheel Tuning](#3-shooter-flywheel-tuning)
4. [Shot Calibration (Hood + Distance Tables)](#4-shot-calibration-hood--distance-tables)
5. [Vision Pose Estimation Tuning](#5-vision-pose-estimation-tuning)
6. [Velocity Compensation Tuning](#6-velocity-compensation-tuning)
7. [Quick Reference — What Lives Where](#7-quick-reference--what-lives-where)

---

## 1. Limelight Setup (New Install)

### 1.1 Physical Mounting

- Mount the Limelight **rigidly to the robot frame** — NOT on the turret.
- Position it where it has a clear view of field AprilTags (rear-facing or elevated is ideal).
- Avoid mounting near motors or areas with heavy vibration.
- Secure all cables — a loose Ethernet cable mid-match will kill your pose estimation.

### 1.2 Wiring

| Connection | Details |
|---|---|
| Power | 12V from PDP/PDH (use a dedicated slot, not shared with motors) |
| Ethernet | To the robot radio or a network switch on the robot |
| Default IP | `10.TE.AM.11` (e.g., team 1234 → `10.12.34.11`) |

### 1.3 Web UI Configuration

1. Connect to the robot's WiFi
2. Open a browser to `http://10.TE.AM.11:5801`
3. **Settings tab:**
   - Set your team number
   - Set the Limelight name to `"limelight"` (must match the string in code)

4. **3D tab — Camera Pose (CRITICAL):**
   - `LL Forward`: distance from robot center to camera along the forward axis (meters)
   - `LL Right`: distance from robot center to camera along the right axis (meters)
   - `LL Up`: height of camera lens from the ground (meters)
   - `LL Roll / Pitch / Yaw`: rotation of the camera in degrees
   - Use the 3D viewer in the UI to verify the camera model looks correct
   - **Measure carefully** — even 2cm of error here translates to pose error on the field

5. **AprilTag Pipeline:**
   - Set the pipeline to "AprilTag" mode
   - Verify tag family is `36h11` (standard for FRC)
   - Upload the correct `.fmap` field map for the 2026 game
     - Generate from the Limelight map generator, or use the default WPILib field layout

6. **If using Limelight 4 (built-in IMU):**
   - The code already handles IMU mode switching automatically:
     - Mode 1 (seeding) during disabled
     - Mode 4 (internal + external assist) during enabled
   - The LL4 must be mounted in **landscape orientation** for IMU mode to work

### 1.4 Verification Checklist

- [ ] Deploy code and connect to robot
- [ ] Open Shuffleboard/AdvantageScope → check `DriveState/Pose`
- [ ] Pose should jump to a reasonable field position when AprilTags are visible
- [ ] Walk robot around the field — pose should track smoothly
- [ ] If pose is mirrored or offset, re-check camera pose in Limelight UI (step 4)
- [ ] Verify `Calibration` tab shows reasonable `Robot X`, `Robot Y`, `Dist to Hub` values

---

## 2. Turret PID Tuning

The turret now runs closed-loop position control on the TalonFX at 1kHz — 20x faster than RIO-side PID.
All gains are configured in `TurretSubsystem.java` constructor.

### 2.1 Current Gains

| Gain | Value | Purpose |
|------|-------|---------|
| KP   | 8.0   | Proportional — main correction force |
| KI   | 0.0   | Integral — leave at 0 unless steady-state error persists |
| KD   | 0.1   | Derivative — dampens oscillation |
| KS   | 0.15  | Static friction feedforward — voltage to overcome stiction |
| KV   | 0.0   | Velocity feedforward — not needed for position control |

### 2.2 Tuning Procedure

1. Open Shuffleboard and look at the `Calibration` tab:
   - `Turret Pos (mech rot)` — current position in mechanism rotations
   - `Turret Target (mech rot)` — where the PID is trying to go
   - `Turret Error Deg` — error in degrees (should settle near 0)

2. Start with KD = 0 and KS = 0. Set KP to a low value like 2.0.

3. Command the turret to aim at the hub (press Start). Watch the error graph:
   - If it slowly creeps to target → increase KP
   - If it overshoots and oscillates → decrease KP
   - If it oscillates once then settles → KP is close, add KD

4. Once KP is set (turret reaches target quickly with 1-2 small oscillations):
   - Add KD starting at 0.05, increase until oscillation is damped
   - Too much KD = sluggish response and motor buzzing/squealing

5. Tune KS (static friction compensation):
   - Command small position changes (a few degrees)
   - If the turret hesitates before moving → increase KS
   - If it lurches at the start of movement → decrease KS
   - KS should be just enough to overcome friction

### 2.3 What to Watch For

- **Oscillation around target**: KP too high or KD too low
- **Motor squealing/buzzing at rest**: KD too high — the derivative term is fighting noise
- **Turret doesn't reach target**: KP too low, or KS too low (friction eating the output)
- **Turret slams into soft limits**: KP too high with no KD — add derivative damping
- **Slow to respond to fast movements**: KP too low — the turret can handle aggressive gains since it's geared 11.515:1

### 2.4 Soft Limits

Hardware-enforced soft limits are set in `TurretSubsystem.java`:
- Forward limit: +0.77 mechanism rotations (+277°)
- Reverse limit: -0.77 mechanism rotations (-277°)
- These are enforced on the motor controller — no code-side clamping needed
- If you need more range, adjust `MAX_MECHANISM_ROTATIONS` and `MIN_MECHANISM_ROTATIONS`

---

## 3. Shooter Flywheel Tuning

Flywheels now use closed-loop velocity control via `VelocityVoltage`. This means shot speed is consistent regardless of battery voltage.
All gains are in `ShooterSubsystem.java` constructor.

### 3.1 Current Gains

| Gain | Value | Purpose |
|------|-------|---------|
| KP   | 0.1   | Proportional — corrects velocity error |
| KI   | 0.0   | Integral — leave at 0 initially |
| KD   | 0.0   | Derivative — usually not needed for flywheels |
| KS   | 0.1   | Static friction — voltage to start spinning |
| KV   | 0.12  | Velocity feedforward — volts per RPS to maintain speed |

### 3.2 Characterizing with SysId (Recommended)

SysId gives you real KS and KV values for your specific motors and mechanism. This is the most accurate way to tune flywheels.

1. In WPILib, open the SysId tool (command palette → "WPILib: Start Tool" → SysId)
2. Configure it for a "Simple Motor" mechanism
3. Run the quasistatic and dynamic tests on the shooter flywheels
4. SysId will output KS and KV values — plug these directly into the `Slot0Configs`
5. After setting KS and KV, tune KP:
   - Start at 0.05
   - Command a target RPS and watch the actual velocity on Shuffleboard
   - Increase KP until the flywheel reaches target quickly without oscillation

### 3.3 Manual Tuning (Without SysId)

If you can't run SysId, tune in this order:

1. Set KP = 0, KS = 0, KV = 0
2. Find KS: slowly increase KS until the flywheels just barely start spinning (try 0.05 → 0.1 → 0.15...)
3. Find KV: command a known RPS (e.g., 50 RPS), measure actual voltage needed, divide voltage by RPS
   - Rough starting point: `KV ≈ 12V / motorFreeSpeedRPS`
   - Kraken X60: `KV ≈ 12 / 106 ≈ 0.113`
   - Falcon 500: `KV ≈ 12 / 100 ≈ 0.12`
4. With KS and KV set, the flywheels should roughly reach target speed open-loop
5. Add KP (start at 0.05) to correct the remaining error

### 3.4 Verification on Shuffleboard

Check the `Calibration` tab:
- `Shooter L Vel (RPS)` / `Shooter R Vel (RPS)` — actual flywheel speeds
- `Shooter Target (RPS)` — commanded speed
- `Ready To Shoot` — true when both flywheels are within 5% of target

What to look for:
- Both flywheels should track the target closely (within ±2 RPS)
- Left and right should be similar — if one lags, check for mechanical binding
- Spin-up time from 0 to 50 RPS should be under 1 second
- No oscillation around the target — if you see hunting, reduce KP

---

## 4. Shot Calibration (Hood + Distance Tables)

The shooter uses interpolation tables to map distance-to-hub → hood position and flywheel RPS.
These tables are in `ShooterSubsystem.java` constructor.

### 4.1 Current Calibration Points

| Distance (m) | Hood Position | Flywheel RPS | ~RPM  |
|---------------|---------------|-------------|-------|
| 1.3           | 0.15          | 35.0        | 2100  |
| 2.1           | 0.25          | 42.0        | 2520  |
| 3.0           | 0.40          | 52.0        | 3120  |
| 3.8           | 0.55          | 62.0        | 3720  |
| 4.7           | 0.70          | 75.0        | 4500  |

These are placeholder values. You need to calibrate with real shots on the field.

### 4.2 Calibration Procedure

1. Place the robot at a known distance from the hub (use a tape measure)
2. Press **Start** to begin the auto-shoot sequence (turret aims, flywheels spin up)
3. Check `Dist to Hub` on the Shuffleboard `Calibration` tab to verify your distance
4. If the shot misses:
   - Press **A button** to nudge the hood UP (increase angle)
   - Press **B button** to nudge the hood DOWN (decrease angle)
   - Each press nudges by 0.02 (configurable in `RobotContainer.java`)
5. Once shots are landing consistently at this distance:
   - Press **D-pad Up** to log the current distance, hood position, and flywheel RPS to the console
6. Move to the next distance and repeat
7. Update the interpolation tables in `ShooterSubsystem.java` with your logged values

### 4.3 Tips for Good Calibration

- Calibrate at 5-7 distances spread across your shooting range (1m to 5m+)
- Do at least 3 shots at each distance to confirm consistency
- Calibrate with a charged battery — velocity control handles voltage sag, but do it anyway
- If shots are inconsistent at the same distance, check:
  - Flywheel spin-up (is `Ready To Shoot` true before feeding?)
  - Turret aim (is `Turret Error Deg` near 0?)
  - Mechanical issues (worn wheels, loose hood servo)
- The interpolation tables will linearly interpolate between your data points, so more points = smoother curve

### 4.4 Updating the Tables

In `ShooterSubsystem.java`, find the calibration tables section and update:

```java
// Hood: distance (meters) → servo position (0.0 to 1.0)
hoodTable.put(1.3, 0.15);   // Replace with your calibrated values
hoodTable.put(2.1, 0.25);
// ... add more points

// Flywheels: distance (meters) → target RPS
shooterRPSTable.put(1.3, 35.0);   // Replace with your calibrated values
shooterRPSTable.put(2.1, 42.0);
// ... add more points
```

---

## 5. Vision Pose Estimation Tuning

Vision fusion is implemented in `Robot.java` → `updateVisionPoseEstimate()`.
It feeds Limelight MegaTag2 poses into the drivetrain's Kalman filter.

### 5.1 How It Works

Every robot loop (~50Hz):
1. Current Pigeon heading + yaw rate is sent to Limelight via `SetRobotOrientation()`
2. Limelight returns a MegaTag2 pose estimate (uses heading + AprilTag detections)
3. Bad updates are rejected (no tags visible, or spinning >360°/s)
4. Good updates are fused into the drivetrain pose estimator with configurable trust levels

### 5.2 Standard Deviation Tuning

The standard deviations control how much the Kalman filter trusts vision vs. odometry:

```java
// In Robot.java → updateVisionPoseEstimate()
m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
    VecBuilder.fill(0.7, 0.7, 9999999));
//                  X    Y    Theta
```

| Parameter | Current Value | What It Means |
|-----------|---------------|---------------|
| X std dev | 0.7           | Trust vision X position within ~0.7 meters |
| Y std dev | 0.7           | Trust vision Y position within ~0.7 meters |
| Theta std dev | 9999999   | Completely ignore vision heading (Pigeon is better) |

**Adjusting trust:**
- Lower values (e.g., 0.3) = trust vision more → pose snaps to vision quickly
- Higher values (e.g., 1.5) = trust vision less → pose changes slowly from vision
- Start at 0.7 and adjust based on testing:
  - If the robot pose jumps around on Shuffleboard → increase stddevs (less trust)
  - If the robot pose drifts over time → decrease stddevs (more trust)
  - If pose is accurate when stationary but bad when moving → the rejection threshold may need tuning

### 5.3 Rejection Threshold Tuning

```java
// In Robot.java → updateVisionPoseEstimate()
if (Math.abs(yawRateDps) > 360) {
    doRejectUpdate = true;
}
```

- Current threshold: 360°/s — rejects vision during fast spins
- If you see bad pose jumps during moderate turning → lower to 270 or 180
- If you're rejecting too many updates during normal driving → raise to 540
- Most teams use 360°/s as a good starting point

### 5.4 Debugging Vision

Open AdvantageScope or Shuffleboard and check:
- `DriveState/Pose` — the fused robot pose (should be smooth and accurate)
- Limelight web UI (`10.TE.AM.11:5801`) — verify tags are being detected
- If pose is mirrored (robot on wrong side of field) → check Limelight camera pose settings
- If pose is offset by a constant amount → re-measure camera position on robot (Section 1.3 step 4)

---

## 6. Velocity Compensation Tuning

Velocity compensation predicts where the robot will be when the shot arrives, so the turret aims ahead.
Implemented in `TurretSubsystem.java` → `aimAtPoseCompensated()` and `AutoShootCommand.java`.

### 6.1 The Latency Constant

```java
// In TurretSubsystem.java → calculateTargetWithCompensation()
// and AutoShootCommand.java → execute()
double latencySeconds = 0.1;
```

This 0.1s value represents the total time from "turret is aimed" to "note reaches the hub."
It includes:
- Feeder delay (0.3s wait after ready, but note is already in motion)
- Note flight time
- Mechanical response time

### 6.2 Tuning Procedure

1. Start with compensation disabled: set `latencySeconds = 0.0` in both files
2. Drive at moderate speed (~1.5 m/s) and shoot at the hub
3. Note where shots miss:
   - Shots land behind where you're going → increase latency (robot moved past the aim point)
   - Shots land ahead of where you're going → decrease latency (overcompensating)
4. Increase in 0.05s increments until shots land consistently while moving
5. Test at different speeds — compensation should work across the range

### 6.3 Important Notes

- Both `TurretSubsystem.java` and `AutoShootCommand.java` have their own `latencySeconds` — keep them in sync
- The compensation only affects turret aim angle and distance calculation, not flywheel speed directly (though distance changes affect the interpolation table lookup)
- At low speeds (<0.3 m/s), compensation has minimal effect — focus testing at 1-2 m/s
- If shots are inconsistent while moving, the issue might be flywheel spin-up time rather than compensation — check `Ready To Shoot` timing

---

## 7. Quick Reference — What Lives Where

### File Locations

| What | File | Section |
|------|------|---------|
| Turret PID (KP, KD, KS) | `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | Constructor → `Slot0Configs` |
| Turret soft limits | `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | `MIN/MAX_MECHANISM_ROTATIONS` |
| Turret gear ratio | `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | `TURRET_GEAR_RATIO = 11.515` |
| Flywheel PID (KP, KS, KV) | `src/main/java/frc/robot/subsystems/ShooterSubsystem.java` | Constructor → `Slot0Configs` |
| Hood interpolation table | `src/main/java/frc/robot/subsystems/ShooterSubsystem.java` | Constructor → `hoodTable.put(...)` |
| Flywheel RPS table | `src/main/java/frc/robot/subsystems/ShooterSubsystem.java` | Constructor → `shooterRPSTable.put(...)` |
| Vision std deviations | `src/main/java/frc/robot/Robot.java` | `updateVisionPoseEstimate()` |
| Vision rejection threshold | `src/main/java/frc/robot/Robot.java` | `yawRateDps > 360` |
| Limelight IMU mode | `src/main/java/frc/robot/Robot.java` | `disabledPeriodic()` / `teleopInit()` |
| Velocity compensation latency | `src/main/java/frc/robot/subsystems/TurretSubsystem.java` + `AutoShootCommand.java` | `latencySeconds = 0.1` |
| Hub target poses | `src/main/java/frc/robot/Constants.java` | `FieldConstants.BLUE_HUB_POSE` / `RED_HUB_POSE` |
| CAN IDs | `src/main/java/frc/robot/Constants.java` | `ShootingConstants`, `DriveTrainConstants`, etc. |
| CANivore bus name | `src/main/java/frc/robot/Constants.java` | `kCANivoreBus = "Carnivore"` |
| PathPlanner auto paths | `src/main/deploy/pathplanner/` | `.auto` and `.path` files |
| Button bindings | `src/main/java/frc/robot/RobotContainer.java` | `configureBindings()` |

### Shuffleboard Calibration Tab Values

| Widget | Source | What It Shows |
|--------|--------|---------------|
| Turret Pos (mech rot) | TurretSubsystem | Current turret position in mechanism rotations |
| Turret Error Deg | TurretSubsystem | Aiming error in degrees |
| Shooter L/R Vel (RPS) | ShooterSubsystem | Actual flywheel speeds |
| Shooter Target (RPS) | ShooterSubsystem | Commanded flywheel speed |
| Ready To Shoot | ShooterSubsystem | True when flywheels are within 5% of target |
| Hood Position | ShooterSubsystem | Current hood servo position (0.0–1.0) |
| Robot X / Robot Y | RobotContainer | Current fused pose from drivetrain |
| Dist to Hub | RobotContainer | Distance from robot to active hub target |

---

*Last updated after implementing vision fusion, onboard turret PID, velocity-controlled flywheels, and shoot-on-the-move compensation.*
