# Shooting-on-the-Move Improvements

Summary of changes made to improve shooting accuracy while the robot is moving.

---

## 1. Critical Bug Fix — Dead Code in AutoShootCommand

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, STEP 3 distance branching

A duplicate `else if (distance > 0.5)` branch was catching all shots beyond 0.5m, stopping the flywheels and labeling them "TRENCH". The actual blended auto-aim code in the next branch was unreachable dead code. The robot was never actually running the hood/flywheel auto-aim logic during matches.

**Fix:** Removed the erroneous branch. The logic now flows: trench → hub inactive → normal shooting (distance > 0.5m) → too close (distance ≤ 0.5m).

---

## 2. Unified Distance Calculations from Turret Pivot

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, STEP 2

**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`  
**Location:** New method `getTurretFieldPosition()`

Distance for power calculations (hood angle + flywheel RPS) now uses the turret pivot position instead of robot center. This matches what the velocity compensation in TurretSubsystem already uses, eliminating ~0.1m inconsistency between where the turret aims and what power the shooter applies.

Added `getTurretFieldPosition(Pose2d)` as a public helper on TurretSubsystem so both systems share the same reference point.

---

## 3. Unified Flight Time Estimation

**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`  
**Location:** `estimateShotSpeed()` — changed from `private` to `public static`

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, STEP 2

AutoShootCommand's power distance calculation now calls `TurretSubsystem.estimateShotSpeed()` instead of using its own separate inline estimate. Both turret aim compensation and power distance compensation now agree on flight time.

---

## 4. Smooth Shot Speed Estimation (No Step Discontinuities)

**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`  
**Location:** `estimateShotSpeed()`

The shot speed function used hard step boundaries (e.g., 5.5 m/s at 1.99m jumping to 4.5 m/s at 2.01m). While moving, distance changes every loop, causing oscillating compensation near boundaries. Now uses linear interpolation: 5.5 m/s at ≤2m, 3.5 m/s at ≥5m, smooth ramp between.

---

## 5. Speed-Scaled Aim Deadband

**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`  
**Location:** `aimAtPose()`

The 1° aim deadband that prevents vision-noise hunting when stationary was also suppressing turret updates while moving. The compensated target shifts continuously during motion, and the deadband was causing the turret to fall behind.

The deadband now scales with robot speed: full 1° when stationary, shrinks to 0.2° at 1.5+ m/s. This preserves noise rejection at rest while allowing tight tracking during motion.

---

## 6. Increased Radial Speed Compensation Factor

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, `aimDistance` calculation

The radial speed compensation multiplier was 0.5 (only compensating for half the expected distance change during flight). Increased to 0.7 for more accurate power adjustment when driving toward or away from the target.

---

## 7. Tighter Flywheel Ready Tolerance

**File:** `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`  
**Location:** `isReadyToShoot()`

Tolerance tightened from `max(10%, 3 RPS)` to `max(7%, 2 RPS)`. While moving, the target RPS shifts every loop as distance changes. The wider tolerance allowed the flywheels to be declared "ready" when they were actually several RPS off the current target. Tighter tolerance ensures shots fire at closer to the intended power.

---

## 8. Removed Asymmetric Hood Bias in Blended Auto-Aim

**File:** `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`  
**Location:** `blendedHubAutoAim()`

The blended hub auto-aim had a bias that pulled the hood toward the table value whenever physics suggested a flatter trajectory. This prevented the physics model from correctly commanding flatter shots at longer distances. The blend is now symmetric — physics can adjust the hood in either direction proportional to the blend weight.

---

## Next Steps — Recommended Future Improvements

These were identified during the code review but not implemented yet. Worth tackling after testing the changes above to see where accuracy stands.

### A. Angular Velocity Compensation for Turret Rotation

**Files:** `TurretSubsystem.java`, possibly `AutoShootCommand.java`

The velocity compensation accounts for the robot translating (driving around) but not rotating (spinning). When the robot spins while shooting, the turret has to track faster and the ball inherits tangential velocity from the turret's own rotation. At typical match rotation rates this is small (~0.5° error), but if the driver is spinning while shooting it can cause noticeable drift. The fix is to add the turret's tangential velocity component to the compensation calculation in `getCompensatedTarget()`.

### B. Latency Compensation for CAN Bus + Servo Delay

**Files:** `TurretSubsystem.java`, `ShooterSubsystem.java`

The turret position read from the TalonFX is ~20ms old (CAN bus round-trip), and the hood servo takes 50-100ms to reach its commanded position. For a robot moving at 2 m/s, that's 4-8cm of uncompensated position error on the turret side, and the hood may not be at the correct angle when the ball feeds. A latency compensator would use the robot's velocity to project the pose forward by the known latency, and the hood command could be sent slightly early based on predicted distance.

### C. Calibrate Passing Shot Tables

**File:** `ShooterSubsystem.java`

All 10 passing shot data points (5 hood + 5 RPS) are marked "NEEDS CALIBRATION". Until calibrated, the blended approach for passing shots is mixing two uncalibrated sources. Either calibrate the tables on the field, or lean fully on physics for passing shots by forcing `physicsWeight = 1.0` when `isPassing` is true.

### D. Run SysId Characterization on Flywheels

**File:** `ShooterSubsystem.java`

The flywheel PID gains (KP=0.1, KS=0.1, KV=0.12) appear to be rough estimates. Running WPILib's SysId tool on the flywheel motors would give you real KS/KV values, which directly improves how fast and accurately the flywheels reach target speed — especially important for on-the-move shots where the target RPS shifts every loop.

### E. Validate `exitVelocityToRPS` Conversion Factor

**File:** `TrajectoryCalculations.java`

The `MOTOR_RPS_PER_MPS` constant (4.25) was back-calculated from a single calibration point and bumped once. As the physics-based aiming gets more weight during moving shots, this conversion becomes critical. Validating it across multiple distances (measure actual ball exit velocity with a radar gun or high-speed video) would improve physics accuracy across the board.

---

## Vision / Limelight Updates (Robot.java)

### 9. Soft-Clamp Replaces Hard Reset for Out-of-Bounds Pose

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `clampPoseToField()`

When the robot drove into a wall, the wheels would spin without the robot moving, causing odometry to drift past the field boundary. The old code called `resetPose()` which hard-resets the Kalman filter — wiping all filter state and causing the turret aim to jump every loop as the pose got yanked back and forth.

Now uses `addVisionMeasurement()` with a 0.5m std dev to gently nudge the pose back onto the field over several loops. The trigger margin was also widened from 0.5m to 1.0m so minor bumper-overhang drift doesn't trigger the correction at all.

### 10. Removed Hard Speed Rejection Gate

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `updateVisionPoseEstimate()`

The old code rejected all vision measurements when driving faster than 3 m/s. This meant the robot got zero vision corrections during fast driving — exactly when odometry drift is worst. Now, speed is handled entirely through std dev scaling in `fuseCameraEstimate()`. Fast driving = high std devs (barely trusted), but measurements still trickle in rather than being completely discarded.

### 11. Tightened Spin Rejection Threshold

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `updateVisionPoseEstimate()`

Spin rejection threshold lowered from 150°/s to 120°/s. MegaTag2 depends on accurate heading, and camera frames blur during rotation. 120°/s is still well above normal driving turns but catches aggressive spins earlier.

### 12. Tightened Jump Rejection Threshold

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `fuseCameraEstimate()`

Jump rejection threshold tightened from 3.0m to 2.0m. A well-calibrated swerve drifts 0.5-1.5m over a full match. A 2m jump from odometry is almost certainly a bad vision read, not legitimate drift correction.

### 13. Steeper Speed-Based Std Dev Scaling

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `fuseCameraEstimate()`

Speed multiplier changed from `1 + v²×0.5 + v×0.5` to `1 + v²`. This makes trust drop off faster at higher speeds (5x at 2 m/s instead of 3x), reducing vision-induced aim wobble while driving fast.

### 14. Stopped Sending yawRate to SetRobotOrientation

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `updateVisionPoseEstimate()`

Per Limelight 4 docs, when using IMU mode 4 (internal IMU + external assist), the LL4's internal IMU handles angular velocity at 1kHz internally. Sending our 50Hz yaw rate was redundant and could add noise. Now passes zeros for all rate parameters, matching the official recommended usage.

### 15. Clarified LL4 IMU Mode Comments

**File:** `src/main/java/frc/robot/Robot.java`  
**Location:** `disabledPeriodic()`, `autonomousInit()`, `teleopInit()`

Updated comments to accurately reflect LL4 IMU mode behavior per current docs. Clarified that the back camera uses mode 0 because it's in portrait orientation (LL4 internal IMU requires landscape mount).

---

## Remaining Next Steps — Now Implemented

### 16. Angular Velocity Compensation for Turret Rotation

**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`  
**Location:** `getCompensatedTarget()`

The velocity compensation now accounts for both translational velocity (robot driving) and tangential velocity from robot rotation. When the robot spins, the turret pivot traces a circle and picks up sideways velocity proportional to `omega × pivot_radius`. This tangential velocity is added to the translational velocity before computing the compensated aim point. At the turret's 0.153m offset from robot center, spinning at 180°/s adds ~0.48 m/s of tangential velocity — enough to shift the aim point by several degrees at typical shooting distances.

### 17. Latency Compensation for Hood Servo

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, new `hoodDistance` calculation

The hood servo takes ~80ms to reach its commanded position. While moving, the distance to the target changes during that 80ms, so the hood arrives at the wrong angle. Now the code tracks the rate of distance change between loops and predicts where the distance will be 80ms from now. The hood and flywheel commands use this predicted distance (`hoodDistance`) instead of the current distance, so the servo arrives at the correct angle by the time the ball feeds. The prediction is clamped to ±20% of the current distance to prevent wild swings from noisy distance readings.

### 18. Passing Shots Use 100% Physics

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** `execute()` method, STEP 3

Since all 10 passing shot table entries are marked "NEEDS CALIBRATION", blending with uncalibrated table values was making passing shots less accurate. Passing shots now always use `physicsWeight = 1.0` (pure physics from `TrajectoryCalculations`). Hub shots still use the speed-based blend (tables at rest, physics while moving). Once the passing tables are calibrated on the field, this can be changed back to blended mode.

### 19. Exit Velocity Telemetry for Conversion Factor Validation

**File:** `src/main/java/frc/robot/commands/AutoShootCommand.java`  
**Location:** Telemetry section in `execute()`

Added two new dashboard values under the Calibration table:
- `Shot Exit Vel (m/s)` — the physics model's predicted ball exit velocity for the current distance. Compare this to actual ball speed measured with a radar gun or high-speed video to validate the `MOTOR_RPS_PER_MPS` conversion factor in `TrajectoryCalculations.exitVelocityToRPS()`.
- `Shot Hood Distance` — the latency-compensated distance being used for hood/flywheel commands. Compare to `Shot Distance` (raw distance) to see how much the latency prediction is shifting things.
