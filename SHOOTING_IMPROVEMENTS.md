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
