# Features & Enhancements — Enable/Disable Guide

These features were stripped out during turret aiming debugging. Add them back one at a time, testing after each.

---

## 1. Turret Pivot Offset
**Status:** DISABLED → RE-ENABLED
**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
**What it does:** Accounts for the turret pivot being 4.5" back and 4" left of robot center. Improves aim accuracy, especially at close range.

**To enable:** In `calculateTargetMechanismRotations`, replace:
```java
// SIMPLIFIED: use robot center directly (no turret pivot offset)
Rotation2d fieldAngle = targetPose.getTranslation()
    .minus(robotPose.getTranslation())
    .getAngle();
```
With:
```java
Translation2d turretFieldPos = robotPose.getTranslation()
    .plus(TURRET_OFFSET.rotateBy(robotPose.getRotation()));
Rotation2d fieldAngle = targetPose.getTranslation()
    .minus(turretFieldPos)
    .getAngle();
```
Also update the debug telemetry to publish `turretFieldPos` instead of `robotPose`.

---

## 2. Limelight Vision Fusion
**Status:** CODE EXISTS, just plug in Limelights
**File:** `src/main/java/frc/robot/Robot.java`
**What it does:** Fuses AprilTag pose estimates from two Limelights (front + back) into the drivetrain's Kalman filter. Corrects position drift from wheel odometry.

**To enable:** Just plug in the Limelights. The code in `updateVisionPoseEstimate()` and `fuseCameraEstimate()` already runs every loop — it skips when no data is available.

**Vision heading seed:** While disabled, `seedHeadingFromVision()` uses MegaTag1 multi-tag solves to auto-correct the Pigeon heading. This means you can place the robot at any angle and it figures out heading from AprilTags.

**Settings to verify in Limelight web UI:**
- Pipeline: AprilTag
- Family: AprilTag Classic 36h11
- Tag size: 165.1mm
- Field map: 2026 REBUILT Welded .fmap uploaded
- LEDs: Off
- Bot dimensions: 0.8636m × 0.8382m (34×33 in with bumpers)
- Front camera pose: Forward=0.127m, Right=0.254m, Up=0.4413m, Roll=0, Pitch=0, Yaw=0
- Back camera pose: Forward=-0.2286m, Right=0.1175m, Up=0.5017m, Roll=-90, Pitch=0, Yaw=180

---

## 3. Aim Deadband (Anti-Hunting)
**Status:** DISABLED
**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
**What it does:** When the turret is within ~2° of target, stops updating the setpoint. Prevents constant micro-corrections from vision pose noise that make the turret "hunt" back and forth.

**To enable:** In `aimAtPose()` and `aimAtPoseCompensated()`, add before `goToPosition()`:
```java
double currentPos = turretMotor.getPosition().getValueAsDouble();
if (Math.abs(currentPos - finalTarget) < 0.006) { // 0.006 rot ≈ 2.2°
    return;
}
```
**When to enable:** After Limelights are running. Not needed without vision.

---

## 4. Zone-Based Smart Targeting
**Status:** DISABLED (simplified to hub only)
**File:** `src/main/java/frc/robot/RobotContainer.java`
**What it does:** Changes the aim target based on where the robot is on the field:
- Own alliance zone → aim at hub
- Near trench boundary → flatten hood to fit under trench
- Neutral zone / opponent's side → aim at corner targets to feed balls back

**To enable:** Replace the simplified `getSmartTarget()` with full zone logic:
```java
public Pose2d getSmartTarget() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    Pose2d robotPose = drivetrain.getState().Pose;
    double x = robotPose.getX();

    if (isRed) {
        if (x >= Constants.FieldConstants.RED_ZONE_START) {
            return Constants.FieldConstants.RED_HUB_POSE; // Own zone → hub
        } else {
            // Neutral or opponent zone → nearest corner
            double y = robotPose.getY();
            return (y > 4.04) ? Constants.FieldConstants.RED_CORNER_LEFT
                              : Constants.FieldConstants.RED_CORNER_RIGHT;
        }
    } else {
        if (x <= Constants.FieldConstants.BLUE_ZONE_END) {
            return Constants.FieldConstants.BLUE_HUB_POSE; // Own zone → hub
        } else {
            double y = robotPose.getY();
            return (y > 4.04) ? Constants.FieldConstants.BLUE_CORNER_LEFT
                              : Constants.FieldConstants.BLUE_CORNER_RIGHT;
        }
    }
}
```
**When to enable:** After basic aiming and Limelights are solid. Needs accurate position to know which zone the robot is in.

---

## 5. Velocity Compensation
**Status:** ENABLED (used in AutoShootCommand)
**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
**What it does:** Predicts where the robot will be in 0.2 seconds and aims there instead. Compensates for robot movement during shot travel time.
**Note:** This is already active in `AutoShootCommand.execute()` via `aimAtPoseCompensated()`. The default command uses `aimAtPose()` (no compensation) which is fine for idle aiming.
