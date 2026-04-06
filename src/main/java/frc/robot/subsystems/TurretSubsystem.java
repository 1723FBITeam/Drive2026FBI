package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * TurretSubsystem — rotates the shooter left/right to aim at the hub.
 *
 * HOW IT WORKS:
 * The turret sits on top of the robot and can spin beyond 360 degrees total.
 * It uses MOTION MAGIC POSITION CONTROL, meaning we tell the motor
 * "go to this angle" and the motor generates a smooth trapezoidal/S-curve
 * velocity profile to get there without skipping gear teeth. The profile
 * limits acceleration and cruise velocity so the small turret pinion
 * doesn't get overwhelmed on large moves.
 *
 * COORDINATE SYSTEM:
 * - 0 position = turret facing the REAR of the robot
 * - Positive rotation = counterclockwise (viewed from above)
 * - Positions are in "mechanism rotations" (1.0 = full 360 turn)
 *
 * TURRET RANGE:
 * - CCW (positive): up to +450 degrees (+1.25 mechanism rotations)
 * - CW (negative):  up to -90 degrees (-0.25 mechanism rotations)
 * - Total travel: 540 degrees
 *
 * Because the turret can go past 360 degrees, any target angle has TWO
 * equivalent positions (e.g., +90 deg and +90-360 = -270 deg). We pick
 * whichever one is in bounds and closest to the current position.
 *
 * RESET BEHAVIOR:
 * When the turret is near a limit and needs to wrap around 360 degrees
 * to reach the target, it enters "resetting" mode — it moves slower
 * and the shooter will NOT fire until the reset is complete.
 *
 * GEAR RATIO: 11.515 motor rotations = 1 turret rotation
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(
        Constants.ShootingConstants.TURRET_MOTOR, Constants.kCANivoreBus);

    // 11.515 motor rotations = 1 full turret rotation (360 degrees)
    private static final double TURRET_GEAR_RATIO = 11.515;

    // ===== TURRET PIVOT OFFSET FROM ROBOT CENTER =====
    // The turret pivot is NOT at the center of the robot.
    // It is 4.5 inches back and 4 inches to the left (from driver's perspective).
    // In robot coordinates: forward is +X, left is +Y
    //   Back 4.5in = -0.1143m in X
    //   Left 4in   = +0.1016m in Y (left is positive in WPILib robot coords)
    private static final Translation2d TURRET_OFFSET = new Translation2d(-0.1143, 0.1016);

    // ===== TURRET TRAVEL LIMITS =====
    // CCW (positive direction, viewed from above): 450 degrees = 450/360 = 1.25 rotations
    // CW (negative direction, viewed from above): 90 degrees = 90/360 = 0.25 rotations
    // Total travel: 540 degrees
    private static final double MAX_MECHANISM_ROTATIONS = 1.25;   // +450° CCW
    private static final double MIN_MECHANISM_ROTATIONS = -0.25;  // -90° CW

    // ===== RESET DETECTION =====
    // When the turret needs to travel more than this many rotations to reach
    // its target, we consider it a "reset" (wrapping around 360 degrees).
    // 0.6 rotations = 216 degrees — any move bigger than this is a reset.
    private static final double RESET_THRESHOLD_ROTATIONS = 0.6;
    // How close the turret must be to the target to consider the reset "done"
    // 0.005 rotations = ~1.8 degrees — tighter than before to prevent firing
    // while still settling from a reset
    private static final double RESET_DONE_TOLERANCE = 0.005;


    // Tracks whether the turret is currently doing a long reset move
    private boolean isResetting = false;

    // ===== AIM OFFSET (adjusted by co-pilot controller 2) =====
    // This offset is added to every aim calculation, in mechanism rotations.
    // D-pad left/right on controller 2 nudges this value during the match.
    // Positive = shift aim counterclockwise, Negative = shift aim clockwise.
    private double aimOffsetRotations = 0.0;
    // Each D-pad press nudges by 1 degree (converted to mechanism rotations)
    private static final double AIM_NUDGE_DEGREES = 1.0;
    private static final double AIM_NUDGE_ROTATIONS = AIM_NUDGE_DEGREES / 360.0;

    // Deadband: ignore aim updates smaller than this to prevent jitter from vision noise
    private static final double AIM_DEADBAND_DEGREES = 1.0;
    private static final double AIM_DEADBAND_ROTATIONS = AIM_DEADBAND_DEGREES / 360.0;

    // ===== MOTION MAGIC SETTINGS =====
    // Motion Magic generates a trapezoidal/S-curve velocity profile so the turret
    // accelerates and decelerates smoothly. This prevents the small gear from
    // skipping teeth on large moves (the old PositionVoltage would slam full voltage
    // instantly, causing the motor to accelerate faster than the gear mesh could handle).
    //
    // All values are in MECHANISM units (turret rotations/sec) because
    // SensorToMechanismRatio is configured. 1.0 rps = 360 deg/sec of turret.
    //
    // TUNING GUIDE:
    //   - If teeth still skip: lower acceleration and/or cruise velocity
    //   - If turret is too slow to track: raise cruise velocity first, then acceleration
    //   - Jerk controls how abruptly acceleration changes (S-curve smoothing)
    private static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.55;  // rps (198 deg/sec)
    private static final double MOTION_MAGIC_ACCELERATION = 1.5;     // rps/s (reaches cruise in 0.37s)
    private static final double MOTION_MAGIC_JERK = 15.0;            // rps/s/s (smooth S-curve)

    // MotionMagicVoltage = "go to this position using a smooth motion profile"
    // Normal aiming uses Slot 0 (full PID gains)
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0)
        .withEnableFOC(false);

    // Slower motion profile for resets — uses Slot 1 with gentler gains
    private final MotionMagicVoltage resetPositionRequest = new MotionMagicVoltage(0)
        .withEnableFOC(false)
        .withSlot(1);

    // DutyCycleOut = simple "spin at X% power" for manual control and resets
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);

    // Dashboard telemetry publishers (essential only)
    private final DoublePublisher ntPosition;
    private final DoublePublisher ntTarget;
    private final DoublePublisher ntError;
    private final BooleanPublisher ntResetting;
    private final DoublePublisher ntAimOffset;
    private int telemetryCounter = 0;
    private double lastAimErrorDegrees = 0.0;

    public TurretSubsystem() {
        // PID + Feedforward gains for position control
        // KP: How aggressively to correct error (higher = faster, may oscillate)
        // KI: Corrects steady-state error over time — helps push through increasing
        //     friction/cable tension as the turret rotates further from center
        // KD: Dampens oscillation (increased slightly to match higher KP)
        // KS: Voltage to overcome static friction — raised because turret gets
        //     stiffer at the extremes of travel
        var slot0 = new Slot0Configs()
            .withKP(30.0)
            .withKI(0.5)
            .withKD(0.2)
            .withKS(0.5)
            .withKV(1.3)   // Velocity feedforward for Motion Magic profile (V per mechanism rps)
            .withKA(0.1)   // Acceleration feedforward for Motion Magic profile
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        // Slot 1: Reset moves (360° wrap-around).
        // Mechanics upgraded — running fast. +25% from last tuning pass.
        var slot1 = new Slot1Configs()
            .withKP(31.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.5)
            .withKV(1.25)
            .withKA(0.10)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        // Motion Magic profile limits — prevents teeth skipping by controlling
        // how fast the motor accelerates. Values are in mechanism units (turret rps)
        // because SensorToMechanismRatio is configured below.
        var motionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION)
            .withMotionMagicJerk(MOTION_MAGIC_JERK);

        // Tell motor about gear ratio so positions are in turret rotations
        var feedback = new FeedbackConfigs()
            .withSensorToMechanismRatio(TURRET_GEAR_RATIO);

        // Hardware soft limits prevent over-rotation even if code has a bug
        var softLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(MAX_MECHANISM_ROTATIONS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(MIN_MECHANISM_ROTATIONS);

        turretMotor.getConfigurator().apply(slot0);
        turretMotor.getConfigurator().apply(slot1);
        turretMotor.getConfigurator().apply(motionMagic);
        turretMotor.getConfigurator().apply(feedback);
        turretMotor.getConfigurator().apply(softLimits);

        // Deadband: ignore output below 4% to prevent buzzing near target
        var motorOutput = new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.04);
        turretMotor.getConfigurator().apply(motorOutput);

        // Zero encoder at startup (turret starts facing rear of robot)
        turretMotor.setPosition(0);

        NetworkTable calTable = NetworkTableInstance.getDefault()
            .getTable("Calibration");
        ntPosition     = calTable.getDoubleTopic("Turret Pos").publish();
        ntTarget       = calTable.getDoubleTopic("Turret Target").publish();
        ntError        = calTable.getDoubleTopic("Turret Error Deg").publish();
        ntResetting    = calTable.getBooleanTopic("Turret Resetting").publish();
        ntAimOffset    = calTable.getDoubleTopic("Turret Aim Offset Deg").publish();
    }

    /** Manually rotate the turret (trigger buttons). Soft limits still enforced. */
    public void rotate(double speed) {
        isResetting = false; // Manual control cancels any reset
        turretMotor.setControl(manualRequest.withOutput(speed));
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /**
     * Returns true if the turret is currently doing a long reset move
     * (wrapping around 360 degrees to get back in range).
     * While resetting, the shooter should NOT fire.
     */
    public boolean isResetting() {
        return isResetting;
    }

    /**
     * Returns true if a reset is about to happen — the turret is near a soft
     * limit and the target would require a large move. AutoShootCommand uses
     * this to stop the feeder ~100ms BEFORE the reset starts, preventing a
     * ball from firing during the wrap-around.
     */
    public boolean isResetImminent(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds speeds) {
        if (isResetting) return true; // Already resetting
        Translation2d compensated = getCompensatedTarget(robotPose, targetPose, speeds);
        Pose2d compensatedPose = new Pose2d(compensated, targetPose.getRotation());
        double targetMechRot = calculateTargetMechanismRotations(robotPose, compensatedPose);
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        return Math.abs(targetMechRot - currentPos) > (RESET_THRESHOLD_ROTATIONS * 0.8);
    }

    /** Nudge the aim offset left (CCW). Called by co-pilot D-pad left. */
    public void nudgeAimLeft() {
        aimOffsetRotations += AIM_NUDGE_ROTATIONS;
    }

    /** Nudge the aim offset right (CW). Called by co-pilot D-pad right. */
    public void nudgeAimRight() {
        aimOffsetRotations -= AIM_NUDGE_ROTATIONS;
    }

    /** Reset the aim offset back to zero. */
    public void resetAimOffset() {
        aimOffsetRotations = 0.0;
    }

    /**
     * Calculate where the turret needs to point to aim at a target.
     *
     * Because the turret range is asymmetric (+420 CCW / -245 CW) and extends
     * past 360 degrees, any desired angle has two equivalent positions separated
     * by 1.0 mechanism rotation (360 degrees). For example, +0.25 rot (90 deg CCW)
     * is equivalent to -0.75 rot (270 deg CW).
     *
     * We pick the candidate that:
     *   1. Is within the turret's physical limits
     *   2. Is closest to the current turret position (shortest move)
     *
     * Steps:
     *   1. Find angle from robot to target on the field
     *   2. Subtract robot heading to get robot-relative angle
     *   3. Subtract 180 because turret zero = rear of robot
     *   4. Convert degrees to mechanism rotations (divide by 360)
     *   5. Generate two candidates (base and base +/- 1.0 rotation)
     *   6. Pick the best in-bounds candidate closest to current position
     */
    private double calculateTargetMechanismRotations(Pose2d robotPose, Pose2d targetPose) {
        // Account for turret pivot being offset from robot center
        // (4.5" back, 4" left from driver's perspective)
        Translation2d turretFieldPos = robotPose.getTranslation()
            .plus(TURRET_OFFSET.rotateBy(robotPose.getRotation()));

        Rotation2d fieldAngle = targetPose.getTranslation()
            .minus(turretFieldPos)
            .getAngle();

        Rotation2d relativeAngle = fieldAngle.minus(robotPose.getRotation());

        // Turret zero = REAR of robot (confirmed). Subtract 180°.
        Rotation2d turretAngle = relativeAngle.minus(Rotation2d.k180deg);

        double base = turretAngle.getDegrees() / 360.0;

        // Generate the two equivalent positions (360 degrees apart)
        double candidateA = base;
        double candidateB = (base > 0) ? base - 1.0 : base + 1.0;

        // Check which candidates are within the turret's physical limits
        boolean aInBounds = candidateA >= MIN_MECHANISM_ROTATIONS && candidateA <= MAX_MECHANISM_ROTATIONS;
        boolean bInBounds = candidateB >= MIN_MECHANISM_ROTATIONS && candidateB <= MAX_MECHANISM_ROTATIONS;

        double currentPos = turretMotor.getPosition().getValueAsDouble();

        // Pick the best candidate
        if (aInBounds && bInBounds) {
            // Both are reachable — pick whichever is closer to current position
            double distA = Math.abs(candidateA - currentPos);
            double distB = Math.abs(candidateB - currentPos);
            return (distA <= distB) ? candidateA : candidateB;
        } else if (aInBounds) {
            return candidateA;
        } else if (bInBounds) {
            return candidateB;
        } else {
            // Neither candidate is in bounds — clamp to the nearest limit
            return MathUtil.clamp(base, MIN_MECHANISM_ROTATIONS, MAX_MECHANISM_ROTATIONS);
        }
    }

    // ===== ITERATIVE VELOCITY COMPENSATION =====
    // Inspired by The Flying Circuits (FRC 2026-Robot).
    // Instead of a simple linear time offset, we iteratively refine the aim point:
    //   1. Calculate time-of-flight to the target
    //   2. Offset the target by robot velocity × flight time
    //   3. Recalculate with the new target (flight time changes because distance changed)
    //   4. Repeat — converges in ~4 iterations
    //
    // This is more accurate than a fixed time offset because the compensation
    // amount depends on the actual shot distance, which changes as we compensate.
    //
    // Number of refinement iterations. 4 is plenty — error is negligible after 3.
    private static final int COMPENSATION_ITERATIONS = 4;

    /**
     * Estimate ball speed in flight based on distance to target.
     * Close shots exit faster (higher RPS, shorter distance = less drag time).
     * Far shots are slower. This makes velocity compensation more accurate
     * than a single fixed constant across all distances.
     *
     * Reduced from original values — on-the-move testing showed compensation
     * was too weak (shots drifting ~2ft in the direction of travel at medium speed).
     * Lower shot speed = longer estimated flight time = more compensation.
     */
    private static double estimateShotSpeed(double distanceMeters) {
        if (distanceMeters < 2.0) return 5.5;
        if (distanceMeters < 3.0) return 4.5;
        if (distanceMeters < 4.0) return 4.0;
        return 3.5;
    }

    /**
     * Sends the turret to a target position, automatically detecting if this
     * is a normal aim or a long reset move. If the move is larger than
     * RESET_THRESHOLD_ROTATIONS, the turret moves slower and sets isResetting=true.
     */
    private void goToPosition(double targetMechRot) {
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        double moveDistance = Math.abs(targetMechRot - currentPos);

        if (moveDistance > RESET_THRESHOLD_ROTATIONS) {
            // Big move — this is a reset (wrapping around).
            // Use Slot 1 (gentle PID) so it approaches smoothly without overshoot.
            isResetting = true;
            turretMotor.setControl(resetPositionRequest.withPosition(targetMechRot));
        } else {
            // Normal aim — check if a previous reset is done
            if (isResetting && moveDistance < RESET_DONE_TOLERANCE) {
                isResetting = false; // Reset complete, switch to normal PID
            }
            if (isResetting) {
                // Still resetting but getting close — stay on gentle PID
                turretMotor.setControl(resetPositionRequest.withPosition(targetMechRot));
            } else {
                // Normal closed-loop position control (Slot 0, full speed)
                turretMotor.setControl(positionRequest.withPosition(targetMechRot));
            }
        }

        ntTarget.set(targetMechRot);
    }

    /**
     * Compute a velocity-compensated aim point using iterative refinement.
     *
     * The ball inherits the robot's velocity when launched. To cancel that out,
     * we shift the virtual target backwards by (robot velocity × time-of-flight).
     *
     * When the robot is stationary (speed < 0.1 m/s), returns the original target.
     */
    public Translation2d getCompensatedTarget(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds robotRelativeSpeeds) {
        Translation2d originalTarget = targetPose.getTranslation();

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, robotPose.getRotation());

        double speed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        if (speed < 0.1) {
            return originalTarget;
        }

        // Account for turret pivot offset from robot center
        Translation2d turretFieldPos = robotPose.getTranslation()
            .plus(TURRET_OFFSET.rotateBy(robotPose.getRotation()));

        Translation2d velocityVector = new Translation2d(
            fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        Translation2d compensated = originalTarget;
        for (int i = 0; i < COMPENSATION_ITERATIONS; i++) {
            double distance = turretFieldPos.getDistance(compensated);
            double shotSpeed = estimateShotSpeed(distance);
            double timeOfFlight = distance / shotSpeed;
            compensated = originalTarget.minus(velocityVector.times(timeOfFlight));
        }
        return compensated;
    }

    /**
     * Aim turret at a target, with iterative velocity compensation.
     *
     * Uses iterative refinement to compute a compensated aim point that accounts
     * for the ball inheriting the robot's velocity. When the robot is stationary,
     * this just aims at the real target — no special case needed.
     *
     * @param robotPose   current robot pose from odometry
     * @param targetPose  field target to aim at (hub, corner, etc.)
     * @param fieldSpeeds current chassis speeds (pass drivetrain.getState().Speeds)
     */
    public void aimAtPose(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        Translation2d compensatedTarget = getCompensatedTarget(robotPose, targetPose, fieldSpeeds);
        Pose2d compensatedPose = new Pose2d(compensatedTarget, targetPose.getRotation());

        double targetMechRot = calculateTargetMechanismRotations(robotPose, compensatedPose);
        double finalTarget = targetMechRot + aimOffsetRotations;
        double currentPos = turretMotor.getPosition().getValueAsDouble();

        // Small deadband: if we're already within ~2° of target, don't update.
        // This prevents constant micro-corrections from vision pose noise
        // that make the turret "hunt" back and forth when the robot is stationary.
        if (Math.abs(currentPos - finalTarget) < AIM_DEADBAND_ROTATIONS) {
            return;
        }
        goToPosition(finalTarget);
    }

    /**
     * Returns true when turret is aimed within ~1.5 degrees of the velocity-compensated
     * target AND not resetting. This now checks against the same compensated aim point
     * that aimAtPose() uses, so moving shots are correctly gated.
     */
    public boolean isAimedAtPose(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds speeds) {
        if (isResetting) return false; // Never "aimed" during a reset
        Translation2d compensated = getCompensatedTarget(robotPose, targetPose, speeds);
        Pose2d compensatedPose = new Pose2d(compensated, targetPose.getRotation());
        double targetMechRot = calculateTargetMechanismRotations(robotPose, compensatedPose);
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPos - targetMechRot) < 0.004; // 0.004 rot = ~1.5 degrees
    }

    @Override
    public void periodic() {
        telemetryCounter++;
        if (telemetryCounter % 5 != 0) return; // ~10Hz instead of 50Hz

        double pos = turretMotor.getPosition().getValueAsDouble();
        ntPosition.set(pos);
        double target = turretMotor.getClosedLoopReference().getValueAsDouble();
        lastAimErrorDegrees = (pos - target) * 360.0;
        ntError.set(lastAimErrorDegrees);
        ntResetting.set(isResetting);
        ntAimOffset.set(aimOffsetRotations * 360.0);
    }

    /**
     * Returns the current turret aim error in degrees.
     * Positive = turret is CCW of target, Negative = CW of target.
     * Used by AutoShootCommand for shot telemetry.
     */
    public double getAimErrorDegrees() {
        return lastAimErrorDegrees;
    }
}
