package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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
 * It uses CLOSED-LOOP POSITION CONTROL, meaning we tell the motor
 * "go to this angle" and the motor PID loop (running at 1000Hz
 * on the TalonFX) handles getting there smoothly.
 *
 * COORDINATE SYSTEM:
 * - 0 position = turret facing the REAR of the robot
 * - Positive rotation = counterclockwise (viewed from above)
 * - Positions are in "mechanism rotations" (1.0 = full 360 turn)
 *
 * TURRET RANGE (asymmetric!):
 * - CCW (positive): up to +420 degrees (+1.1667 mechanism rotations)
 * - CW (negative):  up to -245 degrees (-0.6806 mechanism rotations)
 * - Total travel: 665 degrees
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
    // CCW (positive direction, viewed from above): 420 degrees = 420/360 = 1.1667 rotations
    // CW (negative direction, viewed from above): 245 degrees = 245/360 = 0.6806 rotations
    private static final double MAX_MECHANISM_ROTATIONS = 420.0 / 360.0;   // +1.1667
    private static final double MIN_MECHANISM_ROTATIONS = -245.0 / 360.0;  // -0.6806

    // ===== RESET DETECTION =====
    // When the turret needs to travel more than this many rotations to reach
    // its target, we consider it a "reset" (wrapping around 360 degrees).
    // 0.6 rotations = 216 degrees — any move bigger than this is a reset.
    private static final double RESET_THRESHOLD_ROTATIONS = 0.6;
    // How close the turret must be to the target to consider the reset "done"
    // 0.02 rotations = ~7 degrees
    private static final double RESET_DONE_TOLERANCE = 0.02;
    // Speed multiplier during reset (0.3 = 30% of normal PID output)
    private static final double RESET_SPEED_FRACTION = 0.3;

    // Tracks whether the turret is currently doing a long reset move
    private boolean isResetting = false;

    // Debug print counter (prints every 50 loops ≈ 1 second)
    private int debugCounter = 0;

    // ===== AIM OFFSET (adjusted by co-pilot controller 2) =====
    // This offset is added to every aim calculation, in mechanism rotations.
    // D-pad left/right on controller 2 nudges this value during the match.
    // Positive = shift aim counterclockwise, Negative = shift aim clockwise.
    private double aimOffsetRotations = 0.0;
    // Each D-pad press nudges by 0.5 degrees (converted to mechanism rotations)
    private static final double AIM_NUDGE_DEGREES = 0.5;
    private static final double AIM_NUDGE_ROTATIONS = AIM_NUDGE_DEGREES / 360.0;

    // PositionVoltage = "go to this position and hold it"
    // Normal aiming uses full PID gains
    private final PositionVoltage positionRequest = new PositionVoltage(0)
        .withEnableFOC(false);

    // DutyCycleOut = simple "spin at X% power" for manual control and resets
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);

    // Dashboard telemetry publishers
    private final DoublePublisher ntPosition;
    private final DoublePublisher ntVelocity;
    private final DoublePublisher ntTarget;
    private final DoublePublisher ntDesiredAngle;
    private final DoublePublisher ntError;
    private final BooleanPublisher ntResetting;
    private final DoublePublisher ntAimOffset;
    private final DoublePublisher ntFieldAngle;
    private final DoublePublisher ntRelativeAngle;
    private final DoublePublisher ntTurretFieldX;
    private final DoublePublisher ntTurretFieldY;
    private final DoublePublisher ntBaseMechRot;

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
            .withKS(0.6)
            .withKV(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

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
        turretMotor.getConfigurator().apply(feedback);
        turretMotor.getConfigurator().apply(softLimits);

        // Deadband: ignore output below 4% to prevent buzzing near target
        var motorOutput = new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.04);
        turretMotor.getConfigurator().apply(motorOutput);

        // Zero encoder at startup (turret starts facing rear of robot)
        turretMotor.setPosition(0);

        NetworkTable calTable = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard/Calibration");
        ntPosition     = calTable.getDoubleTopic("Turret Pos (mech rot)").publish();
        ntVelocity     = calTable.getDoubleTopic("Turret Vel (mech rps)").publish();
        ntTarget       = calTable.getDoubleTopic("Turret Target (mech rot)").publish();
        ntDesiredAngle = calTable.getDoubleTopic("Turret Desired Deg").publish();
        ntError        = calTable.getDoubleTopic("Turret Error Deg").publish();
        ntResetting    = calTable.getBooleanTopic("Turret Resetting").publish();
        ntAimOffset    = calTable.getDoubleTopic("Turret Aim Offset (deg)").publish();
        ntFieldAngle   = calTable.getDoubleTopic("Turret Field Angle").publish();
        ntRelativeAngle = calTable.getDoubleTopic("Turret Relative Angle").publish();
        ntTurretFieldX = calTable.getDoubleTopic("Turret Pivot X").publish();
        ntTurretFieldY = calTable.getDoubleTopic("Turret Pivot Y").publish();
        ntBaseMechRot  = calTable.getDoubleTopic("Turret Base MechRot").publish();
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

    /** Nudge the aim offset left (CCW). Called by co-pilot D-pad left. */
    public void nudgeAimLeft() {
        aimOffsetRotations += AIM_NUDGE_ROTATIONS;
        System.out.println(">>> Turret offset: " + String.format("%.1f", aimOffsetRotations * 360.0) + " deg <<<");
    }

    /** Nudge the aim offset right (CW). Called by co-pilot D-pad right. */
    public void nudgeAimRight() {
        aimOffsetRotations -= AIM_NUDGE_ROTATIONS;
        System.out.println(">>> Turret offset: " + String.format("%.1f", aimOffsetRotations * 360.0) + " deg <<<");
    }

    /** Reset the aim offset back to zero. */
    public void resetAimOffset() {
        aimOffsetRotations = 0.0;
        System.out.println(">>> Turret offset RESET to 0 <<<");
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

        // Debug telemetry
        ntFieldAngle.set(fieldAngle.getDegrees());
        ntRelativeAngle.set(relativeAngle.getDegrees());
        ntTurretFieldX.set(turretFieldPos.getX());
        ntTurretFieldY.set(turretFieldPos.getY());

        double base = turretAngle.getDegrees() / 360.0;
        ntBaseMechRot.set(base);

        debugCounter++;
        if (debugCounter % 50 == 0) {
            System.out.println(String.format(
                "AIM: field=%.1f° rel=%.1f° turret=%.1f° base=%.4f rot | pivot=(%.2f,%.2f) target=(%.2f,%.2f) heading=%.1f°",
                fieldAngle.getDegrees(), relativeAngle.getDegrees(), turretAngle.getDegrees(), base,
                turretFieldPos.getX(), turretFieldPos.getY(),
                targetPose.getX(), targetPose.getY(),
                robotPose.getRotation().getDegrees()));
        }

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

    /**
     * Same as above but predicts where the robot WILL BE in ~0.2 seconds.
     * This compensates for note travel time when shooting on the move.
     */
    private double calculateTargetWithCompensation(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        // SIMPLIFIED: no turret pivot offset, just predict future robot center
        double latencySeconds = 0.2;
        Translation2d futurePosition = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * latencySeconds,
                fieldSpeeds.vyMetersPerSecond * latencySeconds));

        Pose2d futurePose = new Pose2d(futurePosition, robotPose.getRotation());
        return calculateTargetMechanismRotations(futurePose, targetPose);
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
            // Big move — this is a reset (wrapping around). Go slower.
            isResetting = true;
            double direction = Math.signum(targetMechRot - currentPos);
            turretMotor.setControl(manualRequest.withOutput(direction * RESET_SPEED_FRACTION));
        } else {
            // Normal aim — check if a previous reset is done
            if (isResetting && moveDistance < RESET_DONE_TOLERANCE) {
                isResetting = false; // Reset complete, back to normal
            }
            // If still resetting (getting close but not done yet), keep slow speed
            if (isResetting) {
                double direction = Math.signum(targetMechRot - currentPos);
                turretMotor.setControl(manualRequest.withOutput(direction * RESET_SPEED_FRACTION));
            } else {
                // Normal closed-loop position control at full speed
                turretMotor.setControl(positionRequest.withPosition(targetMechRot));
            }
        }

        ntDesiredAngle.set(targetMechRot * 360.0);
        ntTarget.set(targetMechRot);
    }

    /** Aim turret at a target (no velocity compensation — good when stationary). */
    public void aimAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetMechRot = calculateTargetMechanismRotations(robotPose, targetPose);
        double finalTarget = targetMechRot + aimOffsetRotations;
        double currentPos = turretMotor.getPosition().getValueAsDouble();

        // Small deadband: if we're already within ~2° of target, don't update.
        // This prevents constant micro-corrections from vision pose noise
        // that make the turret "hunt" back and forth when the robot is stationary.
        if (Math.abs(currentPos - finalTarget) < 0.006) { // 0.006 rot ≈ 2.2°
            return;
        }
        goToPosition(finalTarget);
    }

    /** Aim turret WITH velocity compensation — use when shooting on the move. */
    public void aimAtPoseCompensated(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        double targetMechRot = calculateTargetWithCompensation(robotPose, targetPose, fieldSpeeds);
        double finalTarget = targetMechRot + aimOffsetRotations;
        double currentPos = turretMotor.getPosition().getValueAsDouble();

        // Same deadband as aimAtPose — prevents hunting from vision noise
        if (Math.abs(currentPos - finalTarget) < 0.006) {
            return;
        }
        goToPosition(finalTarget);
    }

    /** Returns true when turret is aimed within ~1.5 degrees of target AND not resetting. */
    public boolean isAimedAtPose(Pose2d robotPose, Pose2d targetPose) {
        if (isResetting) return false; // Never "aimed" during a reset
        double targetMechRot = calculateTargetMechanismRotations(robotPose, targetPose);
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPos - targetMechRot) < 0.004; // 0.004 rot = ~1.5 degrees
    }

    @Override
    public void periodic() {
        double pos = turretMotor.getPosition().getValueAsDouble();
        ntPosition.set(pos);
        ntVelocity.set(turretMotor.getVelocity().getValueAsDouble());
        double target = turretMotor.getClosedLoopReference().getValueAsDouble();
        ntError.set((pos - target) * 360.0);
        ntResetting.set(isResetting);
        ntAimOffset.set(aimOffsetRotations * 360.0); // Show offset in degrees on dashboard
    }
}
