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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * TurretSubsystem — rotates the shooter left/right to aim at the hub.
 *
 * HOW IT WORKS:
 * The turret sits on top of the robot and can spin ~360 degrees.
 * It uses CLOSED-LOOP POSITION CONTROL, meaning we tell the motor
 * "go to this angle" and the motor PID loop (running at 1000Hz
 * on the TalonFX) handles getting there smoothly.
 *
 * COORDINATE SYSTEM:
 * - 0 position = turret facing the REAR of the robot
 * - Positive rotation = counterclockwise (viewed from above)
 * - Positions are in "mechanism rotations" (1.0 = full 360 turn)
 *
 * GEAR RATIO: 11.515 motor rotations = 1 turret rotation
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(
        Constants.ShootingConstants.TURRET_MOTOR, Constants.kCANivoreBus);

    // 11.515 motor rotations = 1 full turret rotation (360 degrees)
    private static final double TURRET_GEAR_RATIO = 11.515;

    // Soft limits: how far the turret can rotate each direction
    // +/-0.5 mechanism rotations = +/-180 degrees = full 360 range
    private static final double MIN_MECHANISM_ROTATIONS = -0.5;
    private static final double MAX_MECHANISM_ROTATIONS = 0.5;

    // PositionVoltage = "go to this position and hold it"
    private final PositionVoltage positionRequest = new PositionVoltage(0)
        .withEnableFOC(false);

    // DutyCycleOut = simple "spin at X% power" for manual control
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);

    // Dashboard telemetry publishers
    private final DoublePublisher ntPosition;
    private final DoublePublisher ntVelocity;
    private final DoublePublisher ntTarget;
    private final DoublePublisher ntDesiredAngle;
    private final DoublePublisher ntError;

    public TurretSubsystem() {
        // PID + Feedforward gains for position control
        // KP: How aggressively to correct error (higher = faster, may oscillate)
        // KI: Corrects steady-state error over time (usually 0)
        // KD: Dampens oscillation (small values)
        // KS: Voltage to overcome static friction
        var slot0 = new Slot0Configs()
            .withKP(20.0)
            .withKI(0.0)
            .withKD(0.1)
            .withKS(0.4)
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

        // Deadband: output zero when duty cycle is below 2% (prevents buzzing)
        var motorOutput = new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.02);
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
    }

    /** Manually rotate the turret (trigger buttons). Soft limits still enforced. */
    public void rotate(double speed) {
        turretMotor.setControl(manualRequest.withOutput(speed));
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /**
     * Calculate where the turret needs to point to aim at a target.
     * 1. Find angle from robot to target on the field
     * 2. Subtract robot heading to get robot-relative angle
     * 3. Subtract 180 because turret zero = rear of robot
     * 4. Convert degrees to mechanism rotations (divide by 360)
     * 5. Clamp to soft limits
     */
    private double calculateTargetMechanismRotations(Pose2d robotPose, Pose2d targetPose) {
        Rotation2d fieldAngle = targetPose.getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle();

        Rotation2d relativeAngle = fieldAngle.minus(robotPose.getRotation());
        Rotation2d turretAngle = relativeAngle.minus(Rotation2d.k180deg);

        double targetMechRot = turretAngle.getDegrees() / 360.0;
        return MathUtil.clamp(targetMechRot, MIN_MECHANISM_ROTATIONS, MAX_MECHANISM_ROTATIONS);
    }

    /**
     * Same as above but predicts where the robot WILL BE in ~0.2 seconds.
     * This compensates for note travel time when shooting on the move.
     */
    private double calculateTargetWithCompensation(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        double latencySeconds = 0.2;
        Translation2d futurePosition = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * latencySeconds,
                fieldSpeeds.vyMetersPerSecond * latencySeconds));

        Pose2d futurePose = new Pose2d(futurePosition, robotPose.getRotation());
        return calculateTargetMechanismRotations(futurePose, targetPose);
    }

    /** Aim turret at a target (no velocity compensation — good when stationary). */
    public void aimAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetMechRot = calculateTargetMechanismRotations(robotPose, targetPose);
        turretMotor.setControl(positionRequest.withPosition(targetMechRot));

        ntDesiredAngle.set(targetMechRot * 360.0);
        ntTarget.set(targetMechRot);
    }

    /** Aim turret WITH velocity compensation — use when shooting on the move. */
    public void aimAtPoseCompensated(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        double targetMechRot = calculateTargetWithCompensation(robotPose, targetPose, fieldSpeeds);
        turretMotor.setControl(positionRequest.withPosition(targetMechRot));

        ntDesiredAngle.set(targetMechRot * 360.0);
        ntTarget.set(targetMechRot);
    }

    /** Returns true when turret is aimed within ~1.5 degrees of target. */
    public boolean isAimedAtPose(Pose2d robotPose, Pose2d targetPose) {
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
    }
}
