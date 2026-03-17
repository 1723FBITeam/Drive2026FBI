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

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(
        Constants.ShootingConstants.TURRET_MOTOR, Constants.kCANivoreBus);

    // Gear ratio: 11.515 motor rotations = 360° of turret travel
    // SensorToMechanismRatio means the motor position is converted to mechanism rotations
    // 1 mechanism rotation = 360° of turret = 11.515 motor rotations
    private static final double TURRET_GEAR_RATIO = 11.515;

    // Soft limits in MECHANISM rotations (turret rotations, not motor rotations)
    // ±0.5 mechanism rotations = ±180° of turret travel (360° total)
    private static final double MIN_MECHANISM_ROTATIONS = -0.5;
    private static final double MAX_MECHANISM_ROTATIONS = 0.5;

    // Onboard closed-loop position control — runs at 1kHz on the TalonFX
    // Deadband: motor outputs zero when error is within this range (prevents buzzing)
    // 0.002 mechanism rotations ≈ 0.72° of turret travel
    private final PositionVoltage positionRequest = new PositionVoltage(0)
        .withEnableFOC(false);

    // Manual control for bumper overrides
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);

    private final DoublePublisher ntPosition;
    private final DoublePublisher ntVelocity;
    private final DoublePublisher ntTarget;
    private final DoublePublisher ntDesiredAngle;
    private final DoublePublisher ntError;

    public TurretSubsystem() {
        // Configure onboard PID + feedforward for position control
        // These run at 1kHz on the TalonFX — 20x faster than RIO-side PID
        var slot0 = new Slot0Configs()
            .withKP(20.0)   // Proportional — tune: increase until oscillation, then back off
            .withKI(0.0)    // Integral — leave at 0 unless you have steady-state error
            .withKD(0.1)    // Derivative — low to avoid screaming
            .withKS(0.4)    // Static friction feedforward — output to just barely start moving
            .withKV(0.0)    // Velocity feedforward — not needed for position control
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        // Tell the TalonFX about the gear ratio so positions are in mechanism rotations
        var feedback = new FeedbackConfigs()
            .withSensorToMechanismRatio(TURRET_GEAR_RATIO);

        // Hardware soft limits — enforced on the motor controller, no code needed
        var softLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(MAX_MECHANISM_ROTATIONS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(MIN_MECHANISM_ROTATIONS);

        turretMotor.getConfigurator().apply(slot0);
        turretMotor.getConfigurator().apply(feedback);
        turretMotor.getConfigurator().apply(softLimits);

        // Deadband — motor outputs zero when duty cycle is below this threshold
        // Prevents buzzing when close to target
        var motorOutput = new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.02);
        turretMotor.getConfigurator().apply(motorOutput);

        // Zero at startup — turret facing rear of robot (toward driver station)
        turretMotor.setPosition(0);

        NetworkTable calTable = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard/Calibration");
        ntPosition     = calTable.getDoubleTopic("Turret Pos (mech rot)").publish();
        ntVelocity     = calTable.getDoubleTopic("Turret Vel (mech rps)").publish();
        ntTarget       = calTable.getDoubleTopic("Turret Target (mech rot)").publish();
        ntDesiredAngle = calTable.getDoubleTopic("Turret Desired Deg").publish();
        ntError        = calTable.getDoubleTopic("Turret Error Deg").publish();
    }

    /** Manual rotate — soft limits are enforced by hardware config */
    public void rotate(double speed) {
        turretMotor.setControl(manualRequest.withOutput(speed));
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /**
     * Calculates the turret target in MECHANISM rotations for a given robot and target pose.
     * 0 mechanism rotations = facing rear of robot.
     * Positive = CCW, negative = CW.
     */
    private double calculateTargetMechanismRotations(Pose2d robotPose, Pose2d targetPose) {
        // Vector from robot to target, then get its field angle
        Rotation2d fieldAngle = targetPose.getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle();

        // Robot-relative angle (subtract robot heading)
        Rotation2d relativeAngle = fieldAngle.minus(robotPose.getRotation());

        // Turret offset: 0 rot = rear of robot, so subtract 180°
        Rotation2d turretAngle = relativeAngle.minus(Rotation2d.k180deg);

        // Convert degrees to mechanism rotations (1 mechanism rotation = 360°)
        double targetMechRot = turretAngle.getDegrees() / 360.0;
        return MathUtil.clamp(targetMechRot, MIN_MECHANISM_ROTATIONS, MAX_MECHANISM_ROTATIONS);
    }

    /**
     * Calculates target with robot velocity compensation.
     * Predicts where the robot will be after a latency period and aims there instead.
     */
    private double calculateTargetWithCompensation(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        // Predict robot position after shot travel time (~0.2s latency compensation)
        double latencySeconds = 0.2;
        Translation2d futurePosition = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * latencySeconds,
                fieldSpeeds.vyMetersPerSecond * latencySeconds));

        // Create a future pose with predicted position but current heading
        Pose2d futurePose = new Pose2d(futurePosition, robotPose.getRotation());
        return calculateTargetMechanismRotations(futurePose, targetPose);
    }

    /**
     * Pose-based turret aiming using onboard TalonFX closed-loop (1kHz).
     * No velocity compensation — use aimAtPoseCompensated() when shooting on the move.
     */
    public void aimAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetMechRot = calculateTargetMechanismRotations(robotPose, targetPose);
        turretMotor.setControl(positionRequest.withPosition(targetMechRot));

        ntDesiredAngle.set(targetMechRot * 360.0);
        ntTarget.set(targetMechRot);
    }

    /**
     * Pose-based turret aiming WITH robot velocity compensation.
     * Use this during shooting sequences for shoot-on-the-move accuracy.
     */
    public void aimAtPoseCompensated(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds fieldSpeeds) {
        double targetMechRot = calculateTargetWithCompensation(robotPose, targetPose, fieldSpeeds);
        turretMotor.setControl(positionRequest.withPosition(targetMechRot));

        ntDesiredAngle.set(targetMechRot * 360.0);
        ntTarget.set(targetMechRot);
    }

    /** Returns true when turret is aimed within ~1.5° of target */
    public boolean isAimedAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetMechRot = calculateTargetMechanismRotations(robotPose, targetPose);
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        // 0.004 mechanism rotations ≈ 1.5° of turret travel
        return Math.abs(currentPos - targetMechRot) < 0.004;
    }

    @Override
    public void periodic() {
        double pos = turretMotor.getPosition().getValueAsDouble();
        ntPosition.set(pos);
        ntVelocity.set(turretMotor.getVelocity().getValueAsDouble());
        double target = turretMotor.getClosedLoopReference().getValueAsDouble();
        ntError.set((pos - target) * 360.0); // Error in degrees
    }
}
