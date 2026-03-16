package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor = new TalonFX(
        Constants.ShootingConstants.TURRET_MOTOR, Constants.kCANivoreBus);

    // Soft limits in motor rotations
    private static final double MIN_ROTATIONS = -2.8;
    private static final double MAX_ROTATIONS = 2.8;

    // Degrees of turret travel per motor rotation — 11.515 rot = 360°
    private static final double DEGREES_PER_ROTATION = 360.0 / 11.515;

    // PID on turret position (rotations)
    private final PIDController positionPID = new PIDController(0.5, 0.0, 0.01);

    private final DoublePublisher ntPosition;
    private final DoublePublisher ntVelocity;
    private final DoublePublisher ntTarget;
    private final DoublePublisher ntDesiredAngle;

    public TurretSubsystem() {
        turretMotor.setPosition(0); // Zero at startup — facing driver station
        positionPID.setTolerance(0.05); // ~3 degrees

        NetworkTable calTable = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard/Calibration");
        ntPosition     = calTable.getDoubleTopic("Turret Pos (rot)").publish();
        ntVelocity     = calTable.getDoubleTopic("Turret Vel (rps)").publish();
        ntTarget       = calTable.getDoubleTopic("Turret Target (rot)").publish();
        ntDesiredAngle = calTable.getDoubleTopic("Turret Desired Deg").publish();
    }

    /** Manual rotate with soft limits enforced */
    public void rotate(double speed) {
        double pos = turretMotor.getPosition().getValueAsDouble();
        if (pos >= MAX_ROTATIONS && speed > 0) speed = 0;
        if (pos <= MIN_ROTATIONS && speed < 0) speed = 0;
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.stopMotor();
        positionPID.reset(); // Clear PID state when stopping to prevent windup
    }

    /**
     * Calculates the turret target in motor rotations for a given robot and target pose.
     * Uses WPILib geometry: Translation2d.minus().getAngle() for the field angle,
     * then Rotation2d.minus() for robot-relative and turret-offset angles.
     *
     * 0 rotations = facing rear of robot.
     * Positive rotations = CCW, negative = CW.
     */
    private double calculateTargetRotations(Pose2d robotPose, Pose2d targetPose) {
        // Vector from robot to target, then get its field angle
        Rotation2d fieldAngle = targetPose.getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle();

        // Robot-relative angle (subtract robot heading)
        Rotation2d relativeAngle = fieldAngle.minus(robotPose.getRotation());

        // Turret offset: 0 rot = rear of robot, so subtract 180°
        Rotation2d turretAngle = relativeAngle.minus(Rotation2d.k180deg);

        // Convert to motor rotations and clamp to soft limits
        double targetRotations = turretAngle.getDegrees() / DEGREES_PER_ROTATION;
        return MathUtil.clamp(targetRotations, MIN_ROTATIONS, MAX_ROTATIONS);
    }

    /** Pose-based turret aiming with PID control. */
    public void aimAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetRotations = calculateTargetRotations(robotPose, targetPose);

        double currentPos = turretMotor.getPosition().getValueAsDouble();
        double error = Math.abs(currentPos - targetRotations);

        // If close enough, don't move — prevents squealing from tiny corrections
        if (error < 0.1) { // ~3° of turret travel
            turretMotor.stopMotor();
            return;
        }

        double output = positionPID.calculate(currentPos, targetRotations);
        output = MathUtil.clamp(output, -0.3, 0.3);

        // Minimum output to overcome static friction
        if (output > 0 && output < 0.05) output = 0.05;
        if (output < 0 && output > -0.05) output = -0.05;

        // Enforce soft limits
        if (currentPos >= MAX_ROTATIONS && output > 0) output = 0;
        if (currentPos <= MIN_ROTATIONS && output < 0) output = 0;

        turretMotor.set(output);

        ntDesiredAngle.set(targetRotations * DEGREES_PER_ROTATION);
        ntTarget.set(targetRotations);
    }

    /** Returns true when turret is aimed close enough */
    public boolean isAimedAtPose(Pose2d robotPose, Pose2d targetPose) {
        double targetRotations = calculateTargetRotations(robotPose, targetPose);
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPos - targetRotations) < 0.05;
    }

    @Override
    public void periodic() {
        ntPosition.set(turretMotor.getPosition().getValueAsDouble());
        ntVelocity.set(turretMotor.getVelocity().getValueAsDouble());
    }
}
