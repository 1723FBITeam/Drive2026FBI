package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

/**
 * ClimberSubsystem — handles endgame climbing.
 * NOTE: Not wired yet! This code is ready for when the hardware is built.
 *
 * The climber has three parts:
 *   1. CLIMB MOTORS (left + right) — pull the robot up onto the chain/bar
 *   2. ELEVATOR MOTOR — extends/retracts a lift mechanism
 *   3. SERVOS (left + right) — lock/unlock climb hooks
 *
 * The climb motors are on the RIO's built-in CAN bus (not the CANivore)
 * because they don't need the faster update rate.
 */
public class ClimberSubsystem extends SubsystemBase {

    // Climb motors — pull the robot up
    private final TalonFX leftClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_LEFT_MOTOR);
    private final TalonFX rightClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_RIGHT_MOTOR);

    // Elevator motor — extends the climbing arm up to reach the bar
    private final TalonFX elevatorMotor = new TalonFX(Constants.ClimberConstants.ELEVATOR_MOTOR);

    // Servos — lock/unlock the climb hooks so we don't fall
    private final Servo leftServo = new Servo(Constants.ClimberConstants.CLIMB_SERVO_LEFT);
    private final Servo rightServo = new Servo(Constants.ClimberConstants.CLIMB_SERVO_RIGHT);

    // Servo preset positions
    private static final double SERVO_RETRACTED = 0.0; // Hooks locked/retracted
    private static final double SERVO_EXTENDED = 1.0;   // Hooks open/extended
    private static final double SERVO_MID = 0.5;        // Halfway position

    public ClimberSubsystem() {
        // Right climb motor is inverted so both motors pull in the same direction
        MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightClimbMotor.getConfigurator().apply(rightConfigs);

        MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftClimbMotor.getConfigurator().apply(leftConfigs);

        MotorOutputConfigs elevatorConfigs = new MotorOutputConfigs();
        elevatorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotor.getConfigurator().apply(elevatorConfigs);
    }

    // ==================== CLIMB MOTORS ====================

    /** Run both climb motors together. @param speed -1.0 to 1.0 */
    public void runClimb(double speed) {
        leftClimbMotor.set(speed);
        rightClimbMotor.set(speed);
    }

    /** Run left climb motor independently (for leveling) */
    public void runLeftClimb(double speed) {
        leftClimbMotor.set(speed);
    }

    /** Run right climb motor independently (for leveling) */
    public void runRightClimb(double speed) {
        rightClimbMotor.set(speed);
    }

    public void stopClimb() {
        leftClimbMotor.stopMotor();
        rightClimbMotor.stopMotor();
    }

    // ==================== ELEVATOR ====================

    /** Run elevator up (positive direction) */
    public void elevatorUp(double speed) {
        elevatorMotor.set(Math.abs(speed));
    }

    /** Run elevator down (negative direction) */
    public void elevatorDown(double speed) {
        elevatorMotor.set(-Math.abs(speed));
    }

    /** Run elevator with manual control. @param speed -1.0 to 1.0 */
    public void runElevator(double speed) {
        elevatorMotor.set(speed);
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    // ==================== SERVOS ====================

    /** Set both servos to a position (0.0 to 1.0) */
    public void setServoPosition(double position) {
        position = MathUtil.clamp(position, 0.0, 1.0);
        leftServo.set(position);
        rightServo.set(position);
    }

    /** Retract hooks (locked position) */
    public void setServoRetracted() {
        setServoPosition(SERVO_RETRACTED);
        System.out.println("Servos: Retracted");
    }

    /** Move hooks to middle position */
    public void setServoMid() {
        setServoPosition(SERVO_MID);
        System.out.println("Servos: Mid Position");
    }

    /** Extend hooks (open position) */
    public void setServoExtended() {
        setServoPosition(SERVO_EXTENDED);
        System.out.println("Servos: Extended");
    }

    // ==================== COMBINED ====================

    /** Stop all climber motors (servos hold their last position) */
    public void stopAll() {
        stopClimb();
        stopElevator();
    }

    /** Called every 20ms — publishes climb positions to dashboard */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climb Position", leftClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Climb Position", rightClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
    }
}
