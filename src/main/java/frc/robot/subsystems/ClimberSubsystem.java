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
 * Climber subsystem - controls climb motors, elevator, and servos for endgame
 */
public class ClimberSubsystem extends SubsystemBase {

    // Climb/Twist motors
    private final TalonFX leftClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_LEFT_MOTOR);
    private final TalonFX rightClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_RIGHT_MOTOR);
    
    // Elevator motor
    private final TalonFX elevatorMotor = new TalonFX(Constants.ClimberConstants.ELEVATOR_MOTOR);
    
    // Servos for climb hooks/locks
    private final Servo leftServo = new Servo(Constants.ClimberConstants.CLIMB_SERVO_LEFT);
    private final Servo rightServo = new Servo(Constants.ClimberConstants.CLIMB_SERVO_RIGHT);

    // Servo position presets
    private static final double SERVO_RETRACTED = 0.0;
    private static final double SERVO_EXTENDED = 1.0;
    private static final double SERVO_MID = 0.5;

    public ClimberSubsystem() {
        // Configure climb motors - right side inverted
        MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightClimbMotor.getConfigurator().apply(rightConfigs);

        MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftClimbMotor.getConfigurator().apply(leftConfigs);

        // Configure elevator
        MotorOutputConfigs elevatorConfigs = new MotorOutputConfigs();
        elevatorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotor.getConfigurator().apply(elevatorConfigs);
    }

    // ========== CLIMB MOTORS ==========
    
    /** Run both climb motors together (-1.0 to 1.0) */
    public void runClimb(double speed) {
        leftClimbMotor.set(speed);
        rightClimbMotor.set(speed);
    }

    /** Run left climb motor independently */
    public void runLeftClimb(double speed) {
        leftClimbMotor.set(speed);
    }

    /** Run right climb motor independently */
    public void runRightClimb(double speed) {
        rightClimbMotor.set(speed);
    }

    public void stopClimb() {
        leftClimbMotor.stopMotor();
        rightClimbMotor.stopMotor();
    }

    // ========== ELEVATOR ==========
    
    /** Run elevator up */
    public void elevatorUp(double speed) {
        elevatorMotor.set(Math.abs(speed)); // Ensure positive
    }

    /** Run elevator down */
    public void elevatorDown(double speed) {
        elevatorMotor.set(-Math.abs(speed)); // Ensure negative
    }

    /** Run elevator motor with manual control (-1.0 to 1.0) */
    public void runElevator(double speed) {
        elevatorMotor.set(speed);
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    // ========== SERVOS ==========
    
    /** Set servo position (0.0 to 1.0) */
    public void setServoPosition(double position) {
        position = MathUtil.clamp(position, 0.0, 1.0);
        leftServo.set(position);
        rightServo.set(position);
    }

    /** Preset positions for servos */
    public void setServoRetracted() {
        setServoPosition(SERVO_RETRACTED);
        System.out.println("Servos: Retracted");
    }

    public void setServoMid() {
        setServoPosition(SERVO_MID);
        System.out.println("Servos: Mid Position");
    }

    public void setServoExtended() {
        setServoPosition(SERVO_EXTENDED);
        System.out.println("Servos: Extended");
    }

    // ========== COMBINED OPERATIONS ==========
    
    /** Stop all climber components */
    public void stopAll() {
        stopClimb();
        stopElevator();
    }

    @Override
    public void periodic() {
        // Add telemetry for debugging
        SmartDashboard.putNumber("Left Climb Position", leftClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Climb Position", rightClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
    }
}
