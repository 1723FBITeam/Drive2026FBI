package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

private final TalonFX intakeLeftMotor =
    new TalonFX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR, "Carnivore");

private final TalonFX intakeRightMotor =
    new TalonFX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR, "Carnivore");

private final TalonFX intakeLeftActivator =
    new TalonFX(Constants.IntakeConstants.DEPLOY_LEFT_MOTOR, "Carnivore");

private final TalonFX intakeRightActivator =
    new TalonFX(Constants.IntakeConstants.DEPLOY_RIGHT_MOTOR, "Carnivore");    

private SlewRateLimiter rateLimiter = new SlewRateLimiter(3);

// PID Constants
private static final double kP_UP = 0.025; // Proportional gain
private static final double kP_DOWN = 0.025; // Proportional gain
private static final double kI = 0.0; // Integral gain
private static final double kD = 0.0; // Derivative gain

private PIDController pid;

private Map<Integer, Double> targetPositions = Map.of(
            0, 1.23,
            1, 1.23);

     private int currentPositionKey = 0;

public IntakeSubsystem() {

MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
intakeLeftMotor.getConfigurator().apply(leftConfigs);
intakeLeftActivator.getConfigurator().apply(leftConfigs);

MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
intakeRightMotor.getConfigurator().apply(rightConfigs);
intakeRightActivator.getConfigurator().apply(rightConfigs);

 pid = new PIDController(kP_UP, kI, kD);
    intakeLeftActivator.setControl(new StaticBrake());
    intakeLeftActivator.setPosition(0);
    intakeRightActivator.setControl(new StaticBrake());
    intakeRightActivator.setPosition(0);
}
public void runIntake(double speed) {
    intakeLeftMotor.set(speed);
    intakeRightMotor.set(speed);
}

public void stopIntake() {
    intakeLeftMotor.stopMotor();
    intakeRightMotor.stopMotor();
}

public void setIntakeUp() {
    pid.setP(kP_UP);
        currentPositionKey = targetPositions.keySet().stream().max(Integer::compareTo).orElse(currentPositionKey);
}

public void setIntakeDown() {
    pid.setP(kP_DOWN);
        currentPositionKey = targetPositions.keySet().stream().min(Integer::compareTo).orElse(currentPositionKey);    
}

@Override
public void periodic() {
    // 1. Get the average position (or just pick one if they are linked)
    double leftPos = intakeLeftActivator.getPosition().getValueAsDouble();
    double rightPos = intakeRightActivator.getPosition().getValueAsDouble();
    double avgPosition = (leftPos + rightPos) / 2.0;

    // 2. Calculate PID based on where the mechanism actually is
    double target = targetPositions.get(currentPositionKey);
    double pidOutput = pid.calculate(avgPosition, target);

    // 3. Apply Rate Limiting ONCE to the output
    double limitedOutput = rateLimiter.calculate(pidOutput);

    // 4. Set both motors to the same limited value
    intakeLeftActivator.set(limitedOutput);
    intakeRightActivator.set(limitedOutput);

    // 5. Log your data
    SmartDashboard.putNumber("Intake/Avg Position", avgPosition);
    SmartDashboard.putNumber("Intake/Target", target);
    SmartDashboard.putNumber("Intake/Output", limitedOutput);
}
}