package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeLeftMotor =
        new TalonFX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR, Constants.kCANivoreBus);
    private final TalonFX intakeRightMotor =
        new TalonFX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR, Constants.kCANivoreBus);
    private final TalonFXS intakeLeftActivator =
        new TalonFXS(Constants.IntakeConstants.DEPLOY_LEFT_MOTOR, Constants.kCANivoreBus);
    private final TalonFXS intakeRightActivator =
        new TalonFXS(Constants.IntakeConstants.DEPLOY_RIGHT_MOTOR, Constants.kCANivoreBus);

    // Deploy speed — simple fixed speed for now until we find positions
    private static final double DEPLOY_SPEED = 0.25;

    private double deployOutput = 0.0; // current deploy motor output

    // Shuffleboard telemetry
    private final DoublePublisher ntLeftPos;
    private final DoublePublisher ntRightPos;
    private final DoublePublisher ntAvgPos;
    private final DoublePublisher ntOutput;

    public IntakeSubsystem() {
        // Configure TalonFXS deploy motors — must set motor type to Minion
        TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();
        fxsConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        fxsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeLeftActivator.getConfigurator().apply(fxsConfig);
        intakeRightActivator.getConfigurator().apply(fxsConfig);
        MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfigs.NeutralMode = NeutralModeValue.Brake;
        intakeLeftMotor.getConfigurator().apply(leftConfigs);

        MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.NeutralMode = NeutralModeValue.Brake;
        intakeRightMotor.getConfigurator().apply(rightConfigs);

        // Zero encoders on startup
        intakeLeftActivator.setPosition(0);
        intakeRightActivator.setPosition(0);

        NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration");
        ntLeftPos  = calTable.getDoubleTopic("Intake L Pos").publish();
        ntRightPos = calTable.getDoubleTopic("Intake R Pos").publish();
        ntAvgPos   = calTable.getDoubleTopic("Intake Avg Pos").publish();
        ntOutput   = calTable.getDoubleTopic("Intake Output").publish();
    }

    public void runIntake(double speed) {
        intakeLeftMotor.set(speed);
        intakeRightMotor.set(speed);
    }

    public void stopIntake() {
        intakeLeftMotor.stopMotor();
        intakeRightMotor.stopMotor();
    }

    /** Run deploy motors forward (out) */
    public void deployOut() {
        intakeLeftActivator.set(DEPLOY_SPEED);
        intakeRightActivator.set(DEPLOY_SPEED);
    }

    /** Run deploy motors reverse (in) */
    public void deployIn() {
        intakeLeftActivator.set(-DEPLOY_SPEED);
        intakeRightActivator.set(-DEPLOY_SPEED);
    }

    /** Stop deploy motors */
    public void stopDeploy() {
        intakeLeftActivator.stopMotor();
        intakeRightActivator.stopMotor();
    }

    @Override
    public void periodic() {
        double leftPos = intakeLeftActivator.getPosition().getValueAsDouble();
        double rightPos = intakeRightActivator.getPosition().getValueAsDouble();

        ntLeftPos.set(leftPos);
        ntRightPos.set(rightPos);
        ntAvgPos.set((leftPos + rightPos) / 2.0);
        ntOutput.set(0);
    }
}
