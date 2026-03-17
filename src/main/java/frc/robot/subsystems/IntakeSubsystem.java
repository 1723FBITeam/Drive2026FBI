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

/**
 * IntakeSubsystem — picks up game pieces (notes) from the ground.
 *
 * The intake has two parts:
 *   1. ROLLERS (TalonFX motors) — spin to grab and pull in notes
 *   2. DEPLOY mechanism (TalonFXS motors) — swings the intake out from
 *      the robot frame so the rollers can reach the ground
 *
 * The deploy motors use TalonFXS (not TalonFX) because they drive smaller
 * "Minion" motors connected via JST cable.
 *
 * Deploy motors are set to BRAKE mode so the intake holds its position
 * when not actively moving. Rollers also brake to grip notes.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Roller motors — spin to grab notes off the ground
    private final TalonFX intakeLeftMotor =
        new TalonFX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR, Constants.kCANivoreBus);
    private final TalonFX intakeRightMotor =
        new TalonFX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR, Constants.kCANivoreBus);

    // Deploy motors — swing the intake out/in (TalonFXS drives smaller Minion motors)
    private final TalonFXS intakeLeftActivator =
        new TalonFXS(Constants.IntakeConstants.DEPLOY_LEFT_MOTOR, Constants.kCANivoreBus);
    private final TalonFXS intakeRightActivator =
        new TalonFXS(Constants.IntakeConstants.DEPLOY_RIGHT_MOTOR, Constants.kCANivoreBus);

    // Deploy speed — how fast the deploy motors run (0.15 = 15% power)
    private static final double DEPLOY_SPEED = 0.15;

    private double deployOutput = 0.0;

    // Dashboard telemetry — shows deploy motor positions for debugging
    private final DoublePublisher ntLeftPos;
    private final DoublePublisher ntRightPos;
    private final DoublePublisher ntAvgPos;
    private final DoublePublisher ntOutput;

    public IntakeSubsystem() {
        // Configure TalonFXS deploy motors
        // MotorArrangement must be set to Minion_JST because these drive
        // smaller motors connected via JST cable (not the built-in motor)
        TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();
        fxsConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        fxsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Hold position when stopped
        intakeLeftActivator.getConfigurator().apply(fxsConfig);
        intakeRightActivator.getConfigurator().apply(fxsConfig);

        // Configure roller motors — left and right spin opposite directions
        // so they both pull notes inward toward the center
        MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfigs.NeutralMode = NeutralModeValue.Brake;
        intakeLeftMotor.getConfigurator().apply(leftConfigs);

        MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.NeutralMode = NeutralModeValue.Brake;
        intakeRightMotor.getConfigurator().apply(rightConfigs);

        // Zero deploy encoders so we can track how far the intake has deployed
        intakeLeftActivator.setPosition(0);
        intakeRightActivator.setPosition(0);

        // Set up dashboard telemetry
        NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration");
        ntLeftPos  = calTable.getDoubleTopic("Intake L Pos").publish();
        ntRightPos = calTable.getDoubleTopic("Intake R Pos").publish();
        ntAvgPos   = calTable.getDoubleTopic("Intake Avg Pos").publish();
        ntOutput   = calTable.getDoubleTopic("Intake Output").publish();
    }

    /**
     * Run the intake rollers to grab notes.
     * @param speed Duty cycle from -1.0 to 1.0 (positive = intake, negative = spit out)
     */
    public void runIntake(double speed) {
        intakeLeftMotor.set(speed);
        intakeRightMotor.set(speed);
    }

    public void stopIntake() {
        intakeLeftMotor.stopMotor();
        intakeRightMotor.stopMotor();
    }

    /** Swing the intake outward (away from robot) to reach the ground */
    public void deployOut() {
        intakeLeftActivator.set(DEPLOY_SPEED);
        intakeRightActivator.set(DEPLOY_SPEED);
    }

    /** Swing the intake inward (back into robot frame) */
    public void deployIn() {
        intakeLeftActivator.set(-DEPLOY_SPEED);
        intakeRightActivator.set(-DEPLOY_SPEED);
    }

    /** Stop the deploy motors (intake holds position due to brake mode) */
    public void stopDeploy() {
        intakeLeftActivator.stopMotor();
        intakeRightActivator.stopMotor();
    }

    /** Called every 20ms — publishes deploy position data to dashboard */
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
