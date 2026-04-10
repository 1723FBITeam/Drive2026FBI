package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
// SmartDashboard removed for deploy limits (we let gravity settle)
import frc.robot.Constants;

/**
 * IntakeSubsystem — picks up game pieces (notes) from the ground.
 *
 * The intake has two parts:
 * 1. ROLLERS (TalonFX motors) — spin to grab and pull in notes
 * 2. DEPLOY mechanism (TalonFXS motors) — swings the intake out from
 * the robot frame so the rollers can reach the ground
 *
 * The deploy motors use TalonFXS (not TalonFX) because they drive smaller
 * "Minion" motors connected via JST cable.
 *
 * Deploy motors are set to BRAKE mode so the intake holds its position
 * when not actively moving. Rollers also brake to grip notes.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Roller motors — spin to grab notes off the ground
    private final TalonFX intakeLeftMotor = new TalonFX(Constants.IntakeConstants.INTAKE_LEFT_MOTOR,
            Constants.kCANivoreBus);
    private final TalonFX intakeRightMotor = new TalonFX(Constants.IntakeConstants.INTAKE_RIGHT_MOTOR,
            Constants.kCANivoreBus);

    // Deploy motors — swing the intake out/in (TalonFXS drives smaller Minion
    // motors)
    private final TalonFXS intakeLeftActivator = new TalonFXS(Constants.IntakeConstants.DEPLOY_LEFT_MOTOR,
            Constants.kCANivoreBus);
    private final TalonFXS intakeRightActivator = new TalonFXS(Constants.IntakeConstants.DEPLOY_RIGHT_MOTOR,
            Constants.kCANivoreBus);

    // Deploy speed — how fast the deploy motors run (0.15 = 15% power)
    private static final double DEPLOY_SPEED = 0.15;

    // We intentionally allow the deploy mechanism to coast (no software limits)
    // so gravity can settle the intake both in and out.

    // Dashboard telemetry — shows deploy motor positions for debugging
    private final DoublePublisher ntLeftPos;
    private final DoublePublisher ntRightPos;
    private final DoublePublisher ntAvgPos;

    private boolean isExtended = false;

    public boolean isExtended() {
        return isExtended;
    }

    private boolean isRunning = false;

    public boolean isRunning() {
        return isRunning;
    }

    public double getAverageDeployPosition() {
        return (intakeLeftActivator.getPosition().getValueAsDouble() +
                intakeRightActivator.getPosition().getValueAsDouble()) / 2.0;
    }

    private final PositionVoltage m_positionControl = new PositionVoltage(0);

    public IntakeSubsystem() {

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

        // ===== CURRENT LIMITS =====
        var rollerCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true);
        intakeLeftMotor.getConfigurator().apply(rollerCurrentLimits);
        intakeRightMotor.getConfigurator().apply(rollerCurrentLimits);

        // Set up dashboard telemetry
        NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Calibration");
        ntLeftPos = calTable.getDoubleTopic("Intake L Pos").publish();
        ntRightPos = calTable.getDoubleTopic("Intake R Pos").publish();
        ntAvgPos = calTable.getDoubleTopic("Intake Avg Pos").publish();

        // Configure TalonFXS deploy motors
        // MotorArrangement must be set to Minion_JST because these drive
        // smaller motors connected via JST cable (not the built-in motor)
        TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();
        fxsConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        // Use COAST so gravity can settle the deploy when motors are stopped
        fxsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // PID gains for position hold
        fxsConfig.Slot0.kP = 10.0;
        fxsConfig.Slot0.kI = 0.0;
        fxsConfig.Slot0.kD = 0.1;
        // Apply once with all settings
        intakeLeftActivator.getConfigurator().apply(fxsConfig);
        intakeRightActivator.getConfigurator().apply(fxsConfig);
    }

    /**
     * Run the intake rollers to grab notes.
     * 
     * @param speed Duty cycle from -1.0 to 1.0 (positive = intake, negative = spit
     *              out)
     */
    public void runIntake(double speed) {
        intakeLeftMotor.set(speed);
        intakeRightMotor.set(speed);
        isRunning = true;
    }

    public void stopIntake() {
        intakeLeftMotor.stopMotor();
        intakeRightMotor.stopMotor();
        isRunning = false;
    }

    /** Swing the intake outward (away from robot) to reach the ground */
    public void deployOut() {
        // Drive deploy motors outward; no software limits so gravity can settle
        // changeConfigsBack();
        intakeLeftActivator.set(DEPLOY_SPEED);
        intakeRightActivator.set(DEPLOY_SPEED);
        isExtended = true;
    }

    /** Swing the intake inward (back into robot frame) */
    public void deployIn() {
        // Drive deploy motors inward; no software limits so gravity can settle
        intakeLeftActivator.set(-DEPLOY_SPEED);
        intakeRightActivator.set(-DEPLOY_SPEED);
        isExtended = false;
    }

    /** Stop the deploy motors (intake holds position due to brake mode) */
    public void stopDeploy() {
        intakeLeftActivator.stopMotor();
        intakeRightActivator.stopMotor();

    }

    // Returns a command that jostles the intake to unstick balls.
    // Only pulses the deploy mechanism in/out — rollers keep running
    // so the ball stays engaged. Safe to call repeatedly.

    // ===== DEPLOY CONSTANTS =====
    // Position threshold for jostle: the deploy encoder position where the intake
    // has retracted enough to "jostle" a stuck ball. Measured from the fully-out
    // position (0.0) — 0.069 rotations ≈ 25° of deploy arm travel inward.
    private static final double JOSTLE_RETRACT_POSITION = 0.069;

    // Maximum time (seconds) to run deploy motors before auto-stopping.
    // Prevents motors from running forever if the mechanism jams.
    private static final double DEPLOY_TIMEOUT_SECONDS = 0.8;

    public Command jostleCommand() {
        return new SequentialCommandGroup(
                // 1. Move the intake in
                new InstantCommand(() -> deployIn(), this),

                // 2. Wait until retracted enough or timeout (whichever comes first)
                Commands.waitUntil(() -> getAverageDeployPosition() >= JOSTLE_RETRACT_POSITION)
                    .withTimeout(DEPLOY_TIMEOUT_SECONDS),

                // 3. Stop the motors and ensure they are in Coast mode so it falls
                new InstantCommand(() -> stopDeploy(), this),

                // 4. Wait a moment for the intake to drop
                new WaitCommand(0.5)
        );
    }

    // public void changeConfigs () {
    // MotorOutputConfigs brakeConfigsShoot = new MotorOutputConfigs();
    // brakeConfigsShoot.NeutralMode = NeutralModeValue.Brake;
    // intakeLeftActivator.getConfigurator().apply(brakeConfigsShoot);
    // intakeRightActivator.getConfigurator().apply(brakeConfigsShoot); }

    // public void changeConfigsBack () {
    // MotorOutputConfigs brakeConfigsShootBack = new MotorOutputConfigs();
    // brakeConfigsShootBack.NeutralMode = NeutralModeValue.Coast;
    // intakeLeftActivator.getConfigurator().apply(brakeConfigsShootBack);
    // intakeRightActivator.getConfigurator().apply(brakeConfigsShootBack); }

    /**
     * * A command that safely stops everything and tucks the intake in.
     * We make this a public method so we can reuse it in the toggle and
     * auto-retract.
     */
    public Command retractCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(this::stopIntake, this),
                new InstantCommand(this::deployIn, this),
                new WaitCommand(DEPLOY_TIMEOUT_SECONDS),
                new InstantCommand(this::stopDeploy, this)).withName("IntakeRetractCommand");
    }

    /**
     * Checks robot velocity and auto-retracts if moving fast in any direction.
     * 
     * @param currentXVelocity Velocity in meters per second (from drivetrain state)
     */
    public void handleAutoRetract(double currentXVelocity) {
        // If running, extended, and moving faster than 0.5 m/s in any direction
        if (this.isRunning && this.isExtended && Math.abs(currentXVelocity) > 0.5) {
            this.retractCommand().schedule();
        }
    }

    /**
     * Checks actual robot movement from Odometry and auto-retracts if necessary.
     * Uses total translational speed (not just backward) so strafing and
     * diagonal movement also trigger retraction.
     * 
     * @param speeds The current ChassisSpeeds from the Pose Estimator/Drivetrain
     *               state.
     */
    public void handleAutoRetract(ChassisSpeeds speeds) {
        double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        if (this.isExtended && this.isRunning && totalSpeed > 2.0) {
            this.retractCommand().schedule();
        }
    }

    // 3. Create the method to hold position
    public void holdPosition(double position) {
        // We apply the position request to both motors
        intakeLeftActivator.setControl(m_positionControl.withPosition(position));
        intakeRightActivator.setControl(m_positionControl.withPosition(position));

        // Optional: Switch to Brake mode for a stronger hold
        setNeutralMode(NeutralModeValue.Brake);
    }

    // Helper to make the container code cleaner
    public void setNeutralMode(NeutralModeValue mode) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = mode;
        intakeLeftActivator.getConfigurator().apply(config);
        intakeRightActivator.getConfigurator().apply(config);
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        telemetryCounter++;
        if (telemetryCounter % 5 != 0)
            return; // ~10Hz

        double leftPos = intakeLeftActivator.getPosition().getValueAsDouble();
        double rightPos = intakeRightActivator.getPosition().getValueAsDouble();

        // No software limits — we intentionally allow the mechanism to coast and
        // settle under gravity. Telemetry continues below.
        ntLeftPos.set(leftPos);
        ntRightPos.set(rightPos);
        ntAvgPos.set((leftPos + rightPos) / 2.0);
    }
}
