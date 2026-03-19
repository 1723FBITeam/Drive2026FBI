// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.AutoShootCommand;

/**
 * RobotContainer — THIS IS THE MOST IMPORTANT FILE.
 *
 * This is where we:
 *   1. Create all subsystems (drivetrain, shooter, turret, intake)
 *   2. Map controller buttons to robot actions
 *   3. Set up default commands (what each subsystem does when idle)
 *   4. Register named commands for PathPlanner autonomous routines
 *   5. Set up the auto chooser on the dashboard
 *
 * Think of this as the "wiring diagram" for the software — it connects
 * inputs (controller buttons) to outputs (motor actions).
 */
public class RobotContainer {
    // ===== SPEED LIMITS =====
    // MaxSpeed: how fast the robot can drive (meters per second)
    // We multiply by 1.0 here — change to 0.5 for half speed during testing!
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // MaxAngularRate: how fast the robot can spin (radians per second)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /**
     * Joystick curve — makes the sticks less sensitive near the center
     * for better precision at low speeds, while still allowing full speed.
     *
     * Blend: 30% linear + 70% cubic
     * - At small inputs (0.1): output is ~0.03 + ~0.0007 = very gentle
     * - At full input (1.0): output is 0.3 + 0.7 = 1.0 (full speed)
     */
    private static double joystickCurve(double input) {
        double cubic = input * input * input;
        return 0.3 * input + 0.7 * cubic;
    }

    // ===== SWERVE DRIVE REQUEST =====
    // FieldCentric means pushing the stick "forward" always drives away from the driver,
    // regardless of which way the robot is facing.
    // Deadband: ignore tiny stick movements (10% of max) to prevent drift
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ===== SUBSYSTEMS =====
    // Each subsystem represents a physical mechanism on the robot
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final frc.robot.subsystems.ClimberSubsystem climber = new frc.robot.subsystems.ClimberSubsystem();
    // ClimberSubsystem — not wired yet, uncomment when ready

    // Telemetry — sends drivetrain data to the dashboard
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ===== SLEW RATE LIMITERS =====
    // These smooth out sudden joystick movements so the robot doesn't jerk.
    // Value of 3.0 means it takes ~0.33 seconds to go from 0 to full speed.
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    // Xbox controller on USB port 0 (driver)
    private final CommandXboxController controller = new CommandXboxController(0);

    // Xbox controller on USB port 1 (co-pilot — live-tunes turret aim and shooter power)
    private final CommandXboxController copilot = new CommandXboxController(1);

    // Auto chooser — dropdown on the dashboard to pick autonomous routine
    private final SendableChooser<Command> autoChooser;

    // Calibration dashboard — plain NetworkTables (arrange in Shuffleboard yourself)
    private final DoublePublisher calRobotX;
    private final DoublePublisher calRobotY;
    private final DoublePublisher calRobotHeading;
    private final DoublePublisher calDistToHub;
    private final DoublePublisher calTargetX;
    private final DoublePublisher calTargetY;
    private edu.wpi.first.networktables.StringPublisher calAlliance;
    private int telemetryCounter = 0;

    // ===== AUTO-AIM MODE =====
    // Tracks whether auto-aim is active. Start button on co-pilot toggles this.
    // When OFF: co-pilot uses triggers to manually aim, B to fire.
    // When ON: turret auto-tracks the hub, triggers disabled.
    private boolean autoAimEnabled = true;

    public RobotContainer() {
        // ===== NAMED COMMANDS FOR PATHPLANNER =====
        // PathPlanner autonomous routines can trigger these by name.
        // For example, a path can say "run Intake" at a certain point.
        NamedCommands.registerCommand("TurretAim", Commands.run(() -> {
            turretSubsystem.aimAtPose(drivetrain.getState().Pose, getSmartTarget(),
                drivetrain.getState().Speeds);
        }, turretSubsystem));
        NamedCommands.registerCommand("Intake", Commands.run(() -> {
            intakeSubsystem.deployOut();
            intakeSubsystem.runIntake(0.35);
        }, intakeSubsystem));
        NamedCommands.registerCommand("StopIntake", Commands.run(() -> {
            intakeSubsystem.stopIntake();
            intakeSubsystem.deployIn();
        }, intakeSubsystem));
        NamedCommands.registerCommand("AutoShoot",
            new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain));

        // ===== AUTO CHOOSER =====
        // Builds a dropdown from all PathPlanner auto files in deploy/pathplanner/autos/
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // ===== CALIBRATION DASHBOARD =====
        // Plain NetworkTables — arrange these in Shuffleboard however you like
        NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Calibration");
        calRobotX    = calTable.getDoubleTopic("Robot X").publish();
        calRobotY    = calTable.getDoubleTopic("Robot Y").publish();
        calRobotHeading = calTable.getDoubleTopic("Robot Heading").publish();
        calDistToHub = calTable.getDoubleTopic("Dist to Hub").publish();
        calTargetX   = calTable.getDoubleTopic("Target X").publish();
        calTargetY   = calTable.getDoubleTopic("Target Y").publish();
        calAlliance  = calTable.getStringTopic("Alliance").publish();

        // ===================================================================
        //                 DRIVER CONTROLLER (USB port 0)
        // ===================================================================
        // The driver only has 4 buttons — drive, shoot, intake, jostle, reset.
        // Everything else is on the co-pilot controller.
        // ===================================================================

        // Y BUTTON — toggle auto-shoot on/off
        // Press once: aims turret, spins flywheels, feeds when ready
        // Press again: stops everything
        controller.y()
            .toggleOnTrue(new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getSmartTarget));

        // X BUTTON — toggle intake on/off
        // Press once: deploys intake out, starts rollers
        // Press again: retracts intake, stops rollers
        //
        // How this works:
        //   1. Deploy out for 0.3s, then stop deploy motor
        //   2. Run rollers until the button is toggled off (RunCommand keeps going)
        //   3. When cancelled: stop rollers, retract for 0.3s, then stop deploy
        //
        // The retract sequence is part of the command chain (not a fire-and-forget
        // schedule call) so we avoid the deprecated Command.schedule() API.
        controller.x()
            .toggleOnTrue(
                // Phase 1: Deploy out, wait, stop deploy, then run rollers
                new SequentialCommandGroup(
                    new InstantCommand(() -> intakeSubsystem.deployOut(), intakeSubsystem),
                    new WaitCommand(0.3),
                    new InstantCommand(() -> intakeSubsystem.stopDeploy()),
                    new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem)
                ).finallyDo(() -> {
                    // Immediately stop rollers and start retracting.
                    // The deploy motor runs at 15% in brake mode — it will hold
                    // position once stopped, so we just need a brief pulse to retract.
                    intakeSubsystem.stopIntake();
                    intakeSubsystem.deployIn();
                    // Schedule a short, independent command to stop the deploy motor
                    // after 0.3s. We schedule it separately because the outer
                    // toggle command may be canceled (interrupting its own
                    // .andThen() chain), so this ensures the stop always runs.
                    new WaitCommand(0.3)
                        .andThen(new InstantCommand(() -> intakeSubsystem.stopDeploy(), intakeSubsystem))
                        .schedule();
                })
                // Phase 2: After cancel/end, immediate retract was scheduled above
            );

        // A BUTTON — jostle intake to unstick balls
        // Quick low-power in/out pulse sequence, won't stress motors
        controller.a()
            .onTrue(intakeSubsystem.jostleCommand());

        // B BUTTON — toggle elevator level-and-return cycle
        // Press: go to level position. Press again: return to home early.
        controller.b()
            .onTrue(new InstantCommand(() -> climber.toggleLevelCycle()));

        // BACK BUTTON — reset field-centric heading
        // Determines the correct heading based on alliance color.
        // Blue alliance: robot faces red wall at startup = 180° in blue-origin coords
        // Red alliance: robot faces blue wall at startup = 0° in blue-origin coords
        controller.back()
            .onTrue(drivetrain.runOnce(() -> {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                drivetrain.seedFieldCentric(isRed ? Rotation2d.kZero : Rotation2d.k180deg);
            }));
        controller.rightBumper()
            .onTrue(new InstantCommand(() -> climber.toggleGearboxLock()));

        // ===================================================================
        //                 CO-PILOT CONTROLLER (USB port 1)
        // ===================================================================
        // Two modes controlled by Start button:
        //
        // AUTO-AIM ON (default):
        //   Turret auto-tracks hub. Triggers disabled.
        //   Y = toggle auto-shoot (aim + spin + feed when ready)
        //   X = toggle intake
        //   A = jostle intake
        //   B = emergency stop shooter
        //   Bumpers = nudge hood up/down
        //   D-pad L/R = aim offset ±0.5°
        //   D-pad U/D = power offset ±1 RPS
        //   Back = reset offsets
        //
        // AUTO-AIM OFF (manual mode):
        //   Turret controlled by triggers. Flywheels still auto-calculated.
        //   Triggers = manually aim turret left/right
        //   B = hold to fire (spin flywheels + feed)
        //   Bumpers = nudge hood up/down
        //   D-pad still works for offsets
        //   Back = reset offsets
        //   Start again = re-enable auto-aim
        // ===================================================================

        // --- Face buttons (same in both modes) ---

        // Y — toggle auto-shoot (co-pilot can trigger shooting for the driver)
        copilot.y()
            .toggleOnTrue(new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getSmartTarget));

        // X — manually move elevator up slowly (hold to move)
        copilot.x()
            .whileTrue(new edu.wpi.first.wpilibj2.command.StartEndCommand(
                () -> climber.runElevator(0.25),
                () -> climber.stopElevator(),
                climber
            ));

        // A — manually move elevator down slowly (hold to move)
        copilot.a()
            .whileTrue(new edu.wpi.first.wpilibj2.command.StartEndCommand(
                () -> climber.runElevator(-0.25),
                () -> climber.stopElevator(),
                climber
            ));

        // B — in auto mode: emergency stop. In manual mode: hold to fire.
        copilot.b()
            .whileTrue(Commands.either(
                // AUTO-AIM ON: emergency stop (instant, not held)
                new InstantCommand(() -> shooterSubsystem.stopAll()),
                // AUTO-AIM OFF (manual mode): spin flywheels at distance-based speed + feed
                Commands.run(() -> {
                    Pose2d pose = drivetrain.getState().Pose;
                    double dist = pose.getTranslation().getDistance(getSmartTarget().getTranslation());
                    if (dist > 0.5) {
                        shooterSubsystem.autoAim(dist);
                    }
                    shooterSubsystem.runFeeder(0.7);
                    shooterSubsystem.runIndexer(0.5);
                }).finallyDo(() -> {
                    shooterSubsystem.stopFeeder();
                    shooterSubsystem.stopIndexer();
                }),
                () -> autoAimEnabled
            ));

        // --- Triggers (manual turret — only when auto-aim is OFF) ---

        // Left Trigger — manually rotate turret CCW/left (only in manual mode)
        copilot.leftTrigger(0.05).and(() -> !autoAimEnabled).whileTrue(Commands.run(
            () -> turretSubsystem.rotate(copilot.getLeftTriggerAxis() * 0.5),
            turretSubsystem).finallyDo(() -> turretSubsystem.stop()));

        // Right Trigger — manually rotate turret CW/right (only in manual mode)
        copilot.rightTrigger(0.05).and(() -> !autoAimEnabled).whileTrue(Commands.run(
            () -> turretSubsystem.rotate(-copilot.getRightTriggerAxis() * 0.5),
            turretSubsystem).finallyDo(() -> turretSubsystem.stop()));

        // --- Bumpers (hood nudge — works in both modes) ---

        // Left Bumper — nudge hood servo UP (hold)
        copilot.leftBumper()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(0.005)));

        // Right Bumper — toggle climber gearbox lock/unlock
        copilot.rightBumper()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(-0.005)));

        // --- D-pad (live tuning offsets — works in both modes) ---

        // D-pad Left — nudge turret aim left (CCW) by 0.5 degrees
        copilot.povLeft()
            .onTrue(new InstantCommand(() -> turretSubsystem.nudgeAimLeft()));

        // D-pad Right — nudge turret aim right (CW) by 0.5 degrees
        copilot.povRight()
            .onTrue(new InstantCommand(() -> turretSubsystem.nudgeAimRight()));

        // D-pad Up — increase shooter power by 1 RPS
        copilot.povUp()
            .onTrue(new InstantCommand(() -> shooterSubsystem.nudgePowerUp()));

        // D-pad Down — decrease shooter power by 1 RPS
        copilot.povDown()
            .onTrue(new InstantCommand(() -> shooterSubsystem.nudgePowerDown()));

        // --- Menu buttons ---

        // Start — toggle auto-aim on/off
        // When OFF: turret stops auto-tracking, co-pilot uses triggers to aim manually
        // When ON: turret resumes auto-tracking the hub
        copilot.start()
            .onTrue(new InstantCommand(() -> {
                autoAimEnabled = !autoAimEnabled;
                if (autoAimEnabled) {
                    System.out.println(">>> AUTO-AIM: ON <<<");
                } else {
                    System.out.println(">>> AUTO-AIM: OFF (manual mode) <<<");
                    turretSubsystem.stop(); // Stop turret so triggers can take over
                }
            }));

        // Back — reset both offsets to zero
        copilot.back()
            .onTrue(new InstantCommand(() -> {
                turretSubsystem.resetAimOffset();
                shooterSubsystem.resetPowerOffset();
            }));

        // ===== IDLE BEHAVIOR =====
        // When the robot is disabled, set swerve modules to idle (no power)
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Register telemetry so drivetrain data shows up on the dashboard
        drivetrain.registerTelemetry(logger::telemeterize);

        // Set initial heading based on alliance color
        // Blue: facing red wall = 180°, Red: facing blue wall = 0° (in blue-origin coords)
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        drivetrain.seedFieldCentric(isRed ? Rotation2d.kZero : Rotation2d.k180deg);

        // ===== DEFAULT COMMANDS =====
        // Default commands run whenever no other command is using that subsystem.

        // Turret default: auto-aim at hub when enabled, idle when manual mode
        turretSubsystem.setDefaultCommand(new RunCommand(
            () -> {
                if (autoAimEnabled) {
                    turretSubsystem.aimAtPose(drivetrain.getState().Pose, getSmartTarget(),
                        drivetrain.getState().Speeds);
                }
                // When auto-aim is off, turret just holds position (no command)
                // Co-pilot uses triggers to manually aim
            },
            turretSubsystem));

        // Drivetrain default: drive using joystick input with cubic curve + slew rate limiting
        // The 0.75 and 0.85 multipliers cap max speed for safety during testing
        // (change to 1.0 for full speed at competition)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                .withRotationalRate(rotationLimiter.calculate(joystickCurve(-controller.getRightX()) * 0.85 * MaxAngularRate))));
    }

    /** Returns the selected autonomous command from the dashboard dropdown */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Manual elevator control command driven by the copilot's left stick Y-axis.
     * Call this to bind a button to hold for manual control, e.g.:
     *   copilot.rightStick().whileTrue(copilotManualElevatorControl());
     */
    public Command copilotManualElevatorControl() {
        return new RunCommand(() -> climber.runElevator(-copilot.getLeftY() * 0.6), climber);
    }

    /**
     * Returns the target to aim at based on alliance color.
     *
     * SIMPLIFIED FOR TESTING — always aims at our hub.
     * Zone-based targeting (corners, trench hood flatten) is disabled
     * until basic aiming is confirmed working.
     *
     * TODO: Re-enable zone logic once turret aiming is verified.
     */
    public Pose2d getSmartTarget() {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        // Safety: if alliance is not set, try to guess from robot position.
        // If we're on the red half of the field (X > 8.27), assume red.
        if (!alliance.isPresent()) {
            double robotX = drivetrain.getState().Pose.getX();
            isRed = robotX > (Constants.FieldConstants.FIELD_LENGTH_METERS / 2.0);
        }

        return isRed ? Constants.FieldConstants.RED_HUB_POSE
                     : Constants.FieldConstants.BLUE_HUB_POSE;
    }


    /** Called from Robot.robotPeriodic() — updates calibration values (~10Hz) */
    public void updateCalibrationTelemetry() {
        telemetryCounter++;
        if (telemetryCounter % 5 != 0) return; // ~10Hz instead of 50Hz

        Pose2d pose = drivetrain.getState().Pose;
        Pose2d target = getSmartTarget();
        double dist = pose.getTranslation().getDistance(target.getTranslation());

        calRobotX.set(pose.getX());
        calRobotY.set(pose.getY());
        calRobotHeading.set(pose.getRotation().getDegrees());
        calDistToHub.set(dist);
        calTargetX.set(target.getX());
        calTargetY.set(target.getY());

        var alliance = DriverStation.getAlliance();
        calAlliance.set(alliance.isPresent()
            ? (alliance.get() == Alliance.Red ? "RED" : "BLUE")
            : "NOT SET");

        SmartDashboard.putBoolean("Auto-Aim", autoAimEnabled);
    }
}
