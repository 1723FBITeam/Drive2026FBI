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

    // Calibration dashboard entries
    private edu.wpi.first.networktables.GenericEntry calRobotX;
    private edu.wpi.first.networktables.GenericEntry calRobotY;
    private edu.wpi.first.networktables.GenericEntry calRobotHeading;
    private edu.wpi.first.networktables.GenericEntry calDistToHub;
    private edu.wpi.first.networktables.GenericEntry calTargetX;
    private edu.wpi.first.networktables.GenericEntry calTargetY;
    private edu.wpi.first.networktables.GenericEntry calAlliance;
    private edu.wpi.first.networktables.GenericEntry calLastDist;
    private edu.wpi.first.networktables.GenericEntry calLastHood;
    private edu.wpi.first.networktables.GenericEntry calShotNum;
    private int calShotCount = 0;

    public RobotContainer() {
        // ===== NAMED COMMANDS FOR PATHPLANNER =====
        // PathPlanner autonomous routines can trigger these by name.
        // For example, a path can say "run Intake" at a certain point.
        NamedCommands.registerCommand("TurretAim", Commands.run(() -> {
            turretSubsystem.aimAtPose(drivetrain.getState().Pose, getSmartTarget());
        }, turretSubsystem));
        NamedCommands.registerCommand("Intake", Commands.run(() -> {
            intakeSubsystem.deployOut();
            intakeSubsystem.runIntake(0.35);
        }, intakeSubsystem));
        NamedCommands.registerCommand("StopIntake", Commands.run(() -> {
            intakeSubsystem.stopIntake();
            intakeSubsystem.deployIn();
        }, intakeSubsystem));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
            new InstantCommand(() -> shooterSubsystem.runFullShooter(0.45)),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooterSubsystem.stopAll())
        ));
        NamedCommands.registerCommand("AutoShoot",
            new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain).withTimeout(3.0));

        // ===== AUTO CHOOSER =====
        // Builds a dropdown from all PathPlanner auto files in deploy/pathplanner/autos/
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Calibration")
            .add("Auto Mode", autoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);

        // ===== CALIBRATION DASHBOARD =====
        // These widgets show up on the "Calibration" tab in Shuffleboard
        // Used for tuning the shooter's distance-based lookup tables
        var calTab = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Calibration");
        calRobotX    = calTab.add("Robot X", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        calRobotY    = calTab.add("Robot Y", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        calRobotHeading = calTab.add("Robot Heading", 0.0).withPosition(2, 2).withSize(1, 1).getEntry();
        calDistToHub = calTab.add("Dist to Hub", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        calTargetX   = calTab.add("Target X", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
        calTargetY   = calTab.add("Target Y", 0.0).withPosition(5, 2).withSize(1, 1).getEntry();
        calAlliance  = calTab.add("Alliance", "???").withPosition(3, 2).withSize(1, 1).getEntry();
        calLastDist  = calTab.add("Last Shot Dist", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        calLastHood  = calTab.add("Last Shot Hood", 0.0).withPosition(6, 1).withSize(1, 1).getEntry();
        calShotNum   = calTab.add("Shot #", 0.0).withPosition(7, 1).withSize(1, 1).getEntry();

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
        controller.x()
            .toggleOnTrue(new SequentialCommandGroup(
                new InstantCommand(() -> intakeSubsystem.deployOut(), intakeSubsystem),
                new WaitCommand(0.3),
                new InstantCommand(() -> intakeSubsystem.stopDeploy()),
                new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem)
            ).finallyDo(() -> {
                intakeSubsystem.deployIn();
                intakeSubsystem.stopIntake();
                new WaitCommand(0.3).andThen(
                    new InstantCommand(() -> intakeSubsystem.stopDeploy())
                ).schedule();
            }));

        // A BUTTON — jostle intake to unstick balls
        // Quick low-power in/out pulse sequence, won't stress motors
        controller.a()
            .onTrue(intakeSubsystem.jostleCommand());

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

        // ===================================================================
        //                 CO-PILOT CONTROLLER (USB port 1)
        // ===================================================================
        // The co-pilot handles shooting, intake, tuning, calibration,
        // and manual overrides so the driver can focus on driving.
        //
        // FACE BUTTONS:
        //   Y = toggle auto-shoot on/off
        //   X = toggle intake on/off
        //   A = jostle intake (unstick balls)
        //   B = emergency stop all shooter motors
        //
        // BUMPERS + TRIGGERS (manual overrides):
        //   Left Trigger  = manually rotate turret left (pressure = speed)
        //   Right Trigger = manually rotate turret right (pressure = speed)
        //   Left Bumper   = nudge hood servo up (hold)
        //   Right Bumper  = nudge hood servo down (hold)
        //
        // D-PAD (live tuning offsets):
        //   Left/Right = nudge turret aim ±0.5 degrees
        //   Up/Down    = nudge shooter power ±1 RPS
        //
        // MENU BUTTONS:
        //   Start = save shot calibration data point
        //   Back  = reset all offsets to zero
        // ===================================================================

        // --- Face buttons ---

        // Y — toggle auto-shoot (co-pilot can trigger shooting for the driver)
        copilot.y()
            .toggleOnTrue(new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getSmartTarget));

        // X — toggle intake on/off
        copilot.x()
            .toggleOnTrue(new SequentialCommandGroup(
                new InstantCommand(() -> intakeSubsystem.deployOut(), intakeSubsystem),
                new WaitCommand(0.3),
                new InstantCommand(() -> intakeSubsystem.stopDeploy()),
                new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem)
            ).finallyDo(() -> {
                intakeSubsystem.deployIn();
                intakeSubsystem.stopIntake();
                new WaitCommand(0.3).andThen(
                    new InstantCommand(() -> intakeSubsystem.stopDeploy())
                ).schedule();
            }));

        // A — jostle intake (unstick balls)
        copilot.a()
            .onTrue(intakeSubsystem.jostleCommand());

        // B — emergency stop all shooter motors
        copilot.b()
            .onTrue(new InstantCommand(() -> shooterSubsystem.stopAll()));

        // --- Bumpers + Triggers (manual overrides) ---

        // Left Trigger — manually rotate turret CCW/left (pressure-sensitive)
        copilot.leftTrigger(0.05).whileTrue(Commands.run(
            () -> turretSubsystem.rotate(copilot.getLeftTriggerAxis() * 0.5),
            turretSubsystem).finallyDo(() -> turretSubsystem.stop()));

        // Right Trigger — manually rotate turret CW/right (pressure-sensitive)
        copilot.rightTrigger(0.05).whileTrue(Commands.run(
            () -> turretSubsystem.rotate(-copilot.getRightTriggerAxis() * 0.5),
            turretSubsystem).finallyDo(() -> turretSubsystem.stop()));

        // Left Bumper — nudge hood servo UP (hold)
        copilot.leftBumper()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(0.005)));

        // Right Bumper — nudge hood servo DOWN (hold)
        copilot.rightBumper()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(-0.005)));

        // --- D-pad (live tuning offsets) ---

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

        // Start — save shot calibration data point
        copilot.start()
            .onTrue(new InstantCommand(() -> {
                Pose2d pose = drivetrain.getState().Pose;
                double dist = pose.getTranslation().getDistance(
                    getSmartTarget().getTranslation());
                double hood = shooterSubsystem.getHoodPosition();
                System.out.println(">>> SHOT DATA: dist=" + String.format("%.2f", dist)
                    + "m, hood=" + String.format("%.3f", hood)
                    + ", x=" + String.format("%.2f", pose.getX())
                    + ", y=" + String.format("%.2f", pose.getY()) + " <<<");
                calShotCount++;
                calLastDist.setDouble(dist);
                calLastHood.setDouble(hood);
                calShotNum.setDouble(calShotCount);
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

        // Turret default: continuously auto-aim at the hub
        turretSubsystem.setDefaultCommand(new RunCommand(
            () -> turretSubsystem.aimAtPose(drivetrain.getState().Pose, getSmartTarget()),
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


    /** Called from Robot.robotPeriodic() — updates calibration values on the dashboard */
    public void updateCalibrationTelemetry() {
        Pose2d pose = drivetrain.getState().Pose;
        Pose2d target = getSmartTarget();
        double dist = pose.getTranslation().getDistance(target.getTranslation());

        calRobotX.setDouble(pose.getX());
        calRobotY.setDouble(pose.getY());
        calRobotHeading.setDouble(pose.getRotation().getDegrees());
        calDistToHub.setDouble(dist);
        calTargetX.setDouble(target.getX());
        calTargetY.setDouble(target.getY());

        var alliance = DriverStation.getAlliance();
        calAlliance.setString(alliance.isPresent()
            ? (alliance.get() == Alliance.Red ? "RED" : "BLUE")
            : "NOT SET");
    }
}
