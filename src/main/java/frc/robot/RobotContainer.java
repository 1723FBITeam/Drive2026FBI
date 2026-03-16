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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.AutoShootCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /**
     * Attempt a joystick curve for better low-speed precision.
     * Attempt a blended curve: 30% linear + 70% cubic.
     * Smoother than pure cubic — less jerky at mid-range while still precise at low speeds.
     */
    private static double joystickCurve(double input) {
        double cubic = input * input * input;
        return 0.3 * input + 0.7 * cubic;
    }

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private final CommandXboxController controller = new CommandXboxController(0);

    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // ClimberSubsystem — not wired yet

    private final SendableChooser<Command> autoChooser;

    // Calibration telemetry
    private edu.wpi.first.networktables.GenericEntry calRobotX;
    private edu.wpi.first.networktables.GenericEntry calRobotY;
    private edu.wpi.first.networktables.GenericEntry calDistToHub;
    private edu.wpi.first.networktables.GenericEntry calLastDist;
    private edu.wpi.first.networktables.GenericEntry calLastHood;
    private edu.wpi.first.networktables.GenericEntry calShotNum;
    private int calShotCount = 0;

    public RobotContainer() {
        // Named commands for PathPlanner autos
        NamedCommands.registerCommand("TurretAim", Commands.run(() -> {
            turretSubsystem.aimAtPose(drivetrain.getState().Pose, getHubPose());
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

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Calibration")
            .add("Auto Mode", autoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);

        // Calibration telemetry — add widgets to Calibration tab
        var calTab = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Calibration");
        calRobotX    = calTab.add("Robot X", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        calRobotY    = calTab.add("Robot Y", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        calDistToHub = calTab.add("Dist to Hub", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        calLastDist  = calTab.add("Last Shot Dist", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        calLastHood  = calTab.add("Last Shot Hood", 0.0).withPosition(6, 1).withSize(1, 1).getEntry();
        calShotNum   = calTab.add("Shot #", 0.0).withPosition(7, 1).withSize(1, 1).getEntry();

        // ===== SINGLE CONTROLLER LAYOUT =====
        // Left stick:    Drive X/Y (cubic curve)
        // Right stick X: Rotation (cubic curve)
        // Left bumper:   Turret rotate left (manual override)
        // Right bumper:  Turret rotate right (manual override)
        // Left trigger:  Free
        // Right trigger: Full shoot (flywheels + feeder + indexer)
        // Y:             Spin up flywheels only (pre-spin)
        // X:             Toggle intake rollers on/off
        // A:             Hood servo nudge up (hold)
        // B:             Hood servo nudge down (hold)
        // Start:         Toggle shoot sequence (spin up → feed)
        // Back:          Reset field-centric heading
        // D-pad Up:      Save shot calibration data point
        // D-pad L/R/D:   Free for climber later

        // Left Bumper — turret rotate left (manual)
        controller.leftBumper()
            .whileTrue(new StartEndCommand(
                () -> turretSubsystem.rotate(-0.15),
                () -> turretSubsystem.stop(),
                turretSubsystem));

        // Right Bumper — turret rotate right (manual)
        controller.rightBumper()
            .whileTrue(new StartEndCommand(
                () -> turretSubsystem.rotate(0.15),
                () -> turretSubsystem.stop(),
                turretSubsystem));

        // Right Trigger — full shoot (flywheels + feeder + indexer)
        controller.rightTrigger()
            .whileTrue(new StartEndCommand(
                () -> shooterSubsystem.runFullShooter(0.7),
                () -> shooterSubsystem.stopAll(),
                shooterSubsystem));

        // Y — spin up flywheels only
        controller.y()
            .whileTrue(new StartEndCommand(
                () -> shooterSubsystem.runFlywheels(0.5),
                () -> shooterSubsystem.stopFlywheels(),
                shooterSubsystem));

        // X — toggle intake rollers on/off
        controller.x()
            .toggleOnTrue(new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem)
                .finallyDo(() -> intakeSubsystem.stopIntake()));

        // A — hood servo up (hold to nudge position)
        controller.a()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(0.005)));

        // B — hood servo down (hold to nudge position)
        controller.b()
            .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(-0.005)));

        // Start — toggle shoot sequence with auto-aim (hood + flywheels scale with distance)
        controller.start()
            .toggleOnTrue(new SequentialCommandGroup(
                // Spin up flywheels using distance-based speed and set hood
                new RunCommand(() -> {
                    double dist = drivetrain.getState().Pose.getTranslation()
                        .getDistance(getHubPose().getTranslation());
                    shooterSubsystem.autoAim(dist);
                }, shooterSubsystem)
                    .until(() -> shooterSubsystem.isReadyToShoot()),
                // Flywheels ready — keep auto-aiming and start feeder + indexer
                new RunCommand(() -> {
                    double dist = drivetrain.getState().Pose.getTranslation()
                        .getDistance(getHubPose().getTranslation());
                    shooterSubsystem.autoAim(dist);
                    shooterSubsystem.runFeeder(0.7);
                    shooterSubsystem.runIndexer(0.3);
                }, shooterSubsystem)
            ).finallyDo(() -> shooterSubsystem.stopAll()));

        // D-pad Up — save current shot data point for calibration
        controller.povUp()
            .onTrue(new InstantCommand(() -> {
                Pose2d pose = drivetrain.getState().Pose;
                double dist = pose.getTranslation().getDistance(
                    getHubPose().getTranslation());
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

        // Back — reset field-centric heading
        // Robot faces drivers at startup = 180° relative to operator forward (away from drivers)
        // seedFieldCentric(180°) tells the system "I'm currently facing 180° from operator forward"
        controller.back()
            .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.k180deg)));

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        // Seed field-centric at startup — robot faces drivers = 180° from operator forward
        drivetrain.seedFieldCentric(Rotation2d.k180deg);

        // Turret default: auto-aim at hub (bumpers override for manual control)
        turretSubsystem.setDefaultCommand(new RunCommand(
            () -> turretSubsystem.aimAtPose(drivetrain.getState().Pose, getHubPose()),
            turretSubsystem));

        // Drivetrain default — blended joystick curve for low-speed precision
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                .withRotationalRate(rotationLimiter.calculate(joystickCurve(-controller.getRightX()) * 0.85 * MaxAngularRate))));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Returns the correct hub pose based on current alliance color */
    private Pose2d getHubPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.FeildConstants.RED_HUB_POSE;
        }
        return Constants.FeildConstants.BLUE_HUB_POSE;
    }

    /** Call from Robot.robotPeriodic() to update calibration dashboard */
    public void updateCalibrationTelemetry() {
        Pose2d pose = drivetrain.getState().Pose;
        double dist = pose.getTranslation().getDistance(getHubPose().getTranslation());
        calRobotX.setDouble(pose.getX());
        calRobotY.setDouble(pose.getY());
        calDistToHub.setDouble(dist);
    }
}
