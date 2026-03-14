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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Controls"); //idk if needed

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);


    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController CommanderController = new CommandXboxController(1);

    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    /* Path follower */
    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

    NamedCommands.registerCommand("TurretAim", Commands.run(() -> {
                        turretSubsystem.aimAtTag();
                }, turretSubsystem));
    NamedCommands.registerCommand("Intake", Commands.run(() -> {
                        intakeSubsystem.setIntakeDown();
                        intakeSubsystem.runIntake(0.35);
                }, intakeSubsystem));    
    NamedCommands.registerCommand("StopIntake", Commands.run(() -> {
                        intakeSubsystem.stopIntake();
                        intakeSubsystem.setIntakeUp();
                }, intakeSubsystem));
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                    new InstantCommand(() -> shooterSubsystem.runFullShooter(0.45)),
                    new WaitCommand(1.5),
                    new InstantCommand(() -> shooterSubsystem.stopAll())
                    ));



                //  autoChooser = AutoBuilder.buildAutoChooser("Tests");
                //  SmartDashboard.putData("Auto Mode", autoChooser);

                    // TRIGGERS
    // Left Trigger - Turret auto-aim at AprilTag
    DriverController.leftTrigger()
     .whileTrue(
        new StartEndCommand(
          () -> turretSubsystem.aimAtTag(),
          () -> turretSubsystem.stop(),
          turretSubsystem));

  // Right trigger - Full shooter (flywheels + feeder + indexer)
    DriverController.rightTrigger()
    .whileTrue(
        new StartEndCommand(
            () -> shooterSubsystem.runFullShooter(7),  // Reduced from 0.7
            () -> shooterSubsystem.stopAll(),
            shooterSubsystem));

    // FACE BUTTONS
    // Y - Run shooter flywheels only (spin up)
    DriverController.y() //TURN THIS INTO DEFAULT COMMAND WHERE FLYWHEELS ARE ALWAYS SPINNING
    .whileTrue(
        new StartEndCommand(
            () -> shooterSubsystem.runFlywheels(0.5),  // Reduced from 0.7
            () -> shooterSubsystem.stopFlywheels(),
            shooterSubsystem));

    // X - Run intake rollers
    DriverController.x() //MAKE IT ON AND OFF, NOT WHILE
    .whileTrue(
        new StartEndCommand(
            () -> intakeSubsystem.runIntake(5),  // Reduced from 0.5
            () -> intakeSubsystem.stopIntake(),
            intakeSubsystem));

    // B - Intake deploy up
    DriverController.b()
    .whileTrue(
        new StartEndCommand(
            () -> intakeSubsystem.setIntakeUp(),
            () -> {},
            intakeSubsystem));
            
    // A - Intake deploy down
    DriverController.a()
    .whileTrue(
        new StartEndCommand(
            () -> intakeSubsystem.setIntakeDown(),
            () -> {},
            intakeSubsystem));

    // BUMPERS
    // Left Bumper - Run feeder only
    DriverController.leftBumper()
    .whileTrue(
        new StartEndCommand(
            () -> shooterSubsystem.runFeeder(5),  // Reduced from 0.5
            () -> shooterSubsystem.stopFeeder(),
            shooterSubsystem));

    // Right Bumper - Run indexer only
    DriverController.rightBumper()
    .whileTrue(
        new StartEndCommand(
            () -> shooterSubsystem.runIndexer(5),  // Reduced from 0.4
            () -> shooterSubsystem.stopIndexer(),
            shooterSubsystem));

    // D-PAD (POV) - CLIMBER CONTROLS
    // POV Up (0°) - Elevator UP
    DriverController.povUp()
      .whileTrue(
          new StartEndCommand(
              () -> climberSubsystem.elevatorUp(0.3),  // Reduced from 0.6
              () -> climberSubsystem.stopElevator(),
              climberSubsystem
          )
      );

    // POV Down (180°) - Elevator DOWN
    DriverController.povDown()
      .whileTrue(
          new StartEndCommand(
              () -> climberSubsystem.elevatorDown(0.3),  // Reduced from 0.6
              () -> climberSubsystem.stopElevator(),
              climberSubsystem
          )
      );

    // POV Right (90°) - Climb motors forward
    DriverController.povRight()
      .whileTrue(
          new StartEndCommand(
              () -> climberSubsystem.runClimb(0.25),  // Reduced from 0.5
              () -> climberSubsystem.stopClimb(),
              climberSubsystem
          )
      );

    // POV Left (270°) - Climb motors reverse
    DriverController.povLeft()
      .whileTrue(
          new StartEndCommand(
              () -> climberSubsystem.runClimb(-0.25),  // Reduced from -0.5
              () -> climberSubsystem.stopClimb(),
              climberSubsystem
          )
      );
      
                configureBindings();
                setLimelight();
                displayPose();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-DriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-DriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // DriverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        DriverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

            // ========== SINGLE CONTROLLER - ALL ROBOT CONTROLS ==========
    // NOTE: All speeds set to LOW values for safe testing



    // BACK/START BUTTONS - SERVO CONTROLS
    // Back button - Servos retracted
    DriverController.back()
      .onTrue(new InstantCommand(() -> climberSubsystem.setServoRetracted(), climberSubsystem));

    // Start button - Servos extended
    DriverController.start()
      .onTrue(new InstantCommand(() -> climberSubsystem.setServoExtended(), climberSubsystem));


    // ========== DEFAULT COMMANDS ==========

    // Turret default: Manual control with right stick, auto-aim when centered
    turretSubsystem.setDefaultCommand(new RunCommand(
        () -> {
          double input = CommanderController.getRightX();
          if (Math.abs(input) > 0.1) {
            turretSubsystem.rotate(input * 0.15); // Reduced from 0.2 for testing
          } else {
            turretSubsystem.aimAtTag(); // Auto-aim when stick centered
          }
        },
        turretSubsystem));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(xLimiter
                .calculate(-DriverController.getLeftY() * 0.7* MaxSpeed))

            .withVelocityY(yLimiter
                .calculate(-DriverController.getLeftX() * 0.7* MaxSpeed))

            .withRotationalRate(rotationLimiter.calculate(
                -DriverController.getRightX() * 0.85* MaxAngularRate))));
        
    }

        private void displayPose(){
                new Thread(() -> {
                        while (true) {

                                Pose2d pose = drivetrain.getState().Pose;
                                SmartDashboard.putNumber("Pose X (m)", pose.getX());
                                SmartDashboard.putNumber("Pose Y (m)", pose.getY());
                                SmartDashboard.putNumber(
                                    "Pose Heading (deg)",
                                    pose.getRotation().getDegrees()
                                );
                                // Sleep for a short duration to prevent overloading CPU
                                Timer.delay(0.05); // 50ms delay
                        }
                }).start();
        }

        private void setLimelight() {
            new Thread(() -> {
                while (true) {
                    // Continuously update Shuffleboard button states
                    // Get the Limelight NetworkTable
                    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                    NetworkTableEntry tx = table.getEntry("tx");
                    NetworkTableEntry ty = table.getEntry("ty");
                    NetworkTableEntry ta = table.getEntry("ta");
                    NetworkTableEntry tid = table.getEntry("tid");

                    // Read values from Limelight
                    double x = tx.getDouble(0.0); // Horizontal offset from crosshair to target
                    double y = ty.getDouble(0.0); // Vertical offset from crosshair to target
                    double area = ta.getDouble(0.0); // Target area (0 to 100%)
                    double id = tid.getDouble(0.0);
                    // Post values to Shuffleboard
                    SmartDashboard.putNumber("Limelight Tag ID", id);
                    SmartDashboard.putNumber("LimelightX", x);
                    SmartDashboard.putNumber("LimelightY", y);
                    SmartDashboard.putNumber("LimelightArea", area);

                    NetworkTable desk = NetworkTableInstance.getDefault().getTable("lemonlight");
                    NetworkTableEntry tx2 = desk.getEntry("tx");
                    NetworkTableEntry ty2 = desk.getEntry("ty");
                    NetworkTableEntry ta2 = desk.getEntry("ta");
                    NetworkTableEntry tid2 = desk.getEntry("tid");

                    // Read values from Limelight
                    double x2 = tx2.getDouble(0.0); // Horizontal offset from crosshair to target
                    double y2 = ty2.getDouble(0.0); // Vertical offset from crosshair to target
                    double area2 = ta2.getDouble(0.0); // Target area (0 to 100%)
                    double id2 = tid2.getDouble(0.0);
                    // Post values to Shuffleboard
                    SmartDashboard.putNumber("Limelight Tag ID", id2);
                    SmartDashboard.putNumber("LimelightX", x2);
                    SmartDashboard.putNumber("LimelightY", y2);
                    SmartDashboard.putNumber("LimelightArea", area2);
                    // Sleep for a short duration to prevent overloading CPU
                    Timer.delay(0.05); // 50ms delay
                        }
                }).start();
        } 

    //public Command getAutonomousCommand() {
    
        //return autoChooser.getSelected();
    //}
 
    public static RobotContainer getInstance() {
        // TODO Auto-generated method stub
        
    throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
    }
}
