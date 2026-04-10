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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
 * 1. Create all subsystems (drivetrain, shooter, turret, intake)
 * 2. Map controller buttons to robot actions
 * 3. Set up default commands (what each subsystem does when idle)
 * 4. Register named commands for PathPlanner autonomous routines
 * 5. Set up the auto chooser on the dashboard
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
        // FieldCentric means pushing the stick "forward" always drives away from the
        // driver,
        // regardless of which way the robot is facing.
        // Deadband: ignore tiny stick movements (10% of max) to prevent drift
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // FieldCentricFacingAngle = drive with left stick, but lock heading to a
        // specific angle.
        // Used by driver D-pad for snap-to-heading (face a specific direction while
        // driving).
        // MaxAbsRotationalRate caps how fast the robot spins to reach the target
        // heading —
        // 0.75 rotations/sec (270°/s) is brisk but controlled, won't feel violent.
        private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(MaxSpeed * 0.1)
                        .withMaxAbsRotationalRate(RotationsPerSecond.of(0.5).in(RadiansPerSecond))
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // ===== SUBSYSTEMS =====
        // Each subsystem represents a physical mechanism on the robot
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final TurretSubsystem turretSubsystem = new TurretSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        // Telemetry — sends drivetrain data to the dashboard
        private final Telemetry logger = new Telemetry(MaxSpeed);

        // ===== PATHPLANNER AUTO LOADER FOR LEFT & RIGHT SWEEP =====
        private Command loadPathfindCommand(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
            
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
            
        } catch (Exception e) {
            // Print an error to the Driver Station so you know it failed
            DriverStation.reportError("Could not load PathPlanner path: " + pathName, false);
            
            // Return a safe dummy command so the robot doesn't crash if the button is pressed
            return Commands.print("Attempted to run missing path: " + pathName); 
        }
        }

        // ===== SLEW RATE LIMITERS =====
        // These smooth out sudden joystick movements so the robot doesn't jerk.
        // Value of 3.0 means it takes ~0.33 seconds to go from 0 to full speed.
        private final SlewRateLimiter xLimiter = new SlewRateLimiter(5.0);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);
        private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

        // Xbox controller on USB port 0 (driver)
        private final CommandXboxController controller = new CommandXboxController(0);

        // Xbox controller on USB port 1 (co-pilot — live-tunes turret aim and shooter
        // power)
        private final CommandXboxController copilot = new CommandXboxController(1);

        // Auto chooser — dropdown on the dashboard to pick autonomous routine
        private final SendableChooser<Command> autoChooser;

        // Field2d — shows the robot's estimated position on a field map in
        // Shuffleboard.
        // Drag this widget from SmartDashboard and change its type to "Field2d" to see
        // the robot icon moving on the field. Great for confirming vision + odometry.
        private final Field2d field2d = new Field2d();

        // Calibration dashboard — plain NetworkTables (arrange in Shuffleboard
        // yourself)
        private final DoublePublisher calRobotX;
        private final DoublePublisher calRobotY;
        private final DoublePublisher calRobotHeading;
        private final DoublePublisher calDistToHub;
        private final DoublePublisher calTargetX;
        private final DoublePublisher calTargetY;
        private edu.wpi.first.networktables.StringPublisher calAlliance;
        private int telemetryCounter = 0;

        // ===== VISION TOGGLE =====
        // When OFF, Limelight vision corrections are completely disabled.
        // Useful if vision is causing problems during a match.
        // Co-pilot right stick press toggles this.
        private boolean visionEnabled = true;

        // ===== PASS TARGET SIDE HYSTERESIS =====
        // Tracks which side (left/right) we're currently passing to.
        // true = left (high Y), false = right (low Y).
        // Once committed, the robot must cross 1m past the midline to flip.
        private boolean passTargetIsLeft = true;

        // ===== TRAJECTORY PHYSICS =====
        // Physics vs table blending is now automatic based on robot speed.
        // No toggle needed — stationary uses tables, moving blends in physics.

        // ===== HUB ACTIVATION STATUS =====
        // Tracks whether our alliance's hub is currently active based on FMS game data.
        // Game message = single char ('R' or 'B') = alliance whose hub goes inactive FIRST.
        // That alliance is ACTIVE in Shifts 2 and 4, INACTIVE in Shifts 1 and 3.
        // The other alliance is the opposite.
        // When no FMS is connected (practice/testing), hub is always treated as active.
        //
        // Match timing (2026 REBUILT):
        //   Auto: 0:20 countdown
        //   Teleop: 2:20 countdown (140 seconds), includes transition
        //     Transition: 2:20 → 2:10 (10s, hub always active)
        //     Shift 1: 2:10 → 1:45  (matchTime 130 → 105, 25s)
        //     Shift 2: 1:45 → 1:20  (matchTime 105 → 80, 25s)
        //     Shift 3: 1:20 → 0:55  (matchTime 80 → 55, 25s)
        //     Shift 4: 0:55 → 0:00  (matchTime 55 → 0, 55s)
        //   Endgame: last 30 seconds (matchTime <= 30)
        //
        // "First inactive" alliance (won auto): ACTIVE in Shifts 2 and 4
        // "Other" alliance (lost auto):         ACTIVE in Shifts 1 and 3
        //
        // PRE_ACTIVATE_SECONDS: start shooting this many seconds before the hub
        // turns on — balls in flight still count due to the 3-second scoring delay.
        private static final double PRE_ACTIVATE_SECONDS = 3.0;

    public RobotContainer() {
        // Wire up the robot X supplier for trench hood safety
        shooterSubsystem.setRobotXSupplier(() -> drivetrain.getState().Pose.getX());
        shooterSubsystem.setRobotYSupplier(() -> drivetrain.getState().Pose.getY());

        // ===== NAMED COMMANDS FOR PATHPLANNER =====
        // PathPlanner autonomous routines can trigger these by name.
        // The turret default command already auto-aims at the hub, so no
        // separate turret tracking command is needed.

        // AutoShoot — aims turret at hub, spins flywheels, feeds when ready.
        // Hub-only targeting for auto (driver controls pass targeting in teleop).
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getAllianceHub, this::isHubActive));

        // StopShooter — kills flywheels, feeder, indexer, and flattens hood.
        // Use when leaving alliance zone to collect — don't waste power spinning.
        NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> {
            shooterSubsystem.stopAll();
            shooterSubsystem.setHoodPosition(0.0);
        }, shooterSubsystem));

        // Intake — deploy intake and run rollers to collect balls.
        NamedCommands.registerCommand("Intake", Commands.run(() -> {
            intakeSubsystem.deployOut();
            intakeSubsystem.runIntake(0.35);
        }, intakeSubsystem));

        // StopIntake — retract intake and stop rollers.
        NamedCommands.registerCommand("StopIntake", Commands.run(() -> {
            intakeSubsystem.stopIntake();
            intakeSubsystem.deployIn();
        }, intakeSubsystem));

        // Climb — removed from robot for this competition.

        // ===== AUTO CHOOSER =====
        // Builds a dropdown from all PathPlanner auto files in
        // deploy/pathplanner/autos/
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Publish Field2d to SmartDashboard. In Shuffleboard, find "Field" under
        // SmartDashboard and drag it onto your layout — it auto-detects as a Field2d widget.
        // This shows the robot's estimated position from odometry + vision fusion.
        // It works without Limelight (odometry only) — vision just corrects drift.
        SmartDashboard.putData("Field", field2d);

        // ===== CALIBRATION DASHBOARD =====
        // Plain NetworkTables — arrange these in Shuffleboard however you like
        NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Calibration");
        calRobotX = calTable.getDoubleTopic("Robot X").publish();
        calRobotY = calTable.getDoubleTopic("Robot Y").publish();
        calRobotHeading = calTable.getDoubleTopic("Robot Heading").publish();
        calDistToHub = calTable.getDoubleTopic("Dist to Hub").publish();
        calTargetX = calTable.getDoubleTopic("Target X").publish();
        calTargetY = calTable.getDoubleTopic("Target Y").publish();
        calAlliance = calTable.getStringTopic("Alliance").publish();

        // ===================================================================
        // DRIVER CONTROLLER (USB port 0)
        // ===================================================================
        // Y = toggle auto-shoot on/off
        // X = toggle intake on/off
        // A = jostle intake (hold)
        // B = emergency stop pathfinding
        // Right Trigger = speed boost to 100% (hold, default is 40%)
        // Left Trigger = manual feeder + indexer (hold)
        // Left Bumper = pathfind and follow Left Sweep
        // Right Bumper = pathfind and follow Right Sweep
        // Back = reset field-centric heading
        // D-pad = snap-to-heading (face specific direction while driving)
        // Left Stick = drive (field-centric)
        // Right Stick = rotate
        // ===================================================================

                // Y BUTTON — toggle auto-shoot on/off
        // Press once: aims turret, spins flywheels, feeds when ready, and spins spindexer
        // Press again: stops everything
        controller.y()
                .toggleOnTrue(new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getSmartTarget, this::isHubActive)
                        .alongWith(
                            new StartEndCommand(
                                () -> shooterSubsystem.runIndexer(0.5), // Start the spindexer at 50% speed
                                () -> shooterSubsystem.stopIndexer()    // Stop the spindexer when toggled off
                            )
                        ));

        // LEFT TRIGGER — manual feeder + indexer (hold)
        // While held: runs feeder at 65% and indexer at 50%
        // Releases: stops both
//         controller.leftTrigger()
//         .whileTrue(new StartEndCommand(
//         () -> {
//             shooterSubsystem.runFeeder(0.65);
//             shooterSubsystem.runIndexer(0.50);
//         }, // Start: Run feeder and indexer
//         () -> {
//             shooterSubsystem.stopFeeder();
//             shooterSubsystem.stopIndexer();
//         }, // End: Stop both
//         shooterSubsystem // Subsystem requirement
//     ));

        // LEFT BUMPER — pathfind and follow "Left Sweep"
        controller.leftBumper().onTrue(loadPathfindCommand("Left Sweep"));

        // RIGHT BUMPER — pathfind and follow "Right Sweep"
        controller.rightBumper().onTrue(loadPathfindCommand("Right Sweep"));

        //EMERGENCY STOP PATHFINDING: If the driver needs to take back control during a path, they can press the B button to cancel the current path and drive manually.
        controller.b().onTrue(Commands.runOnce(() -> {
        System.out.println("DRIVER TOOK OVER: Pathfinding Cancelled!");
        }, drivetrain));

        // RIGHT TRIGGER — speed boost (hold)
        // While held: sets speed multiplier to 1.0 (full speed)
        // Released: returns to 0.4 (40% speed — the default)
        controller.rightTrigger()
                .whileTrue(new StartEndCommand(
                        () -> drivetrain.setSpeedMultiplier(1.0),
                        () -> drivetrain.setSpeedMultiplier(0.4)));

        // LEFT TRIGGER — speed reduction (hold)
        // While held: sets speed multiplier to 0.2 (reduced speed)
        // Released: returns to 0.4 (40% speed — the default)
        controller.leftTrigger()
                .whileTrue(new StartEndCommand(
                        () -> drivetrain.setSpeedMultiplier(0.2),
                        () -> drivetrain.setSpeedMultiplier(0.4)));

        // X BUTTON — toggle intake on/off
        // Press once: deploys intake out, starts rollers
        // Press again: retracts intake, stops rollers
        controller.x().toggleOnTrue(
                // --- COMMAND TO START (Deploy & Spin) ---
                new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::deployOut, intakeSubsystem),
                        new WaitCommand(0.3),
                        new InstantCommand(intakeSubsystem::stopDeploy, intakeSubsystem),
                        // This RunCommand keeps the rollers spinning until the button is toggled OFF
                        new RunCommand(() -> intakeSubsystem.runIntake(0.35), intakeSubsystem))
                        .finallyDo((interrupted) -> {
                            // --- COMMAND TO STOP (Retract & Stop) ---
                            // This only runs when the toggle is turned OFF (interrupting the RunCommand)
                            new SequentialCommandGroup(
        new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem),
        new InstantCommand(intakeSubsystem::deployIn, intakeSubsystem),
        new WaitCommand(0.3),
        new InstantCommand(intakeSubsystem::stopDeploy, intakeSubsystem),
        new WaitCommand(0.5),
        // GRAB CURRENT AVG POSITION AND HOLD
        new InstantCommand(() -> {
            double currentAvg = intakeSubsystem.getAverageDeployPosition();
            intakeSubsystem.holdPosition(currentAvg);
            // Optionally force it to brake mode here if it was in coast
        }, intakeSubsystem)).schedule();
        }));

        // A BUTTON — jostle intake to unstick balls
        // While held: continuously repeat the jostle sequence (with a short gap) until
        // released.
        controller.a()
                .whileTrue(new RepeatCommand(intakeSubsystem.jostleCommand()));

        // B BUTTON — (free)

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
        // RIGHT BUMPER — (free — climber removed)

        // --- D-pad (snap-to-heading — face a specific direction while driving) ---
        // While held: drive normally with left stick, but robot heading snaps to the
        // D-pad direction. Releasing returns to normal joystick rotation control.
        // Directions are from the DRIVER'S perspective (field-centric, alliance-aware):
        // Up = face away from driver wall (toward opponent)
        // Down = face toward driver wall
        // Left = face left
        // Right = face right
        //
        // Uses FieldCentricFacingAngle which has its own heading PID controller.
        // This does NOT affect the turret or shooter — only drivetrain rotation.

        // Configure the heading controller PID for snap-to-heading
        // P=4.0 gives a moderate rotation speed — won't whip around violently.
        // D=0.3 dampens oscillation so it doesn't overshoot the target heading.
        facingAngle.HeadingController.setPID(4.0, 0.0, 0.3);
        facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Helper: get the field-relative target heading from a driver-relative angle.
        // On blue alliance, driver faces 0° (toward red wall), so "away" = 0°.
        // On red alliance, driver faces 180° (toward blue wall), so "away" = 180°.
        // The operator perspective offset handles this automatically in the swerve
        // request, so we use fixed angles from the driver's perspective:
        // Up (away) = 0°, Down (toward) = 180°, Left = 90°, Right = -90°

        // Additional helper: a BooleanSupplier that returns true if the driver is trying
        // to manually rotate with the right stick. This is used to interrupt the
        // snap-to-heading command if the driver takes control of rotation.
        // This checks if the driver is trying to manually rotate with the right stick.
        // We use > 0.1 to account for minor stick drift (deadband).
        java.util.function.BooleanSupplier driverTakesOverRotation = 
            () -> Math.abs(controller.getRightX()) > 0.1;

        // Up = face away from driver wall (0°)
        controller.povUp()
                .onTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kZero)))
                // Keeps running UNTIL you touch the right joystick
                .until(driverTakesOverRotation));

        // Down = face toward driver wall (180°)
        controller.povDown()
                .onTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.k180deg)))
                .until(driverTakesOverRotation));

        // Left = face left (90°)
        controller.povLeft()
                .onTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kCCW_90deg)))
                .until(driverTakesOverRotation));

        // Right = face right (-90°)
        controller.povRight()
                .onTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kCW_90deg)))
                .until(driverTakesOverRotation));

        // ===================================================================
        // CO-PILOT CONTROLLER (USB port 1)
        // ===================================================================
        // Turret always auto-tracks the hub. Co-pilot handles live tuning
        // and emergency controls.
        //
        // Y = jostle intake
        // X = (free)
        // A = (free)
        // B = emergency stop shooter
        // Left Bumper = nudge hood up (hold)
        // Right Bumper = nudge hood down (hold)
        // D-pad Left/Right = aim offset ±1°
        // D-pad Up/Down = power offset ±1 RPS
        // Right Stick Press = toggle vision on/off
        // Back = (free)
        // ===================================================================

        // --- Face buttons ---

        // Y — jostle intake
        copilot.y()
                .whileTrue(new RepeatCommand(intakeSubsystem.jostleCommand()));

        // X — (free)

        // A — (free)

        // B — emergency stop all shooter motors
        copilot.b()
                .onTrue(new InstantCommand(() -> shooterSubsystem.stopAll()));

        // --- Bumpers (hood nudge) ---

        // Left Bumper — nudge hood servo UP (hold)
        copilot.leftBumper()
                .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(0.005)));

        // Right Bumper — nudge hood servo DOWN (hold)
        copilot.rightBumper()
                .whileTrue(Commands.run(() -> shooterSubsystem.nudgeHood(-0.005)));

        // --- D-pad (live tuning offsets — works in both modes) ---

        // D-pad Left — nudge turret aim left (CCW) by 1 degree
        copilot.povLeft()
                .onTrue(new InstantCommand(() -> turretSubsystem.nudgeAimLeft()));

        // D-pad Right — nudge turret aim right (CW) by 1 degree
        copilot.povRight()
                .onTrue(new InstantCommand(() -> turretSubsystem.nudgeAimRight()));

        // D-pad Up — increase shooter power by 1 RPS
        copilot.povUp()
                .onTrue(new InstantCommand(() -> shooterSubsystem.nudgePowerUp()));

        // D-pad Down — decrease shooter power by 1 RPS
        copilot.povDown()
                .onTrue(new InstantCommand(() -> shooterSubsystem.nudgePowerDown()));

        // --- Menu buttons ---

        // Right Stick Press — toggle Limelight vision on/off
        // If vision is causing problems during a match, co-pilot can kill it instantly.
        copilot.rightStick()
                .onTrue(new InstantCommand(() -> {
                    visionEnabled = !visionEnabled;
                    System.out.println(">>> VISION: " + (visionEnabled ? "ON" : "OFF") + " <<<");
                }));

        // Back — (free — trajectory toggle removed, blending is automatic) // ===== IDLE BEHAVIOR =====
        // When the robot is disabled, set swerve modules to idle (no power)
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Register telemetry so drivetrain data shows up on the dashboard
        drivetrain.registerTelemetry(logger::telemeterize);

        // NOTE: Operator perspective (for field-centric driving) is handled
        // automatically by CommandSwerveDrivetrain.periodic() using
        // setOperatorPerspectiveForward(). No need to seed it here.
        // For auto, PathPlanner's resetOdom handles the starting pose.

        // ===== DEFAULT COMMANDS =====
        // Default commands run whenever no other command is using that subsystem.

        // Turret default: always auto-aim at the smart target
        turretSubsystem.setDefaultCommand(new RunCommand(
                () -> turretSubsystem.aimAtPose(drivetrain.getState().Pose, getSmartTarget(),
                        drivetrain.getState().Speeds),
                turretSubsystem));

        // Shooter default: flywheels off when not actively shooting.
        // AutoShootCommand requires the shooter subsystem, so this default is
        // automatically interrupted when shooting and resumes when shooting stops.
        shooterSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    shooterSubsystem.stopFlywheels();
                },
                shooterSubsystem));

        // Drivetrain default: drive using joystick input with cubic curve + slew rate
        // limiting
        // The 0.75 and 0.85 multipliers cap max speed for safety during testing
        // (change to 1.0 for full speed at competition)
        //
        // TRENCH SAFETY: When approaching the trench zone with the hood up,
        // auto-slow to 40% to give the servo time to retract before impact.
        // Only affects teleop joystick driving — auto paths are unaffected.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double speedScale = 0.75;

                    // Slow down near trench if hood is raised — need time for servo to retract.
                    // 20% speed if hood is up (aggressive slow to protect hardware).
                    // Normal speed if hood is already flat.
                    double robotX = drivetrain.getState().Pose.getX();
                    double robotY = drivetrain.getState().Pose.getY();
                    boolean nearTrench = (Constants.FieldConstants.isInTrenchZone(robotX)
                            || Constants.FieldConstants.isNearTrenchZone(robotX))
                            && Constants.FieldConstants.isOnTrenchSide(robotY);
                    if (nearTrench
                            && shooterSubsystem.getHoodPosition() > Constants.FieldConstants.TRENCH_HOOD_THRESHOLD) {
                        speedScale = 0.20;
                    }

                    // ✅ GET SPEEDS FROM POSE
                    // This gets the actual physical velocity of the robot from the Pose Estimator
                    ChassisSpeeds currentRobotSpeeds = drivetrain.getState().Speeds;

                    // ✅ PASS TO INTAKE
                    intakeSubsystem.handleAutoRetract(currentRobotSpeeds);
                    // 🔽 APPLY LIMITERS + MULTIPLIER (UNCHANGED LOGIC)
                        double mult = drivetrain.getSpeedMultiplier();

                    return drive
                            .withVelocityX(
                                    xLimiter.calculate(
                                            joystickCurve(-controller.getLeftY()) * speedScale * MaxSpeed * mult))
                            .withVelocityY(
                                    yLimiter.calculate(
                                            joystickCurve(-controller.getLeftX()) * speedScale * MaxSpeed * mult))
                            .withRotationalRate(rotationLimiter
                                    .calculate(joystickCurve(-controller.getRightX()) * 0.85 * MaxAngularRate * mult));
                }));
    }

        /** Returns the selected autonomous command from the dashboard dropdown */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * Returns the hub pose for the current alliance. Used by auto commands
         * that should always aim at the hub regardless of field position.
         */
        public Pose2d getAllianceHub() {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return Constants.FieldConstants.RED_HUB_POSE;
                }
                return Constants.FieldConstants.BLUE_HUB_POSE;
        }

        /**
         * Returns the target to aim at based on alliance color and field position.
         *
         * When the robot is on its own side (inside the alliance zone), aim at the hub.
         * When the robot crosses the alliance line into neutral/opponent territory,
         * switch to a passing target — lob toward our alliance wall, ~4m in from
         * whichever side wall is closer.
         */
        public Pose2d getSmartTarget() {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

                // Safety: if alliance is not set, try to guess from robot position.
                if (!alliance.isPresent()) {
                        double robotX = drivetrain.getState().Pose.getX();
                        isRed = robotX > (Constants.FieldConstants.FIELD_LENGTH_METERS / 2.0);
                }

                double robotX = drivetrain.getState().Pose.getX();
                double robotY = drivetrain.getState().Pose.getY();
                double fieldMidY = Constants.FieldConstants.FIELD_WIDTH_METERS / 2.0;
                double hysteresis = Constants.FieldConstants.PASS_Y_HYSTERESIS;

                // Update left/right side with hysteresis — must cross 1m past midline to flip.
                // If currently targeting left (high Y), only switch to right when Y drops below
                // midline - buffer.
                // If currently targeting right (low Y), only switch to left when Y rises above
                // midline + buffer.
                if (passTargetIsLeft && robotY < fieldMidY - hysteresis) {
                        passTargetIsLeft = false;
                } else if (!passTargetIsLeft && robotY > fieldMidY + hysteresis) {
                        passTargetIsLeft = true;
                }

                if (isRed) {
                        // Red alliance zone: X > RED_ZONE_START (12.51m)
                        // Past alliance line = X < RED_ZONE_START
                        if (robotX < Constants.FieldConstants.RED_ZONE_START) {
                                return passTargetIsLeft
                                                ? Constants.FieldConstants.RED_PASS_LEFT
                                                : Constants.FieldConstants.RED_PASS_RIGHT;
                        }
                        return Constants.FieldConstants.RED_HUB_POSE;
                } else {
                        // Blue alliance zone: X < BLUE_ZONE_END (4.03m)
                        // Past alliance line = X > BLUE_ZONE_END
                        if (robotX > Constants.FieldConstants.BLUE_ZONE_END) {
                                return passTargetIsLeft
                                                ? Constants.FieldConstants.BLUE_PASS_LEFT
                                                : Constants.FieldConstants.BLUE_PASS_RIGHT;
                        }
                        return Constants.FieldConstants.BLUE_HUB_POSE;
                }
        }

        /** Called from Robot.robotPeriodic() — updates calibration values (~10Hz) */
        public void updateCalibrationTelemetry() {
                // Update Field2d every loop (50Hz) so the robot icon moves smoothly
                Pose2d pose = drivetrain.getState().Pose;
                field2d.setRobotPose(pose);

                // Show aim targets on the field map:
                // "Target" = the fixed smart target (hub or pass zone)
                // "Aim Point" = the velocity-compensated point the turret is
                //   actually aiming at. When moving left, this shifts right of
                //   the hub. When stationary, it sits on top of the target.
                Pose2d target = getSmartTarget();
                field2d.getObject("Target").setPose(target);

                Translation2d compensated = turretSubsystem.getCompensatedTarget(
                        pose, target, drivetrain.getState().Speeds);
                field2d.getObject("Aim Point").setPose(
                        new Pose2d(compensated, target.getRotation()));

                telemetryCounter++;
                if (telemetryCounter % 5 != 0)
                        return; // ~10Hz for the rest

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

                SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
                SmartDashboard.putBoolean("Hub Active", isHubActive());
                SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

                // Robot speed for at-rest vs moving diagnostics
                ChassisSpeeds speeds = drivetrain.getState().Speeds;
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                        speeds, pose.getRotation());
                double robotSpeed = Math.hypot(
                        fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
                SmartDashboard.putNumber("Robot Speed m/s", robotSpeed);
                SmartDashboard.putBoolean("At Rest", robotSpeed < 0.1);

                // Zone debug — shows exactly what getSmartTarget decided
                boolean inTrench = Constants.FieldConstants.isInTrenchZone(pose.getX());
                SmartDashboard.putBoolean("In Trench", inTrench);
                SmartDashboard.putString("Target Zone",
                                inTrench ? "TRENCH"
                                                : target.equals(Constants.FieldConstants.BLUE_HUB_POSE) ? "BLUE HUB"
                                                                : target.equals(Constants.FieldConstants.RED_HUB_POSE)
                                                                                ? "RED HUB"
                                                                                : "PASSING");
        }

        /** Returns true if the co-pilot has vision enabled (right stick toggle). */
        public boolean isVisionEnabled() {
                return visionEnabled;
        }

        /**
         * Returns true if our alliance's hub is currently active (or about to be).
         *
         * When no FMS is connected (practice/testing), always returns true so
         * shooting is never blocked during practice.
         *
         * During a real match, uses the game-specific message ('R' or 'B' = alliance
         * whose hub goes inactive first) plus match time to determine the current shift.
         * Returns true PRE_ACTIVATE_SECONDS before the hub actually turns on, since
         * balls in flight still count due to the 3-second scoring delay.
         */
        public boolean isHubActive() {
                String gameMsg = DriverStation.getGameSpecificMessage();
                double matchTime = DriverStation.getMatchTime();

                // No FMS connected (practice/testing) — always active
                if (gameMsg == null || gameMsg.isEmpty() || matchTime < 0) {
                        return true;
                }

                // During auto, hub is always active for both alliances
                if (DriverStation.isAutonomous()) {
                        return true;
                }

                // Transition period (2:20 → 2:10, matchTime 140 → 130): always active
                if (matchTime > 130) {
                        return true;
                }

                // Endgame (last 30s): hub is always active for both alliances
                if (matchTime <= 30) {
                        return true;
                }

                // Determine if we are the "first inactive" alliance (won auto)
                var alliance = DriverStation.getAlliance();
                if (!alliance.isPresent()) return true; // Safety fallback

                boolean isRed = alliance.get() == Alliance.Red;
                boolean weGoInactiveFirst = (isRed && gameMsg.equals("R"))
                                         || (!isRed && gameMsg.equals("B"));

                // Shift boundaries (matchTime counting down):
                //   Shift 1: 130 → 105  (2:10 → 1:45)
                //   Shift 2: 105 →  80  (1:45 → 1:20)
                //   Shift 3:  80 →  55  (1:20 → 0:55)
                //   Shift 4:  55 →   0  (0:55 → 0:00, includes endgame)
                //
                // "First inactive" (won auto): ACTIVE in Shifts 2 and 4
                // "Other" (lost auto):         ACTIVE in Shifts 1 and 3
                int shift;
                if (matchTime > 105) shift = 1;
                else if (matchTime > 80) shift = 2;
                else if (matchTime > 55) shift = 3;
                else shift = 4;

                boolean activeInShift;
                if (weGoInactiveFirst) {
                        activeInShift = (shift == 2 || shift == 4);
                } else {
                        activeInShift = (shift == 1 || shift == 3);
                }

                if (activeInShift) return true;

                // Not active yet — but check if we're within PRE_ACTIVATE_SECONDS
                // of the next activation boundary (balls in flight still count)
                double nextActivation;
                if (weGoInactiveFirst) {
                        // Active in shifts 2 (starts at 105) and 4 (starts at 55)
                        if (matchTime > 105) nextActivation = 105;
                        else if (matchTime > 55 && matchTime <= 80) nextActivation = 55;
                        else return false;
                } else {
                        // Active in shifts 1 (starts at 130) and 3 (starts at 80)
                        if (matchTime > 80 && matchTime <= 105) nextActivation = 80;
                        else return false;
                }

                return matchTime <= nextActivation + PRE_ACTIVATE_SECONDS;
        }

        /**
         * Convert a driver-relative heading to a field-relative heading based on
         * alliance.
         * On blue alliance, the driver faces 0° (toward red wall) — no offset needed.
         * On red alliance, the driver faces 180° — rotate the target by 180°.
         *
         * @param driverRelative the heading from the driver's perspective (0° = away
         *                       from driver)
         * @return field-relative heading for the swerve request
         */
        private Rotation2d getAllianceRotation(Rotation2d driverRelative) {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                if (isRed) {
                        return driverRelative.plus(Rotation2d.k180deg);
                }
                return driverRelative;
        }
}
