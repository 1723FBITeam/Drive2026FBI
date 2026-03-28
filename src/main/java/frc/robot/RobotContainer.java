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
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final frc.robot.subsystems.ClimberSubsystem climber = new frc.robot.subsystems.ClimberSubsystem();
    // ClimberSubsystem — not wired yet, uncomment when ready

    // Telemetry — sends drivetrain data to the dashboard
    private final Telemetry logger = new Telemetry(MaxSpeed);

    PathPlannerPath path;
    {

        try {
            path = PathPlannerPath.fromPathFile("Left Sweep");
        } catch (Exception e) {
            e.printStackTrace();
            path = null;
        }
    }

    Command followPath = null;
    {

        if (path != null) {
            followPath = AutoBuilder.followPath(path);
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

    // ===== TRAJECTORY PHYSICS TOGGLE =====
    // When true, ALL shots (hub and passing) use physics-based trajectory
    // calculations.
    // When false, ALL shots use the interpolation tables.
    // Co-pilot Back button toggles this at runtime.
    // During auto, interpolation tables are always used regardless of this toggle.
    private boolean useTrajectoryPassing = Constants.FieldConstants.USE_TRAJECTORY_PASSING_DEFAULT;

    public RobotContainer() {
        // Wire up the robot X supplier for trench hood safety
        shooterSubsystem.setRobotXSupplier(() -> drivetrain.getState().Pose.getX());

        // ===== NAMED COMMANDS FOR PATHPLANNER =====
        // PathPlanner autonomous routines can trigger these by name.
        // The turret default command already auto-aims at the hub, so no
        // separate turret tracking command is needed.

        // AutoShoot — aims turret at hub, spins flywheels, feeds when ready.
        // Hub-only targeting for auto (driver controls pass targeting in teleop).
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getAllianceHub, () -> false));

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

        // Climb — start the climber level cycle for endgame.
        NamedCommands.registerCommand("Climb", Commands.runOnce(() -> {
            climber.toggleLevelCycle();
        }));

        // ===== AUTO CHOOSER =====
        // Builds a dropdown from all PathPlanner auto files in
        // deploy/pathplanner/autos/
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Publish Field2d to the Calibration tab so it's with the other debug data.
        // In Shuffleboard, drag "Calibration/Field" and set widget type to "Field2d".
        NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration")
                .getSubTable("Field")
                .getEntry(".type").setString("Field2d");
        SmartDashboard.putData("Calibration/Field", field2d);

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
        // The driver only has 4 buttons — drive, shoot, intake, jostle, reset.
        // Everything else is on the co-pilot controller.
        // ===================================================================

        // Y BUTTON — toggle auto-shoot on/off
        // Press once: aims turret, spins flywheels, feeds when ready
        // Press again: stops everything
        controller.y()
                .toggleOnTrue(new AutoShootCommand(turretSubsystem, shooterSubsystem, drivetrain, this::getSmartTarget,
                        this::isTrajectoryPassingEnabled));

        // controller.rightTrigger()
        // .whileTrue(new StartEndCommand(
        // () -> {
        // shooterSubsystem.runFlywheelsRPS(30.0);
        // shooterSubsystem.runFeeder(0.4);
        // shooterSubsystem.runIndexer(0.3);
        // },
        // () -> shooterSubsystem.stopAll(),
        // shooterSubsystem));
        // controller.leftTrigger()
        // .whileTrue(new StartEndCommand(
        // () -> {
        // shooterSubsystem.runFlywheelsRPS(25.0);
        // },
        // () -> {
        // },
        // shooterSubsystem));

        if (followPath != null) {
            controller.leftTrigger().onTrue(followPath);
        }

        controller.leftBumper()
                .whileTrue(new StartEndCommand(
                        () -> drivetrain.setSpeedMultiplier(0.4),
                        () -> drivetrain.setSpeedMultiplier(1.0)));

        // X BUTTON — toggle intake on/off
        // Press once: deploys intake out, starts rollers
        // Press again: retracts intake, stops rollers
        //
        // How this works:
        // 1. Deploy out for 0.3s, then stop deploy motor
        // 2. Run rollers until the button is toggled off (RunCommand keeps going)
        // 3. When cancelled: stop rollers, retract for 0.3s, then stop deploy
        //
        // The retract sequence is part of the command chain (not a fire-and-forget
        // schedule call) so we avoid the deprecated Command.schedule() API.
        // controller.x()
        // .toggleOnTrue(
        // // Phase 1: Deploy out, wait, stop deploy, then run rollers
        // new SequentialCommandGroup(
        // new InstantCommand(() -> intakeSubsystem.deployOut(), intakeSubsystem),
        // new WaitCommand(0.3),
        // new InstantCommand(() -> intakeSubsystem.stopDeploy()),
        // new RunCommand(() -> intakeSubsystem.runIntake(0.5),
        // intakeSubsystem)).finallyDo(() -> {
        // // Immediately stop rollers and start retracting.
        // intakeSubsystem.stopIntake();
        // intakeSubsystem.deployIn();
        // })
        // // Phase 2: After cancel/end, give deploy motor 0.3s to retract, then stop it
        // .andThen(new WaitCommand(0.3))
        // .andThen(new InstantCommand(() -> intakeSubsystem.stopDeploy())));

        controller.x().toggleOnTrue(
                // --- COMMAND TO START (Deploy & Spin) ---
                new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::deployOut, intakeSubsystem),
                        new WaitCommand(0.3),
                        new InstantCommand(intakeSubsystem::stopDeploy, intakeSubsystem),
                        // This RunCommand keeps the rollers spinning until the button is toggled OFF
                        new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem))
                        .finallyDo((interrupted) -> {
                            // --- COMMAND TO STOP (Retract & Stop) ---
                            // This only runs when the toggle is turned OFF (interrupting the RunCommand)
                            new SequentialCommandGroup(
                                    new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem),
                                    new InstantCommand(intakeSubsystem::deployIn, intakeSubsystem),
                                    new WaitCommand(0.3),
                                    new InstantCommand(intakeSubsystem::stopDeploy, intakeSubsystem),
                                    new WaitCommand(0.5),
                                    new InstantCommand(intakeSubsystem::changeConfigs, intakeSubsystem)).schedule(); // .schedule()
                                                                                                                  // allows
                                                                                                                  // this
                                                                                                                  // to
                                                                                                                  // run
                                                                                                                  // even
                                                                                                                  // after
                                                                                                                  // the
                                                                                                                  // parent
                                                                                                                  // is
                                                                                                                  // dead
                        }));

        // A BUTTON — jostle intake to unstick balls
        // While held: continuously repeat the jostle sequence (with a short gap) until
        // released.
        controller.a()
                .whileTrue(new RepeatCommand(intakeSubsystem.jostleCommand()));

        // B BUTTON — toggle elevator between home and preset (index 1)
        // Press once: go to preset target (index 1 = 29.0). Press again: return to home
        // (index 0 = 0.0).
        controller.b()
                .onTrue(new InstantCommand(() -> climber.togglePresetIndex(1)));

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

        controller.povUp()
                .whileTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kZero))));

        controller.povDown()
                .whileTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.k180deg))));

        controller.povLeft()
                .whileTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kCCW_90deg))));

        controller.povRight()
                .whileTrue(drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(xLimiter.calculate(joystickCurve(-controller.getLeftY()) * 0.75 * MaxSpeed))
                        .withVelocityY(yLimiter.calculate(joystickCurve(-controller.getLeftX()) * 0.75 * MaxSpeed))
                        .withTargetDirection(getAllianceRotation(Rotation2d.kCW_90deg))));

        // ===================================================================
        // CO-PILOT CONTROLLER (USB port 1)
        // ===================================================================
        // Turret always auto-tracks the hub. Co-pilot handles tuning,
        // elevator, intake jostle, and emergency stop.
        //
        // Y = jostle intake (hold)
        // X = move elevator up (hold)
        // A = move elevator down (hold)
        // B = emergency stop shooter
        // Bumpers = nudge hood up/down
        // D-pad L/R = aim offset ±1°
        // D-pad U/D = power offset ±1 RPS
        // Right Stick Press = toggle vision on/off
        // Back = toggle trajectory physics (all shots)
        // ===================================================================

        // --- Face buttons ---

        // Y — jostle intake (co-pilot can trigger jostle for the driver)
        copilot.y()
                .onTrue(intakeSubsystem.jostleCommand());

        // X — manually move elevator up slowly (hold to move)
        copilot.x()
                .whileTrue(new edu.wpi.first.wpilibj2.command.StartEndCommand(
                        () -> climber.runElevator(0.25),
                        () -> climber.stopElevator(),
                        climber));

        // A — manually move elevator down slowly (hold to move)
        copilot.a()
                .whileTrue(new edu.wpi.first.wpilibj2.command.StartEndCommand(
                        () -> climber.runElevator(-0.25),
                        () -> climber.stopElevator(),
                        climber));

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

        // Back — toggle trajectory physics for ALL shots (hub and passing)
        // OFF (default): all shots use interpolation tables (safe, proven)
        // ON: all shots use physics-based trajectory (experimental, more accurate at
        // range)
        // During auto, interpolation tables are always used regardless of this toggle.
        copilot.back()
                .onTrue(new InstantCommand(() -> {
                    useTrajectoryPassing = !useTrajectoryPassing;
                    SmartDashboard.putBoolean("Trajectory Passing", useTrajectoryPassing);
                    System.out.println(">>> TRAJECTORY PASSING: " + (useTrajectoryPassing ? "ON" : "OFF") + " <<<");
                })); // ===== IDLE BEHAVIOR =====
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

                    // Slow down near trench if hood is raised
                    double robotX = drivetrain.getState().Pose.getX();
                    boolean nearTrench = Constants.FieldConstants.isInTrenchZone(robotX)
                            || Constants.FieldConstants.isNearTrenchZone(robotX);
                    if (nearTrench
                            && shooterSubsystem.getHoodPosition() > Constants.FieldConstants.TRENCH_HOOD_THRESHOLD) {
                        speedScale = 0.4; // Slow enough for hood servo to retract
                    }

                    // ✅ DEFINE THESE FIRST
                    double xInput = joystickCurve(-controller.getLeftY()) * speedScale * MaxSpeed;
                    double yInput = joystickCurve(-controller.getLeftX()) * speedScale * MaxSpeed;
                    double rotInput = joystickCurve(-controller.getRightX()) * 0.85 * MaxAngularRate;

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

        telemetryCounter++;
        if (telemetryCounter % 5 != 0)
            return; // ~10Hz for the rest

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

        SmartDashboard.putBoolean("Vision Enabled", visionEnabled);

        // Zone debug — shows exactly what getSmartTarget decided
        boolean inTrench = Constants.FieldConstants.isInTrenchZone(pose.getX());
        SmartDashboard.putBoolean("In Trench", inTrench);
        SmartDashboard.putString("Target Zone",
                inTrench ? "TRENCH"
                        : target.equals(Constants.FieldConstants.BLUE_HUB_POSE) ? "BLUE HUB"
                                : target.equals(Constants.FieldConstants.RED_HUB_POSE) ? "RED HUB"
                                        : "PASSING");
    }

    /** Returns true if the co-pilot has vision enabled (right stick toggle). */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    /**
     * Returns true if trajectory physics is enabled for passing shots (co-pilot
     * Back toggle).
     */
    public boolean isTrajectoryPassingEnabled() {
        return useTrajectoryPassing;
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
