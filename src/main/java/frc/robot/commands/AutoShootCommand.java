package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.Supplier;

/**
 * AutoShootCommand — the "smart shoot" command that coordinates everything:
 *
 * This command runs continuously (it never finishes on its own) and does:
 *   1. Aims the turret at the hub, compensating for robot movement
 *   2. Calculates distance to the hub (using predicted future position)
 *   3. Sets hood angle and flywheel speed based on that distance
 *   4. Once the turret is aimed AND flywheels are at speed, feeds the note
 *
 * The 0.3-second FEED_DELAY prevents feeding before the flywheels are
 * truly stable (the "ready" check might briefly flicker true).
 *
 * This command is used in two places:
 *   - Y button (toggle on/off during teleop)
 *   - PathPlanner autos (registered as "AutoShoot" named command)
 */
public class AutoShootCommand extends Command {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain drivetrain;

    // Wait this long after "ready" before actually feeding (prevents premature shots)
    private static final double FEED_DELAY = 0.3;
    private double readyTimestamp = 0.0;  // When we first detected "ready to shoot"
    private boolean feeding = false;       // Are we currently feeding a note?

    // Which target to aim at — supplied dynamically so it changes as the robot
    // moves between zones (hub on own side, corners on opponent's side)
    private final Supplier<Pose2d> targetSupplier;
    private Pose2d targetPose = Constants.FieldConstants.BLUE_HUB_POSE;

    /**
     * Creates an AutoShootCommand with a dynamic target supplier.
     * The supplier is called every loop so the target updates as the robot
     * moves between field zones.
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                            CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.turret = turret;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;
        addRequirements(turret, shooter);
    }

    /**
     * Convenience constructor — falls back to alliance-based hub targeting
     * (for PathPlanner autos that don't have access to getSmartTarget).
     */
    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                            CommandSwerveDrivetrain drivetrain) {
        this(turret, shooter, drivetrain, () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return Constants.FieldConstants.RED_HUB_POSE;
            }
            return Constants.FieldConstants.BLUE_HUB_POSE;
        });
    }

    /** Called once when the command starts */
    @Override
    public void initialize() {
        feeding = false;
        readyTimestamp = 0.0;
        targetPose = targetSupplier.get();
    }

    /** Called every 20ms while the command is running */
    @Override
    public void execute() {
        // Update target every loop — it changes as the robot crosses field zones
        targetPose = targetSupplier.get();
        // Get current robot position and velocity from the drivetrain
        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds;

        // STEP 1: Aim the turret with velocity compensation
        // This predicts where the robot will be in 0.2s and aims there instead
        turret.aimAtPose(robotPose, targetPose, fieldSpeeds);

        // STEP 2: Calculate distance using the same compensated target
        // This keeps the distance calculation consistent with where the turret is aiming
        Translation2d compensatedTarget = turret.getCompensatedTarget(robotPose, targetPose, fieldSpeeds);
        double distance = robotPose.getTranslation().getDistance(compensatedTarget);

        // STEP 3: Set hood angle and flywheel speed based on distance
        // If we're in a trench zone, force hood flat and stop flywheels (22.25in clearance)
        boolean inTrench = Constants.FieldConstants.isInTrenchZone(robotPose.getX());
        if (inTrench) {
            shooter.setHoodPosition(0.0);
            shooter.stopFlywheels();
        } else if (distance > 0.5) {
            // autoAim() uses the interpolation tables in ShooterSubsystem
            shooter.autoAim(distance);
        }

        // STEP 4: Check if we're ready to fire
        boolean aimed = turret.isAimedAtPose(robotPose, targetPose);
        boolean flywheelsReady = shooter.isReadyToShoot();
        boolean turretResetting = turret.isResetting();

        // Start the indexer early to keep notes moving toward the feeder
        // BUT NOT while the turret is resetting or in the trench zone
        if (flywheelsReady && !turretResetting && !inTrench) {
            shooter.runIndexer(0.5);
        }

        // Only feed when turret is aimed, flywheels are at speed, NOT resetting, and NOT in trench
        if (aimed && flywheelsReady && !turretResetting && !inTrench) {
            // Start a timer — we wait FEED_DELAY seconds before actually feeding
            // This prevents feeding during brief "ready" flickers
            if (readyTimestamp == 0.0) {
                readyTimestamp = Timer.getFPGATimestamp();
            }
            if (Timer.getFPGATimestamp() - readyTimestamp >= FEED_DELAY) {
                // We've been ready long enough — fire!
                shooter.runFeeder(0.85);
                shooter.runIndexer(0.6);
                feeding = true;
            }
        } else {
            // Not ready — reset the timer and stop feeding
            readyTimestamp = 0.0;
            if (feeding) {
                shooter.stopFeeder();
                shooter.stopIndexer();
                feeding = false;
            }
        }
    }

    /** Called when the command ends (either interrupted or cancelled) */
    @Override
    public void end(boolean interrupted) {
        turret.stop();
        shooter.stopAll();
        shooter.setHoodPosition(0.0); // Flatten hood when not shooting
    }

    /** This command runs forever until cancelled (toggle off or auto timeout) */
    @Override
    public boolean isFinished() {
        return false;
    }
}
