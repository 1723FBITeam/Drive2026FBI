package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
 *   2. Calculates distance to the REAL target for hood/flywheel lookup
 *   3. Sets hood angle and flywheel speed based on that distance
 *   4. Once the turret is aimed AND flywheels are at speed, feeds the note
 *
 * FEED DELAY is speed-dependent:
 *   - Stationary (< 0.3 m/s): 0.10s — turret is stable, fire fast
 *   - Moving (>= 0.3 m/s): 0.30s — wait for compensation to settle
 *
 * AIMED LOOP COUNT: turret must be on-target for 3 consecutive loops (~60ms)
 * before feeding is allowed. This prevents firing during brief flickers
 * caused by impacts or vision jumps.
 */
public class AutoShootCommand extends Command {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain drivetrain;

    private double readyTimestamp = 0.0;
    private boolean feeding = false;
    // Consecutive loops the turret has been on-target (prevents firing on flickers)
    private int aimedLoopCount = 0;
    // Minimum consecutive aimed loops before allowing feed (3 loops = ~60ms)
    private static final int MIN_AIMED_LOOPS = 3;

    private final Supplier<Pose2d> targetSupplier;
    private Pose2d targetPose = Constants.FieldConstants.BLUE_HUB_POSE;

    // Tracks which aiming method is currently active (for dashboard display)
    private String shotMethod = "IDLE";

    // ===== SHOT TELEMETRY (for at-rest calibration testing) =====
    private final DoublePublisher ntShotDistance;
    private final DoublePublisher ntShotHood;
    private final DoublePublisher ntShotRPS;
    private final DoublePublisher ntShotSpeed;
    private final BooleanPublisher ntShotAimed;
    private final BooleanPublisher ntShotReady;
    private final BooleanPublisher ntShotFeeding;
    private final StringPublisher ntShotState;
    private final DoublePublisher ntShotTurretError;
    private final DoublePublisher ntShotFeedDelay;
    private final StringPublisher ntShotMethod;
    private final DoublePublisher ntShotCompensationDeg;
    private int telemetryCounter = 0;

    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                            CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.turret = turret;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;
        addRequirements(turret, shooter);

        // Publish shot telemetry under "Shot" table for easy Shuffleboard layout
        NetworkTable shotTable = NetworkTableInstance.getDefault().getTable("Calibration");
        ntShotDistance   = shotTable.getDoubleTopic("Shot Distance").publish();
        ntShotHood       = shotTable.getDoubleTopic("Shot Hood Cmd").publish();
        ntShotRPS        = shotTable.getDoubleTopic("Shot RPS Cmd").publish();
        ntShotSpeed      = shotTable.getDoubleTopic("Shot Robot Speed").publish();
        ntShotAimed      = shotTable.getBooleanTopic("Shot Aimed").publish();
        ntShotReady      = shotTable.getBooleanTopic("Shot Flywheels Ready").publish();
        ntShotFeeding    = shotTable.getBooleanTopic("Shot Feeding").publish();
        ntShotState      = shotTable.getStringTopic("Shot State").publish();
        ntShotTurretError = shotTable.getDoubleTopic("Shot Turret Error Deg").publish();
        ntShotFeedDelay  = shotTable.getDoubleTopic("Shot Feed Delay").publish();
        ntShotMethod     = shotTable.getStringTopic("Shot Method").publish();
        ntShotCompensationDeg = shotTable.getDoubleTopic("Shot Vel Comp Deg").publish();
    }

    @Override
    public void initialize() {
        feeding = false;
        readyTimestamp = 0.0;
        aimedLoopCount = 0;
        targetPose = targetSupplier.get();
    }

    @Override
    public void execute() {
        // Update target every loop — it changes as the robot crosses field zones
        targetPose = targetSupplier.get();
        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds;

        // Robot translational speed (for feed delay and telemetry)
        ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            fieldSpeeds, robotPose.getRotation());
        double robotSpeed = Math.hypot(
            fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond);

        // STEP 1: Aim the turret with velocity compensation
        turret.aimAtPose(robotPose, targetPose, fieldSpeeds);

        // STEP 2: Calculate distances.
        // Real distance: used for target detection (hub vs passing) and telemetry.
        //
        // Power distance: only adjusts for the component of velocity that's
        // toward or away from the target. Lateral movement (strafing) should
        // only affect turret aim direction, NOT power — strafing doesn't change
        // how far the ball needs to travel to reach the hub.
        double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());

        // Get velocity component along the robot-to-target axis
        ChassisSpeeds fieldRelSpeeds2 = ChassisSpeeds.fromRobotRelativeSpeeds(
            fieldSpeeds, robotPose.getRotation());
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        double distToTarget = toTarget.getNorm();
        double radialSpeed = 0.0;
        if (distToTarget > 0.1) {
            // Dot product of velocity and unit vector toward target = closing speed
            // Positive = moving toward target, Negative = moving away
            Translation2d unitToTarget = toTarget.div(distToTarget);
            radialSpeed = fieldRelSpeeds2.vxMetersPerSecond * unitToTarget.getX()
                        + fieldRelSpeeds2.vyMetersPerSecond * unitToTarget.getY();
        }
        // Estimate how much closer/farther we'll be when the ball arrives
        double shotSpeed = (distance < 3.0) ? 5.5 : 4.0;
        double approxFlightTime = distance / shotSpeed;
        double aimDistance = distance - radialSpeed * approxFlightTime * 0.5;
        // Don't let it go negative or wildly different from real distance
        aimDistance = edu.wpi.first.math.MathUtil.clamp(aimDistance, distance * 0.7, distance * 1.3);

        // Full compensated target still used for turret aim (includes lateral shift)
        Translation2d compensatedTarget = turret.getCompensatedTarget(robotPose, targetPose, fieldSpeeds);

        // STEP 3: Set hood angle and flywheel speed based on distance
        boolean inTrench = Constants.FieldConstants.isInTrenchZone(robotPose.getX());
        // Detect if we're aiming at a passing target (not the hub).
        boolean isPassing = !targetPose.equals(Constants.FieldConstants.BLUE_HUB_POSE)
                         && !targetPose.equals(Constants.FieldConstants.RED_HUB_POSE);

        if (inTrench) {
            shooter.setHoodPosition(0.0);
            shooter.stopFlywheels();
            shotMethod = "TRENCH";
        } else if (distance > 0.5) {
            // Blend table and physics based on robot speed.
            // At rest: 100% table (proven, calibrated)
            // At full speed (2+ m/s): 100% physics (adapts to changing distance)
            // In between: smooth blend — no hard switching
            double physicsWeight = edu.wpi.first.math.MathUtil.clamp(robotSpeed / 2.0, 0.0, 1.0);

            if (isPassing) {
                shooter.blendedPassAutoAim(aimDistance, physicsWeight);
                shotMethod = String.format("PASS: BLEND %.0f%%", physicsWeight * 100);
            } else {
                shooter.blendedHubAutoAim(aimDistance, physicsWeight);
                shotMethod = String.format("HUB: BLEND %.0f%%", physicsWeight * 100);
            }
        }

        // STEP 4: Check if we're ready to fire
        boolean aimed = turret.isAimedAtPose(robotPose, targetPose, fieldSpeeds);
        boolean flywheelsReady = shooter.isReadyToShoot();
        boolean turretResetting = turret.isResetting();
        // Check if a reset is about to happen — stop feeder BEFORE the turret starts spinning
        boolean resetImminent = turret.isResetImminent(robotPose, targetPose, fieldSpeeds);

        // Track consecutive aimed loops for the FIRST shot only.
        // Once feeding starts, we don't require re-stabilization.
        if (aimed) {
            aimedLoopCount++;
        } else {
            aimedLoopCount = 0;
        }
        boolean stableAim = aimedLoopCount >= MIN_AIMED_LOOPS;

        // Always run indexer when flywheels are spinning and not in trench/reset
        if (!resetImminent && !inTrench && shooter.getTargetRPS() > 0) {
            shooter.runIndexer(0.5);
        } else if (!feeding) {
            shooter.stopIndexer();
        }

        // Speed-dependent feed delay (only applies to FIRST shot):
        // Stationary: 0.10s — turret is stable, fire fast
        // Moving: 0.30s — wait for velocity compensation to settle
        double feedDelay = robotSpeed < 0.3 ? 0.10 : 0.30;

        // FEEDING LOGIC:
        // To START feeding (first shot): need stable aim + flywheels ready + feed delay elapsed.
        // Once feeding: KEEP feeding. Only stop for hard safety reasons:
        //   - Turret is resetting (big 360° wrap — would fire wildly off-target)
        //   - In trench zone (hood must be flat, can't shoot)
        // Flywheel dips from ball loading are expected — they recover between balls.
        // Brief aim wobbles while moving are OK — the ball is already in the flywheels
        // by the time the turret shifts a degree or two.
        if (feeding) {
            // Already feeding — only stop for hard safety
            if (resetImminent || inTrench) {
                shooter.stopFeeder();
                shooter.stopIndexer();
                feeding = false;
                readyTimestamp = 0.0;
                aimedLoopCount = 0;
            } else {
                shooter.runFeeder(0.65);
                shooter.runIndexer(0.45);
            }
        } else {
            // Not yet feeding — require full ready check for first shot
            if (stableAim && flywheelsReady && !resetImminent && !inTrench) {
                if (readyTimestamp == 0.0) {
                    readyTimestamp = Timer.getFPGATimestamp();
                }
                if (Timer.getFPGATimestamp() - readyTimestamp >= feedDelay) {
                    shooter.runFeeder(0.65);
                    shooter.runIndexer(0.45);
                    feeding = true;
                }
            } else {
                readyTimestamp = 0.0;
            }
        }

        // ===== SHOT TELEMETRY (for calibration and debugging) =====
        telemetryCounter++;
        if (telemetryCounter % 5 == 0) {
            ntShotDistance.set(distance);
            ntShotHood.set(shooter.getHoodPosition());
            ntShotRPS.set(shooter.getTargetRPS());
            ntShotSpeed.set(robotSpeed);
            ntShotAimed.set(aimed);
            ntShotReady.set(flywheelsReady);
            ntShotFeeding.set(feeding);
            ntShotTurretError.set(turret.getAimErrorDegrees());
            ntShotFeedDelay.set(feedDelay);

            // How much velocity compensation is shifting the aim (degrees).
            // 0 when stationary, positive/negative when moving.
            // If shots consistently miss in one direction while moving,
            // this tells you if compensation is too much or too little.
            Translation2d compensated = turret.getCompensatedTarget(robotPose, targetPose, fieldSpeeds);
            double compDistDeg = Math.toDegrees(Math.atan2(
                compensated.getY() - targetPose.getTranslation().getY(),
                compensated.getX() - targetPose.getTranslation().getX()));
            double rawAngle = Math.toDegrees(Math.atan2(
                targetPose.getTranslation().getY() - robotPose.getTranslation().getY(),
                targetPose.getTranslation().getX() - robotPose.getTranslation().getX()));
            double compAngle = Math.toDegrees(Math.atan2(
                compensated.getY() - robotPose.getTranslation().getY(),
                compensated.getX() - robotPose.getTranslation().getX()));
            ntShotCompensationDeg.set(compAngle - rawAngle);

            // Human-readable state for quick glance on dashboard
            String state;
            if (inTrench) {
                state = "TRENCH - NO SHOOT";
            } else if (turretResetting) {
                state = "TURRET RESETTING";
            } else if (!aimed) {
                state = "AIMING...";
            } else if (!flywheelsReady) {
                state = "SPINNING UP...";
            } else if (!stableAim) {
                state = "STABILIZING AIM";
            } else if (readyTimestamp > 0 && Timer.getFPGATimestamp() - readyTimestamp < feedDelay) {
                state = "FEED DELAY";
            } else if (feeding) {
                state = "FIRING";
            } else {
                state = "READY";
            }
            ntShotState.set(state);
            ntShotMethod.set(shotMethod);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        shooter.stopAll();
        shooter.setHoodPosition(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
