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
import frc.robot.TrajectoryCalculations;
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
    private final Supplier<Boolean> hubActiveSupplier;
    private Pose2d targetPose = Constants.FieldConstants.BLUE_HUB_POSE;

    // Tracks which aiming method is currently active (for dashboard display)
    private String shotMethod = "IDLE";

    // ===== LATENCY COMPENSATION =====
    // The hood servo takes ~80ms to reach its commanded position.
    // By predicting where the distance will be 80ms from now and commanding
    // that hood position now, the servo arrives at the right angle by the
    // time the ball feeds. We track the previous distance to estimate the
    // rate of change.
    private static final double HOOD_LATENCY_SECONDS = 0.08; // 80ms servo response
    private double previousDistance = 0.0;

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
    private final DoublePublisher ntShotExitVelocity;  // Physics-predicted ball exit velocity (m/s)
    private final DoublePublisher ntShotHoodDistance;   // Latency-compensated distance used for hood
    private int telemetryCounter = 0;

    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                            CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier,
                            Supplier<Boolean> hubActiveSupplier) {
        this.turret = turret;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;
        this.hubActiveSupplier = hubActiveSupplier;
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
        ntShotExitVelocity = shotTable.getDoubleTopic("Shot Exit Vel (m/s)").publish();
        ntShotHoodDistance = shotTable.getDoubleTopic("Shot Hood Distance").publish();
    }

    @Override
    public void initialize() {
        feeding = false;
        readyTimestamp = 0.0;
        aimedLoopCount = 0;
        previousDistance = 0.0;
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

        // STEP 2: Calculate distances using turret pivot position for consistency
        // with the velocity compensation in TurretSubsystem.
        Translation2d turretFieldPos = turret.getTurretFieldPosition(robotPose);
        double distance = turretFieldPos.getDistance(targetPose.getTranslation());

        // Get velocity component along the turret-to-target axis.
        // Radial speed: positive = closing on target, negative = moving away.
        // Only radial motion affects power — lateral strafing changes aim
        // direction but not how far the ball needs to travel.
        Translation2d toTarget = targetPose.getTranslation().minus(turretFieldPos);
        double distToTarget = toTarget.getNorm();
        double radialSpeed = 0.0;
        if (distToTarget > 0.1) {
            Translation2d unitToTarget = toTarget.div(distToTarget);
            radialSpeed = fieldRelSpeeds.vxMetersPerSecond * unitToTarget.getX()
                        + fieldRelSpeeds.vyMetersPerSecond * unitToTarget.getY();
        }

        // Estimate how much closer/farther we'll be when the ball arrives.
        // Uses the same estimateShotSpeed function as turret velocity compensation
        // so both systems agree on flight time.
        double shotSpeed = TurretSubsystem.estimateShotSpeed(distance);
        double approxFlightTime = distance / shotSpeed;
        double aimDistance = distance - radialSpeed * approxFlightTime * 0.7;
        // Clamp to prevent negative or wildly divergent values
        aimDistance = edu.wpi.first.math.MathUtil.clamp(aimDistance, distance * 0.7, distance * 1.3);

        // Latency compensation: the hood servo takes ~80ms to move. Predict
        // where the distance will be 80ms from now and use that for hood/flywheel
        // commands so the servo arrives at the right angle by the time the ball feeds.
        double distanceRate = (previousDistance > 0.1) ? (aimDistance - previousDistance) / 0.02 : 0.0;
        double hoodDistance = aimDistance + distanceRate * HOOD_LATENCY_SECONDS;
        hoodDistance = edu.wpi.first.math.MathUtil.clamp(hoodDistance, aimDistance * 0.8, aimDistance * 1.2);
        previousDistance = aimDistance;

        // Full compensated target still used for turret aim (includes lateral shift)
        Translation2d compensatedTarget = turret.getCompensatedTarget(robotPose, targetPose, fieldSpeeds);

        // STEP 3: Set hood angle and flywheel speed based on distance
        boolean inTrench = Constants.FieldConstants.isInTrenchZone(robotPose.getX())
                        && Constants.FieldConstants.isOnTrenchSide(robotPose.getY());
        // Also check the wider "near trench" zone on the NEUTRAL side only.
        // When approaching from neutral zone (fast), flatten hood early so the
        // drivetrain speed limiter doesn't slam us to 20%.
        // On the alliance side, we want to shoot as close to the trench as possible,
        // so we only flatten when actually in the trench (inTrench above).
        boolean nearTrenchFromNeutral = Constants.FieldConstants.isNearTrenchZone(robotPose.getX())
                        && !Constants.FieldConstants.isInTrenchZone(robotPose.getX())
                        && Constants.FieldConstants.isOnTrenchSide(robotPose.getY());
        // Detect if we're aiming at a passing target (not the hub).
        boolean isPassing = !targetPose.equals(Constants.FieldConstants.BLUE_HUB_POSE)
                         && !targetPose.equals(Constants.FieldConstants.RED_HUB_POSE);

        // Check if our hub is active (or about to be).
        // When hub is off and we're aiming at the hub (not passing), stop shooting
        // to save power for driving/collecting. Passing is always allowed.
        boolean hubActive = hubActiveSupplier.get();
        boolean hubShotBlocked = !isPassing && !hubActive;

        if (inTrench) {
            // In the trench — hood flat, no shooting
            shooter.setHoodPosition(0.0);
            shooter.stopFlywheels();
            shotMethod = "TRENCH";
        } else if (nearTrenchFromNeutral) {
            // Approaching trench — flatten hood early so drivetrain doesn't slow down,
            // but keep flywheels spinning so we're ready to shoot when we exit
            shooter.setHoodPosition(0.0);
            shotMethod = "NEAR TRENCH";
        } else if (hubShotBlocked) {
            shooter.stopFlywheels();
            shotMethod = "HUB INACTIVE";
        } else if (distance > 0.5) {
            // Blend table and physics based on robot speed.
            // At rest: 100% table (proven, calibrated)
            // At full speed (2+ m/s): 100% physics (adapts to changing distance)
            // In between: smooth blend — no hard switching
            //
            // Exception: passing shots always use 100% physics because the
            // passing tables haven't been calibrated yet. Physics-only is
            // more accurate than blending with uncalibrated table values.
            double physicsWeight = edu.wpi.first.math.MathUtil.clamp(robotSpeed / 2.0, 0.0, 1.0);

            if (isPassing) {
                shooter.blendedPassAutoAim(hoodDistance, 1.0);
                shotMethod = "PASS: PHYSICS";
            } else {
                shooter.blendedHubAutoAim(hoodDistance, physicsWeight);
                shotMethod = String.format("HUB: BLEND %.0f%%", physicsWeight * 100);
            }
        } else {
            // Too close to shoot accurately (< 0.5m) — hold fire
            shooter.setHoodPosition(0.0);
            shooter.stopFlywheels();
            shotMethod = "TOO CLOSE";
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
        if (!resetImminent && !inTrench && !nearTrenchFromNeutral && shooter.getTargetRPS() > 0) {
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
            if (resetImminent || inTrench || nearTrenchFromNeutral || hubShotBlocked) {
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
            if (stableAim && flywheelsReady && !resetImminent && !inTrench && !nearTrenchFromNeutral && !hubShotBlocked) {
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

            // Latency-compensated distance used for hood/flywheel commands
            ntShotHoodDistance.set(hoodDistance);

            // Physics-predicted exit velocity for this distance.
            // Compare this to actual ball speed (radar gun or video) to
            // validate the exitVelocityToRPS conversion factor.
            double heightDiff = isPassing
                ? Constants.FieldConstants.PASSING_TARGET_HEIGHT_METERS - Constants.FieldConstants.TURRET_HEIGHT_METERS
                : Constants.FieldConstants.HUB_TARGET_HEIGHT_METERS - Constants.FieldConstants.TURRET_HEIGHT_METERS;
            double aoa = isPassing
                ? TrajectoryCalculations.getPassingAngleOfAttack(distance)
                : TrajectoryCalculations.getHubAngleOfAttack(distance);
            double[] solution = TrajectoryCalculations.solve(distance, heightDiff, aoa);
            ntShotExitVelocity.set(solution != null ? solution[0] : 0.0);

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
