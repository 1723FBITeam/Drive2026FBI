package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Coordinates turret aiming, hood/flywheel auto-aim, and firing.
 * Now includes robot velocity compensation for shoot-on-the-move accuracy.
 *
 * Sequence:
 * 1. Turret aims toward hub using robot pose + velocity compensation
 * 2. Distance is calculated from predicted future position to hub
 * 3. Hood and flywheel speed are set via interpolation tables
 * 4. Once turret is aimed AND flywheels are at speed, feeder + indexer fire
 */
public class AutoShootCommand extends Command {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain drivetrain;

    private static final double FEED_DELAY = 0.3;
    private double readyTimestamp = 0.0;
    private boolean feeding = false;
    private Pose2d targetPose = Constants.FieldConstants.BLUE_HUB_POSE;

    public AutoShootCommand(TurretSubsystem turret, ShooterSubsystem shooter,
                            CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        addRequirements(turret, shooter);
    }

    @Override
    public void initialize() {
        feeding = false;
        readyTimestamp = 0.0;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targetPose = Constants.FieldConstants.RED_HUB_POSE;
        } else {
            targetPose = Constants.FieldConstants.BLUE_HUB_POSE;
        }
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds;

        // 1. Aim turret with velocity compensation for shoot-on-the-move
        turret.aimAtPoseCompensated(robotPose, targetPose, fieldSpeeds);

        // 2. Calculate distance from predicted future position to hub
        // Use same latency compensation as turret for consistent behavior
        double latencySeconds = 0.2;
        Translation2d futurePosition = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * latencySeconds,
                fieldSpeeds.vyMetersPerSecond * latencySeconds));
        double distance = futurePosition.getDistance(targetPose.getTranslation());
        SmartDashboard.putNumber("Auto Shoot Distance", distance);

        // 3. Set hood + flywheel speed based on compensated distance
        if (distance > 0.5) {
            shooter.autoAim(distance);
        }

        // 4. Check if ready to fire
        boolean aimed = turret.isAimedAtPose(robotPose, targetPose);
        boolean flywheelsReady = shooter.isReadyToShoot();
        SmartDashboard.putBoolean("Auto Shoot Aimed", aimed);
        SmartDashboard.putBoolean("Auto Shoot Flywheels", flywheelsReady);

        // Start indexer as soon as flywheels are ready (keeps notes moving)
        if (flywheelsReady) {
            shooter.runIndexer(0.3);
        }

        // Only start feeder when turret is also aimed
        if (aimed && flywheelsReady) {
            if (readyTimestamp == 0.0) {
                readyTimestamp = Timer.getFPGATimestamp();
            }
            if (Timer.getFPGATimestamp() - readyTimestamp >= FEED_DELAY) {
                shooter.runFeeder(0.7);
                shooter.runIndexer(0.3);
                feeding = true;
            }
        } else {
            readyTimestamp = 0.0;
            if (feeding) {
                shooter.stopFeeder();
                shooter.stopIndexer();
                feeding = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        shooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
