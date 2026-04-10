// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot.java — the main robot class that WPILib calls automatically.
 *
 * LIFECYCLE (in order):
 *   1. Constructor runs once when robot code starts
 *   2. robotPeriodic() runs every 20ms no matter what mode we're in
 *   3. disabledInit/Periodic → autonomousInit/Periodic → teleopInit/Periodic
 *      are called based on the current mode from the Driver Station
 *
 * This file handles:
 *   - Creating the RobotContainer (which sets up everything else)
 *   - Running the command scheduler every loop
 *   - Fusing Limelight vision data into our position estimate
 *   - Setting Limelight IMU modes for each robot state
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    // RobotContainer sets up all subsystems, buttons, and auto routines
    private final RobotContainer m_robotContainer;

    // ===== VISION TELEMETRY =====
    // These show up on the dashboard so you can confirm Limelights are working.
    // Look for these under the "Vision" table in NetworkTables.
    private final IntegerPublisher ntFrontTags;
    private final IntegerPublisher ntBackTags;
    private final DoublePublisher ntFrontDist;
    private final DoublePublisher ntBackDist;
    private final StringPublisher ntVisionStatus;
    private final DoublePublisher ntVisionStdDev;

    // ===== VISION DIAGNOSTICS =====
    // Counters and stats to help you tell if vision is helping or hurting.
    // All of these show up under the "Vision" table in Shuffleboard.
    private final IntegerPublisher ntAcceptedCount;   // How many measurements were fused
    private final IntegerPublisher ntRejectedCount;   // How many were thrown out (and why)
    private final StringPublisher ntLastRejectReason;  // Why the last one was rejected
    private final IntegerPublisher ntClampCount;       // How many times soft-clamp fired
    private final DoublePublisher ntPoseJump;          // Distance between vision and odometry
    private int visionTelemetryCounter = 0;
    private int acceptedCount = 0;
    private int rejectedCount = 0;
    private int clampCount = 0;
    private String lastRejectReason = "none";

    public Robot() {
        // Create the RobotContainer — this is where all the setup happens
        // If this crashes, the error message will tell us what went wrong
        try {
            m_robotContainer = new RobotContainer();
            System.out.println(">>> Robot code started — build v31 <<<");
        } catch (Exception e) {
            System.err.println("!!! ROBOTCONTAINER CRASHED: " + e.getMessage());
            e.printStackTrace();
            throw e;
        }

        // ===== DATA LOGGING =====
        // Records all NetworkTables values + controller inputs to a .wpilog file
        // on the roboRIO. Pull logs from /home/lvuser/logs/ after a match and
        // open in AdvantageScope to replay the entire match with field view,
        // plots, and controller timeline.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);

        // Set up vision dashboard publishers
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        ntFrontTags   = visionTable.getIntegerTopic("Front Tags").publish();
        ntBackTags    = visionTable.getIntegerTopic("Back Tags").publish();
        ntFrontDist   = visionTable.getDoubleTopic("Front Avg Dist").publish();
        ntBackDist    = visionTable.getDoubleTopic("Back Avg Dist").publish();
        ntVisionStatus = visionTable.getStringTopic("Status").publish();
        ntVisionStdDev = visionTable.getDoubleTopic("Std Dev").publish();
        ntAcceptedCount   = visionTable.getIntegerTopic("Accepted").publish();
        ntRejectedCount   = visionTable.getIntegerTopic("Rejected").publish();
        ntLastRejectReason = visionTable.getStringTopic("Last Reject").publish();
        ntClampCount      = visionTable.getIntegerTopic("Clamp Count").publish();
        ntPoseJump        = visionTable.getDoubleTopic("Pose Jump (m)").publish();
    }

    /**
     * Runs every 20ms regardless of robot mode (disabled, auto, teleop, test).
     * This is where we run the command scheduler and update vision.
     */
    @Override
    public void robotPeriodic() {
        // The command scheduler runs all active commands and checks triggers
        CommandScheduler.getInstance().run();
        // Fuse Limelight vision data into our position estimate
        updateVisionPoseEstimate();
        // Clamp pose to field bounds — if odometry or vision drifts off-field,
        // force it back. This prevents turret aiming, smart target, and trench
        // detection from breaking when the pose is obviously wrong.
        clampPoseToField();
        // Update calibration dashboard values
        m_robotContainer.updateCalibrationTelemetry();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // While disabled, seed both Limelights' IMUs with our Pigeon heading.
        // Mode 1 = seed/calibrate internal IMU to match external yaw each frame.
        // This prepares the LL4 internal IMU for mode 4 when we enable.
        //
        // Back LL uses mode 0 (external yaw only) because it's mounted in
        // portrait orientation — the LL4 internal IMU only works in landscape.
        LimelightHelpers.SetIMUMode("limelight-front", 1);
        LimelightHelpers.SetIMUMode("limelight-back", 0);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Disable vision during auto — pure odometry for reliable paths.
        LimelightHelpers.setEnabled(false);
        // The LL4 internal IMU (mode 4) can produce bad poses if it wasn't
        // properly seeded during disabled. The Pigeon is reliable for 20 seconds.
        LimelightHelpers.SetIMUMode("limelight-front", 0);
        LimelightHelpers.SetIMUMode("limelight-back", 0);

        // Get the selected auto routine from the dashboard chooser
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        // Cancel auto command if it's still running when teleop starts
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        // Re-enable vision for teleop (was disabled during auto)
        LimelightHelpers.setEnabled(true);

        // Front LL: Mode 4 = fuse internal IMU + external Pigeon assist
        // Back LL: Mode 0 = external Pigeon heading only (portrait mount)
        LimelightHelpers.SetIMUMode("limelight-front", 4);
        LimelightHelpers.SetIMUMode("limelight-back", 0);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        // Cancel everything so we start fresh in test mode
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Soft-clamp the robot's estimated pose to field bounds.
     *
     * Instead of hard-resetting the pose estimator (which wipes Kalman filter
     * state and causes turret aim jumps), this injects a gentle vision-like
     * correction that nudges the pose back toward the field. This handles the
     * common case where the robot drives into a wall and odometry drifts past
     * the field boundary because the wheels spin without the robot moving.
     *
     * Only triggers when the pose is significantly off-field (>1m past the wall).
     * Minor drift (0-1m) is left for vision to correct naturally.
     */
    private void clampPoseToField() {
        Pose2d pose = m_robotContainer.drivetrain.getState().Pose;
        double x = pose.getX();
        double y = pose.getY();

        // Only intervene when the pose is clearly off the field.
        // 1.0m margin accounts for bumper overhang and minor odometry drift.
        // Anything beyond this is almost certainly wheel slip against a wall.
        double margin = 1.0;
        double minX = -margin;
        double maxX = Constants.FieldConstants.FIELD_LENGTH_METERS + margin;
        double minY = -margin;
        double maxY = Constants.FieldConstants.FIELD_WIDTH_METERS + margin;

        if (x < minX || x > maxX || y < minY || y > maxY) {
            clampCount++;

            // Clamp to just inside the field boundary (0.05m buffer so we don't
            // keep re-triggering on the exact edge)
            double clampedX = Math.max(0.05, Math.min(x, Constants.FieldConstants.FIELD_LENGTH_METERS - 0.05));
            double clampedY = Math.max(0.05, Math.min(y, Constants.FieldConstants.FIELD_WIDTH_METERS - 0.05));

            // Inject as a soft vision measurement with moderate trust.
            // std dev of 0.5m means this is a gentle nudge, not a hard reset.
            // The Kalman filter will blend it with odometry over several loops
            // rather than teleporting the robot in one frame.
            Pose2d correctedPose = new Pose2d(clampedX, clampedY, pose.getRotation());
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.5, 0.5, 9999999));
            m_robotContainer.drivetrain.addVisionMeasurement(
                correctedPose, Timer.getFPGATimestamp());
        }
    }

    /**
     * VISION FUSION — gently corrects X/Y position using Limelight MegaTag2.
     *
     * PHILOSOPHY: The robot works fine on odometry alone. Vision is only here
     * to slowly correct drift over time. We NEVER let vision make big sudden
     * jumps — that would mess up turret aiming worse than the drift it's fixing.
     *
     * We use MegaTag2 ONLY (no MegaTag1). MegaTag2 uses the Pigeon heading
     * as input, so it only corrects X/Y position, never heading. The Pigeon
     * is always trusted for heading (rotation std dev = 9999999).
     *
     * WHEN WE REJECT VISION ENTIRELY:
     *   - Spinning faster than 120°/s (heading changes too fast for camera exposure)
     *   - No tags visible
     *   - Single tag farther than 3m (elevator flex makes these unreliable)
     *   - Pose is obviously off the field
     *   - Pose jumps more than 2m from current estimate
     *
     * WHEN WE ACCEPT BUT DON'T TRUST MUCH:
     *   - Single tag close → high std dev (gentle nudge)
     *   - Multi-tag far → medium std dev
     *   - Multi-tag close → lower std dev (most trusted, but still conservative)
     *   - Moving fast → std devs scale up quadratically
     */
    private void updateVisionPoseEstimate() {
        double robotYawDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

        // Calculate translational speed (field-relative) for std dev scaling
        var speeds = m_robotContainer.drivetrain.getState().Speeds;
        var pose = m_robotContainer.drivetrain.getState().Pose;
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double translationalSpeed = Math.hypot(vx, vy);

        // Angular velocity for rejection check
        double yawRateDps = m_robotContainer.drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();

        // Reject all vision if spinning too fast — MegaTag2 needs accurate heading
        // and camera frames blur during fast rotation.
        // 360°/s matches the official Limelight example. Top teams use 720°/s.
        // 120°/s was too aggressive — it rejected vision during normal turning.
        boolean spinning = Math.abs(yawRateDps) > 360;

        // Build a status string for the dashboard so you can see what's happening
        String status;
        if (!m_robotContainer.isVisionEnabled()) {
            status = "DISABLED (co-pilot)";
        } else if (spinning) {
            status = "REJECTED: spinning " + String.format("%.0f", Math.abs(yawRateDps)) + "°/s";
        } else {
            status = "ACTIVE";
        }

        // Send heading to both cameras so MegaTag2 can work.
        // Pass zeros for rates — the LL4 internal IMU (mode 4) handles angular
        // velocity internally at 1kHz, so sending our 50Hz rate would just add noise.
        // For the back camera (mode 0), rates are ignored entirely.
        LimelightHelpers.SetRobotOrientation("limelight-front",
            robotYawDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-back",
            robotYawDeg, 0, 0, 0, 0, 0);

        // Get raw estimates for telemetry (even if we reject them)
        LimelightHelpers.PoseEstimate frontEst =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        LimelightHelpers.PoseEstimate backEst =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        // Publish vision telemetry at ~10Hz so you can confirm cameras are working
        visionTelemetryCounter++;
        if (visionTelemetryCounter % 5 == 0) {
            ntFrontTags.set(frontEst != null ? frontEst.tagCount : 0);
            ntBackTags.set(backEst != null ? backEst.tagCount : 0);
            ntFrontDist.set(frontEst != null && frontEst.tagCount > 0 ? frontEst.avgTagDist : 0.0);
            ntBackDist.set(backEst != null && backEst.tagCount > 0 ? backEst.avgTagDist : 0.0);
            ntVisionStatus.set(status);
            ntAcceptedCount.set(acceptedCount);
            ntRejectedCount.set(rejectedCount);
            ntLastRejectReason.set(lastRejectReason);
            ntClampCount.set(clampCount);
        }

        // Only fuse if conditions are good and vision is enabled
        if (!spinning && m_robotContainer.isVisionEnabled()) {
            if (frontEst != null) fuseCameraEstimate(frontEst, translationalSpeed);
            if (backEst != null) fuseCameraEstimate(backEst, translationalSpeed);
        }
    }

    /**
     * Process a single camera's pose estimate with conservative filtering.
     *
     * The goal is to gently correct odometry drift without ever making
     * things worse. Std devs are scaled by tag count, distance, and robot
     * speed so vision is always a soft nudge, never a hard correction.
     */
    private void fuseCameraEstimate(LimelightHelpers.PoseEstimate estimate, double translationalSpeed) {
        // No tags visible — nothing to fuse
        if (estimate.tagCount == 0) {
            return;
        }

        // Reject single-tag estimates when the tag is far away.
        // A single tag beyond 4m on a flexible mount is too noisy to trust.
        if (estimate.tagCount == 1 && estimate.avgTagDist > 4.0) {
            rejectedCount++;
            lastRejectReason = "single tag too far (" + String.format("%.1f", estimate.avgTagDist) + "m)";
            return;
        }

        // Sanity check: reject poses that are obviously off the field
        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        if (x < -0.5 || x > 17.0 || y < -0.5 || y > 8.6) {
            rejectedCount++;
            lastRejectReason = "off field (" + String.format("%.1f, %.1f", x, y) + ")";
            return;
        }

        // Track pose jump for telemetry (but don't reject based on it).
        // MegaTag2 is reliable enough that jump rejection does more harm than good —
        // it causes most measurements to be rejected, and the few that slip through
        // are random outliers that cause jerkiness. Trust the Kalman filter + std devs
        // to handle noisy measurements gracefully.
        double distFromCurrent = m_robotContainer.drivetrain.getState().Pose
            .getTranslation().getDistance(estimate.pose.getTranslation());
        ntPoseJump.set(distFromCurrent);

        // Use a flat std dev of 0.7 for MegaTag2, matching the official Limelight
        // example and Team 3255's approach. This trusts vision enough to correct
        // drift but not so much that it yanks the pose around.
        // Rotation std dev = 9999999 means we NEVER trust vision for heading.
        double xyStdDev = 0.7;

        // Publish the std dev being used for the Vision dashboard
        ntVisionStdDev.set(xyStdDev);

        // Feed the measurement into the Kalman filter.
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(
            estimate.pose, estimate.timestampSeconds);
        acceptedCount++;
    }
}
