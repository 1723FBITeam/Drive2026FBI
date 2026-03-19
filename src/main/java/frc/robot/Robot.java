// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
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
    private int visionTelemetryCounter = 0;

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
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);

        // Set up vision dashboard publishers
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        ntFrontTags   = visionTable.getIntegerTopic("Front Tags").publish();
        ntBackTags    = visionTable.getIntegerTopic("Back Tags").publish();
        ntFrontDist   = visionTable.getDoubleTopic("Front Avg Dist").publish();
        ntBackDist    = visionTable.getDoubleTopic("Back Avg Dist").publish();
        ntVisionStatus = visionTable.getStringTopic("Status").publish();
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
        // Update calibration dashboard values
        m_robotContainer.updateCalibrationTelemetry();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // While disabled, seed both Limelights' IMUs with our Pigeon heading.
        // Front LL: Mode 1 = seed/calibrate internal IMU from Pigeon heading.
        //   This prepares the front camera's IMU for mode 4 when we enable.
        // Back LL: Mode 0 = ignore internal IMU entirely, use only Pigeon heading.
        //   The back camera is mounted in portrait on the elevator — its IMU
        //   data is unreliable due to the mounting angle and flex.
        LimelightHelpers.SetIMUMode("limelight-front", 1);
        LimelightHelpers.SetIMUMode("limelight-back", 0);

        // NOTE: We do NOT seed heading from MegaTag1 here.
        // MegaTag1 can give bad heading with few tags or on a practice field.
        // Instead, use the Back button on the driver controller to reset heading,
        // or let MegaTag2 gently correct X/Y once enabled.
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Front LL: Mode 4 = fuse internal IMU + external Pigeon assist (best accuracy)
        // Back LL: Mode 0 = external Pigeon heading only (skip unreliable internal IMU)
        LimelightHelpers.SetIMUMode("limelight-front", 4);
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
        // Front LL: Mode 4 = fuse internal IMU + external Pigeon assist
        // Back LL: Mode 0 = external Pigeon heading only
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
     *   - Spinning faster than 150°/s (heading unreliable → MegaTag2 input is bad)
     *   - Driving faster than 3 m/s (camera blur + lag makes estimates noisy)
     *   - Single tag farther than 3m (elevator flex makes these unreliable)
     *   - Pose is obviously off the field
     *
     * WHEN WE ACCEPT BUT DON'T TRUST MUCH:
     *   - Single tag close → high std dev (gentle nudge)
     *   - Multi-tag far → medium std dev
     *   - Multi-tag close → lower std dev (most trusted, but still conservative)
     */
    private void updateVisionPoseEstimate() {
        double robotYawDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRateDps = m_robotContainer.drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();

        // Calculate translational speed from chassis speeds
        var speeds = m_robotContainer.drivetrain.getState().Speeds;
        double translationalSpeed = Math.hypot(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // Reject all vision if spinning too fast — MegaTag2 needs accurate heading
        // 150°/s is conservative: even moderate turns can throw off the estimate
        boolean spinning = Math.abs(yawRateDps) > 150;

        // Reject all vision if driving too fast — camera blur + processing lag
        // means the estimate is for where we WERE, not where we ARE
        boolean drivingFast = translationalSpeed > 3.0;

        // Build a status string for the dashboard so you can see what's happening
        String status;
        if (spinning) {
            status = "REJECTED: spinning " + String.format("%.0f", Math.abs(yawRateDps)) + "°/s";
        } else if (drivingFast) {
            status = "REJECTED: driving " + String.format("%.1f", translationalSpeed) + " m/s";
        } else {
            status = "ACTIVE";
        }

        // Send heading to both cameras so MegaTag2 can work
        // (we always send this, even if we're going to reject the result)
        LimelightHelpers.SetRobotOrientation("limelight-front",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-back",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);

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
        }

        // Only fuse if conditions are good
        if (!spinning && !drivingFast) {
            if (frontEst != null) fuseCameraEstimate(frontEst);
            if (backEst != null) fuseCameraEstimate(backEst);
        }
    }

    /**
     * Process a single camera's pose estimate with conservative filtering.
     *
     * The goal is to GENTLY correct odometry drift without ever making
     * things worse. All std devs are high (1.0+) so vision is always
     * a soft nudge, never a hard correction.
     *
     * On a practice field with only a few tags, you'll mostly get single-tag
     * estimates. These are accepted but barely trusted (std dev 3.0-5.0).
     * That's fine — over many loops, even light corrections add up.
     */
    private void fuseCameraEstimate(LimelightHelpers.PoseEstimate estimate) {
        // No tags visible — nothing to fuse
        if (estimate.tagCount == 0) {
            return;
        }

        // Reject single-tag estimates when the tag is far away.
        // With elevator flex, a single tag beyond 3m is too noisy to trust at all.
        if (estimate.tagCount == 1 && estimate.avgTagDist > 3.0) {
            return;
        }

        // Sanity check: reject poses that are obviously off the field
        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        if (x < -1.0 || x > 17.5 || y < -1.0 || y > 9.5) {
            return;
        }

        // Jump rejection: if vision says we're more than 1m from where odometry
        // thinks we are, something is wrong — reject it. This prevents sudden
        // teleports from a bad tag read. Real drift accumulates slowly, so a
        // 1m gap means the vision estimate is almost certainly wrong.
        double distFromCurrent = m_robotContainer.drivetrain.getState().Pose
            .getTranslation().getDistance(estimate.pose.getTranslation());
        if (distFromCurrent > 1.0) {
            return;
        }

        // Scale trust based on quality — CONSERVATIVE values.
        // Higher std dev = less trust = vision is just a gentle nudge.
        // The robot should drive fine without vision; this only fixes slow drift.
        double xyStdDev;
        if (estimate.tagCount >= 2) {
            // Multi-tag MegaTag2: the most reliable estimate we can get
            if (estimate.avgTagDist < 3.0) {
                xyStdDev = 0.7;  // Close multi-tag — good confidence
            } else if (estimate.avgTagDist < 5.0) {
                xyStdDev = 1.2;  // Medium distance multi-tag
            } else {
                xyStdDev = 2.0;  // Far multi-tag — still okay but less trusted
            }
        } else {
            // Single tag: much less reliable, especially on a flexible mount
            if (estimate.avgTagDist < 1.5) {
                xyStdDev = 2.0;  // Very close single tag — decent
            } else if (estimate.avgTagDist < 2.5) {
                xyStdDev = 3.5;  // Medium single tag — light nudge only
            } else {
                xyStdDev = 5.0;  // Far single tag (2.5-3m) — barely trust it
            }
        }

        // Feed the measurement into the Kalman filter.
        // Rotation std dev = 9999999 means we NEVER trust vision for heading.
        // The Pigeon gyro is always the authority on which way we're facing.
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(
            estimate.pose, estimate.timestampSeconds);
    }
}
