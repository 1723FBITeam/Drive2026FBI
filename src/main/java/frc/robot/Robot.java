// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
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

    public Robot() {
        // Create the RobotContainer — this is where all the setup happens
        // If this crashes, the error message will tell us what went wrong
        try {
            m_robotContainer = new RobotContainer();
            System.out.println(">>> Robot code started — build v30 <<<");
        } catch (Exception e) {
            System.err.println("!!! ROBOTCONTAINER CRASHED: " + e.getMessage());
            e.printStackTrace();
            throw e;
        }
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);
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
        // Mode 1 = "use external heading only" — calibrates the IMU so it's ready when we enable.
        LimelightHelpers.SetIMUMode("limelight-front", 1);
        LimelightHelpers.SetIMUMode("limelight-back", 1);

        // VISION HEADING SEED:
        // While disabled, if a Limelight gets a good multi-tag solve,
        // use that heading to seed the Pigeon gyro. This means you can
        // place the robot at ANY angle and it will figure out its heading
        // from the AprilTags before the match starts.
        seedHeadingFromVision("limelight-front");
        seedHeadingFromVision("limelight-back");
    }

    /**
     * If the camera sees multiple tags, use MegaTag1 to get heading.
     * MegaTag1 doesn't need a heading input — it solves purely from
     * tag geometry, so it works even when the gyro heading is totally wrong.
     * Only runs while disabled so it doesn't fight the gyro during a match.
     */
    private void seedHeadingFromVision(String limelightName) {
        // Use MegaTag1 (NOT MegaTag2) because MegaTag1 can solve heading
        // from scratch using multiple tags. MegaTag2 needs a good heading
        // input which we don't have yet.
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount < 2) {
            return; // Only trust multi-tag solves for heading
        }

        // Sanity check — reject obviously bad poses
        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        if (x < -1.0 || x > 17.5 || y < -1.0 || y > 9.5) {
            return;
        }

        // Seed the drivetrain with the vision-derived pose (including heading)
        m_robotContainer.drivetrain.resetPose(estimate.pose);
        System.out.println(">>> VISION SEED from " + limelightName
            + ": heading=" + String.format("%.1f", estimate.pose.getRotation().getDegrees())
            + "° pos=(" + String.format("%.2f", x) + ", " + String.format("%.2f", y) + ")"
            + " tags=" + estimate.tagCount + " <<<");
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Mode 4 = "internal IMU + external assist" — best accuracy for MegaTag2
        LimelightHelpers.SetIMUMode("limelight-front", 4);
        LimelightHelpers.SetIMUMode("limelight-back", 4);

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
        // Same Limelight mode as auto — best accuracy for vision fusion
        LimelightHelpers.SetIMUMode("limelight-front", 4);
        LimelightHelpers.SetIMUMode("limelight-back", 4);
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
     * VISION FUSION — combines Limelight camera data with wheel odometry.
     *
     * We run two Limelights (front + back) for full-field AprilTag coverage.
     * Each camera independently estimates our position, and both get fused
     * into the drivetrain's Kalman filter.
     *
     * IMPORTANT: Both cameras are mounted on the ELEVATOR, and the belly pan
     * is not very rigid. This means:
     *   - The cameras vibrate/flex more than a rigid frame mount
     *   - Single-tag estimates from far away are unreliable
     *   - We need aggressive filtering to reject bad frames
     *   - We scale trust based on how many tags we see and how far away they are
     *
     * Every loop (~50Hz):
     *   1. Send our current heading to BOTH Limelights (needed for MegaTag2)
     *   2. Get each camera's pose estimate
     *   3. Reject bad updates (spinning, too far, single-tag at distance, etc.)
     *   4. Scale trust based on tag count and distance
     *   5. Feed good updates into the Kalman filter
     */
    private void updateVisionPoseEstimate() {
        double robotYawDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRateDps = m_robotContainer.drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();

        // Reject all vision if spinning too fast — heading is unreliable
        // Lowered from 360 to 270 because elevator flex amplifies errors during turns
        boolean spinning = Math.abs(yawRateDps) > 270;

        // Send heading to both cameras so MegaTag2 can work
        LimelightHelpers.SetRobotOrientation("limelight-front",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-back",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);

        // Fuse front camera
        if (!spinning) {
            fuseCameraEstimate("limelight-front");
        }

        // Fuse back camera
        if (!spinning) {
            fuseCameraEstimate("limelight-back");
        }
    }

    /**
     * Process a single camera's pose estimate with quality filtering.
     *
     * Because the cameras are on an elevator (flexible mount), we:
     *   1. Reject single-tag estimates when the tag is far away (>4m) — too noisy
     *   2. Scale trust based on tag count and average distance:
     *      - Multi-tag close up → trust a lot (std dev 0.5)
     *      - Multi-tag far away → trust moderately (std dev 1.0)
     *      - Single tag close  → trust lightly (std dev 1.5)
     *      - Single tag medium → trust very lightly (std dev 2.5)
     *   3. Reject any estimate where the pose is obviously off the field
     *
     * This prevents the turret from twitching when a single far-away tag
     * gives a bad reading due to elevator vibration or belly pan flex.
     */
    private void fuseCameraEstimate(String limelightName) {
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // No data or no tags visible — skip
        if (estimate == null || estimate.tagCount == 0) {
            return;
        }

        // Reject single-tag estimates when the tag is far away.
        // With elevator flex, a single tag at 4+ meters gives unreliable pose.
        if (estimate.tagCount == 1 && estimate.avgTagDist > 4.0) {
            return;
        }

        // Sanity check: reject poses that are obviously off the field
        // (can happen with bad single-tag solves or partially occluded tags)
        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        if (x < -1.0 || x > 17.5 || y < -1.0 || y > 9.5) {
            return;
        }

        // Scale trust based on how good this measurement is.
        // More tags + closer = lower std dev = more trust.
        // Fewer tags + farther = higher std dev = less trust.
        double xyStdDev;
        if (estimate.tagCount >= 2) {
            // Multi-tag: much more reliable, especially with MegaTag2
            if (estimate.avgTagDist < 3.0) {
                xyStdDev = 0.5;  // Close multi-tag — very trustworthy
            } else if (estimate.avgTagDist < 5.0) {
                xyStdDev = 0.8;  // Medium distance multi-tag — still good
            } else {
                xyStdDev = 1.2;  // Far multi-tag — decent but not great
            }
        } else {
            // Single tag: less reliable, especially on a flexible mount
            if (estimate.avgTagDist < 2.0) {
                xyStdDev = 1.5;  // Close single tag — okay
            } else if (estimate.avgTagDist < 3.5) {
                xyStdDev = 2.5;  // Medium single tag — light trust
            } else {
                xyStdDev = 4.0;  // Far single tag (3.5-4m) — barely trust it
            }
        }

        // Feed the measurement into the Kalman filter
        // Rotation std dev stays at 9999999 — we always trust the Pigeon for heading
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(
            estimate.pose, estimate.timestampSeconds);
    }
}
