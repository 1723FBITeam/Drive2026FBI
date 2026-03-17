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
        // While disabled, seed the Limelight's internal IMU with our Pigeon heading.
        // Mode 1 = "use external heading only" — this calibrates the Limelight's IMU
        // so it's ready when we enable.
        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Mode 4 = "internal IMU + external assist" — best accuracy for MegaTag2
        // during autonomous when the robot is moving
        LimelightHelpers.SetIMUMode("limelight", 4);

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
        LimelightHelpers.SetIMUMode("limelight", 4);
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
     * Called every robot loop (~50Hz). Here's what happens:
     *   1. We tell the Limelight our current heading (from the Pigeon IMU)
     *      so it can do MegaTag2 calculations
     *   2. The Limelight sees AprilTags and estimates our field position
     *   3. We feed that estimate into the drivetrain's Kalman filter
     *
     * The Kalman filter blends wheel odometry (very smooth but drifts over time)
     * with vision (jumpy but doesn't drift) to get the best of both worlds.
     *
     * We REJECT vision updates when:
     *   - No AprilTags are visible (nothing to calculate from)
     *   - Robot is spinning fast (>360 deg/s) — heading is unreliable
     *
     * Standard deviations tell the filter how much to trust each source:
     *   - XY: 0.7m (moderate trust in vision position)
     *   - Theta: 9999999 (basically ignore vision heading — Pigeon is way better)
     */
    private void updateVisionPoseEstimate() {
        // Tell Limelight our current heading so MegaTag2 can work
        double robotYawDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRateDps = m_robotContainer.drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("limelight",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);

        // Get the Limelight's position estimate (in WPILib blue-origin coordinates)
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        boolean doRejectUpdate = false;

        // No tags visible = no data to use
        if (mt2 == null || mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Spinning too fast = heading is unreliable, vision will be wrong
        if (Math.abs(yawRateDps) > 360) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // Feed vision data to the Kalman filter
            // Trust XY moderately (0.7m std dev), ignore heading entirely (9999999)
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.7, 0.7, 9999999));
            m_robotContainer.drivetrain.addVisionMeasurement(
                mt2.pose, mt2.timestampSeconds);
        }
    }
}
