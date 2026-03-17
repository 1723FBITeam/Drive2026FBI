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

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
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

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        updateVisionPoseEstimate();
        m_robotContainer.updateCalibrationTelemetry();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Seed Limelight's internal IMU with our Pigeon heading while disabled
        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Switch Limelight to internal IMU + external assist for best MT2 accuracy
        LimelightHelpers.SetIMUMode("limelight", 4);

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
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        // Switch Limelight to internal IMU + external assist for best MT2 accuracy
        LimelightHelpers.SetIMUMode("limelight", 4);
        // NOTE: Removed hardcoded resetPose — vision fusion handles pose correction now
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Fuses Limelight MegaTag2 pose estimates into the drivetrain's Kalman filter.
     * Called every robot loop (~50Hz). MegaTag2 requires us to feed it our current
     * heading via SetRobotOrientation each frame.
     *
     * We reject vision updates when:
     * - No tags are visible
     * - Robot is spinning fast (>360°/s) — heading is unreliable during fast rotation
     *
     * Standard deviations: we trust vision XY (0.7m) but ignore vision theta
     * (9999999) because our Pigeon IMU heading is far more accurate.
     */
    private void updateVisionPoseEstimate() {
        // Feed our current heading to Limelight for MegaTag2 calculations
        double robotYawDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRateDps = m_robotContainer.drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("limelight",
            robotYawDeg, yawRateDps, 0, 0, 0, 0);

        // Get MegaTag2 pose estimate (always use wpiBlue origin)
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        boolean doRejectUpdate = false;

        if (mt2 == null || mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Reject during fast spins — MegaTag2 needs stable heading
        if (Math.abs(yawRateDps) > 360) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // Trust vision XY moderately, ignore vision heading entirely
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.7, 0.7, 9999999));
            m_robotContainer.drivetrain.addVisionMeasurement(
                mt2.pose, mt2.timestampSeconds);
        }
    }
}
