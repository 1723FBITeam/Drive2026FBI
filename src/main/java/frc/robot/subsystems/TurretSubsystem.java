package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase {

private final TalonFX turretMotor = new TalonFX(Constants.ShootingConstants.TURRET_MOTOR);
    private final PIDController aimPID = new PIDController(0.1, 0.0, 0.001);

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");

    public void rotate(double speed) {
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /** Called by the command to auto-aim */
    public void aimAtTag() {
        limelightTable.getEntry("pipeline").setDouble(0.0);

        double tx = limelightTable.getEntry("tx").getDouble(0.0);

        // if theres no target it doesnt
        
        if (tx == 0) {
            turretMotor.set(0);
            return;
        } else {
            //double turretSpeed = 0.2;
             double turretSpeed = aimPID.calculate(tx, 0.0);
             turretSpeed = MathUtil.clamp(turretSpeed, -0.2, 0.2);
            turretMotor.set(-turretSpeed);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight TX",
                limelightTable.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Limelight ID Found:",
                limelightTable.getEntry("fID").getDouble(0.0));
    }
}




// SparkMaxConfig config = new SparkMaxConfig();

// config.idleMode(IdleMode.kBrake);
// config.smartCurrentLimit(20);

// turretMotor.configure(config);

// turretMotor.restoreFactoryDefaults();
// turretMotor.setIdleMode (SparkMax.IdleMode.kBrake);
// turretMotor.setSmartCurrentLimit(20);
