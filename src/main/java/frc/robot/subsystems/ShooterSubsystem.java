package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


/**
 * Unified shooter subsystem - controls flywheels, feeder, and indexer as a coordinated unit.
 * Turret aiming is handled separately in TurretSubsystem for independent tracking.
 */
public class ShooterSubsystem extends SubsystemBase {

  // Shooter flywheels
  private final TalonFX leftShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_LEFT_MOTOR, Constants.kCANivoreBus);
  private final TalonFX rightShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_RIGHT_MOTOR, Constants.kCANivoreBus);
  
  // Feeder - feeds notes into shooter
  private final TalonFX feederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_MOTOR, Constants.kCANivoreBus);
  
  // Indexer/Spinner - positions notes for feeding
  private final TalonFX indexerMotor = new TalonFX(Constants.ShootingConstants.SPINNER_MOTOR, Constants.kCANivoreBus);
  
  private final DutyCycleOut shooterControl = new DutyCycleOut(0);

  private final Servo hoodServo;
  private static final double HOOD_MIN = Constants.ShootingConstants.HOOD_MIN;
  private static final double HOOD_MAX = Constants.ShootingConstants.HOOD_MAX;
  private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  private final DoublePublisher ntHoodPos;
  private final DoublePublisher ntLeftVel;
  private final DoublePublisher ntRightVel;
  private final BooleanPublisher ntReady;

  
  public ShooterSubsystem() {
    // Configure shooter motors - inversions swapped to fix direction
    MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
    rightConfigs.Inverted = InvertedValue.CounterClockwise_Positive;  // Flipped
    rightConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    rightShooterMotor.getConfigurator().apply(rightConfigs);

    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.Clockwise_Positive;  // Flipped
    leftConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    leftShooterMotor.getConfigurator().apply(leftConfigs);

    // Configure feeder motor - opposite of shooter motors
    MotorOutputConfigs feederConfigs = new MotorOutputConfigs();
    feederConfigs.Inverted = InvertedValue.Clockwise_Positive;  // Separate config for feeder
    feederConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    feederMotor.getConfigurator().apply(feederConfigs);

    // Configure indexer
    MotorOutputConfigs indexerConfigs = new MotorOutputConfigs();
    indexerConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    indexerMotor.getConfigurator().apply(indexerConfigs);

    // ===== SHOT CALIBRATION TABLES =====
    // Maps distance (meters from hub) → hood position (0.0–1.0) and flywheel speed (0.0–1.0)
    // 
    // HOW TO CALIBRATE:
    // 1. Press Start to begin shooting (auto-aim sets hood + flywheels from these tables)
    // 2. Check "Dist to Hub" on the Calibration tab to see your distance
    // 3. Use A/B buttons to manually nudge hood until shots land
    // 4. Press D-pad Up to log the shot data (distance + hood position)
    // 5. Update the tables below with your real data points
    //
    // FORMAT: table.put(distanceInMeters, value)
    //   hoodTable:    0.0 = all the way down, 1.0 = all the way up
    //   shooterTable: 0.0 = stopped, 1.0 = full speed
    //
    // Tips:
    // - More data points = smoother interpolation between distances
    // - If overshooting: lower hood value or lower flywheel speed at that distance
    // - If undershooting: raise hood value or raise flywheel speed at that distance
    // - The table interpolates linearly between points, so 2.5m will blend 2.1 and 3.0 values
    hoodTable.put(1.3, 0.15);
    hoodTable.put(2.1, 0.25);
    hoodTable.put(3.0, 0.40);
    hoodTable.put(3.8, 0.55);
    hoodTable.put(4.7, 0.70);

    shooterTable.put(1.3, 0.35);
    shooterTable.put(2.1, 0.42);
    shooterTable.put(3.0, 0.52);
    shooterTable.put(3.8, 0.62);
    shooterTable.put(4.7, 0.75);

    NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration");
    ntHoodPos  = calTable.getDoubleTopic("Hood Position").publish();
    ntLeftVel  = calTable.getDoubleTopic("Shooter L Vel").publish();
    ntRightVel = calTable.getDoubleTopic("Shooter R Vel").publish();
    ntReady    = calTable.getBooleanTopic("Ready To Shoot").publish();

    // Initialize servo with correct pulse range for REV SRS (270° range)
    hoodServo = new Servo(Constants.ShootingConstants.HOOD_SERVO);
    hoodServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
    hoodServo.set(0.0);
  }

  // ========== SHOOTER FLYWHEELS ==========
  
  /** Spin up shooter flywheels only (-1.0 to 1.0) */
  public void runFlywheels(double speed) {
    shooterControl.Output = speed;
    leftShooterMotor.setControl(shooterControl);
    rightShooterMotor.setControl(shooterControl);
  }

  public void stopFlywheels() {
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
  }

  // ========== FEEDER ==========
  
  /** Run feeder to push note into shooter (-1.0 to 1.0) */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  // ========== INDEXER/SPINNER ==========
  
  /** Run indexer to position notes (-1.0 to 1.0) */
  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

  // ========== COMBINED OPERATIONS ==========
  
  /** Run entire shooting sequence - flywheels, feeder, and indexer */
  public void runFullShooter(double speed) {
    runFlywheels(speed);
    runFeeder(speed);
    runIndexer(speed);
  }

  /** Stop all shooter components */
  public void stopAll() {
    stopFlywheels();
    stopFeeder();
    stopIndexer();
  }

  /** Check if flywheels are at target speed using velocity threshold */
  public boolean isReadyToShoot() {
    double leftVel = Math.abs(leftShooterMotor.getVelocity().getValueAsDouble());
    double rightVel = Math.abs(rightShooterMotor.getVelocity().getValueAsDouble());
    // Flywheels need to be spinning at least 3 rps to be considered ready
    return leftVel > 3.0 && rightVel > 3.0;
  }

  // ========== HOOD ==========

  /** Auto-aim hood and flywheels based on distance using interpolation tables */
  public void autoAim(double distanceMeters) {
    double hoodPos = hoodTable.get(distanceMeters);
    double shooterSpeed = shooterTable.get(distanceMeters);
    setHoodPosition(hoodPos);
    runFlywheels(shooterSpeed);
  }

  private double currentHoodPosition = 0.0; // Start at bottom

  /** Set hood position (clamped to HOOD_MIN..HOOD_MAX) */
  public void setHoodPosition(double pos) {
    pos = MathUtil.clamp(pos, HOOD_MIN, HOOD_MAX);
    currentHoodPosition = pos;
    hoodServo.set(pos);
  }

  /** Nudge hood position by a delta amount (positive = up, negative = down) */
  public void nudgeHood(double delta) {
    setHoodPosition(currentHoodPosition + delta);
  }

  /** Get current hood position for calibration readout */
  public double getHoodPosition() {
    return currentHoodPosition;
  }

  /**
   * Set hood servo raw value — for continuous mode servos:
   * 0.5 = stop, >0.5 = one direction, <0.5 = other direction
   */
  public void setHoodSpeed(double value) {
    currentHoodPosition = value;
    hoodServo.set(value);
  }

  @Override
  public void periodic() {
    ntLeftVel.set(leftShooterMotor.getVelocity().getValueAsDouble());
    ntRightVel.set(rightShooterMotor.getVelocity().getValueAsDouble());
    ntHoodPos.set(currentHoodPosition);
    ntReady.set(isReadyToShoot());
  }
}
