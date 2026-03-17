package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
 * Unified shooter subsystem — flywheels now use closed-loop velocity control
 * via Phoenix 6 VelocityVoltage for consistent shot speed regardless of battery state.
 * Feeder and indexer remain duty-cycle controlled.
 */
public class ShooterSubsystem extends SubsystemBase {

  // Shooter flywheels — closed-loop velocity control
  private final TalonFX leftShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_LEFT_MOTOR, Constants.kCANivoreBus);
  private final TalonFX rightShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_RIGHT_MOTOR, Constants.kCANivoreBus);

  // Feeder — duty cycle (doesn't need precise speed)
  private final TalonFX feederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_MOTOR, Constants.kCANivoreBus);

  // Indexer/Spinner — duty cycle
  private final TalonFX indexerMotor = new TalonFX(Constants.ShootingConstants.SPINNER_MOTOR, Constants.kCANivoreBus);

  // Closed-loop velocity request for flywheels — runs at 1kHz on TalonFX
  private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0)
      .withEnableFOC(false); // Set true if you have Pro license

  private final Servo hoodServo;
  private static final double HOOD_MIN = Constants.ShootingConstants.HOOD_MIN;
  private static final double HOOD_MAX = Constants.ShootingConstants.HOOD_MAX;

  // Interpolation tables now map distance → RPS (rotations per second) instead of duty cycle
  // This makes shots repeatable regardless of battery voltage
  private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRPSTable = new InterpolatingDoubleTreeMap();

  // Target RPS for ready-to-shoot detection
  private double targetRPS = 0.0;

  private final DoublePublisher ntHoodPos;
  private final DoublePublisher ntLeftVel;
  private final DoublePublisher ntRightVel;
  private final DoublePublisher ntTargetVel;
  private final BooleanPublisher ntReady;

  public ShooterSubsystem() {
    // Configure flywheel PID + feedforward for velocity control
    // These gains need to be tuned — run SysId on your shooter to get real KS/KV values
    var flywheelGains = new Slot0Configs()
        .withKP(0.1)    // Proportional — corrects velocity error
        .withKI(0.0)    // Integral — leave at 0 initially
        .withKD(0.0)    // Derivative — usually not needed for flywheels
        .withKS(0.1)    // Static friction — voltage to just barely start spinning
        .withKV(0.12);  // Velocity feedforward — volts per RPS to maintain speed

    // Configure shooter motors — inversions for correct spin direction
    MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
    rightConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfigs.NeutralMode = NeutralModeValue.Coast;
    rightShooterMotor.getConfigurator().apply(rightConfigs);
    rightShooterMotor.getConfigurator().apply(flywheelGains);

    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leftConfigs.NeutralMode = NeutralModeValue.Coast;
    leftShooterMotor.getConfigurator().apply(leftConfigs);
    leftShooterMotor.getConfigurator().apply(flywheelGains);

    // Configure feeder motor
    MotorOutputConfigs feederConfigs = new MotorOutputConfigs();
    feederConfigs.Inverted = InvertedValue.Clockwise_Positive;
    feederConfigs.NeutralMode = NeutralModeValue.Coast;
    feederMotor.getConfigurator().apply(feederConfigs);

    // Configure indexer
    MotorOutputConfigs indexerConfigs = new MotorOutputConfigs();
    indexerConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfigs.NeutralMode = NeutralModeValue.Coast;
    indexerMotor.getConfigurator().apply(indexerConfigs);

    // ===== SHOT CALIBRATION TABLES =====
    // Maps distance (meters from hub) → hood position (0.0–1.0) and flywheel speed (RPS)
    //
    // HOW TO CALIBRATE:
    // 1. Press Start to begin shooting (auto-aim sets hood + flywheels from these tables)
    // 2. Check "Dist to Hub" on the Calibration tab to see your distance
    // 3. Use A/B buttons to manually nudge hood until shots land
    // 4. Press D-pad Up to log the shot data (distance + hood position)
    // 5. Update the tables below with your real data points
    //
    // FORMAT: table.put(distanceInMeters, value)
    //   hoodTable:       0.0 = all the way down, 1.0 = all the way up
    //   shooterRPSTable: target flywheel speed in rotations per second
    //
    // To convert from old duty cycle values: RPS ≈ dutyCycle × maxFreeSpeedRPS
    // For Kraken X60: max free speed ≈ 106 RPS (6380 RPM)
    // For Falcon 500: max free speed ≈ 100 RPS (6000 RPM)
    // Hood table: distance in meters → servo position (0.0 = flat, 1.0 = max angle)
    hoodTable.put(1.3, 0.15);
    hoodTable.put(2.1, 0.25);
    hoodTable.put(3.0, 0.40);
    hoodTable.put(3.8, 0.55);
    hoodTable.put(4.7, 0.70);

    // Flywheel table: distance in meters → flywheel speed in RPS (rotations per second)
    shooterRPSTable.put(1.3, 18.0);   // ~1080 RPM — close range
    shooterRPSTable.put(2.1, 24.0);   // ~1440 RPM
    shooterRPSTable.put(3.0, 30.0);   // ~1800 RPM
    shooterRPSTable.put(3.8, 36.0);   // ~2160 RPM
    shooterRPSTable.put(4.7, 42.0);   // ~2520 RPM — long range

    NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration");
    ntHoodPos   = calTable.getDoubleTopic("Hood Position").publish();
    ntLeftVel   = calTable.getDoubleTopic("Shooter L Vel (RPS)").publish();
    ntRightVel  = calTable.getDoubleTopic("Shooter R Vel (RPS)").publish();
    ntTargetVel = calTable.getDoubleTopic("Shooter Target (RPS)").publish();
    ntReady     = calTable.getBooleanTopic("Ready To Shoot").publish();

    // Initialize servo with correct pulse range for REV SRS (270° range)
    hoodServo = new Servo(Constants.ShootingConstants.HOOD_SERVO);
    hoodServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
    hoodServo.set(0.0);
  }

  // ========== SHOOTER FLYWHEELS (Closed-Loop Velocity) ==========

  /** Spin up flywheels to a target speed in rotations per second */
  public void runFlywheelsRPS(double rps) {
    targetRPS = rps;
    leftShooterMotor.setControl(flywheelVelocity.withVelocity(rps));
    rightShooterMotor.setControl(flywheelVelocity.withVelocity(rps));
  }

  /** Legacy method — run flywheels at a duty cycle (for manual/override use) */
  public void runFlywheels(double speed) {
    targetRPS = speed * 106.0; // Approximate RPS for telemetry
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void stopFlywheels() {
    targetRPS = 0.0;
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
  }

  // ========== FEEDER ==========

  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  // ========== INDEXER/SPINNER ==========

  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

  // ========== COMBINED OPERATIONS ==========

  /** Run entire shooting sequence — flywheels at duty cycle, feeder, and indexer */
  public void runFullShooter(double speed) {
    runFlywheels(speed);
    runFeeder(speed);
    runIndexer(speed);
  }

  public void stopAll() {
    stopFlywheels();
    stopFeeder();
    stopIndexer();
  }

  /**
   * Check if flywheels are at target speed.
   * Uses a tolerance of 5% of target RPS for closed-loop,
   * or falls back to minimum threshold for open-loop.
   */
  public boolean isReadyToShoot() {
    double leftVel = Math.abs(leftShooterMotor.getVelocity().getValueAsDouble());
    double rightVel = Math.abs(rightShooterMotor.getVelocity().getValueAsDouble());

    if (targetRPS > 0) {
      // Closed-loop: within 5% of target
      double tolerance = targetRPS * 0.05;
      return Math.abs(leftVel - targetRPS) < tolerance
          && Math.abs(rightVel - targetRPS) < tolerance;
    }
    // Fallback: spinning at least 3 RPS
    return leftVel > 3.0 && rightVel > 3.0;
  }

  // ========== HOOD ==========

  /** Auto-aim hood and flywheels based on distance using interpolation tables */
  public void autoAim(double distanceMeters) {
    double hoodPos = hoodTable.get(distanceMeters);
    double rps = shooterRPSTable.get(distanceMeters);
    setHoodPosition(hoodPos);
    runFlywheelsRPS(rps);
  }

  private double currentHoodPosition = 0.0;

  public void setHoodPosition(double pos) {
    pos = MathUtil.clamp(pos, HOOD_MIN, HOOD_MAX);
    currentHoodPosition = pos;
    hoodServo.set(pos);
  }

  public void nudgeHood(double delta) {
    setHoodPosition(currentHoodPosition + delta);
  }

  public double getHoodPosition() {
    return currentHoodPosition;
  }

  public void setHoodSpeed(double value) {
    currentHoodPosition = value;
    hoodServo.set(value);
  }

  @Override
  public void periodic() {
    ntLeftVel.set(leftShooterMotor.getVelocity().getValueAsDouble());
    ntRightVel.set(rightShooterMotor.getVelocity().getValueAsDouble());
    ntTargetVel.set(targetRPS);
    ntHoodPos.set(currentHoodPosition);
    ntReady.set(isReadyToShoot());
  }
}
