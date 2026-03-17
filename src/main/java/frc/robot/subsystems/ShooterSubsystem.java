package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 * ShooterSubsystem — controls everything needed to score a note:
 *   - Two flywheel motors (spin up to launch the note)
 *   - A feeder motor (pushes the note into the flywheels)
 *   - An indexer/spinner motor (queues notes before feeding)
 *   - A hood servo (adjusts the launch angle up/down)
 *
 * The flywheels use CLOSED-LOOP velocity control, meaning we tell them
 * "spin at 30 rotations per second" and the motor controller adjusts voltage
 * automatically to maintain that speed. This makes shots consistent even
 * as the battery drains during a match.
 *
 * The hood angle and flywheel speed are set automatically based on distance
 * to the hub using interpolation tables (see the calibration section below).
 */
public class ShooterSubsystem extends SubsystemBase {

  // ===== MOTORS =====
  // Two flywheel motors spin in opposite directions to launch the note
  private final TalonFX leftShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_LEFT_MOTOR, Constants.kCANivoreBus);
  private final TalonFX rightShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_RIGHT_MOTOR, Constants.kCANivoreBus);

  // Feeder motor — pushes notes from the indexer into the spinning flywheels
  private final TalonFX feederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_MOTOR, Constants.kCANivoreBus);

  // Indexer/spinner motor — moves notes into position before feeding
  private final TalonFX indexerMotor = new TalonFX(Constants.ShootingConstants.SPINNER_MOTOR, Constants.kCANivoreBus);

  // ===== CONTROL REQUESTS =====
  // VelocityVoltage tells the motor "maintain this speed (in rotations per second)"
  // The motor's built-in PID loop runs at 1000Hz (1000 times per second) to hold that speed
  // FOC (Field Oriented Control) is disabled — enable it if you have a CTRE Pro license
  private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0)
      .withEnableFOC(false);

  // ===== HOOD SERVO =====
  // The hood is a servo that tilts the shooter up/down to change launch angle
  // Position 0.0 = flat (low angle), 1.0 = tilted up (high angle)
  private final Servo hoodServo;
  private static final double HOOD_MIN = Constants.ShootingConstants.HOOD_MIN;
  private static final double HOOD_MAX = Constants.ShootingConstants.HOOD_MAX;

  // ===== INTERPOLATION TABLES =====
  // These are lookup tables: you give them a distance, they give back a value.
  // Between known data points, they estimate (interpolate) the right value.
  // Example: if 2m → hood 0.25 and 4m → hood 0.55, then 3m → hood ~0.40
  private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRPSTable = new InterpolatingDoubleTreeMap();

  // Tracks what speed we asked the flywheels to spin at (for ready-to-shoot check)
  private double targetRPS = 0.0;

  // ===== POWER OFFSET (adjusted by co-pilot controller 2) =====
  // This offset is added to every flywheel RPS command.
  // D-pad up/down on controller 2 nudges this value during the match.
  // Positive = more power (shots go further), Negative = less power (shots fall shorter).
  private double rpsOffset = 0.0;
  // Each D-pad press nudges by 1 RPS (~60 RPM)
  private static final double RPS_NUDGE = 1.0;

  // ===== DASHBOARD TELEMETRY =====
  // These publish values to Shuffleboard so we can see what's happening in real time
  private final DoublePublisher ntHoodPos;
  private final DoublePublisher ntLeftVel;
  private final DoublePublisher ntRightVel;
  private final DoublePublisher ntTargetVel;
  private final BooleanPublisher ntReady;
  private final DoublePublisher ntRpsOffset;

  public ShooterSubsystem() {
    // --- PID + Feedforward gains for flywheel velocity control ---
    // These numbers tell the motor controller HOW to maintain the target speed.
    // KP: "If I'm 1 RPS too slow, add this much voltage" — increase until it responds fast but doesn't oscillate
    // KI: "If I've been off-target for a while, add more" — usually leave at 0
    // KD: "If the error is changing fast, slow down" — usually not needed for flywheels
    // KS: Voltage needed to overcome friction (motor won't spin below this)
    // KV: Voltage per RPS to maintain speed (the main feedforward term)
    // TIP: Run SysId characterization to find real KS/KV values for your specific shooter
    var flywheelGains = new Slot0Configs()
        .withKP(0.1)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.1)
        .withKV(0.12);

    // --- Motor configurations ---
    // Each motor needs to know: which direction is "positive" and what to do when stopped
    // The left and right flywheels spin opposite directions (one CW, one CCW) so the
    // note gets launched straight out instead of curving

    // Right flywheel: CounterClockwise = positive direction, Coast when stopped
    MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
    rightConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfigs.NeutralMode = NeutralModeValue.Coast; // Coast = spin freely when stopped (less wear)
    rightShooterMotor.getConfigurator().apply(rightConfigs);
    rightShooterMotor.getConfigurator().apply(flywheelGains);

    // Left flywheel: Clockwise = positive direction (opposite of right)
    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leftConfigs.NeutralMode = NeutralModeValue.Coast;
    leftShooterMotor.getConfigurator().apply(leftConfigs);
    leftShooterMotor.getConfigurator().apply(flywheelGains);

    // Feeder motor config
    MotorOutputConfigs feederConfigs = new MotorOutputConfigs();
    feederConfigs.Inverted = InvertedValue.Clockwise_Positive;
    feederConfigs.NeutralMode = NeutralModeValue.Coast;
    feederMotor.getConfigurator().apply(feederConfigs);

    // Indexer motor config
    MotorOutputConfigs indexerConfigs = new MotorOutputConfigs();
    indexerConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfigs.NeutralMode = NeutralModeValue.Coast;
    indexerMotor.getConfigurator().apply(indexerConfigs);

    // ===== SHOT CALIBRATION TABLES =====
    // These tables map distance (meters from hub) to hood position and flywheel speed.
    //
    // HOW TO CALIBRATE:
    // 1. Drive to a known distance from the hub
    // 2. Press Start to begin auto-shoot (sets hood + flywheels from these tables)
    // 3. Check "Dist to Hub" on the Calibration tab in Shuffleboard
    // 4. Use A/B buttons to manually nudge hood until shots land consistently
    // 5. Press D-pad Up to log the shot data (distance + hood position)
    // 6. Update the tables below with your real data points
    //
    // FORMAT: table.put(distanceInMeters, value)

    // Hood table: distance → servo position (0.0 = flat, 1.0 = max angle up)
    hoodTable.put(1.3, 0.13);
    hoodTable.put(2.1, 0.21);
    hoodTable.put(3.0, 0.30);
    hoodTable.put(3.8, 0.38);
    hoodTable.put(4.7, 0.47);

    // Flywheel speed table: distance → speed in Rotations Per Second (RPS)
    shooterRPSTable.put(1.3, 18.9);   // ~1134 RPM
    shooterRPSTable.put(2.1, 24.6);   // ~1476 RPM
    shooterRPSTable.put(3.0, 28.0);   // ~1680 RPM
    shooterRPSTable.put(3.8, 32.5);   // ~1950 RPM
    shooterRPSTable.put(4.7, 37.0);   // ~2220 RPM

    // Set up dashboard publishers for the Calibration tab
    NetworkTable calTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Calibration");
    ntHoodPos   = calTable.getDoubleTopic("Hood Position").publish();
    ntLeftVel   = calTable.getDoubleTopic("Shooter L Vel (RPS)").publish();
    ntRightVel  = calTable.getDoubleTopic("Shooter R Vel (RPS)").publish();
    ntTargetVel = calTable.getDoubleTopic("Shooter Target (RPS)").publish();
    ntReady     = calTable.getBooleanTopic("Ready To Shoot").publish();
    ntRpsOffset = calTable.getDoubleTopic("Shooter RPS Offset").publish();

    // Initialize the hood servo with the correct pulse width range for REV Smart Robot Servo
    // The pulse range (500-2500 microseconds) defines the servo's full range of motion (270°)
    hoodServo = new Servo(Constants.ShootingConstants.HOOD_SERVO);
    hoodServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
    hoodServo.set(0.0); // Start flat
  }

  // ==================== FLYWHEEL CONTROLS ====================

  /**
   * Spin up flywheels to a target speed using closed-loop velocity control.
   * The co-pilot's RPS offset is added automatically.
   * @param rps Target speed in Rotations Per Second (e.g., 30.0 = 1800 RPM)
   */
  public void runFlywheelsRPS(double rps) {
    double adjustedRPS = rps + rpsOffset;
    if (adjustedRPS < 0) adjustedRPS = 0; // Don't spin backwards
    targetRPS = adjustedRPS;
    leftShooterMotor.setControl(flywheelVelocity.withVelocity(adjustedRPS));
    rightShooterMotor.setControl(flywheelVelocity.withVelocity(adjustedRPS));
  }

  /**
   * Run flywheels at a simple duty cycle (percentage of full power).
   * Less accurate than RPS control — use for manual override only.
   * @param speed Duty cycle from 0.0 (stopped) to 1.0 (full power)
   */
  public void runFlywheels(double speed) {
    targetRPS = speed * 106.0; // Rough RPS estimate for telemetry (Kraken max ≈ 106 RPS)
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  /** Stop both flywheel motors */
  public void stopFlywheels() {
    targetRPS = 0.0;
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
  }

  // ==================== FEEDER CONTROLS ====================

  /**
   * Run the feeder motor to push notes into the flywheels.
   * @param speed Duty cycle from -1.0 to 1.0 (positive = feed forward)
   */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  // ==================== INDEXER/SPINNER CONTROLS ====================

  /**
   * Run the indexer to move notes into feeding position.
   * @param speed Duty cycle from -1.0 to 1.0
   */
  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

  // ==================== COMBINED OPERATIONS ====================

  /**
   * Run the entire shooting system at once (flywheels + feeder + indexer).
   * Uses simple duty cycle control — for auto-aim, use autoAim() instead.
   */
  public void runFullShooter(double speed) {
    runFlywheels(speed);
    runFeeder(speed);
    runIndexer(speed);
  }

  /** Emergency stop — kills all shooter motors */
  public void stopAll() {
    stopFlywheels();
    stopFeeder();
    stopIndexer();
  }

  // ==================== CO-PILOT POWER OFFSET ====================

  /** Nudge flywheel power UP (+1 RPS). Called by co-pilot D-pad up. */
  public void nudgePowerUp() {
    rpsOffset += RPS_NUDGE;
    System.out.println(">>> Shooter RPS offset: " + String.format("%+.1f", rpsOffset) + " RPS <<<");
  }

  /** Nudge flywheel power DOWN (-1 RPS). Called by co-pilot D-pad down. */
  public void nudgePowerDown() {
    rpsOffset -= RPS_NUDGE;
    System.out.println(">>> Shooter RPS offset: " + String.format("%+.1f", rpsOffset) + " RPS <<<");
  }

  /** Reset the power offset back to zero. */
  public void resetPowerOffset() {
    rpsOffset = 0.0;
    System.out.println(">>> Shooter RPS offset RESET to 0 <<<");
  }

  /**
   * Are the flywheels spinning fast enough to shoot?
   * Returns true when both flywheels are within 5% of the target speed.
   * The AutoShootCommand checks this before feeding a note.
   */
  public boolean isReadyToShoot() {
    // Read actual flywheel speeds from the motor encoders
    double leftVel = Math.abs(leftShooterMotor.getVelocity().getValueAsDouble());
    double rightVel = Math.abs(rightShooterMotor.getVelocity().getValueAsDouble());

    if (targetRPS > 0) {
      // Closed-loop mode: check if within 5% of target
      double tolerance = targetRPS * 0.05;
      return Math.abs(leftVel - targetRPS) < tolerance
          && Math.abs(rightVel - targetRPS) < tolerance;
    }
    // Fallback: just check if spinning at all (at least 3 RPS)
    return leftVel > 3.0 && rightVel > 3.0;
  }

  // ==================== HOOD CONTROLS ====================

  /**
   * Auto-aim: set hood angle and flywheel speed based on distance to the hub.
   * Uses the interpolation tables defined in the constructor.
   * @param distanceMeters Distance from the robot to the hub in meters
   */
  public void autoAim(double distanceMeters) {
    double hoodPos = hoodTable.get(distanceMeters);      // Look up hood angle for this distance
    double rps = shooterRPSTable.get(distanceMeters);     // Look up flywheel speed for this distance
    setHoodPosition(hoodPos);
    runFlywheelsRPS(rps);
  }

  // Tracks the current hood position (servo doesn't have feedback, so we track it ourselves)
  private double currentHoodPosition = 0.0;

  /**
   * Set the hood to a specific position.
   * @param pos Position from 0.0 (flat) to 1.0 (max angle up). Clamped to safe range.
   */
  public void setHoodPosition(double pos) {
    pos = MathUtil.clamp(pos, HOOD_MIN, HOOD_MAX); // Don't let it go past physical limits
    currentHoodPosition = pos;
    hoodServo.set(pos);
  }

  /**
   * Nudge the hood position by a small amount (used by A/B buttons for calibration).
   * @param delta Amount to change (positive = up, negative = down)
   */
  public void nudgeHood(double delta) {
    setHoodPosition(currentHoodPosition + delta);
  }

  /** Get the current hood position (0.0 to 1.0) */
  public double getHoodPosition() {
    return currentHoodPosition;
  }

  /**
   * Called automatically every 20ms by the command scheduler.
   * Publishes current shooter data to the dashboard for monitoring.
   */
  @Override
  public void periodic() {
    ntLeftVel.set(leftShooterMotor.getVelocity().getValueAsDouble());
    ntRightVel.set(rightShooterMotor.getVelocity().getValueAsDouble());
    ntTargetVel.set(targetRPS);
    ntHoodPos.set(currentHoodPosition);
    ntReady.set(isReadyToShoot());
    ntRpsOffset.set(rpsOffset);
  }
}
