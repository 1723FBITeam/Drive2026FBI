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
import frc.robot.TrajectoryCalculations;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.function.DoubleSupplier;

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
  private final TalonFX leftFeederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_LEFT_MOTOR, Constants.kCANivoreBus);
  private final TalonFX rightFeederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_RIGHT_MOTOR, Constants.kCANivoreBus);

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
  //
  // HUB tables: calibrated for scoring into the hub (1–6.5m)
  // PASSING tables: calibrated for lobbing across the field (5–13m)
  //   Passing shots need more hood angle (higher arc) and more power.
  //   Biased toward overshooting — better to overshoot than undershoot a pass.
  private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRPSTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap passHoodTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap passRPSTable = new InterpolatingDoubleTreeMap();

  // ===== FLYWHEEL PRE-SPIN =====
  // Idle speed to keep flywheels warm when not actively shooting.
  // Cuts spin-up time from ~0.8s to ~0.2s when AutoShoot activates.
  public static final double IDLE_PRESPIN_RPS = 18.0;

  // Tracks what speed we asked the flywheels to spin at (for ready-to-shoot check)
  private double targetRPS = 0.0;

  // ===== TRENCH SAFETY =====
  // Robot position suppliers — set by RobotContainer so periodic() can
  // force the hood flat when driving through a trench zone on the trench side.
  private DoubleSupplier robotXSupplier = () -> 0.0;
  private DoubleSupplier robotYSupplier = () -> 0.0;

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
    rightFeederMotor.getConfigurator().apply(rightConfigs); // Feeder spins same direction as right flywheel

    // Left flywheel: Clockwise = positive direction (opposite of right)
    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leftConfigs.NeutralMode = NeutralModeValue.Coast;
    leftShooterMotor.getConfigurator().apply(leftConfigs);
    leftShooterMotor.getConfigurator().apply(flywheelGains);
    leftFeederMotor.getConfigurator().apply(leftConfigs); // Feeder spins same direction as right flywheel

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
    // CALIBRATION LOG:
    //   1.3m (52in): too close — hood goes flat, ball exits forward. Need dedicated close-range entry.
    //   2.2m (87in): confirmed good (screenshot data)
    //   3.35m (123in): made it but hood slightly too high — lowered from 0.336 to 0.31
    //   3.71m (146in): diagonal shot, made it well — confirmed good
    //   4.03m: screenshot data looks good
    //   5.06m (199in): made 1/missed 1, hood too angled — lowered from 0.55 to 0.48
    hoodTable.put(1.0, 0.15);   // Close range — give it some angle so ball arcs up
    hoodTable.put(1.3, 0.15);   // Raised from 0.13 — flat hood can't score at this range
    hoodTable.put(2.1, 0.21);
    hoodTable.put(3.0, 0.28);
    hoodTable.put(3.35, 0.31);  // NEW — tested at 123in, slightly less hood than interp gave
    hoodTable.put(3.8, 0.36);
    hoodTable.put(4.7, 0.44);
    hoodTable.put(5.1, 0.48);   // NEW — tested at 199in, was 0.55 (too much), lowered
    hoodTable.put(5.5, 0.52);   // Adjusted down from 0.55 based on 5.1m result
    hoodTable.put(6.5, 0.60);   // Adjusted down from 0.63 based on trend

    // Flywheel speed table: distance → speed in Rotations Per Second (RPS)
    // CALIBRATION LOG:
    //   1.3m: 20 RPS way too low at close range — raised to 25
    //   5.06m: made 1/missed 1 at 42 RPS — keep as-is, hood was the problem
    shooterRPSTable.put(1.0, 25.0);   // NEW — close range needs more power to arc up
    shooterRPSTable.put(1.3, 25.0);   // Raised from 20.0 — was way too weak up close
    shooterRPSTable.put(2.1, 27.5);
    shooterRPSTable.put(2.2, 28.5);
    shooterRPSTable.put(3.0, 30.6);
    shooterRPSTable.put(3.35, 32.0);  // NEW — tested at 123in
    shooterRPSTable.put(3.8, 34.4);
    shooterRPSTable.put(4.7, 37.5);
    shooterRPSTable.put(5.1, 42.0);   // NEW — tested at 199in, power seemed OK
    shooterRPSTable.put(5.5, 44.0);   // Adjusted from 42.0
    shooterRPSTable.put(6.5, 48.0);   // Adjusted from 47.0

    // ===== PASSING SHOT CALIBRATION TABLES =====
    // For lobbing balls across the field to teammates (5–13m range).
    // Target: land 0.5–1m inside our alliance zone. Don't over-power.
    //
    // Hood: higher values = more arc.
    passHoodTable.put(5.0, 0.55);    // NEEDS CALIBRATION
    passHoodTable.put(7.0, 0.62);    // NEEDS CALIBRATION
    passHoodTable.put(9.0, 0.68);    // NEEDS CALIBRATION
    passHoodTable.put(11.0, 0.73);   // NEEDS CALIBRATION
    passHoodTable.put(13.0, 0.78);   // NEEDS CALIBRATION

    // RPS: pulled back from original values to avoid overshooting
    passRPSTable.put(5.0, 38.0);     // NEEDS CALIBRATION (was 45)
    passRPSTable.put(7.0, 42.0);     // NEEDS CALIBRATION (was 48)
    passRPSTable.put(9.0, 46.0);     // NEEDS CALIBRATION (was 52)
    passRPSTable.put(11.0, 49.0);    // NEEDS CALIBRATION (was 55)
    passRPSTable.put(13.0, 52.0);    // NEEDS CALIBRATION (was 58)

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
    leftFeederMotor.set(speed);
    rightFeederMotor.set(speed);
  }

  public void stopFeeder() {
    leftFeederMotor.stopMotor();
    rightFeederMotor.stopMotor();
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
  }

  /** Nudge flywheel power DOWN (-1 RPS). Called by co-pilot D-pad down. */
  public void nudgePowerDown() {
    rpsOffset -= RPS_NUDGE;
  }

  /** Reset the power offset back to zero. */
  public void resetPowerOffset() {
    rpsOffset = 0.0;
  }

  /**
   * Set the robot X position supplier for trench safety.
   * Call this once from RobotContainer after construction.
   * periodic() uses this to force the hood flat in trench zones.
   */
  public void setRobotXSupplier(DoubleSupplier supplier) {
    this.robotXSupplier = supplier;
  }

  public void setRobotYSupplier(DoubleSupplier supplier) {
    this.robotYSupplier = supplier;
  }

  /**
   * Are the flywheels spinning fast enough to shoot?
   * Returns true when both flywheels are within 5% of the target speed.
   * The AutoShootCommand checks this before feeding a note.
   */
  public boolean isReadyToShoot() {
    double leftVel = Math.abs(leftShooterMotor.getVelocity().getValueAsDouble());
    double rightVel = Math.abs(rightShooterMotor.getVelocity().getValueAsDouble());

    if (targetRPS > 0) {
      // Use the LARGER of 10% relative tolerance or 3 RPS absolute tolerance.
      // The absolute floor prevents the ready check from flickering when the
      // target RPS is changing rapidly (e.g., distance shifting while moving).
      // At 30 RPS: max(3.0, 3.0) = 3 RPS window (27–33)
      // At 40 RPS: max(4.0, 3.0) = 4 RPS window (36–44)
      double tolerance = Math.max(targetRPS * 0.10, 3.0);
      return Math.abs(leftVel - targetRPS) < tolerance
          && Math.abs(rightVel - targetRPS) < tolerance;
    }
    return leftVel > 3.0 && rightVel > 3.0;
  }

  // ==================== HOOD CONTROLS ====================

  /**
   * Auto-aim: set hood angle and flywheel speed based on distance.
   * Uses blended table + physics approach — see blendedHubAutoAim / blendedPassAutoAim.
   * The standalone methods below are kept private for use by the blended methods.
   */

  /**
   * Blended hub auto-aim: smoothly mixes table and physics based on weight.
   * @param distanceMeters distance to target
   * @param physicsWeight 0.0 = pure table, 1.0 = pure physics
   */
  public void blendedHubAutoAim(double distanceMeters, double physicsWeight) {
    // Table values
    double clampedDist = MathUtil.clamp(distanceMeters, 1.0, 6.5);
    double tableHood = hoodTable.get(clampedDist);
    double tableRPS = shooterRPSTable.get(clampedDist);

    // Physics values
    double heightDiff = Constants.FieldConstants.HUB_TARGET_HEIGHT_METERS
                      - Constants.FieldConstants.TURRET_HEIGHT_METERS;
    double aoa = TrajectoryCalculations.getHubAngleOfAttack(distanceMeters);
    double[] solution = TrajectoryCalculations.solve(distanceMeters, heightDiff, aoa);

    double physicsHood = tableHood;
    double physicsRPS = tableRPS;
    if (solution != null) {
      physicsHood = TrajectoryCalculations.launchAngleToHoodPosition(solution[1]);
      physicsRPS = TrajectoryCalculations.exitVelocityToRPS(solution[0]);
    }

    // Blend
    double hood = (1.0 - physicsWeight) * tableHood + physicsWeight * physicsHood;
    double rps = (1.0 - physicsWeight) * tableRPS + physicsWeight * physicsRPS;

    // If blended hood is flatter than table, pull back toward table
    if (hood > tableHood) {
      hood = (hood + tableHood) / 2.0;
    }

    setHoodPosition(MathUtil.clamp(hood, HOOD_MIN, HOOD_MAX));
    runFlywheelsRPS(MathUtil.clamp(rps, 0.0, 60.0));
  }

  /**
   * Blended passing auto-aim: smoothly mixes table and physics based on weight.
   * @param distanceMeters distance to target
   * @param physicsWeight 0.0 = pure table, 1.0 = pure physics
   */
  public void blendedPassAutoAim(double distanceMeters, double physicsWeight) {
    // Table values
    double clampedDist = MathUtil.clamp(distanceMeters, 5.0, 13.0);
    double tableHood = passHoodTable.get(clampedDist);
    double tableRPS = passRPSTable.get(clampedDist);

    // Physics values
    double heightDiff = Constants.FieldConstants.PASSING_TARGET_HEIGHT_METERS
                      - Constants.FieldConstants.TURRET_HEIGHT_METERS;
    double aoa = TrajectoryCalculations.getPassingAngleOfAttack(distanceMeters);
    double[] solution = TrajectoryCalculations.solve(distanceMeters, heightDiff, aoa);

    double physicsHood = tableHood;
    double physicsRPS = tableRPS;
    if (solution != null) {
      physicsHood = TrajectoryCalculations.launchAngleToHoodPosition(solution[1]);
      physicsRPS = TrajectoryCalculations.exitVelocityToRPS(solution[0]);
    }

    // Blend
    double hood = (1.0 - physicsWeight) * tableHood + physicsWeight * physicsHood;
    double rps = (1.0 - physicsWeight) * tableRPS + physicsWeight * physicsRPS;

    setHoodPosition(MathUtil.clamp(hood, HOOD_MIN, HOOD_MAX));
    runFlywheelsRPS(MathUtil.clamp(rps, 0.0, 60.0));
  }

  /**
   * Auto-aim for passing shots (lobbing to a teammate across the field).
   * Uses dedicated passing interpolation tables with more hood angle and
   * more power than hub shots. Biased toward overshooting.
   *
   * @param distanceMeters horizontal distance from robot to the passing target
   */
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

  /** Get the current target flywheel RPS (what we commanded, including offset) */
  public double getTargetRPS() {
    return targetRPS;
  }

  /**
   * Called automatically every 20ms by the command scheduler.
   * Publishes current shooter data to the dashboard for monitoring.
   */
  private int telemetryCounter = 0;

  @Override
  public void periodic() {
    // ===== TRENCH SAFETY (runs every loop, 50Hz) =====
    // Force hood flat when in a trench zone AND on the trench side of the field.
    // Does NOT trigger when going over the bump (middle of field).
    if (Constants.FieldConstants.isInTrenchZone(robotXSupplier.getAsDouble())
        && Constants.FieldConstants.isOnTrenchSide(robotYSupplier.getAsDouble())) {
      if (currentHoodPosition > Constants.FieldConstants.TRENCH_HOOD_THRESHOLD) {
        setHoodPosition(0.0);
      }
    }

    // Telemetry at ~10Hz
    telemetryCounter++;
    if (telemetryCounter % 5 != 0) return;

    ntLeftVel.set(leftShooterMotor.getVelocity().getValueAsDouble());
    ntRightVel.set(rightShooterMotor.getVelocity().getValueAsDouble());
    ntTargetVel.set(targetRPS);
    ntHoodPos.set(currentHoodPosition);
    ntReady.set(isReadyToShoot());
    ntRpsOffset.set(rpsOffset);
  }
}
