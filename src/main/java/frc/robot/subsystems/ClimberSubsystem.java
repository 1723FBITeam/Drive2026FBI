package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

/**
 * ClimberSubsystem — handles endgame climbing.
 * NOTE: Not wired yet! This code is ready for when the hardware is built.
 *
 * The climber has three parts:
 *   1. CLIMB MOTORS (left + right) — pull the robot up onto the chain/bar
 *   2. ELEVATOR MOTOR — extends/retracts a lift mechanism
 *   3. SERVOS (left + right) — lock/unlock climb hooks
 *
 * The climb motors are on the RIO's built-in CAN bus (not the CANivore)
 * because they don't need the faster update rate.
 */
public class ClimberSubsystem extends SubsystemBase {

    // Climb motors — pull the robot up
    // private final TalonFX leftClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_LEFT_MOTOR);
    // private final TalonFX rightClimbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_RIGHT_MOTOR);

    // Elevator motor — extends/retracts the climbing arm
    private final TalonFX elevatorMotor = new TalonFX(Constants.ClimberConstants.ELEVATOR_MOTOR, Constants.kCANivoreBus);

    // Servo that locks/unlocks the gearbox (single servo controlling the lock)
    private final Servo elevatorServo = new Servo(Constants.ClimberConstants.ELEVATOR_SERVO);

    // ==================== ELEVATOR PID (position control) ====================
    // Very conservative PID: feedforward does the heavy lifting, PID just gently corrects
    private final PIDController elevatorPID = new PIDController(0.1, 0.0, 0.0);
    private double elevatorTargetPosition = 0.0; // rotations
    private boolean elevatorPositionControlEnabled = false;
    private static final double ELEVATOR_TOLERANCE = 0.01; // rotations

    // Runtime tuning defaults (can be changed on Shuffleboard)
    private static final double DEFAULT_MAX_OUTPUT = 0.25;  // max percent output from PID (reduced for safe testing)

     private Map<Integer, Double> targetPositions = Map.of(
            0, 0.0,
            1, 29.0);

    // Last PID loop telemetry (recorded each periodic while PID is active)
    private double lastPidOutput = 0.0;
    private double lastPidError = 0.0;

    // Level-and-return cycle state: records starting (home) pos then goes to level and back
    private boolean levelCycleActive = false;
    private double levelCycleHomePos = 0.0;
    private int levelCycleStage = 0; // 0=idle, 1=going up, 2=returning home
    private final Timer levelCycleTimer = new Timer();

    // Default positions (can be adjusted on Shuffleboard)
    private static final double DEFAULT_LEVEL_POS = 5.0; // rotations (example)
    private static final double DEFAULT_HOME_POS = 0.0;

    // Servo lock positions
    private static final double GEARBOX_LOCK_POS = 0.5;
    private static final double GEARBOX_UNLOCK_POS = 1.0;

    // Tracks whether gearbox is currently locked
    private boolean gearboxLocked = false;

    // Servo preset positions
    private static final double SERVO_RETRACTED = 0.0; // Hooks locked/retracted
    private static final double SERVO_EXTENDED = 1.0;   // Hooks open/extended
    private static final double SERVO_MID = 0.5;        // Halfway position

    public ClimberSubsystem() {
        // Right climb motor is inverted so both motors pull in the same direction
        // MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
        // rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        // rightClimbMotor.getConfigurator().apply(rightConfigs);

        // MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        // leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        // leftClimbMotor.getConfigurator().apply(leftConfigs);

        MotorOutputConfigs elevatorConfigs = new MotorOutputConfigs();
        elevatorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotor.getConfigurator().apply(elevatorConfigs);

        // Configure PID tolerance so we can use atSetpoint() if needed
        elevatorPID.setTolerance(ELEVATOR_TOLERANCE);

        // Ensure gearbox is unlocked at robot start so elevator can move freely.
        // Some hardware rests the servo at the locked position (0.5) — unlock here.
        unlockGearbox();
        SmartDashboard.putBoolean("Climber Gearbox Locked", gearboxLocked);
    }

    // ==================== CLIMB MOTORS ====================

    // /** Run both climb motors together. @param speed -1.0 to 1.0 */
    // public void runClimb(double speed) {
    //     leftClimbMotor.set(speed);
    //     rightClimbMotor.set(speed);
    // }

    // /** Run left climb motor independently (for leveling) */
    // public void runLeftClimb(double speed) {
    //     leftClimbMotor.set(speed);
    // }

    // /** Run right climb motor independently (for leveling) */
    // public void runRightClimb(double speed) {
    //     rightClimbMotor.set(speed);
    // }

    // public void stopClimb() {
    //     leftClimbMotor.stopMotor();
    //     rightClimbMotor.stopMotor();
    // }

    // ==================== ELEVATOR ====================

    /** Run elevator up (positive direction) */
    public void elevatorUp(double speed) {
        // Manual control cancels position control
        elevatorPositionControlEnabled = false;
        elevatorMotor.set(Math.abs(speed));
    }

    /** Run elevator down (negative direction) */
    public void elevatorDown(double speed) {
        elevatorMotor.set(-Math.abs(speed));
    }

    /** Run elevator with manual control. @param speed -1.0 to 1.0 */
    public void runElevator(double speed) {
        // Manual control cancels position control
        elevatorPositionControlEnabled = false;
        elevatorMotor.set(speed);
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    // ==================== SERVOS ====================

    /** Set both servos to a position (0.0 to 1.0) */
    public void setServoPosition(double position) {
        position = MathUtil.clamp(position, 0.0, 1.0);
        // leftServo.set(position);
        // rightServo.set(position);
        elevatorServo.set(position);
    }

    /** Retract hooks (locked position) */
    public void setServoRetracted() {
        setServoPosition(SERVO_RETRACTED);
        System.out.println("Servos: Retracted");
    }

    /** Move hooks to middle position */
    public void setServoMid() {
        setServoPosition(SERVO_MID);
        System.out.println("Servos: Mid Position");
    }

    /** Extend hooks (open position) */
    public void setServoExtended() {
        setServoPosition(SERVO_EXTENDED);
        System.out.println("Servos: Extended");
    }

    // ==================== COMBINED ====================

    /** Stop all climber motors (servos hold their last position) */
    public void stopAll() {
        // stopClimb();
        stopElevator();
    }

    // ==================== POSITION CONTROL API ====================

    /** Enable elevator position control to a specific target (rotations). */
    public void enableElevatorPositionControl(double targetRotations) {
        elevatorTargetPosition = targetRotations;
        elevatorPID.reset();
        // Prevent integrator windup during large moves
        elevatorPID.setIntegratorRange(-0.2, 0.2);
        elevatorPositionControlEnabled = true;
        System.out.println(">>> Climber: position control ENABLED -> target=" + String.format("%.3f", targetRotations) + " rot <<<");
    }

    /** Start a level-and-return cycle: record current position, go to level, then return home. */
    public void startLevelAndReturn() {
        if (levelCycleActive) return; // already running
        levelCycleHomePos = elevatorMotor.getPosition().getValueAsDouble();
        double levelPos = SmartDashboard.getNumber("Climber Level Pos", DEFAULT_LEVEL_POS);
        levelCycleActive = true;
        levelCycleStage = 1;
        levelCycleTimer.reset();
        levelCycleTimer.start();
        enableElevatorPositionControl(levelPos);
    }

    /** Toggle elevator between level and home. Press same button to start cycle, press again while cycling returns to home immediately. */
    public void toggleLevelCycle() {
        if (levelCycleActive) {
            // Already in cycle — skip remaining stages and return home immediately
            levelCycleStage = 2;
            enableElevatorPositionControl(levelCycleHomePos);
            System.out.println(">>> Climber: level cycle interrupted, returning home <<<");
        } else {
            // Start new level cycle
            startLevelAndReturn();
        }
    }

    /** Go to a predefined indexed target (from targetPositions map). */
    public void goToTargetIndex(int index) {
        if (!targetPositions.containsKey(index)) {
            System.out.println(">>> Climber: target index " + index + " not found <<<");
            return;
        }
        double pos = targetPositions.get(index);
        enableElevatorPositionControl(pos);
        System.out.println(">>> Climber: moving to preset target[" + index + "] = " + pos + " rot <<<");
    }

    /**
     * Toggle between a preset index and home (index 0).
     * If currently targeting the preset (within tolerance), go to home; otherwise go to the preset.
     */
    public void togglePresetIndex(int index) {
        if (!targetPositions.containsKey(index)) {
            System.out.println(">>> Climber: target index " + index + " not found <<<");
            return;
        }
        double preset = targetPositions.get(index);
        double home = targetPositions.getOrDefault(0, 0.0);

        // If position control is enabled and the current target equals the preset (within tolerance), go home
        if (elevatorPositionControlEnabled && Math.abs(elevatorTargetPosition - preset) < ELEVATOR_TOLERANCE) {
            enableElevatorPositionControl(home);
            System.out.println(">>> Climber: toggle -> returning HOME (" + String.format("%.3f", home) + " rot) <<<");
        } else {
            enableElevatorPositionControl(preset);
            System.out.println(">>> Climber: toggle -> moving to PRESET[" + index + "] (" + String.format("%.3f", preset) + " rot) <<<");
        }
    }

    /** Returns true if the level-and-return cycle is active. */
    public boolean isLevelCycleActive() {
        return levelCycleActive;
    }

    // ==================== GEARBOX LOCK/UNLOCK ====================

    /** Lock the gearbox using the servo. */
    public void lockGearbox() {
        setServoPosition(GEARBOX_LOCK_POS);
        gearboxLocked = true;
        System.out.println(">>> Climber: Gearbox LOCKED <<<");
    }

    /** Unlock the gearbox using the servo. */
    public void unlockGearbox() {
        setServoPosition(GEARBOX_UNLOCK_POS);
        gearboxLocked = false;
        System.out.println(">>> Climber: Gearbox UNLOCKED <<<");
    }

    /** Toggle the gearbox lock state. */
    public void toggleGearboxLock() {
        if (gearboxLocked) unlockGearbox(); else lockGearbox();
    }

    /** Returns true if the gearbox is currently locked. */
    public boolean isGearboxLocked() {
        return gearboxLocked;
    }

    /** Called every 20ms — publishes climb positions to dashboard */
    @Override
    public void periodic() {
        double elevPos = elevatorMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Position", elevPos);
      

        // Dashboard tuning values (live)
        double newP = SmartDashboard.getNumber("Climber kP", elevatorPID.getP());
        double newI = SmartDashboard.getNumber("Climber kI", elevatorPID.getI());
        double newD = SmartDashboard.getNumber("Climber kD", elevatorPID.getD());
        if (newP != elevatorPID.getP() || newI != elevatorPID.getI() || newD != elevatorPID.getD()) {
            elevatorPID.setP(newP);
            elevatorPID.setI(newI);
            elevatorPID.setD(newD);
        }

        SmartDashboard.putNumber("Climber Target Position", elevatorTargetPosition);
        SmartDashboard.putBoolean("Climber PID Enabled", elevatorPositionControlEnabled);

        // Dashboard one-shot triggers
        if (SmartDashboard.getBoolean("Climb Go To Level", false)) {
            double levelPos = SmartDashboard.getNumber("Climber Level Pos", DEFAULT_LEVEL_POS);
            enableElevatorPositionControl(levelPos);
            SmartDashboard.putBoolean("Climb Go To Level", false);
        }
        if (SmartDashboard.getBoolean("Climb Go Home", false)) {
            double homePos = SmartDashboard.getNumber("Climber Home Pos", DEFAULT_HOME_POS);
            enableElevatorPositionControl(homePos);
            SmartDashboard.putBoolean("Climb Go Home", false);
        }
        if (SmartDashboard.getBoolean("Climb Start Level Cycle", false)) {
            startLevelAndReturn();
            SmartDashboard.putBoolean("Climb Start Level Cycle", false);
        }

        // Run PID if enabled
        if (elevatorPositionControlEnabled) {
            // Live tunables
            double maxOut = SmartDashboard.getNumber("Climber Max Output", DEFAULT_MAX_OUTPUT);
            double ff = SmartDashboard.getNumber("Climber FF", 0.15); // feedforward to hold against gravity
            
            lastPidError = elevatorTargetPosition - elevPos;
            
            // Gentle PID correction: mostly feedforward, tiny PID nudge only
            double pidOut = elevatorPID.calculate(elevPos, elevatorTargetPosition);
            double output = ff + pidOut; // feedforward + small PID correction
            double motorOut = MathUtil.clamp(output, -maxOut, maxOut);
            
            // Publish diagnostics
            SmartDashboard.putNumber("Climber PID pidOut", pidOut);
            SmartDashboard.putNumber("Climber PID FF", ff);
            SmartDashboard.putNumber("Climber Motor Output", motorOut);
            SmartDashboard.putNumber("Climber PID Error", lastPidError);
            
            lastPidOutput = motorOut;
            elevatorMotor.set(lastPidOutput);
            
            // Telemetry
            String compact = String.format("pos=%.3f tgt=%.3f err=%.3f out=%.3f",
                elevPos, elevatorTargetPosition, lastPidError, lastPidOutput);
            SmartDashboard.putString("Climber PID Telemetry", compact);

            // Print telemetry
            System.out.println(">>> Climber: pos=" + String.format("%.3f", elevPos)
                + " target=" + String.format("%.3f", elevatorTargetPosition)
                + " err=" + String.format("%.3f", lastPidError)
                + " out=" + String.format("%.3f", lastPidOutput) + " <<<");

            // Check reached target
            if (Math.abs(elevPos - elevatorTargetPosition) < ELEVATOR_TOLERANCE) {
                stopElevator();
                elevatorPositionControlEnabled = false;
                SmartDashboard.putBoolean("Climber PID Enabled", false);
                System.out.println(">>> Climber: reached target " + String.format("%.3f", elevatorTargetPosition) + " rot <<<");

                // If we're in the level-return cycle and just finished stage 1 (going up), start return
                if (levelCycleActive && levelCycleStage == 1) {
                    levelCycleStage = 2;
                    enableElevatorPositionControl(levelCycleHomePos);
                } else if (levelCycleActive && levelCycleStage == 2) {
                    // Finished return
                    levelCycleActive = false;
                    levelCycleStage = 0;
                    levelCycleTimer.stop();
                    System.out.println(">>> Climber: level-and-return complete <<<");
                }
            }
        }
    }
}
