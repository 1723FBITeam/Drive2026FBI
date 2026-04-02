package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Tracks alliance shift timing for the 2026 REBUILT game.
 * 
 * ALLIANCE SHIFTS (from game manual):
 * - Match starts with BOTH hubs active for the first few seconds
 * - Then hubs alternate: Blue active → Red active → Blue active...
 * - Each alliance gets their hub active for a set duration
 * 
 * This class:
 * 1. Calculates when your hub is active based on match time
 * 2. Publishes status to Shuffleboard (green = active, red = inactive)
 * 3. Rumbles controllers 1 second before your hub becomes active
 * 
 * TIMING (adjust these constants based on actual game manual):
 * - INITIAL_BOTH_ACTIVE: Both hubs active at match start
 * - SHIFT_PERIOD: How long each alliance's hub stays active
 * - First shift happens after initial period ends
 */
public class AllianceShiftTracker {
    
    // ===== TIMING CONSTANTS (ADJUST BASED ON GAME MANUAL) =====
    // Initial period where BOTH hubs are active (seconds)
    private static final double INITIAL_BOTH_ACTIVE_DURATION = 5.0;
    
    // How long each alliance's hub stays active during alternating phase (seconds)
    private static final double SHIFT_PERIOD = 20.0;
    
    // Total teleop time (2:15 = 135 seconds for most FRC games)
    private static final double TELEOP_DURATION = 135.0;
    
    // How early to start rumbling before hub becomes active (seconds)
    private static final double RUMBLE_WARNING_TIME = 1.0;
    
    // Rumble intensity (0.0 to 1.0)
    private static final double RUMBLE_INTENSITY = 0.8;
    
    // ===== NETWORKTABLES PUBLISHERS =====
    private final BooleanPublisher hubActivePub;
    private final StringPublisher hubStatusTextPub;
    private final DoublePublisher timeUntilShiftPub;
    private final DoublePublisher matchTimePub;
    private final StringPublisher currentPhasePub;
    
    // ===== CONTROLLERS FOR RUMBLE =====
    private final CommandXboxController driverController;
    private final CommandXboxController copilotController;
    
    // ===== STATE TRACKING =====
    private boolean wasHubActive = false;
    private boolean isRumbling = false;
    private double lastRumbleStartTime = -999;
    
    /**
     * Creates a new AllianceShiftTracker.
     * 
     * @param driver The driver's controller (for rumble feedback)
     * @param copilot The co-pilot's controller (for rumble feedback)
     */
    public AllianceShiftTracker(CommandXboxController driver, CommandXboxController copilot) {
        this.driverController = driver;
        this.copilotController = copilot;
        
        // Set up NetworkTables publishers under "AllianceShift" table
        NetworkTable table = NetworkTableInstance.getDefault().getTable("AllianceShift");
        hubActivePub = table.getBooleanTopic("Hub Active").publish();
        hubStatusTextPub = table.getStringTopic("Status").publish();
        timeUntilShiftPub = table.getDoubleTopic("Time Until Shift").publish();
        matchTimePub = table.getDoubleTopic("Match Time").publish();
        currentPhasePub = table.getStringTopic("Phase").publish();
        
        // Initialize with default values
        hubActivePub.set(false);
        hubStatusTextPub.set("WAITING");
        timeUntilShiftPub.set(0.0);
        matchTimePub.set(0.0);
        currentPhasePub.set("PRE-MATCH");
    }
    
    /**
     * Call this every robot periodic loop (in Robot.java robotPeriodic).
     * Updates the hub status, publishes to Shuffleboard, and handles rumble.
     */
    public void update() {
        // Only track during teleop
        if (!DriverStation.isTeleopEnabled()) {
            hubActivePub.set(false);
            hubStatusTextPub.set("NOT IN TELEOP");
            currentPhasePub.set("DISABLED");
            stopRumble();
            return;
        }
        
        double matchTime = Timer.getMatchTime(); // Counts DOWN from teleop start
        double elapsedTime = TELEOP_DURATION - matchTime; // Convert to elapsed time
        
        // Clamp elapsed time to valid range
        elapsedTime = Math.max(0, Math.min(elapsedTime, TELEOP_DURATION));
        
        matchTimePub.set(matchTime);
        
        // Determine current phase and hub status
        boolean hubActive;
        double timeUntilNextShift;
        String phase;
        
        if (elapsedTime < INITIAL_BOTH_ACTIVE_DURATION) {
            // Initial phase: BOTH hubs active
            hubActive = true;
            timeUntilNextShift = INITIAL_BOTH_ACTIVE_DURATION - elapsedTime;
            phase = "BOTH ACTIVE";
        } else {
            // Alternating phase: calculate which alliance is active
            double timeInAltPhase = elapsedTime - INITIAL_BOTH_ACTIVE_DURATION;
            int shiftNumber = (int) (timeInAltPhase / SHIFT_PERIOD);
            double timeInCurrentShift = timeInAltPhase % SHIFT_PERIOD;
            timeUntilNextShift = SHIFT_PERIOD - timeInCurrentShift;
            
            // Determine if OUR hub is active based on alliance and shift number
            // Even shifts = Blue active, Odd shifts = Red active (adjust if needed)
            boolean isBlueShift = (shiftNumber % 2) == 0;
            boolean weAreBlue = isBlueAlliance();
            
            hubActive = (weAreBlue == isBlueShift);
            phase = isBlueShift ? "BLUE ACTIVE" : "RED ACTIVE";
        }
        
        // Publish status
        hubActivePub.set(hubActive);
        hubStatusTextPub.set(hubActive ? "HUB ACTIVE" : "HUB INACTIVE");
        timeUntilShiftPub.set(timeUntilNextShift);
        currentPhasePub.set(phase);
        
        // Handle rumble warning
        handleRumbleWarning(hubActive, timeUntilNextShift, elapsedTime);
        
        // Track state change for logging
        if (hubActive != wasHubActive) {
            System.out.println(">>> ALLIANCE SHIFT: Hub is now " + (hubActive ? "ACTIVE" : "INACTIVE") + " <<<");
            wasHubActive = hubActive;
        }
    }
    
    /**
     * Handles controller rumble to warn drivers before their hub becomes active.
     */
    private void handleRumbleWarning(boolean hubCurrentlyActive, double timeUntilShift, double elapsedTime) {
        // Only rumble if hub is currently INACTIVE and about to become ACTIVE
        boolean shouldRumble = !hubCurrentlyActive 
                && timeUntilShift <= RUMBLE_WARNING_TIME 
                && timeUntilShift > 0
                && elapsedTime >= INITIAL_BOTH_ACTIVE_DURATION; // Don't rumble during initial phase
        
        if (shouldRumble && !isRumbling) {
            // Start rumbling
            startRumble();
            lastRumbleStartTime = Timer.getFPGATimestamp();
            isRumbling = true;
        } else if (!shouldRumble && isRumbling) {
            // Stop rumbling
            stopRumble();
            isRumbling = false;
        }
        
        // Safety: stop rumble after 1.5 seconds max (in case timing is off)
        if (isRumbling && (Timer.getFPGATimestamp() - lastRumbleStartTime) > 1.5) {
            stopRumble();
            isRumbling = false;
        }
    }
    
    private void startRumble() {
        driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
        copilotController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
    }
    
    private void stopRumble() {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        copilotController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
    
    /**
     * Returns true if our alliance is Blue.
     */
    private boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        // Default to blue if not set
        return !alliance.isPresent() || alliance.get() == DriverStation.Alliance.Blue;
    }
    
    /**
     * Returns true if our hub is currently active (can score).
     * Use this in other parts of code to make decisions based on hub status.
     */
    public boolean isHubActive() {
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }
        
        double matchTime = Timer.getMatchTime();
        double elapsedTime = TELEOP_DURATION - matchTime;
        elapsedTime = Math.max(0, Math.min(elapsedTime, TELEOP_DURATION));
        
        if (elapsedTime < INITIAL_BOTH_ACTIVE_DURATION) {
            return true; // Both hubs active initially
        }
        
        double timeInAltPhase = elapsedTime - INITIAL_BOTH_ACTIVE_DURATION;
        int shiftNumber = (int) (timeInAltPhase / SHIFT_PERIOD);
        boolean isBlueShift = (shiftNumber % 2) == 0;
        boolean weAreBlue = isBlueAlliance();
        
        return (weAreBlue == isBlueShift);
    }
    
    /**
     * Returns the time in seconds until the next alliance shift.
     */
    public double getTimeUntilNextShift() {
        if (!DriverStation.isTeleopEnabled()) {
            return 0;
        }
        
        double matchTime = Timer.getMatchTime();
        double elapsedTime = TELEOP_DURATION - matchTime;
        elapsedTime = Math.max(0, Math.min(elapsedTime, TELEOP_DURATION));
        
        if (elapsedTime < INITIAL_BOTH_ACTIVE_DURATION) {
            return INITIAL_BOTH_ACTIVE_DURATION - elapsedTime;
        }
        
        double timeInAltPhase = elapsedTime - INITIAL_BOTH_ACTIVE_DURATION;
        double timeInCurrentShift = timeInAltPhase % SHIFT_PERIOD;
        return SHIFT_PERIOD - timeInCurrentShift;
    }
}