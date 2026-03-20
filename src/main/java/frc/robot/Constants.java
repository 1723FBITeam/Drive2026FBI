package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * All robot-wide constants live here — CAN IDs, PWM ports, field positions, etc.
 *
 * WHY: Keeping all "magic numbers" in one file makes it easy to change wiring
 * or field measurements without hunting through every subsystem file.
 *
 * CAN ID RANGES (so we don't accidentally reuse an ID):
 *   1-20  = Drivetrain (swerve modules)
 *   21-30 = Shooting system (flywheels, turret, feeder, indexer)
 *   31-40 = Intake system (rollers, deploy motors)
 *   41-50 = Climber system (climb motors, elevator)
 */
public class Constants {

    /**
     * The name of our CANivore bus. All non-drivetrain motors are on this bus.
     * The drivetrain uses its own CAN bus configured in TunerConstants.
     * If you plug a motor into the wrong bus, it won't show up!
     */
    public static final CANBus kCANivoreBus = new CANBus("Carnivore");

    /**
     * Shooting system CAN IDs and settings.
     * The shooter has: two flywheel motors, a turret motor, a feeder motor,
     * an indexer/spinner motor, and a hood servo (on PWM, not CAN).
     */
    public static final class ShootingConstants {
        public static final int SHOOTER_LEFT_MOTOR = 21;  // Left flywheel
        public static final int SHOOTER_RIGHT_MOTOR = 22; // Right flywheel
        public static final int TURRET_MOTOR = 23;        // Rotates the whole shooter
        public static final int FEEDER_MOTOR = 24;        // Pushes notes into flywheels
        public static final int SPINNER_MOTOR = 25;       // Indexes/queues notes

        // Hood servo is on PWM port 0 (not CAN — it's a simple servo)
        public static final int HOOD_SERVO = 0;

        // Hood servo travel limits (0.0 = flat, 1.0 = max angle up)
        public static final double HOOD_MIN = 0.0;
        public static final double HOOD_MAX = 0.85;
    }

    /**
     * Intake system CAN IDs.
     * The intake has: two roller motors (spin to grab notes) and
     * two deploy motors (swing the intake out from the robot frame).
     */
    public static final class IntakeConstants {
        public static final int INTAKE_LEFT_MOTOR = 31;  // Left roller
        public static final int INTAKE_RIGHT_MOTOR = 32; // Right roller
        public static final int DEPLOY_LEFT_MOTOR = 33;  // Swings intake out/in
        public static final int DEPLOY_RIGHT_MOTOR = 34; // Swings intake out/in
    }

    /**
     * Climber system CAN IDs and PWM ports.
     * The climber has: two climb motors, an elevator motor, and two servos.
     * NOTE: Not wired yet — these IDs are reserved for when we build it.
     */
    public static final class ClimberConstants {
        public static final int CLIMB_LEFT_MOTOR = 41;
        public static final int CLIMB_RIGHT_MOTOR = 42;
        public static final int ELEVATOR_MOTOR = 43;

        // Servos use PWM ports (not CAN IDs)
        // public static final int CLIMB_SERVO_LEFT = 1;
        // public static final int CLIMB_SERVO_RIGHT = 2;
        public static final int ELEVATOR_SERVO = 1;

    }

    /**
     * Field positions and zones used for auto-aiming.
     * All positions are in meters, using the WPILib "blue origin" coordinate system:
     *   - (0, 0) is the bottom-left corner when looking from the blue alliance wall
     *   - X increases toward the red alliance wall
     *   - Y increases toward the left (from blue's perspective)
     *
     * FIELD ZONES (2026 REBUILT):
     *   Blue Alliance Zone:  X = 0 to 4.03m
     *   Neutral Zone:        X = 4.03m to 12.51m (includes trenches at the boundaries)
     *   Red Alliance Zone:   X = 12.51m to 16.54m
     *
     * TRENCH: 22.25in (56.5cm) clearance — hood must be flat to fit under.
     *   Trenches run along the guardrails at the boundary between alliance zones
     *   and the neutral zone (near the BUMPs at ~4.03m from each wall).
     */
    public static final class FieldConstants {
        // Total field dimensions in meters (from 2026 REBUILT WPILib AprilTag field JSON)
        // Length: 16.541m (651.2in)
        // Width:  8.069m  (317.7in)
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

        // ===== ZONE BOUNDARIES (X coordinates, blue origin) =====
        // Alliance zone depth from 2026 game manual: 158.6in ≈ 4.03m
        // Neutral zone depth: 283in ≈ 7.19m (between the two alliance zones)
        public static final double ALLIANCE_ZONE_DEPTH = 4.03;
        // Neutral zone starts where alliance zone ends
        public static final double BLUE_ZONE_END = ALLIANCE_ZONE_DEPTH;           // 4.03m
        public static final double RED_ZONE_START = FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH; // 12.51m
        // ===== HYSTERESIS =====
        // When the robot is near the Y midline, we don't want the pass target
        // to flip between left and right every loop. This buffer means the robot
        // must cross 1m past the midline before the target switches sides.
        public static final double PASS_Y_HYSTERESIS = 1.0; // meters past midline to switch

        // ===== TRENCH ZONE =====
        // Trench positions derived from official WPILib 2026 AprilTag field JSON.
        // Trench AprilTags sit on the trench arms at these X coordinates:
        //   Blue trench: tags 23/28 at X=4.588 (alliance side), tags 17/22 at X=4.663 (neutral side)
        //   Red trench:  tags 1/6 at X=11.878 (alliance side), tags 7/12 at X=11.953 (neutral side)
        // Clearance underneath: 22.25in (56.52cm) — hood MUST be flat to fit.
        // TRENCH_BUFFER adds extra margin so the hood flattens before we reach it.
        public static final double TRENCH_BUFFER = 0.55;  // extra margin on each side (meters)
        // Blue trench X range: 4.588 to 4.663 (+ buffer on each side)
        public static final double BLUE_TRENCH_MIN_X = 4.588 - TRENCH_BUFFER;  // ~4.088
        public static final double BLUE_TRENCH_MAX_X = 4.663 + TRENCH_BUFFER;  // ~5.163
        // Red trench X range: 11.878 to 11.953 (+ buffer on each side)
        public static final double RED_TRENCH_MIN_X = 11.878 - TRENCH_BUFFER;  // ~11.378
        public static final double RED_TRENCH_MAX_X = 11.953 + TRENCH_BUFFER;  // ~12.453

        /** Returns true if the given X position is within a trench zone (either alliance). */
        public static boolean isInTrenchZone(double robotX) {
            return (robotX >= BLUE_TRENCH_MIN_X && robotX <= BLUE_TRENCH_MAX_X)
                || (robotX >= RED_TRENCH_MIN_X && robotX <= RED_TRENCH_MAX_X);
        }

        // Hood position threshold for trench clearance — if the hood is above this
        // value near the trench, the drivetrain slows down to give the servo time to retract.
        // Set this to the max hood position that still clears the trench (0.0 = fully flat).
        public static final double TRENCH_HOOD_THRESHOLD = 0.05;
        private static final double TRENCH_APPROACH_MARGIN = 0.5; // meters beyond the normal buffer

        /** Returns true if the robot is approaching a trench zone (wider than isInTrenchZone).
         *  Used to auto-slow the drivetrain so the hood servo has time to retract. */
        public static boolean isNearTrenchZone(double robotX) {
            return (robotX >= BLUE_TRENCH_MIN_X - TRENCH_APPROACH_MARGIN && robotX <= BLUE_TRENCH_MAX_X + TRENCH_APPROACH_MARGIN)
                || (robotX >= RED_TRENCH_MIN_X - TRENCH_APPROACH_MARGIN && robotX <= RED_TRENCH_MAX_X + TRENCH_APPROACH_MARGIN);
        }

        // ===== HUB POSITIONS =====
        // Derived from 2026 WPILib AprilTag field JSON.
        // Blue hub center: midpoint of tags 25/26 (x=4.022) and 19/20 (x=5.229),
        //                  midpoint of tags 27/18 (y=3.431) and 24/21 (y=4.638)
        public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.626, 4.035, new Rotation2d());
        // Red hub center: midpoint of tags 3/4 (x=11.312) and 9/10 (x=12.519)
        public static final Pose2d RED_HUB_POSE = new Pose2d(11.916, 4.035, new Rotation2d());

        // ===== PASSING TARGETS (for lobbing from opponent's side) =====
        // When the robot crosses the alliance line into neutral/opponent territory,
        // lob the ball just barely inside our alliance zone so a teammate can grab it.
        // X is 0.5m inside the alliance line; Y is 4m from whichever side wall is closer.
        public static final double PASSING_TARGET_Y_INSET = 2.25;    // 2.25m from side wall

        // ===== TURRET AND TARGET HEIGHTS (for 3D trajectory physics) =====
        // Turret launch point height above the floor (measure from CAD or robot).
        // This is the Z coordinate where the ball leaves the shooter.
        public static final double TURRET_HEIGHT_METERS = 0.5334;  // ~21 inches — measured from floor to ball exit point
        // Passing target landing height — ground level (ball should land on the floor)
        public static final double PASSING_TARGET_HEIGHT_METERS = 0.0;

        // Set to true to use physics-based trajectory for passing shots.
        // Set to false to fall back to the interpolation tables (safe for matches).
        // This is the compile-time default — co-pilot Back button toggles at runtime.
        public static final boolean USE_TRAJECTORY_PASSING_DEFAULT = false;

        public static final Pose2d BLUE_PASS_LEFT  = new Pose2d(
            BLUE_ZONE_END - 0.5, FIELD_WIDTH_METERS - PASSING_TARGET_Y_INSET, new Rotation2d());
        public static final Pose2d BLUE_PASS_RIGHT = new Pose2d(
            BLUE_ZONE_END - 0.5, PASSING_TARGET_Y_INSET, new Rotation2d());
        // Red passing targets (just inside red alliance zone)
        public static final Pose2d RED_PASS_LEFT   = new Pose2d(
            RED_ZONE_START + 0.5, FIELD_WIDTH_METERS - PASSING_TARGET_Y_INSET, new Rotation2d());
        public static final Pose2d RED_PASS_RIGHT  = new Pose2d(
            RED_ZONE_START + 0.5, PASSING_TARGET_Y_INSET, new Rotation2d());

    }
}
