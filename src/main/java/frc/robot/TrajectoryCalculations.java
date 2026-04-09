package frc.robot;

/**
 * TrajectoryCalculations — projectile physics for passing shots.
 *
 * Instead of using interpolation tables (which are calibrated for hub shots),
 * passing shots use real physics to calculate the required exit velocity and
 * launch angle to lob a ball to a target at a given distance and height.
 *
 * The math solves the standard 2D projectile equations:
 *   x(t) = v₀ · cos(θ) · t
 *   z(t) = v₀ · sin(θ) · t - ½gt²
 *
 * Given a desired angle of attack (the angle the ball hits the ground at the
 * target), we can solve for the required exit velocity and launch angle.
 *
 * Inspired by TheFlyingCircuits/2026-Robot TurretCalculations.java.
 */
public class TrajectoryCalculations {

    private static final double GRAVITY = 9.81; // m/s²

    /**
     * Solve for the required exit velocity and launch angle to hit a target.
     *
     * @param horizontalDistance  XY distance from turret to target (meters)
     * @param heightDifference    target Z minus turret Z (negative = target is below turret)
     * @param angleOfAttackDeg    desired angle of attack at the target (negative = ball coming down)
     *                            e.g., -40 means the ball arrives at 40° below horizontal
     * @return double[3]: [0] = exit velocity (m/s), [1] = launch angle (degrees), [2] = time of flight (seconds)
     *         Returns null if the solution is physically impossible (e.g., NaN from sqrt of negative)
     */
    public static double[] solve(double horizontalDistance, double heightDifference, double angleOfAttackDeg) {
        // Convert angle of attack to radians
        double aoaRad = Math.toRadians(angleOfAttackDeg);

        // Time of flight from the projectile equation:
        //   z = v₀z·t - ½g·t²
        //   x = v₀x·t
        // With angle of attack constraint, we get:
        //   t = sqrt( (2/g) · (Δz - d·tan(aoa)) )
        // where d = horizontal distance, Δz = height difference, aoa = angle of attack
        double inner = (2.0 / GRAVITY) * (heightDifference - horizontalDistance * Math.tan(aoaRad));
        if (inner < 0) {
            return null; // Physically impossible — can't reach target with this angle of attack
        }

        double timeOfFlight = Math.sqrt(inner);
        if (timeOfFlight < 0.01) {
            return null; // Target is essentially at the turret — no meaningful solution
        }

        // Horizontal and vertical velocity components
        double vXY = horizontalDistance / timeOfFlight;
        double vZ = (heightDifference + 0.5 * GRAVITY * timeOfFlight * timeOfFlight) / timeOfFlight;

        // Total exit velocity and launch angle
        double exitVelocity = Math.hypot(vXY, vZ);
        double launchAngleDeg = Math.toDegrees(Math.atan2(vZ, vXY));

        return new double[] { exitVelocity, launchAngleDeg, timeOfFlight };
    }

    /**
     * Get the angle of attack for a hub shot based on distance.
     * The ball needs to arrive at a steep enough angle to clear the 6ft hub.
     * Close range = very steep descent, far range = moderate descent.
     *
     * Range: -75° (close, ~1m) to -55° (far, ~7m)
     * Steepened from original values — ball was falling short of 6ft clearance.
     *
     * @param distanceMeters horizontal distance to the hub
     * @return angle of attack in degrees (negative = ball descending)
     */
    public static double getHubAngleOfAttack(double distanceMeters) {
        double minDist = 1.0;
        double maxDist = 7.0;
        double steepAngle = -75.0;
        double flatAngle = -55.0;

        double clamped = Math.max(minDist, Math.min(maxDist, distanceMeters));
        double fraction = (clamped - minDist) / (maxDist - minDist);
        return steepAngle + fraction * (flatAngle - steepAngle);
    }

    /**
     * Get the angle of attack for a passing shot based on distance.
     * Closer = steeper descent, farther = flatter trajectory.
     *
     * Range: -50° (close, ~3m) to -30° (far, ~10m)
     * Linearly interpolated between min and max distance.
     *
     * @param distanceMeters horizontal distance to the passing target
     * @return angle of attack in degrees (negative = ball descending)
     */
    public static double getPassingAngleOfAttack(double distanceMeters) {
        double minDist = 3.0;   // meters — closest expected passing distance
        double maxDist = 10.0;  // meters — farthest expected passing distance
        double steepAngle = -50.0;  // degrees at close range (steep lob)
        double flatAngle = -30.0;   // degrees at far range (flatter arc)

        // Clamp distance to the expected range
        double clamped = Math.max(minDist, Math.min(maxDist, distanceMeters));
        double fraction = (clamped - minDist) / (maxDist - minDist);
        return steepAngle + fraction * (flatAngle - steepAngle);
    }

    /**
     * Convert a required launch angle (degrees) to a hood servo position (0.0–1.0).
     *
     * The hood is INVERTED: servo 0.0 = steepest (most straight up),
     * higher servo values = flatter (more forward, lower launch angle).
     *
     * Adjusted based on calibration data:
     *   Servo 0.0 → ~80° above horizontal (nearly straight up)
     *   Servo 1.0 → ~60° above horizontal (angled forward)
     *   Total range: ~20° of launch angle change
     *
     * @param launchAngleDeg desired launch angle in degrees above horizontal
     * @return hood servo position (0.0 to 1.0), clamped to valid range
     */
    public static double launchAngleToHoodPosition(double launchAngleDeg) {
        double HOOD_MAX_ANGLE_DEG = 80.0;  // launch angle at servo 0.0 (steepest)
        double HOOD_MIN_ANGLE_DEG = 60.0;  // launch angle at servo 1.0 (flattest)
        double range = HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG; // 20°
        double position = (HOOD_MAX_ANGLE_DEG - launchAngleDeg) / range;
        return Math.max(0.0, Math.min(1.0, position));
    }

    /**
     * Convert a required exit velocity (m/s) to flywheel RPS.
     *
     * Hardware: 2× Kraken X60 motors (free speed 6271 RPM = 104.5 RPS) driving
     * 4-inch (0.1016m) flywheel wheels through a 24T:15T HTD 5mm belt reduction.
     * Gear ratio: 24/15 = 1.6:1 (wheels spin 1.6× motor speed).
     *
     * Theoretical conversion:
     *   Wheel surface speed = motor_RPS × 1.6 × π × 0.1016 = motor_RPS × 0.5107 m/s
     *   So theoretically: 1 / 0.5107 = 1.96 motor RPS per m/s of surface speed
     *
     * Back-calculated from calibration data:
     *   At 4.7m hub shot: 37.5 motor RPS produces a successful shot
     *   Estimated exit velocity at 4.7m: ~10 m/s
     *   Effective conversion: 37.5 / 10.0 = 3.75 motor RPS per m/s exit velocity
     *
     * The difference (3.75 vs 1.96) means ~52% transfer efficiency from wheel
     * surface to ball exit velocity. This is normal for compliant wheels launching
     * a compressible game piece — energy is lost to ball compression and slip.
     *
     * @param exitVelocityMPS required ball exit velocity in m/s
     * @return flywheel speed in motor RPS (what to command the TalonFX)
     */
    public static double exitVelocityToRPS(double exitVelocityMPS) {
        // 4.5 motor RPS per m/s of ball exit velocity.
        // Bumped from 4.25 — shots were consistently clipping the near lip
        // or barely making it in, indicating ~5% more power is needed.
        // Includes 24T:15T gear ratio (1.6:1) + ~44% transfer efficiency.
        double MOTOR_RPS_PER_MPS = 4.5;
        return exitVelocityMPS * MOTOR_RPS_PER_MPS;
    }
}
