# FRC Robot Code — Team Project

## Controller Layout — Driver (Controller 1, USB port 0)

The driver handles driving, shooting, intake, and field reset.

| Button/Input     | Action                                           |
|------------------|--------------------------------------------------|
| Left Stick       | Drive forward/backward and strafe left/right     |
| Right Stick X    | Rotate the robot                                 |
| Y Button         | Toggle auto-shoot on/off (aim + spin + feed)     |
| X Button         | Toggle intake on/off (deploy + rollers)          |
| A Button         | Jostle intake (unstick balls)                    |
| B Button         | Toggle elevator level-and-return cycle           |
| Right Bumper     | Toggle climber gearbox lock/unlock               |
| Back             | Reset field-centric heading (alliance-aware)     |

## Controller Layout — Co-Pilot (Controller 2, USB port 1)

The co-pilot handles shooting overrides, elevator, tuning, and manual overrides.
Two modes controlled by Start button:

### Auto-Aim ON (default)
Turret auto-tracks the hub. Triggers disabled.

| Button/Input     | Action                                           |
|------------------|--------------------------------------------------|
| Y Button         | Toggle auto-shoot on/off                         |
| X Button         | Move elevator up slowly (hold)                   |
| A Button         | Move elevator down slowly (hold)                 |
| B Button         | Emergency stop all shooter motors                |
| Left Bumper      | Nudge hood servo up (hold)                       |
| Right Bumper     | Nudge hood servo down (hold)                     |
| D-pad Left       | Nudge turret aim left (CCW) by 0.5 degrees       |
| D-pad Right      | Nudge turret aim right (CW) by 0.5 degrees       |
| D-pad Up         | Increase shooter power by 1 RPS (~60 RPM)        |
| D-pad Down       | Decrease shooter power by 1 RPS (~60 RPM)        |
| Start            | Toggle auto-aim OFF (enter manual mode)          |
| Right Stick Press| Toggle Limelight vision on/off                   |
| Back             | Toggle trajectory passing mode (physics vs tables)|

### Auto-Aim OFF (manual mode)
Co-pilot aims turret with triggers. Flywheels still auto-calculated from distance.

| Button/Input     | Action                                           |
|------------------|--------------------------------------------------|
| Left Trigger     | Manually rotate turret left (pressure = speed)   |
| Right Trigger    | Manually rotate turret right (pressure = speed)  |
| B Button (hold)  | Fire — spins flywheels + feeds ball              |
| Left Bumper      | Nudge hood servo up (hold)                       |
| Right Bumper     | Nudge hood servo down (hold)                     |
| D-pad            | Same offsets as auto mode                        |
| Start            | Toggle auto-aim back ON                          |

Aim/power offsets accumulate — pressing D-pad Left 3 times shifts aim by 1.5 degrees. Current offsets are shown on the Calibration tab in Shuffleboard.

---

## What Is This?

This is the code for our FRC competition robot. It controls a swerve drive base with a turret-mounted shooter, an intake system, and a climber. The robot can automatically aim and shoot at the hub using vision (Limelight) and sensor fusion.

## How the Robot Works (Big Picture)

1. The driver uses an Xbox controller to drive the robot around the field using swerve drive (the robot can move in any direction while facing any direction).
2. The intake picks up game pieces (notes) from the ground.
3. The turret automatically rotates to face the scoring target no matter which way the robot is facing.
4. The shooter spins up flywheels and adjusts the hood angle based on distance, then feeds the note to score.
5. A co-pilot on a second controller can manage shooting/intake and live-tune aim and power offsets during a match.
6. The robot uses zone-based smart targeting — it aims at the hub on our side, flattens the hood automatically near the trench, and lobs passing shots when on the opponent's side.
7. The climber is used during endgame to climb. The elevator has a level-and-return cycle and a gearbox lock.

## Project Structure — Where to Find Things

```
src/main/java/frc/robot/
│
├── Main.java                        — Entry point. Just starts the robot. Don't touch this.
├── Robot.java                       — Robot lifecycle (init, periodic, auto, teleop). Runs vision fusion.
├── RobotContainer.java              — MOST IMPORTANT FILE. Sets up all subsystems, controller buttons, and auto routines.
├── Constants.java                   — All CAN IDs, PWM ports, and field measurements in one place.
├── Telemetry.java                   — Sends drivetrain data to the dashboard for debugging.
├── LimelightHelpers.java            — Helper library for talking to the Limelight camera (don't edit).
│
├── subsystems/                      — Each mechanism on the robot is a "subsystem"
│   ├── CommandSwerveDrivetrain.java — Swerve drive (generated by CTRE Tuner X). Handles driving + PathPlanner autos.
│   ├── ShooterSubsystem.java       — Flywheels, feeder, indexer, and hood servo. Has distance-based auto-aim tables.
│   ├── TurretSubsystem.java        — Rotates the shooter to face the hub. Uses closed-loop position control.
│   ├── IntakeSubsystem.java        — Intake rollers + deploy mechanism to pick up notes.
│   └── ClimberSubsystem.java       — Climb motors, elevator, gearbox lock, and level cycle for endgame.
│
├── commands/                        — Complex actions that coordinate multiple subsystems
│   └── AutoShootCommand.java        — Auto-aims turret, sets hood+flywheel by distance, feeds when ready.
│
└── generated/
    └── TunerConstants.java          — Swerve module configs generated by Tuner X. Motor IDs, gear ratios, PID gains.
```

## Key Concepts for New Students

### Subsystems and Commands
- A "subsystem" represents a physical mechanism (drivetrain, shooter, intake, etc.)
- A "command" is an action the robot performs (drive, shoot, intake a note)
- Only ONE command can use a subsystem at a time — this prevents conflicts
- "Default commands" run when nothing else is using that subsystem

### Swerve Drive
- Each of the 4 wheels can spin AND steer independently
- This lets the robot drive in any direction while facing any direction
- "Field-centric" means pushing the stick forward always drives away from the driver, no matter which way the robot faces

### Turret Range and Reset
- The turret can rotate 420° counterclockwise and 245° clockwise from center (665° total travel)
- Because the range exceeds 360°, any target angle has two equivalent positions — the code picks whichever is in bounds and closest
- When the turret needs to wrap around 360° to reach a target (a "reset"), it uses gentler PID gains (Slot 1) to avoid overshoot
- The shooter will NOT fire during a turret reset — it waits until the turret finishes and is aimed again

### Closed-Loop Control
- "Open loop" = set motor to X% power and hope for the best
- "Closed loop" = tell the motor "go THIS fast" or "go to THIS position" and it adjusts itself using PID
- The shooter flywheels use closed-loop velocity (consistent shot speed regardless of battery)
- The turret uses closed-loop position (accurate aiming) with two PID slots:
  - Slot 0 (normal aim): KP=30, KI=0.5, KD=0.2 — aggressive tracking
  - Slot 1 (reset moves): KP=8, KD=0.5, KV=0.5 — gentle approach to avoid overshoot

### Vision (Limelight + MegaTag2)
- The Limelight camera sees AprilTags on the field to figure out where the robot is
- This "vision pose" is fused with wheel odometry using a Kalman filter for accuracy
- The robot uses this position to calculate distance and angle to the hub
- Co-pilot can toggle vision off (right stick press) if it's causing problems during a match

### Interpolation Tables
- The shooter has lookup tables that map distance → hood angle and flywheel speed
- Between known data points, it interpolates (estimates) the right values
- You calibrate these by shooting from different distances and recording what works

### Two Shooting Methods (Hood + Flywheel Calculation)
The turret aiming is always the same — it tracks the target using closed-loop position control with velocity compensation. What changes between the two methods is how the hood angle and flywheel speed are calculated:

- **Interpolation Tables (`autoAim`)** — the proven, match-safe method. Uses hand-calibrated lookup tables in `ShooterSubsystem.java` that map distance to hood position and flywheel RPS. Accurate for hub shots within the calibrated range (1.3m–4.7m). Used for all hub shots, and for passing shots when trajectory mode is OFF.

- **Trajectory Physics (`passAutoAim`)** — the experimental method. Uses real projectile math in `TrajectoryCalculations.java` to solve for the required exit velocity and launch angle given the distance and height difference. Better for passing shots at longer/unusual distances where the interpolation tables weren't calibrated. If the physics solver can't find a solution, it automatically falls back to `autoAim`.

The co-pilot Back button toggles between these for passing shots at runtime (defaults to OFF = interpolation tables). Hub shots always use interpolation tables regardless of the toggle. The current mode is shown on the dashboard as "Trajectory Passing".

**To make trajectory physics accurate, you need to:**
1. Count the pulley teeth on the flywheel system (to validate the motor RPS → ball speed conversion)
2. Measure the actual hood angle at servo position 1.0 (currently estimated at 60°)
3. Measure `TURRET_HEIGHT_METERS` from the floor to the ball exit point (currently estimated at 0.46m)

### Zone-Based Smart Targeting (2026 REBUILT)
The field is divided into zones, and the robot automatically changes what it aims at:

- **Own Alliance Zone** (0–4.03m from your wall): Aim at the hub. Normal shooting.
- **Trench Zone** (±0.5m around the trench arms at ~4.6m blue / ~11.9m red): Hood automatically flattens to 0.0 so the robot fits under the trench (22.25in / 56.5cm clearance). Flywheels stop, no feeding.
- **Neutral Zone / Opponent's Side** (past the alliance line): Aim at a passing target — lob the ball 0.5m inside our alliance line, 2.25m from whichever side wall is closer. Left/right side has 1m hysteresis so the target doesn't flip-flop near the Y midline.

The target updates every 20ms as the robot moves, so transitions between zones are seamless. The driver presses Y once and the system handles everything automatically. This logic lives in `RobotContainer.java` → `getSmartTarget()`. Zone boundaries and positions are defined in `Constants.java` → `FieldConstants`.

**During autonomous**, the robot only shoots at the hub (no passing targets). PathPlanner named commands use `getAllianceHub()` directly.

## Iterative Velocity Compensation (Shooting While Moving)

When auto-shoot is active (Y button), the turret compensates for robot movement using an iterative refinement algorithm inspired by [The Flying Circuits (FRC 2026-Robot)](https://github.com/TheFlyingCircuits/2026-Robot).

Instead of a simple "predict where the robot will be in X seconds" approach, the system:
1. Calculates the time-of-flight to the target based on distance and estimated shot speed
2. Shifts the virtual target backwards by (robot velocity × flight time)
3. Recalculates — because shifting the target changes the distance, which changes the flight time
4. Repeats 4 times until the answer converges

This is more accurate than a fixed time offset because the compensation amount adapts to the actual shot distance. The same compensated target is used for both turret aiming and distance-based hood/flywheel calculations, keeping everything consistent.

**Key constants in `TurretSubsystem.java`:**
- `SHOT_SPEED_MPS` (default 10.0 m/s) — estimated average ball speed in flight. Tune by measuring actual shot speed.
- `COMPENSATION_ITERATIONS` (default 4) — number of refinement passes. 4 is plenty.

**How to tune:**
1. Drive sideways at a consistent speed past the hub with auto-shoot on
2. If shots consistently land **behind** you (where you were): increase `SHOT_SPEED_MPS` (ball is faster than estimated, so less compensation needed — wait, actually decrease it so flight time increases and compensation increases)
3. If shots consistently land **ahead** of you: increase `SHOT_SPEED_MPS` (shorter estimated flight time = less compensation)
4. Adjust in increments of 1.0 m/s

## PathPlanner Named Commands

These are registered in `RobotContainer` for use in PathPlanner autonomous routines:

| Command Name  | What It Does                                              |
|---------------|-----------------------------------------------------------|
| `AutoShoot`   | Aims turret at hub, spins flywheels, feeds when ready     |
| `StopShooter` | Stops flywheels, feeder, indexer, flattens hood           |
| `Intake`      | Deploys intake out and runs rollers                       |
| `StopIntake`  | Stops rollers and retracts intake                         |
| `Climb`       | Starts the climber level cycle                            |

The turret default command handles hub tracking automatically — no separate named command needed for turret aiming during auto.

## How to Calibrate the Shooter

1. Drive to a known distance from the hub
2. Press Y to start the auto-shoot sequence
3. Check "Dist to Hub" on the Calibration tab in Shuffleboard
4. Use co-pilot bumpers to nudge the hood until shots land consistently
5. Use co-pilot D-pad Up/Down to nudge flywheel power
6. Update the tables in `ShooterSubsystem.java` with your new data points

## Building and Deploying

- Build: `./gradlew build`
- Deploy to robot: Use WPILib VS Code extension (Ctrl+Shift+P → "Deploy Robot Code")
- Note: `JAVA_HOME` may not be set in your shell — use the WPILib deploy command instead of `./gradlew deploy` directly.

## Dependencies

- WPILib 2026
- CTRE Phoenix 6 (TalonFX, CANcoder, Pigeon 2)
- PathPlanner (autonomous path following)
- Limelight (vision processing — MegaTag2 with AprilTags)
