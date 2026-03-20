# Suggested Enhancements

Inspired by analysis of [TheFlyingCircuits/2026-Robot](https://github.com/TheFlyingCircuits/2026-Robot) and comparison with our current codebase.

---

## 1. 3D Trajectory Physics for Passing Shots

**Priority:** Medium  
**Difficulty:** Hard  
**Files affected:** `TurretSubsystem.java`, new `TrajectoryCalculations.java`

**What:** Our current system treats the ball as a flat line traveling at `SHOT_SPEED_MPS = 10.0`. This works fine for hub shots (interpolation tables handle it), but for long passing shots (6–10m) the ball arcs significantly due to gravity. The Flying Circuits use real projectile physics — given a target position (including height), an angle of attack, and gravity, they calculate the exact exit velocity and launch angle needed.

**Why it helps:** Passing shots would land more accurately at long range. Currently our extrapolated hood/speed values are just linear guesses past 4.7m.

**How to implement:**
- Create a `TrajectoryCalculations` utility class with a `projectileSolve(targetXY, targetZ, angleOfAttack, turretHeight)` method
- Returns required exit velocity and launch angle
- Use this for passing shots instead of the interpolation tables (keep tables for hub shots since they're calibrated)
- Need to know turret height off the ground (measure from CAD or robot)

---

## 2. Distance-Adaptive Angle of Attack

**Priority:** Medium  
**Difficulty:** Medium  
**Files affected:** `ShooterSubsystem.java` or new utility class

**What:** The Flying Circuits adjust the angle of attack continuously based on distance — steeper when close to the hub, flatter when far away. They use different angle ranges for hub shots vs passing shots. Our hood table is similar in concept but only covers 5 discrete data points from 1.3–4.7m.

**Why it helps:** A continuous function is smoother than interpolating between 5 points, and it naturally extends to longer distances without relying on extrapolation.

**How to implement:**
- Add a method like `getAngleOfAttack(targetType, distanceMeters)` that linearly maps distance to an angle range
- Hub: e.g., -75° at 1m to -53° at 5.65m
- Passing: e.g., -40° at 2m to -30° at 7.5m
- Could replace or supplement the existing hood interpolation table

---

## 3. Pre-Aim (Turret Always Tracking, Flywheels Only on Demand)

**Priority:** High  
**Difficulty:** Easy  
**Files affected:** `RobotContainer.java`, `AutoShootCommand.java`

**What:** The Flying Circuits separate "aiming" from "shooting." Their turret always tracks the target (pre-aims), but flywheels only spin and the indexer only feeds when the driver presses a button. This means the turret is already locked on target the instant the driver decides to shoot — zero aim delay.

**Why it helps:** Faster time-to-first-shot. Currently our turret only starts tracking when Y is pressed (via AutoShootCommand). The turret default command does track the hub, but if auto-shoot isn't active, the flywheels are cold and the hood isn't set.

**How to implement:**
- The turret default command already auto-aims — this part is done
- Option A: Have the default command also pre-set the hood and pre-spin flywheels at low RPM (wastes some battery but instant shots)
- Option B: Keep current behavior but add a "pre-aim only" mode where the turret tracks but flywheels stay off until Y is pressed (this is basically what we have now)
- The real win is making sure the turret default command is always running `aimAtPose()` even when auto-shoot is off

---

## 4. Dual Tolerances (Tighter to Start, Looser to Continue)

**Priority:** High  
**Difficulty:** Easy  
**Files affected:** `AutoShootCommand.java`, `TurretSubsystem.java`, `ShooterSubsystem.java`

**What:** The Flying Circuits use two sets of tolerances:
- **Not shooting tolerances** (tight): turret must be within 1.2° and flywheels within 0.2 m/s to START feeding
- **While shooting tolerances** (loose): turret can drift to 10° and flywheels to 8 m/s before feeding STOPS

This prevents the shooter from cutting off mid-burst if the turret wobbles slightly or the flywheels dip momentarily.

**Why it helps:** Our current system uses a single tolerance (1.5° for turret, 5% for flywheels). If the turret drifts past 1.5° while feeding, it stops — then re-aims — then starts again, causing stuttery shot strings. Hysteresis would give smoother, more consistent multi-ball shots.

**How to implement:**
- Add `isAimedAtPose()` overload or parameter for tolerance
- In `AutoShootCommand`, track `isShooting` state:
  - If not shooting: use tight tolerance (e.g., 1.5° turret, 5% flywheels)
  - If already shooting: use loose tolerance (e.g., 5° turret, 15% flywheels)
- Switch back to tight tolerance when feeding stops

```java
// Example in AutoShootCommand.execute():
double aimTolerance = feeding ? 0.014 : 0.004;  // ~5° vs ~1.5° in mechanism rotations
boolean aimed = turret.isAimedAtPose(robotPose, targetPose, aimTolerance);
```

---

## 5. LED Feedback System

**Priority:** High  
**Difficulty:** Medium  
**Files affected:** New `LedSubsystem.java`, `RobotContainer.java`

**What:** The Flying Circuits have a full LED subsystem with:
- Alliance-color heartbeat pattern (pulses in blue or red)
- Traffic light progress bar (red → yellow → green) showing flywheel spinup progress
- Intake animations (orange blobs moving when intaking, green when game piece detected)
- Shot-fired flash animation
- Wipe transitions between patterns

**Why it helps:** The driver and human player can instantly see robot state without looking at a dashboard:
- Green = ready to shoot
- Red/yellow = spinning up
- Orange animation = intaking
- Flash = shot fired

**How to implement:**
- Create `LedSubsystem` using WPILib's `AddressableLED` and `AddressableLEDBuffer`
- Wire an LED strip to a PWM port on the roboRIO
- Add patterns: solid alliance color (idle), traffic light (spinup), green (ready), flash (shot)
- Trigger pattern changes from `AutoShootCommand` and intake commands
- Use `temporarilySwitchPattern()` approach so patterns return to default after completing

**Hardware needed:** WS2812B LED strip (e.g., 60 LEDs), connected to a roboRIO PWM port

---

## 6. Vision Trust Reset Command

**Priority:** Low  
**Difficulty:** Easy  
**Files affected:** `RobotContainer.java`, `CommandSwerveDrivetrain.java`

**What:** The Flying Circuits have a command that forces the pose estimator to fully trust the next vision reading, even if it's far from the current estimate. This is useful when you know the robot's pose is wrong (after being bumped, after a bad auto, etc.).

**Why it helps:** Our Back button resets the heading, but if the XY position is wrong (e.g., odometry drifted during a long match), there's no way to force-correct it. A "trust vision now" button would snap the robot's estimated position to what the Limelight sees.

**How to implement:**
- Add a method to `CommandSwerveDrivetrain` that temporarily sets vision standard deviations to near-zero for one update cycle
- Bind to a button (e.g., co-pilot Back, since it's now free after removing trench mode)
- The command should wait until the Limelight actually sees a tag before applying

---

## 7. Game Piece Auto-Pickup Assist

**Priority:** Low  
**Difficulty:** Hard  
**Files affected:** New command, `RobotContainer.java`, requires vision pipeline

**What:** The Flying Circuits use a camera to detect game piece clusters on the field and can auto-drive toward the closest one while the driver maintains partial control (driver can override rotation to reject a target).

**Why it helps:** Faster cycling — the driver doesn't have to perfectly line up with game pieces. The robot assists by steering toward the nearest one.

**How to implement:**
- Requires a camera pipeline that can detect game pieces (Limelight neural network pipeline, or a separate camera)
- Create a command that blends driver input with auto-aim toward the detected game piece
- Driver rotation input overrides the auto-aim (so they can reject a target)
- Only active while a button is held

**Hardware needed:** Camera with game piece detection capability (Limelight can do this with a trained neural network pipeline)

---

## Implementation Priority

| # | Enhancement | Priority | Effort | Impact |
|---|-------------|----------|--------|--------|
| 4 | Dual tolerances | High | Easy | Smoother shot strings |
| 3 | Pre-aim separation | High | Easy | Faster first shot |
| 5 | LED feedback | High | Medium | Driver awareness |
| 1 | 3D trajectory physics | Medium | Hard | Better passing accuracy |
| 2 | Distance-adaptive angle | Medium | Medium | Smoother hood control |
| 6 | Vision trust reset | Low | Easy | Recovery from drift |
| 7 | Game piece auto-pickup | Low | Hard | Faster cycling |
