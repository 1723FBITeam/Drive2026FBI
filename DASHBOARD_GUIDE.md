# Shuffleboard Dashboard Guide

## Saving & Sharing Layouts

Shuffleboard layouts are saved as `.json` files. To share across computers/team members:

1. Set up your layout in Shuffleboard (drag widgets, resize, arrange tabs)
2. **File → Save As** → save as `shuffleboard.json` in the project root
3. Commit to Git so everyone has it
4. On another computer: **File → Open** → select the file from the repo
5. Optional: **File → Preferences → Default Layout** → point to the file so it loads automatically

The layout file lives at: `C:\Code\FBI\Drive2026FBI\shuffleboard.json`

Layouts reference NetworkTables keys by name — as long as the robot code is the same, the layout works on any computer.

---

## Priority Dashboard Items

Set up your dashboard with these items in order of importance. Drag them from the NetworkTables source panel on the left side of Shuffleboard.

### Tier 1 — Must Have (driver/co-pilot needs these during a match)

| NetworkTables Path | Type | What It Shows |
|---|---|---|
| `SmartDashboard/Auto Mode` | Chooser | Autonomous routine selector |
| `SmartDashboard/Auto-Aim` | Boolean | Auto-aim ON/OFF status (green = auto, red = manual) |
| `Calibration/Alliance` | String | RED, BLUE, or NOT SET |
| `Calibration/Dist to Hub` | Double | Distance to hub in meters |
| `Calibration/Turret Pos` | Double | Current turret position (mechanism rotations) |
| `Calibration/Turret Target` | Double | Where turret is trying to aim |
| `Shuffleboard/Calibration/Ready To Shoot` | Boolean | Flywheels at speed (green = ready) |

### Tier 2 — Important for Tuning (co-pilot watches these)

| NetworkTables Path | Type | What It Shows |
|---|---|---|
| `Calibration/Turret Aim Offset Deg` | Double | Current aim offset from D-pad (degrees) |
| `Shuffleboard/Calibration/Shooter RPS Offset` | Double | Current power offset from D-pad (RPS) |
| `Shuffleboard/Calibration/Shooter Target (RPS)` | Double | Target flywheel speed |
| `Shuffleboard/Calibration/Shooter L Vel (RPS)` | Double | Left flywheel actual speed |
| `Shuffleboard/Calibration/Shooter R Vel (RPS)` | Double | Right flywheel actual speed |
| `Shuffleboard/Calibration/Hood Position` | Double | Current hood servo position (0.0–1.0) |
| `Calibration/Turret Error Deg` | Double | How far off the turret is from target (degrees) |
| `Calibration/Turret Resetting` | Boolean | Turret doing a long wrap-around move |

### Tier 3 — Vision & Position (useful for debugging)

| NetworkTables Path | Type | What It Shows |
|---|---|---|
| `Vision/Front Tags` | Integer | How many AprilTags the front camera sees (0 = no data) |
| `Vision/Back Tags` | Integer | How many AprilTags the back camera sees (0 = no data) |
| `Vision/Front Avg Dist` | Double | Average distance to tags (front camera, meters) |
| `Vision/Back Avg Dist` | Double | Average distance to tags (back camera, meters) |
| `Vision/Status` | String | ACTIVE, or why vision is rejected (spinning, driving fast) |
| `Calibration/Robot X` | Double | Robot X position on field (meters) |
| `Calibration/Robot Y` | Double | Robot Y position on field (meters) |
| `Calibration/Robot Heading` | Double | Robot heading (degrees) |
| `Calibration/Target X` | Double | Hub X position being aimed at |
| `Calibration/Target Y` | Double | Hub Y position being aimed at |

### Tier 4 — Intake & Climber (check occasionally)

| NetworkTables Path | Type | What It Shows |
|---|---|---|
| `Shuffleboard/Calibration/Intake L Pos` | Double | Left deploy motor position |
| `Shuffleboard/Calibration/Intake R Pos` | Double | Right deploy motor position |
| `Shuffleboard/Calibration/Intake Avg Pos` | Double | Average deploy position |
| `SmartDashboard/Left Climb Position` | Double | Left climber position |
| `SmartDashboard/Right Climb Position` | Double | Right climber position |
| `SmartDashboard/Elevator Position` | Double | Elevator position |

---

## Recommended Layout

### Tab 1: "Match" (driver/co-pilot during competition)
- Auto Mode chooser (top left, wide)
- Auto-Aim boolean (large, color-coded)
- Alliance string
- Ready To Shoot boolean (large, color-coded)
- Dist to Hub
- Turret Aim Offset Deg
- Shooter RPS Offset

### Tab 2: "Tuning" (co-pilot during practice)
- All Tier 2 items
- Robot Heading
- Turret Pos and Turret Target side by side
- Turret Error Deg

### Tab 3: "Debug" (pit crew / programming)
- All Tier 3 position items
- Intake positions
- Climber positions

---

## Tips

- Use **Boolean Box** widget type for Ready To Shoot and Auto-Aim — they show green/red which is easy to see from across the field
- Right-click any widget → **Change to...** to pick a different display type (graph, dial, text, etc.)
- Graphs are useful for flywheel velocity — you can see if it's oscillating
- Keep the Match tab clean — only what the driver/co-pilot needs at a glance
- All telemetry updates at ~10Hz to keep Shuffleboard responsive
