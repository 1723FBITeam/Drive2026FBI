# Post-Match Analysis Guide

## Before the Match

1. Deploy the latest code (has DataLogManager enabled)
2. Clear old logs: SFTP to roboRIO → delete everything in `/home/lvuser/logs/`
3. Record a video from the stands — try to capture the full field
4. Note the match start time on your phone so you can sync video later

## After the Match

### Step 1: Pull the Log File

- Connect laptop to robot (USB or WiFi)
- SFTP to `roborio-1723-frc.local` (user: `admin` or `lvuser`?, password: blank)
- Download the newest `.wpilog` file from `/home/lvuser/logs/`
- Or use WPILib Start Menu → "RoboRIO Data Log Download Tool"

### Step 2: Open in AdvantageScope

- File → Open Log File → select the `.wpilog`
- The timeline at the bottom shows the full match

### Step 3: Set Up These Tabs

#### Tab 1: Field View (2D Field)
- Add `SmartDashboard/Field/Robot`
- Scrub the timeline to watch the robot's path through the match
- Look for: unexpected jumps (vision issue), weird paths (driver or auto issue)

#### Tab 2: Driver Inputs (Line Graph)
- `DriverStation/Joystick0/Axis0` — left stick X (strafe)
- `DriverStation/Joystick0/Axis1` — left stick Y (forward/back)
- `DriverStation/Joystick0/Axis4` — right stick X (rotation)
- Look for: jerky inputs = driver issue, smooth inputs but jerky robot = code issue

#### Tab 3: Vision Health (Line Graph)
- `Vision/Front Tags` and `Vision/Back Tags` — tag count per camera
- `Vision/Accepted` and `Vision/Rejected` — running totals
- `Vision/Pose Jump (m)` — distance between vision and odometry
- `Vision/Status` — text showing current vision state
- Look for: high rejected count = vision config issue, large pose jumps = heading or offset issue

#### Tab 4: Shooting Performance (Line Graph)
- `Calibration/Shot Distance` — distance to target
- `Calibration/Shot Hood Cmd` — hood servo position
- `Calibration/Shot RPS` — flywheel speed commanded
- `Calibration/Shot State` — AIMING / SPINNING UP / FIRING / TRENCH etc.
- `Calibration/Shot Feeding` — true when balls are being fed
- Look for: long AIMING periods = turret tracking issue, SPINNING UP = flywheels slow to recover

#### Tab 5: Turret Tracking (Line Graph)
- `Calibration/Turret Pos` — actual turret position
- `Calibration/Turret Target` — where it's trying to go
- `Calibration/Turret Error Deg` — difference between actual and target
- `Calibration/Turret Resetting` — true during 360° wrap-arounds
- Look for: large sustained error = turret can't keep up, frequent resets = robot spinning too much

#### Tab 6: Controller Buttons (Line Graph)
- `DriverStation/Joystick0/Button1` through `Button12` — driver buttons
- `DriverStation/Joystick1/Button1` through `Button12` — co-pilot buttons
- Look for: when auto-shoot was toggled, when intake was activated, unexpected button presses

### Step 4: Key Questions to Answer

| Question | Where to Look |
|----------|--------------|
| Was vision working? | Vision tab — Accepted should climb, tags > 0 |
| Was the robot shaky? | Compare Driver Inputs vs Field View — smooth sticks + jerky path = code issue |
| Why did a shot miss? | Shooting tab at that timestamp — check Shot State, distance, hood, turret error |
| Why didn't it shoot? | Shot State will say AIMING, SPINNING UP, TRENCH, HUB INACTIVE, etc. |
| Was the turret fast enough? | Turret tab — error should stay under 3° during normal tracking |
| Did the trench slow us down? | Field View + Shot State — look for TRENCH or NEAR TRENCH states |
| Was hub timing correct? | Check `SmartDashboard/Hub Active` against match time |

### Step 5: Save Your Layout

File → Save Layout — so you don't have to set up tabs every time.

### Step 6: Sync with Video

- Match time is in the log timeline (bottom of AdvantageScope)
- Auto starts at T=0, teleop at ~T=20s
- Scrub to the moment in the video where something happened and check what the code was doing

## Quick Reference: NetworkTables Keys

| Key | What It Shows |
|-----|--------------|
| `Vision/Front Tags` | Number of AprilTags seen by front camera |
| `Vision/Back Tags` | Number of AprilTags seen by back camera |
| `Vision/Accepted` | Total vision measurements fused |
| `Vision/Rejected` | Total vision measurements rejected |
| `Vision/Last Reject` | Why the last measurement was rejected |
| `Vision/Pose Jump (m)` | Distance between vision and odometry poses |
| `Calibration/Shot State` | Current shooting state (FIRING, AIMING, etc.) |
| `Calibration/Shot Distance` | Distance to target in meters |
| `Calibration/Turret Error Deg` | Turret aim error in degrees |
| `SmartDashboard/Hub Active` | Whether our hub is currently active |
| `SmartDashboard/Vision Enabled` | Whether co-pilot has vision on/off |
