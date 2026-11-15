# FTC24006 Robot Tuning Guide (from zero)

Complete workflow for bringing a brand-new robot (or a rebuilt one) from bare
hardware to a calibrated, auto-aiming competition machine. Work through the
stages in order — each one depends on the ones before it.

**Tuning order**: mechanical checks → servo endpoints → turret alignment →
drivetrain/odometry → Limelight setup → odo-field calibration → verification →
shooter characterization → aim model.

---

## 0. Conventions used everywhere

- **FTC Dashboard** (`http://192.168.43.1:8080` from the Driver Station laptop,
  phone hotspot or RC WiFi Direct): every tunable in this codebase is a
  `public static` registered with the dashboard. Values changed there are
  **live** but **reset when the Robot Controller app restarts**.
- **Copy-to-source rule**: the dashboard is for finding values; when one is
  final, edit it into the Java source as the new default. Tuned values that
  only ever lived on the dashboard are lost on restart.
- **Units**: Pedro/Drivetrain positions are **inches/radians** (field frame).
  Limelight math is **meters/radians** internally; telemetry shows both.
- Compile check after code edits:
  ```powershell
  $env:JAVA_HOME = "C:\Program Files\Android\Android Studio\jbr"
  .\gradlew.bat ":TeamCode:compileDebugJavaWithJavac"
  ```
  (from `FtcRobotController/`)

---

## 1. Bench setup

### 1.1 Driver Station configuration

Create/edit the configuration file on the Driver Station. Names must match the
code exactly:

| Device | Config name | Type |
|---|---|---|
| Drive motors | `frontLeft`, `frontRight`, `backLeft`, `backRight` | DcMotorEx (REV hub) |
| Odometry | `pinpoint` | goBILDA Pinpoint (I2C) |
| Shooter flywheels | `shooterLeft`, `shooterRight` | DcMotorEx |
| Shooter gate | `shootLimit` | Servo |
| Hood | `hood` | Servo |
| Turret | `turretServoL`, `turretServoR` | Servo (500-2500 us range set in code) |
| Intake motors | `intakeL`, `intakeR` | DcMotorEx |
| Flowers | `flowerL`, `flowerR` | Servo |
| Camera | `limelight` | Limelight 3A (Ethernet Device, USB 3.0 into Control Hub) |

### 1.2 Quick sanity pass

1. Boot the Control Hub, connect the Driver Station, activate the config.
2. Open the FTC Dashboard — confirm it connects.
3. Run any teleop and verify: motors spin, no `NullPointerException` in the
   Driver Station error log (a missing/renamed device shows up here).

---

## 2. Servo endpoints and positions — "Servo Tune"

OpMode: **Servo Tune** (`ServoTune.java`, Tuning group). One unified tuner for
all servo positions.

**Controls**

| Input | Action |
|---|---|
| `X` | Cycle target: hood → shootLimit open → shootLimit closed → flower INTAKING → INTAKING_FLOWER → STOP → OUTTAKING → flower R offset |
| dpad up/down | fine ±0.005 per loop (hold to repeat) |
| dpad left/right | coarse ±0.05 |
| `A` | Reset to 0.5 (flower offset → 0) |
| Dashboard `ServoTune/tuneValue` | Type an **exact position** — the current target follows it |

**Workflow per mechanism**

1. Cycle to the target, nudge until the mechanism is physically where it
   should be (gate fully open/closed, hood angle, flower up/down).
2. Fine-tune with dpad up/down; for repeatable comparisons, read the number
   off telemetry and type exact values into `tuneValue`.
3. Copy the final value into the source:
   - hood → `Shooter.hoodPos`
   - shootLimit → `Shooter.shootLimitOpen` / `shootLimitClosed`
   - flowers → `Intake.flowerPosIntaking` / `flowerPosIntakingFlower` /
     `flowerPosStop` / `flowerPosOuttaking`
   - left/right flower pair alignment → `Intake.flowerOffset`
4. Behavior check: run the real teleop and confirm each state (intake, stop,
   outtake, flower pickup) looks right.

Note: flower runtime positions read those statics directly, so a tuned value
works in matches immediately — copying to source only makes it survive app
restarts.

---

## 3. Turret alignment and scale

### 3.1 Servo pair alignment — "Turret Servo Align"

OpMode: **Turret Servo Align**. The two turret servos must point the same
direction; if one horn is a tooth off, trim it out here.

1. Hold the turret physically straight (or eyeball both servo arms parallel).
2. dpad up/down ±0.001, bumpers ±0.01, `A` reset — adjusts `Turret.ServoLoff`
   (applied to the left servo only) and re-centers every loop.
3. When the turret holds straight, copy `ServoLoff` into `Turret.java`.

### 3.2 Angle scale (`arctoDegree`)

Convention: servo **0.5 = turret pointing at robot front**. Servo range 0..1
is the turret's full 360° rotation; **0 and 1 are the same direction**
(straight backward) — that seam is the "turning point".

Calibrate degrees→servo-units:

1. Dashboard: `Turret/arctoDegree` (starts at last year's 0.155 — assume it's
   wrong for new hardware).
2. Command known angles (e.g. via `settoangle(0)` vs `settoangle(90)` in a
   bench test) and measure the turret's actual rotation.
3. `arctoDegree = servo position change / degrees rotated`. For a full ±180°
   sweep across the servo's ±0.5 travel, expect roughly `0.5/180 ≈ 0.0028`.
4. **Sign check**: command +10°. If the turret rotates the wrong way (CW vs
   CCW), negate `arctoDegree`.

### 3.3 Dead zone at the wrap point

`Turret.deadZoneDeg` (default 5): commands within ±5° of straight backward
(the servo 0↔1 seam, `>355°` or `<5°` in the turret's 0-360 frame) snap to the
nearest end. This stops the turret from swinging a full turn when the aim
wobbles across backward. Widen it if the turret still thrashes near the seam;
shrink it if you need rearward precision.

---

## 4. Drivetrain and odometry

### 4.1 Pinpoint sanity

1. Run a teleop, drive straight ~5 ft forward: `Odo` x should increase by ~60,
   y stay near 0, heading near 0.
2. Spin in place 10 times: x/y should barely move, heading should accumulate
   ±20π rad (wrap is fine).
3. If axes are swapped or signs flipped, fix the Pinpoint mounting parameters
   (x/y reversal, encoder directions) before anything else.

### 4.2 Pedro Pathing follower tuning

Follow the official order (https://pedropathing.com/docs): robot mass and
moment of inertia → drive radius → drive vector PID → heading PID →
translational + heading path following PIDs. Do this after the Pinpoint is
sane, since every test reads odometry.

### 4.3 Zeroing

`dpad_up` in teleops zeroes the odometry pose wherever the robot sits. In
matches, always start the auto from the pose the auto expects.

---

## 5. Limelight setup

One-time web UI configuration (http://limelight.ftc:5801 or the device IP):

1. **Pipeline slot 3** holds the AprilTag config (`Limelight.pipelineIndex = 3`
   in code — if you move it in the web UI, move the constant too).
2. AprilTag pipeline: enable **Full 3D**.
3. Enable **MegaTag2** (uses the gyro yaw we feed it every loop).
4. **Robot Coordinate Space**: enter the camera mount offset from robot center
   — required for MT2 accuracy. Do this physically, with a tape measure.
5. Field map: **not** uploaded by code at init. Run **TurnRed** or **TurnBlue**
   (Tuning group) once before matches/verification — they upload the correct
   Hive map (red tags 30-37, blue 38-45) and it **persists on the device**.
   Sanity check on their telemetry: visible tag IDs must match the alliance.

The code re-asserts the pipeline every second (a web UI tab can steal it) and
shows `LL map/orient POST` health on calibration opmodes.

---

## 6. Odometry → field calibration — "LL Calibration"

OpMode: **LL Calibration** (`LLCalibration.java`). Fits the transform (and
collects data for the MLP) mapping odometry pose → Limelight field pose.

**Controls**: drive normally; `X` = capture (robust average of the last ~20
readings, outlier-rejected — **park first**), `Y` = solve + report, `A` =
clear, `B` = force re-upload the other alliance's map, dpad_up = export CSV.

**Workflow**

1. Run **TurnRed** first (map on device).
2. Drive around so tags are visible; telemetry must show `LL field (m)` values
   (not "no MT2 pose") and `LL pipeline active 3`.
3. **Spread matters**: capture 20+ samples across as much of the field as
   possible — different positions AND headings. A tight cluster of samples
   produces a model that only works inside that box (learned the hard way:
   20 samples in a 30×19 in box → seductive numbers, invalid everywhere else).
4. `Y` to solve. Read the report:
   - **Translation RMS** target: **< 1.5 in good, < 2.5 in usable**.
   - **Rotation RMS** should be ~0.00° (MT2 yaw is fed from the Pinpoint).
   - Per-sample residuals > 3× RMS are flagged — recapture those spots.
5. dpad_up writes `/sdcard/FIRST/llcal_samples.csv`; pull it:
   ```powershell
   adb pull /sdcard/FIRST/llcal_samples.csv tools\llcal_samples.csv
   ```
   (USB or `adb connect <ip>:5555` after pairing; if WiFi adb misbehaves,
   `adb disconnect` + `adb kill-server` and use USB.)

---

## 7. Train and deploy the model

On the laptop (only dependency: numpy):

```powershell
cd c:\Users\Taven\Documents\FTC\FTC24006
python tools\train_mlp.py tools\llcal_samples.csv            # evaluate only
python tools\train_mlp.py tools\llcal_samples.csv --save tools\mlp_red.json
```

**Read the LOO (leave-one-out) table, not the train RMS.** The train set
flatters the net; LOO is the honest number.

| LOO result | Meaning / action |
|---|---|
| SE(3) ≤ MLP | The rigid transform wins — use it, skip the net |
| MLP < SE(3) by a real margin (≥ ~0.3 in median) | Net earns its place |
| MLP much better but samples are clustered | Local artifact — recapture with field-wide spread before trusting it |

Deploy: the trained artifact is regenerated into `RedMlp.java` (emitter
script; ask whoever maintains the tooling, currently it lives outside the
repo), then recompile. `LLVerification` needs no changes.

Architecture note: current champion is 6 inputs (LL x/y/z/rpy) → 3 hidden
layers of 6 → 2 outputs (odo x/y), ~140 params, chosen by grid search over
depth × width with LOO. If retraining on new data, re-run the sweep — the
winner depends on the dataset.

---

## 8. Verify — "LL Verification"

OpMode: **LL Verification** (`LLVerification.java`). No anchor, no setup spot.

1. Run **TurnRed** first.
2. Drive anywhere with tags in view; the first valid LL reading becomes the
   **bridge** (one odo↔LL pair). Park and press `X` anytime to re-bridge clean.
3. Drive your actual routes and read:
   - **Diff odo** — odometry drift vs the Limelight since the bridge (starts
     0, grows with drift). This is the number that matters for "how far can I
     trust the odo between tag sightings".
   - **Diff MLP** — the recorded net's error on top of the bridge. Only
     meaningful near the calibration data's coverage.
   - Verdicts: GOOD < 3 in && < 3°.

If Diff odo reads ~10 in after real driving, that's genuine Pinpoint-vs-LL
disagreement, not a software bug (the loop math is exact at the bridge by
construction — verified offline).

---

## 9. Shooter + aim data collection — "Aim Tune"

OpMode: **Aim Tune** (`AimTune.java`). The turret is locked at robot front;
**you aim by turning the whole robot**, so the recorded heading IS the field
aim angle.

**Setup (dashboard)**

| Static | Meaning |
|---|---|
| `AimModel/targetX`, `targetY` | Goal position in field frame, inches — **measure and set before collecting** (red-frame convention) |
| `Shooter/aimRPM` | Flywheel target (0 = off; runs continuously otherwise) |
| `Shooter/hoodPos` | Hood position, applied live |
| `AimTune/flyTime` | Your stopwatch ball flight time — set before each capture |

**Controls**: drive normally; `A` (hold) feeds balls; `X` = capture sample;
`Y` = export `/sdcard/FIRST/aim_samples.csv`.

**Collection protocol (~50 samples)**

1. Grid of positions: ~8-10 distances × ~5 lateral offsets across the shooting
   range. 2D coverage is required — the model input is (x, y).
2. At each spot: set `aimRPM`/`hoodPos`, turn the robot until balls score
   consistently, set `flyTime`, press `X` while parked and scoring.
3. Sample = {x, y, heading, aimRPM, hoodPos, flyTime}. Telemetry shows the
   live **dTheta residual** (your aimed heading minus geometric bearing) —
   that's exactly what the model learns.
4. Leave 3-4 distances completely unsampled as holdout.
5. Repeat 2-3 spots at very different robot headings — the solution should be
   heading-independent (if not, the turret/shooter has an asymmetry worth
   knowing about).

**Aim model plan** (once data exists): fit script compares pure-atan2 /
residual-MLP / absolute-MLP for the angle and poly / MLP for the ballistics
via LOO; winner is emitted into `AimModel.java` (signature `predict(x, y) →
{θ_field, rpm, hood, tAir}` is already wired). `AimSolver.solve(x, y, heading,
vx, vy)` then outputs the robot-relative turret angle (heading-subtracted,
wrapped, ±5° dead zone at backward) plus moving-shot lead compensation
(`−v_robot · tAir`).

---

## 10. Troubleshooting

**LL field pose all zeros / "no MT2 pose"** — check in this order (each one
produced this symptom at least once):
1. MegaTag2 disabled in the web UI.
2. Wrong pipeline slot (web UI says 3? code wants 3?).
3. Field map uploaded without field dimensions (the code's `HiveFieldMap`
   subclass re-adds them — if you change map code, verify with the web UI's
   map view).
4. Wrong alliance map for the visible tags (red tags 30-37 / blue 38-45).

**Pipeline keeps reverting**: an open web UI tab fights the code. The 1 Hz
re-assert guard handles it; close the tab anyway.

**`adb pull` over WiFi hangs**: `adb disconnect` then `adb kill-server`; use
USB and Android Studio's Device Explorer instead.

**Dashboard statics missing**: this app's DEX scan misses some classes —
registration is done explicitly in constructors (`withConfigRoot` +
`ReflectionConfig`). New tunable classes need the same two lines.

**Servo won't hold a tuned value**: values reset on app restart — copy into
source (see §0).

---

*Branch: `2027State` (robot "Albert", BIOBUZZ 2026-27). Update this guide when
the tuning stack changes.*
