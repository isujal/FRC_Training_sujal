# Test Bench – how to use it

Pre-season testing for any mechanism: roller tests, position tests, stress tests, and live tuning from
Elastic **without redeploying**.

---

## 1. The one rule about units

| What | Unit | Example |
|---|---|---|
| Speed | **RPM of the mechanism output** (roller / wheel), not the motor | `TargetRPM = 1800` |
| Position | **degrees of the mechanism** (or inches/meters if you chose that) | `PresetA (deg) = 120` |
| Gains (kP, kS, kV, kG…) | **volts** | `kV = 0.12` |

The gear ratio in the config converts between motor and mechanism, inside the motor controller. With
`gearRatio = 1` (the default), mechanism RPM = motor RPM. Telemetry shows both `Speed RPM` (mechanism)
and `Motor RPM`.

The competition code (`frc.robot.subsystems`) uses the same idea: every `...RPM(...)` method takes
**motor RPM** and divides by 60 before sending, because Phoenix wants rotations per **second**.

---

## 2. Turning the test bench on and off

`src/main/java/frc/robot/Constants.java`:

```java
public static final boolean kTestBenchMode = true;   // true = test bench, false = competition robot
```

Deploy after changing it. Exactly one of the two "containers" is created, so they never fight over CAN IDs:

- `true`  → `frc.robot.testbench.TestBenchContainer`
- `false` → `frc.robot.RobotContainer` (your normal robot, unchanged)

**Set it back to `false` before any match.**

---

## 3. Files (all in `src/main/java/frc/robot/testbench/`)

| File | What it is | Do you edit it? |
|---|---|---|
| `MechanismPresets.java` | **The list of mechanisms** (CAN IDs, gear ratio, limits…) | **Yes – this is the only one** |
| `MechanismConfig.java` | All the settings a mechanism can have | Rarely |
| `RollerMechanism.java` | Preset for spinning things | No |
| `PositionMechanism.java` | Preset for arms / hoods / elevators | No |
| `RollerTests.java` / `PositionTests.java` | The automatic tests | Only to add a test |
| `TestMechanism.java` | Shared part: safety, telemetry, live config | No |
| `TestCommand.java` | Shared part of every test (timing, verdict, results) | No |
| `TunableNumber.java` | A number you can change live in Elastic | No |
| `TestBenchContainer.java` | Controller buttons, Elastic buttons, optional drive | Rarely |

---

## 4. Mechanical brings a new mechanism

1. Open `MechanismPresets.java`.
2. Copy a template at the bottom (roller, arm or elevator).
3. Fill in: name, CAN IDs (leader + followers, `true` = follower spins the opposite way), gear ratio,
   range/start position for an arm, and **a low `maxVolts` (e.g. 4–6 V) for the first run**.
4. Comment out mechanisms that are not plugged in (otherwise CAN errors).
5. Deploy **once**. Everything after this is live.

Example:

```java
new RollerMechanism(MechanismConfig.named("Gripper")
    .leader(25).follower(26, true)
    .gearRatio(4.0)            // 4:1 -> TargetRPM is gripper-wheel RPM
    .maxVolts(6.0)
    .currentLimits(40, 30)
    .targetRPM(800)),
```

Position mechanisms assume they are sitting at `startPosition` when the robot turns on. Put the arm
there before power-on, or run **Home To Hard Stop**.

---

## 5. Elastic setup (one time, ~5 minutes)

1. Connect to the robot, open Elastic.
2. Make a tab "Test Bench" and drag in from the NetworkTables tree:
   - `SmartDashboard/TestBench/Mechanism` → **ComboBox Chooser** (picks what the controller drives)
   - `SmartDashboard/TestBench/E-STOP ALL`, `Clear All Faults`, `Print All Tunables`,
     `Reset Tunables To Code` → **Command** buttons
   - `SmartDashboard/TestBench/Selected Last Result`, `Battery V`, `Tunables Changed From Code`
   - `SmartDashboard/TestBench/Tunables Report` → **Text Display** (make it tall: it is one line per
     changed value)
3. For each mechanism, make a tab (e.g. "Intake Rollers"):
   - Live values from `SmartDashboard/TestBench/Intake Rollers/…`: `Speed RPM`, `Target RPM (tunable)`,
     `Stator A`, `Max Temp C`, `Fault`, `OK (no fault)`, `Running`, `Last Result`.
     Use **Graph** widgets for Speed RPM and Stator A.
   - The test buttons: `SmartDashboard/TestBench/Intake Rollers/Tests/…` → Command widgets.
   - **The tunables**: `Preferences/Intake Rollers/…` → **Text Field** (or Number Slider).
     These are the values you change live.
4. `File → Save Layout` and commit the JSON to the repo so the whole team has it.

---

## 6. Controllers

**Tester controller – USB port 1** (drives whichever mechanism is selected in Elastic):

| Button | Roller | Position mechanism |
|---|---|---|
| **A** | run forward at `TargetRPM` (until B) | go to `PresetA` |
| **Y** | run reverse at `TargetRPM` | go to `PresetB` |
| **B** | **stop** + clear fault | **hold where it is** + clear fault |
| **X** | test: Run Until Current Spike | test: Step Response |
| **POV up / down** | `TargetRPM` ± `RPMStep` (live) | jog ± `JogStep` |
| **Right / left trigger** | open-loop volts ± (× `MaxVolts`) | open-loop volts ± (× `ManualVolts`); release = hold |
| **Start (hold 1 s)** | – | re-zero: "it is at startPosition now" |
| **Back** | **E-STOP every mechanism** | same |

**Driver controller – port 0** (only if `kIncludeDrive = true` in `TestBenchContainer`):
30% speed field-centric drive, LB = reset heading.

---

## 7. Live tuning ("make the roller faster")

1. Robot enabled in Teleop, roller running (press A).
2. In Elastic change `Preferences/Intake Rollers/TargetRPM` from 1800 to 2200 → it changes **right away**.
   Or press POV up on the tester controller (+100 RPM each press).
3. Gains (`Gains/kP`, `kV`…), `MaxVolts`, `StatorLimitA`, `SupplyLimitA`, Motion Magic speeds: change
   them the same way. They are re-sent to the motor automatically (the console prints
   "new gains/limits sent to motors").
4. Values are **saved on the roboRIO**, so they survive a reboot.
5. When you are happy, read **`TestBench/Tunables Report`** in Elastic. It lists only the values you
   changed, e.g. `Intake Rollers/TargetRPM = 2200   (code: 1800)`, and updates by itself. Copy those
   into the code (`MechanismPresets.java` or the competition subsystem).
   (**Print All Tunables** prints the full list, changed or not, to the Driver Station console / RioLog.)
6. **Reset Tunables To Code** throws all saved values away. Do this after copying the values into the
   code. Otherwise the saved values keep overriding the code, and `Tunables Changed From Code` stays
   above 0 as a reminder.

---

## 8. Safety (always on, every mechanism)

| Protection | Default | Tunable at |
|---|---|---|
| Stall cut-off: current > X with no movement for Y s | 60 A, 1.0 s | `<name>/Safety/StallCurrentA`, `StallTimeS` |
| Temperature cut-off | 75 °C | `<name>/Safety/TempCutoffC` |
| Max volts | from config | `<name>/MaxVolts` |
| Current limits | from config | `<name>/StatorLimitA`, `SupplyLimitA` |
| Soft limits (position) | config range | – |
| E-stop | Back button / Elastic | – |

A tripped mechanism **stays off** and shows the reason in `Fault`. Press **B** (or the Elastic
"Stop + Clear Fault" button) to clear it. Brake/Coast buttons work while disabled, so mechanical can
move things by hand.

---

## 9. The tests and what to tell mechanical

Start a test with its Elastic button (click again to cancel) or X for the quick test. Every test ends with
**PASS / WARN / FAIL** + a one-line note, shown in `TestBench/<name>/Last Result`. The numbers are in
`TestBench/<name>/Results/<test>/…`. A summary line is also printed to the console and saved in the log.

Test settings are tunable under `Preferences/<name>/Test/…`. They are read when the test **starts**.

### Rollers

| Test | What it does | What to tell mechanical |
|---|---|---|
| **Run Until Current Spike** | runs at TargetRPM until current > `SpikeCurrentA`, then stops (or holds `HoldVoltsAfterSpike`, e.g. a gripper) | time to grab the piece, peak current; good threshold for game-piece detection |
| **Endurance Soak** | runs `SoakMinutes` (or until `SoakTempLimitC`) | temperature rise per minute, average current, speed steadiness (rubbing / belt slip), battery sag |
| **Jam Detect + Auto Unjam** | on a spike: reverse `UnjamTimeS`, retry, count jams | how often it jams with real game pieces |
| **Speed Ramp + Free Speed Check** | 2 V, 4 V, … steps, **no game piece** | % of expected free speed: < 85% = friction / rubbing / wrong ratio. Also suggests **kS and kV** → type them into the Gains |
| **Flywheel Spin-up + Recovery** | spin to TargetRPM, then feed pieces | spin-up time, RPM dip per shot, recovery time |

### Position mechanisms

| Test | What it does | What to tell mechanical |
|---|---|---|
| **Home To Hard Stop** | drives at `HomeVolts` into the stop until current > `HomeCurrentA`, then zeroes | – (sets the zero) |
| **Range Sweep** | slow min → max → min | peak current **and at which angle**, the worst lag and where = tight / binding spot |
| **Step Response** | Preset A → Preset B | rise time, overshoot, settling time (PID tuning) |
| **Repeatability** | A ↔ B `RepeatCycles` times | error spread, move time, temperature rise (gearbox stress test) |
| **Hold Test** | hold still `HoldTimeS` | holding current: high = needs counterbalance / gas spring / more reduction |
| **Find kS + kG** | slow voltage ramp up and down until it moves | suggested **kS and kG**. For an arm, run it near horizontal |

Note: positions come from the **motor** encoder. Backlash or a slipping belt between the motor and the
mechanism is not visible yet. That comes when we add a CANcoder.

---

## 10. A typical first test of a new mechanism

1. Add it to `MechanismPresets.java` with `maxVolts(4)`, deploy.
2. Elastic: select it, set **Coast** and let mechanical turn it by hand. Check that `Speed RPM` / `Position`
   moves the right way (wrong sign → flip `.inverted(...)`; follower fighting → flip its `true/false`).
3. Enable. Gently use the **triggers**. Check every motor under `Motors/…` has similar current.
   One follower at 0 A or much higher than the rest = wiring / CAN ID / direction problem.
4. Roller: **Speed Ramp** → copy suggested kS / kV into Gains → press A, tune `TargetRPM` live.
   Arm: **Home**, **Find kS + kG** → copy into Gains → raise `kP` until **Step Response** passes.
5. Raise `MaxVolts` step by step to the real value.
6. Stress: **Endurance Soak** / **Repeatability**. Record the results.
7. **Print All Tunables** → copy the final values into code → **Reset Tunables To Code**.

---

## 11. Graphs after the test

Every value is also logged to a `.wpilog` file (on a USB stick in the roboRIO if one is plugged in, else
`/home/lvuser/logs`). Open it in **AdvantageScope** to plot current, RPM and temperature over the whole
session, and show the graphs to mechanical.
