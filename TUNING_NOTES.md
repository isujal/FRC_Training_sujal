# Tuning notes: what every gain does

All gains live in Elastic under `Preferences/<mechanism>/Gains/...` and Motion Magic under
`Preferences/<mechanism>/MotionMagic/...`. Change them while the robot is enabled; they are sent to the
motor immediately.

## 1. What the motor actually does every 1 ms

```
Volts out =  kS · sign(velocity)          ← beat friction
          +  kV · target velocity         ← hold the speed
          +  kA · target acceleration     ← speed up / slow down
          +  kG  (× cos(angle) for an ARM) ← fight gravity
          +  kP · error                   ← correct what's left over
          +  kI · (error added up over time)
          +  kD · (how fast the error is changing)
```

- **kS, kV, kA, kG = feedforward.** These are *predicted* volts, computed from where we want to be,
  before any error happens. Get them right and the mechanism is already almost on target.
- **kP, kI, kD = feedback (PID).** These *react* to the error that is left.

Rule of thumb: **feedforward does ~90% of the work, PID cleans up the last 10%.** Tune feedforward first.

## 2. Units in OUR code

Everything is measured at the **mechanism** (after the gear ratio), and output is **volts**.

| Gain | Unit | Example |
|---|---|---|
| kS | V | `0.15` = 0.15 V just to start moving |
| kV | V per (mechanism rotation / s) | Roller `0.12` → 1000 RPM (16.7 rps) needs 2 V |
| kA | V per (mechanism rotation / s²) | |
| kG | V | `0.4` = 0.4 V to hold against gravity (arm: when horizontal) |
| kP | V per rotation of error (position) · V per rps of error (velocity) | Pivot `250` → 1° error = 250/360 = **0.69 V** |
| kI | V per (rotation × second) of error | |
| kD | V per (rotation / s) of error change | |
| Cruise | deg/s (or in/s) | `90` = at most a quarter turn per second |
| Accel | deg/s² | `180` = reaches 90 deg/s in 0.5 s |
| Jerk | deg/s³ | `0` = off |

The theoretical kV is easy to compute: **kV = 12 V ÷ (mechanism free speed in rotations per second)**.

- Kraken X60 on a 100:1 arm: free speed = 6000 / 60 / 100 = 1 rps, so kV = 12.
- Falcon intake, 1:1: 6380 / 60 = 106 rps, so kV = 0.113.

## 3. Each gain, precisely

### kS — static friction (volts)
- **What:** the voltage needed just to *start* moving. It is added in the direction of motion.
- **Too low:** a position mechanism stops a little short of the target. A roller is sluggish at low RPM.
- **Too high:** it buzzes or chatters around the target, flipping direction.
- **How to find it:** run **Find kS + kG** (position) or **Speed Ramp** (roller). Typical values: 0.05–0.3 V
  for a bare motor, 0.2–1 V on a real mechanism.

### kV — velocity (volts per rps)
- **What:** volts per unit of speed. This is the main term for rollers and flywheels.
- **Too low:** a roller settles **below** the target RPM. An arm lags behind the motion profile.
- **Too high:** a roller settles **above** the target. An arm runs ahead of the profile and overshoots.
- **How to find it:** start with 12 ÷ free-speed rps (already in the code), then use the **Speed Ramp**
  result. Fine-tune by watching `Error RPM` with kP = 0: change kV until the error is ~0.

### kA — acceleration (volts per rps²)
- **What:** extra volts while speeding up or slowing down. This matters for heavy or high-inertia things:
  flywheels, long arms.
- **Too low:** it lags during spin-up / at the start of a move, then catches up.
- **Too high:** it jumps ahead at the start of a move.
- **How to find it:** leave it at **0** until everything else works. Then raise it in small steps (e.g. 0.01 for
  a roller, 0.1 for an arm) while watching the error during acceleration. SysId gives the exact value.

### kG — gravity (volts), position mechanisms only
- **What:** the voltage to hold the mechanism still against gravity.
  `ELEVATOR` = the same everywhere. `ARM` = kG × cos(angle): full at horizontal, 0 straight up/down.
- **Too low:** it sags and drifts down. Holding needs lots of kP.
- **Too high:** it drifts up.
- **How to find it:** **Find kS + kG**. On a bare motor with nothing attached, kG ≈ 0.

### kP — proportional (the main PID term)
- **What:** volts per unit of error. "The further away, the harder I push."
- **Too low:** slow, stops short, gets pushed around easily.
- **Too high:** overshoots and oscillates (bounces back and forth around the target).
- **How to tune:** double it until it starts to overshoot / oscillate in **Step Response**, then back off to ~60–70%
  of that value.

### kI — integral
- **What:** adds up the error over time and pushes harder the longer an error stays. It fixes a small error that
  never goes away.
- **Danger:** it "winds up": it overshoots badly and oscillates slowly.
- **How to tune:** **keep it at 0.** A steady error almost always means kS or kG is wrong; fix those instead.
  Only if a tiny error remains, try a very small value (e.g. kP / 100).

### kD — derivative
- **What:** reacts to how *fast* the error is changing, like a brake or shock absorber. It reduces overshoot.
- **Too low (0):** more overshoot.
- **Too high:** buzzing / jitter / a whining motor (it amplifies sensor noise).
- **How to tune:** if kP gives a little overshoot you can't remove otherwise, add kD in small steps
  (e.g. kP / 50, then kP / 20).

### Cruise velocity (deg/s) — Motion Magic
- **What:** the top speed of the motion profile.
- **Rule:** it must be **below what the motor can actually reach** at `MaxVolts`. Otherwise the mechanism can't
  follow the profile, lags, then overshoots. Safe start: **~70%** of `free speed × MaxVolts / 12`.
- **Example:** New Arm (100:1) at 6 V can do 360 × 6 / 12 = 180 deg/s, so cruise 90–120 is fine.

### Acceleration (deg/s²) — Motion Magic
- **What:** how quickly the profile speeds up to cruise and slows down to stop.
  Time to reach cruise = cruise ÷ accel.
- **Too high:** jerky starts. Current spikes, belts skip, it overshoots at the end.
- **Too low:** slow, lazy moves.
- **Start:** accel = 2 × cruise (reaches full speed in 0.5 s).

### Jerk (deg/s³) — Motion Magic
- **What:** how quickly the *acceleration* itself ramps up. It rounds the corners of the motion (an "S-curve")
  so starts and stops are smooth instead of a sudden kick. Time to reach full accel = accel ÷ jerk.
- **0 = off** (trapezoid profile, a sudden change in acceleration).
- **When to use:** long or heavy arms and elevators that shake at the start or end of a move.
  **Start:** jerk = 10 × accel (≈ 0.1 s of smoothing), raise smoothing by lowering it.

## 4. Other settings you will touch

| Setting | What it does |
|---|---|
| `MaxVolts` | Hard cap on output voltage. **Start low (4–6 V) on anything new.** |
| `StatorLimitA` | Current limit at the motor. Limits **torque/force**, protects the mechanism. |
| `SupplyLimitA` | Current limit from the battery. Protects breakers / battery. |
| `Tolerance (deg)` | How close counts as "at target". Used by the tests' PASS/FAIL. |

## 5. Tuning order (follow this, it saves hours)

**Position mechanism (arm, pivot, hood, elevator):**
1. Set `MaxVolts` low, `kP` small, `kI = kD = kA = 0`.
2. **Find kS + kG** → type in kS and kG.
3. kV: keep the 12 ÷ free-speed value.
4. Cruise ≈ 70% of the reachable speed, Accel = 2 × Cruise.
5. **Step Response**: raise kP until it overshoots a bit, then back off. Add kD if needed.
6. **Repeatability** and **Hold Test** to confirm.
7. Only then: kA, Jerk, and raising Cruise / Accel / MaxVolts to the real values.

**Roller / flywheel:**
1. **Speed Ramp** → type in kS and kV. With kP = 0 the RPM should already be close.
2. Add a small kP (0.05–0.3) to remove what is left.
3. kA if spin-up is slow (watch **Flywheel Spin-up**).
4. kI and kD: normally 0 for rollers.
