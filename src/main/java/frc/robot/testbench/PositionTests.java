package frc.robot.testbench;

/**
 * The automatic tests for a {@link PositionMechanism}. Settings are live-tunable in Elastic under
 * Preferences/&lt;mechanism&gt;/Test/... and are read when the test STARTS.
 *
 * <pre>
 *  Home To Hard Stop  drive slowly into the hard stop until current rises, then zero the encoder
 *  Range Sweep        move slowly min -> max -> min: peak current and worst tracking error, and WHERE
 *                     (finds binding / tight spots)
 *  Step Response      Preset A -> Preset B: rise time, overshoot, settling time, final error (PID tuning)
 *  Repeatability      A <-> B many times: error spread, move time, temperature (gearbox stress test)
 *  Hold Test          hold still for a while: holding current (too much gravity load? counterbalance?)
 *  Find kS + kG       ramp the voltage slowly until it moves up and down -> suggested kS and kG
 * </pre>
 *
 * NOTE: all positions come from the MOTOR encoder. Backlash or a slipping belt between motor and
 * mechanism cannot be seen yet. That comes when we add a CANcoder on the mechanism shaft.
 */
public class PositionTests {

    private final PositionMechanism m;
    private final TunableNumber homeVolts, homeCurrentA, homePosition, homeTimeoutS;
    private final TunableNumber sweepSpeed, sweepMargin;
    private final TunableNumber moveTimeoutS, settleS, cycles;
    private final TunableNumber holdTimeS, holdWarnA;
    private final TunableNumber rampVoltsPerS, rampMaxVolts, moveThreshold;

    PositionTests(PositionMechanism m) {
        this.m = m;
        String p = m.getName() + "/Test/";
        String u = " (" + m.getConfig().unitName + ")";
        double range = m.getConfig().maxPosition - m.getConfig().minPosition;
        homeVolts     = new TunableNumber(p + "HomeVolts (sign = direction)", -1.0);
        homeCurrentA  = new TunableNumber(p + "HomeCurrentA", 15.0);
        homePosition  = new TunableNumber(p + "HomePosition" + u, m.getConfig().minPosition);
        homeTimeoutS  = new TunableNumber(p + "HomeTimeoutS", 5.0);
        sweepSpeed    = new TunableNumber(p + "SweepSpeed" + u + " per s", Math.max(1.0, range / 8.0));
        sweepMargin   = new TunableNumber(p + "SweepMargin" + u, 0.03 * range);
        moveTimeoutS  = new TunableNumber(p + "MoveTimeoutS", 3.0);
        settleS       = new TunableNumber(p + "SettleS", 0.3);
        cycles        = new TunableNumber(p + "RepeatCycles", 10.0);
        holdTimeS     = new TunableNumber(p + "HoldTimeS", 5.0);
        holdWarnA     = new TunableNumber(p + "HoldWarnA", 20.0);
        rampVoltsPerS = new TunableNumber(p + "kSkG RampVoltsPerS", 0.25);
        rampMaxVolts  = new TunableNumber(p + "kSkG MaxVolts", 4.0);
        moveThreshold = new TunableNumber(p + "kSkG MoveThreshold" + u + " per s", Math.max(0.5, range / 100.0));
    }

    // ═════════════════════════════════════════════════════════════════════════
    TestCommand home() {
        return new TestCommand(m, "Home To Hard Stop") {
            Debounce hit;
            double startPos;
            boolean found;

            @Override protected void start() {
                hit = new Debounce(0.1);
                startPos = m.getPosition();
                found = false;
                m.setStallCheckPaused(true);   // we are stalling on purpose
                m.setSoftLimitsEnabled(false); // we must be allowed past the soft limit
            }

            @Override protected boolean loop(double t) {
                m.runVolts(homeVolts.get());
                boolean high = t > 0.25 && m.getStatorCurrentA() > homeCurrentA.get();
                if (hit.update(high, t)) {
                    m.setEncoderPosition(homePosition.get());
                    found = true;
                    return true;
                }
                return t > homeTimeoutS.get();
            }

            @Override protected void cleanUp() {
                m.setStallCheckPaused(false);
                m.setSoftLimitsEnabled(true);
            }

            @Override protected Verdict report() {
                result("Found", found ? 1 : 0);
                result("TravelBeforeHome", Math.abs(m.getPosition() - startPos));
                if (!found) {
                    note("Hard stop not found before timeout. Check HomeVolts direction / raise HomeVolts.");
                    return Verdict.FAIL;
                }
                note("Homed. Encoder set to " + homePosition.get() + " " + m.getConfig().unitName);
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    TestCommand sweep() {
        return new TestCommand(m, "Range Sweep") {
            int phase;              // 0 = go to low end, 1 = sweep up, 2 = sweep down
            double low, high, setpoint, speed, phaseStart, lastT;
            double peakA, peakAAt, maxErr, maxErrAt, sumA, minReached, maxReached;
            int n;

            @Override protected void start() {
                low = m.getConfig().minPosition + sweepMargin.get();
                high = m.getConfig().maxPosition - sweepMargin.get();
                speed = Math.abs(sweepSpeed.get());
                phase = 0; phaseStart = 0; lastT = 0;
                peakA = maxErr = sumA = 0; n = 0;
                minReached = Double.MAX_VALUE; maxReached = -Double.MAX_VALUE;
                m.setTarget(low);
            }

            @Override protected boolean loop(double t) {
                double dt = t - lastT;
                lastT = t;
                if (phase == 0) {
                    m.setTarget(low);
                    if (m.atTarget() || t - phaseStart > moveTimeoutS.get()) {
                        phase = 1; setpoint = m.getPosition(); phaseStart = t;
                    }
                    return false;
                }
                // Move the setpoint at a constant slow speed and record how well it follows.
                double dir = phase == 1 ? 1 : -1;
                setpoint += dir * speed * dt;
                m.setTrackingSetpoint(setpoint, dir * speed);
                double pos = m.getPosition(), a = m.getStatorCurrentA(), err = Math.abs(setpoint - pos);
                if (a > peakA) { peakA = a; peakAAt = pos; }
                if (err > maxErr) { maxErr = err; maxErrAt = pos; }
                sumA += a; n++;
                minReached = Math.min(minReached, pos);
                maxReached = Math.max(maxReached, pos);
                if (phase == 1 && setpoint >= high) { setpoint = high; phase = 2; }
                if (phase == 2 && setpoint <= low) return true;
                return false;
            }

            @Override protected Verdict report() {
                result("PeakCurrentA", peakA);
                result("PeakCurrentAt", peakAAt);
                result("AvgCurrentA", n > 0 ? sumA / n : 0);
                result("MaxTrackingError", maxErr);
                result("MaxTrackingErrorAt", maxErrAt);
                result("MinReached", n > 0 ? minReached : 0);
                result("MaxReached", n > 0 ? maxReached : 0);
                String u = m.getConfig().unitName;
                if (maxErr > 5 * m.getTolerance()) {
                    note(String.format("Lags up to %.2f %s near %.1f %s: binding there, or kP/kG too low.",
                            maxErr, u, maxErrAt, u));
                    return Verdict.WARN;
                }
                note(String.format("Smooth. Peak %.0f A at %.1f %s.", peakA, peakAAt, u));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    TestCommand stepResponse() {
        return new TestCommand(m, "Step Response") {
            boolean stepping;
            double from, to, stepStart, riseTime, overshoot, settledAt;
            Debounce settled;

            @Override protected void start() {
                from = m.getPresetA(); to = m.getPresetB();
                stepping = false; riseTime = -1; overshoot = 0; settledAt = -1;
                settled = new Debounce(settleS.get());
                m.setTarget(from);
            }

            @Override protected boolean loop(double t) {
                if (!stepping) {
                    m.setTarget(from);
                    if (settled.update(m.atTarget(), t) || t > moveTimeoutS.get()) {
                        stepping = true; stepStart = t;
                        settled = new Debounce(settleS.get());
                        m.setTarget(to);
                    }
                    return false;
                }
                double ts = t - stepStart;
                double dir = Math.signum(to - from);
                double pos = m.getPosition();
                double progress = (pos - from) * dir;          // how far towards "to"
                double distance = Math.abs(to - from);
                if (riseTime < 0 && progress >= 0.9 * distance) riseTime = ts;
                overshoot = Math.max(overshoot, progress - distance);
                if (settled.update(m.atTarget(), t)) { settledAt = ts - settleS.get(); return true; }
                return ts > moveTimeoutS.get();
            }

            @Override protected Verdict report() {
                String u = m.getConfig().unitName;
                result("RiseTime90S", riseTime);
                result("Overshoot", overshoot);
                result("SettlingTimeS", settledAt);
                result("FinalError", m.getError());
                if (settledAt < 0) {
                    note(String.format("Did not settle within %.1f %s. Raise kP, add kS/kG, or check for binding.",
                            m.getTolerance(), u));
                    return Verdict.FAIL;
                }
                if (overshoot > 2 * m.getTolerance()) {
                    note(String.format("Overshoot %.2f %s: lower kP, raise kD, or lower Accel.", overshoot, u));
                    return Verdict.WARN;
                }
                note(String.format("Rise %.2f s, settled %.2f s, overshoot %.2f %s.", riseTime, settledAt, overshoot, u));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    TestCommand repeatability() {
        return new TestCommand(m, "Repeatability") {
            int moves, totalMoves, timeouts;
            boolean goingToA;
            double moveStart, firstAPos, lastAPos, minErrA, maxErrA, sumMoveTime, peakA, startTemp;
            Debounce arrived;

            @Override protected void start() {
                totalMoves = 2 * (int) Math.max(1, cycles.get());
                moves = 0; timeouts = 0; goingToA = true;
                moveStart = 0; firstAPos = Double.NaN; lastAPos = Double.NaN;
                minErrA = Double.MAX_VALUE; maxErrA = -Double.MAX_VALUE;
                sumMoveTime = 0; peakA = 0; startTemp = m.getMaxTempC();
                arrived = new Debounce(settleS.get());
            }

            @Override protected boolean loop(double t) {
                double goal = goingToA ? m.getPresetA() : m.getPresetB();
                m.setTarget(goal);
                peakA = Math.max(peakA, m.getStatorCurrentA());
                boolean done = arrived.update(m.atTarget(), t);
                boolean timedOut = t - moveStart > moveTimeoutS.get();
                if (done || timedOut) {
                    if (timedOut && !done) timeouts++;
                    sumMoveTime += t - moveStart;
                    if (goingToA && moves > 0) { // first trip to A is just "getting into position"
                        double pos = m.getPosition(), err = pos - goal;
                        if (Double.isNaN(firstAPos)) firstAPos = pos;
                        lastAPos = pos;
                        minErrA = Math.min(minErrA, err);
                        maxErrA = Math.max(maxErrA, err);
                    }
                    moves++;
                    goingToA = !goingToA;
                    moveStart = t;
                    arrived = new Debounce(settleS.get());
                }
                return moves > totalMoves;
            }

            @Override protected Verdict report() {
                boolean haveA = !Double.isNaN(firstAPos);
                double spread = haveA ? maxErrA - minErrA : 0;
                result("MovesDone", moves);
                result("MoveTimeouts", timeouts);
                result("AvgMoveTimeS", moves > 0 ? sumMoveTime / moves : 0);
                result("ErrorSpreadAtA", spread);
                result("WorstErrorAtA", haveA ? Math.max(Math.abs(minErrA), Math.abs(maxErrA)) : 0);
                result("DriftAtA", haveA ? lastAPos - firstAPos : 0);
                result("PeakCurrentA", peakA);
                result("TempRiseC", m.getMaxTempC() - startTemp);
                String u = m.getConfig().unitName;
                if (timeouts > 0) {
                    note(timeouts + " move(s) did not reach the target in time. Tune gains / check binding.");
                    return Verdict.WARN;
                }
                if (spread > 2 * m.getTolerance()) {
                    note(String.format("Inconsistent: lands within a %.2f %s spread. Check gains and loose parts.", spread, u));
                    return Verdict.WARN;
                }
                note(String.format("%d moves, spread %.2f %s, +%.1f C.", moves, spread, u, m.getMaxTempC() - startTemp));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    TestCommand hold() {
        return new TestCommand(m, "Hold Test") {
            double holdAt, sumA, peakA, sumV, maxDrift;
            int n;

            @Override protected void start() {
                holdAt = m.getPosition();
                sumA = peakA = sumV = maxDrift = 0; n = 0;
                m.setTarget(holdAt);
            }

            @Override protected boolean loop(double t) {
                m.setTarget(holdAt);
                if (t > 0.5) {
                    double a = m.getStatorCurrentA();
                    sumA += a; peakA = Math.max(peakA, a);
                    sumV += Math.abs(m.getAppliedVolts());
                    maxDrift = Math.max(maxDrift, Math.abs(m.getPosition() - holdAt));
                    n++;
                }
                return t > holdTimeS.get();
            }

            @Override protected Verdict report() {
                double avgA = n > 0 ? sumA / n : 0;
                result("HoldPosition", holdAt);
                result("AvgHoldCurrentA", avgA);
                result("PeakHoldCurrentA", peakA);
                result("AvgHoldVolts", n > 0 ? sumV / n : 0);
                result("MaxDrift", maxDrift);
                String u = m.getConfig().unitName;
                if (avgA > holdWarnA.get()) {
                    note(String.format("Needs %.0f A just to hold at %.1f %s: heavy gravity load. Consider counterbalance / gas spring / higher ratio.",
                            avgA, holdAt, u));
                    return Verdict.WARN;
                }
                if (maxDrift > m.getTolerance()) {
                    note(String.format("Drifted %.2f %s while holding: raise kP or add kG.", maxDrift, u));
                    return Verdict.WARN;
                }
                note(String.format("Holds at %.1f %s with %.1f A.", holdAt, u, avgA));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    /**
     * Up-volts = kG + kS, down-volts = kG - kS  =>  kG = (up + down) / 2, kS = (up - down) / 2.
     * For an ARM, run this with the arm near horizontal (that is where kG is measured).
     */
    TestCommand findKsKg() {
        return new TestCommand(m, "Find kS + kG") {
            int phase;            // 0 = ramp up, 1 = pause, 2 = ramp down
            double volts, lastT, phaseStart, upVolts, downVolts;

            @Override protected void start() {
                phase = 0; volts = 0; lastT = 0; phaseStart = 0;
                upVolts = Double.NaN; downVolts = Double.NaN;
            }

            @Override protected boolean loop(double t) {
                double dt = t - lastT;
                lastT = t;
                double rate = Math.abs(rampVoltsPerS.get());
                double max = Math.abs(rampMaxVolts.get());
                double thr = Math.abs(moveThreshold.get());
                switch (phase) {
                    case 0 -> {
                        volts += rate * dt;
                        m.runVolts(volts);
                        if (m.getVelocityUnits() > thr) { upVolts = volts; phase = 1; phaseStart = t; }
                        else if (volts > max) { phase = 1; phaseStart = t; }
                    }
                    case 1 -> {
                        m.runVolts(0);
                        if (t - phaseStart > 0.75) { phase = 2; volts = 0; }
                    }
                    default -> {
                        volts -= rate * dt;
                        m.runVolts(volts);
                        if (m.getVelocityUnits() < -thr) { downVolts = volts; return true; }
                        if (volts < -max) return true;
                    }
                }
                return false;
            }

            @Override protected Verdict report() {
                result("VoltsToMoveUp", upVolts);
                result("VoltsToMoveDown", downVolts);
                if (Double.isNaN(upVolts) || Double.isNaN(downVolts)) {
                    note("Did not move in one direction within kSkG MaxVolts (or cancelled). Raise MaxVolts carefully.");
                    return Verdict.WARN;
                }
                double kg = (upVolts + downVolts) / 2.0, ks = (upVolts - downVolts) / 2.0;
                result("Suggested kS", ks);
                result("Suggested kG", kg);
                note(String.format("Suggested kS=%.3f kG=%.3f - type them into Gains/kS and Gains/kG.", ks, kg));
                return Verdict.PASS;
            }
        };
    }
}
