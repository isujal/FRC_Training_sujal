package frc.robot.testbench;

import edu.wpi.first.wpilibj.RobotController;

/**
 * The automatic tests for a {@link RollerMechanism}. Each test's settings are live-tunable in
 * Elastic under Preferences/&lt;mechanism&gt;/Test/...  and are read when the test STARTS.
 *
 * <pre>
 *  Run Until Current Spike   run at TargetRPM until current jumps (game piece in / stall), then stop
 *                            (or keep holding with HoldVolts, e.g. for a gripper)
 *  Endurance Soak            run for N minutes: temperature rise, current, speed steadiness, battery sag
 *  Jam Detect + Auto Unjam   on a current spike, reverse briefly and retry; counts jams
 *  Speed Ramp + Free Speed   steps the voltage up; measures real free speed vs expected (friction /
 *                            wrong gear ratio) and suggests kS and kV
 *  Flywheel Spin-up          time to reach speed, RPM dip per shot and recovery time
 * </pre>
 */
public class RollerTests {

    private final RollerMechanism roller;

    // Shared current-spike settings
    private final TunableNumber spikeCurrentA, ignoreStartS, spikeDebounceS, timeoutS;
    // Run until current
    private final TunableNumber holdVolts;
    // Soak
    private final TunableNumber soakMinutes, soakTempLimitC;
    // Jam
    private final TunableNumber unjamTimeS, unjamSpeedScale, maxJams;
    // Ramp
    private final TunableNumber rampMaxVolts, rampStepVolts, rampStepHoldS;

    RollerTests(RollerMechanism roller) {
        this.roller = roller;
        String p = roller.getName() + "/Test/";
        spikeCurrentA   = new TunableNumber(p + "SpikeCurrentA", 30.0);
        ignoreStartS    = new TunableNumber(p + "IgnoreStartS", 0.3);  // spin-up current is not a spike
        spikeDebounceS  = new TunableNumber(p + "SpikeDebounceS", 0.08);
        timeoutS        = new TunableNumber(p + "TimeoutS", 15.0);
        holdVolts       = new TunableNumber(p + "HoldVoltsAfterSpike", 0.0);
        soakMinutes     = new TunableNumber(p + "SoakMinutes", 2.0);
        soakTempLimitC  = new TunableNumber(p + "SoakTempLimitC", 70.0);
        unjamTimeS      = new TunableNumber(p + "UnjamTimeS", 0.3);
        unjamSpeedScale = new TunableNumber(p + "UnjamSpeedScale", 0.5);
        maxJams         = new TunableNumber(p + "MaxJams", 3.0);
        rampMaxVolts    = new TunableNumber(p + "RampMaxVolts", 10.0);
        rampStepVolts   = new TunableNumber(p + "RampStepVolts", 2.0);
        rampStepHoldS   = new TunableNumber(p + "RampStepHoldS", 1.5);
    }

    // ═════════════════════════════════════════════════════════════════════════
    /** Run at TargetRPM until the current spikes (= game piece in, or something blocked it). */
    TestCommand runUntilCurrent() {
        return new TestCommand(roller, "Run Until Current Spike") {
            Debounce spike;
            double peakA, detectedAt, hold;

            @Override protected void start() {
                spike = new Debounce(spikeDebounceS.get());
                peakA = 0; detectedAt = -1; hold = holdVolts.get();
            }

            @Override protected boolean loop(double t) {
                if (detectedAt < 0) {
                    roller.runRPM(roller.getTargetRPM());
                    if (t > ignoreStartS.get()) {
                        peakA = Math.max(peakA, roller.getStatorCurrentA());
                        if (spike.update(roller.getStatorCurrentA() > spikeCurrentA.get(), t)) {
                            detectedAt = t;
                            if (hold == 0) return true;   // stop right away
                        }
                    }
                    return t > timeoutS.get();
                }
                roller.runVolts(hold);                    // gripper style: keep squeezing until B
                return false;
            }

            @Override protected boolean stopByUserIsNormal() { return detectedAt >= 0; }

            @Override protected Verdict report() {
                result("SpikeDetectedAtS", detectedAt);
                result("PeakCurrentA", peakA);
                result("SpikeThresholdA", spikeCurrentA.get());
                if (detectedAt < 0) {
                    note("No spike before timeout. Lower SpikeCurrentA, or the piece never reached the roller.");
                    return Verdict.WARN;
                }
                note(String.format("Spike after %.2f s (peak %.0f A)", detectedAt, peakA));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    /** Stress test: run for SoakMinutes and watch temperature, current and speed. */
    TestCommand soak() {
        return new TestCommand(roller, "Endurance Soak") {
            double startTemp, maxTemp, sumA, peakA, minRpm, maxRpm, sumRpm, minBattery, elapsed;
            int n;
            boolean hitTempLimit;

            @Override protected void start() {
                startTemp = roller.getMaxTempC(); maxTemp = startTemp;
                sumA = 0; peakA = 0; sumRpm = 0; n = 0;
                minRpm = Double.MAX_VALUE; maxRpm = -Double.MAX_VALUE;
                minBattery = RobotController.getBatteryVoltage();
                hitTempLimit = false; elapsed = 0;
            }

            @Override protected boolean loop(double t) {
                roller.runRPM(roller.getTargetRPM());
                elapsed = t;
                maxTemp = Math.max(maxTemp, roller.getMaxTempC());
                minBattery = Math.min(minBattery, RobotController.getBatteryVoltage());
                if (t > 1.5) { // ignore spin-up
                    double rpm = Math.abs(roller.getVelocityRPM());
                    double a = roller.getStatorCurrentA();
                    sumA += a; peakA = Math.max(peakA, a);
                    sumRpm += rpm; minRpm = Math.min(minRpm, rpm); maxRpm = Math.max(maxRpm, rpm);
                    n++;
                }
                if (roller.getMaxTempC() >= soakTempLimitC.get()) { hitTempLimit = true; return true; }
                return t >= soakMinutes.get() * 60.0;
            }

            @Override protected Verdict report() {
                double minutes = Math.max(1e-6, elapsed / 60.0);
                double avgRpm = n > 0 ? sumRpm / n : 0;
                double variationPct = (n > 0 && avgRpm > 1) ? (maxRpm - minRpm) / avgRpm * 100.0 : 0;
                result("StartTempC", startTemp);
                result("MaxTempC", maxTemp);
                result("TempRiseC", maxTemp - startTemp);
                result("TempRisePerMinC", (maxTemp - startTemp) / minutes);
                result("AvgStatorA", n > 0 ? sumA / n : 0);
                result("PeakStatorA", peakA);
                result("AvgRPM", avgRpm);
                result("MinRPM", n > 0 ? minRpm : 0);
                result("MaxRPM", n > 0 ? maxRpm : 0);
                result("SpeedVariationPct", variationPct);
                result("MinBatteryV", minBattery);
                if (hitTempLimit) {
                    note(String.format("Reached %.0f C after %.1f min - motor runs hot at this load.",
                            soakTempLimitC.get(), minutes));
                    return Verdict.WARN;
                }
                if (variationPct > 10.0) {
                    note(String.format("Speed varied %.0f%% - look for binding, rubbing or belt slip.", variationPct));
                    return Verdict.WARN;
                }
                note(String.format("Steady. +%.1f C over %.1f min, avg %.0f A.", maxTemp - startTemp, minutes,
                        n > 0 ? sumA / n : 0));
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    /** Run forward; whenever current spikes, reverse for UnjamTimeS, then try again. */
    TestCommand jam() {
        return new TestCommand(roller, "Jam Detect + Auto Unjam") {
            Debounce spike;
            int jams;
            double phaseStart;
            boolean reversing, gaveUp;

            @Override protected void start() {
                spike = new Debounce(spikeDebounceS.get());
                jams = 0; phaseStart = 0; reversing = false; gaveUp = false;
            }

            @Override protected boolean loop(double t) {
                double sinceStart = t - phaseStart;
                if (reversing) {
                    roller.runRPM(-roller.getTargetRPM() * unjamSpeedScale.get());
                    if (sinceStart > unjamTimeS.get()) { reversing = false; phaseStart = t; }
                } else {
                    roller.runRPM(roller.getTargetRPM());
                    boolean high = sinceStart > ignoreStartS.get()
                            && roller.getStatorCurrentA() > spikeCurrentA.get();
                    if (spike.update(high, t)) {
                        jams++;
                        if (jams > maxJams.get()) { gaveUp = true; return true; }
                        reversing = true; phaseStart = t;
                    }
                }
                return t > timeoutS.get();
            }

            @Override protected Verdict report() {
                result("Jams", jams);
                if (gaveUp) { note("Still jammed after " + (int) maxJams.get() + " retries."); return Verdict.FAIL; }
                if (jams > 0) { note(jams + " jam(s), all cleared by reversing."); return Verdict.WARN; }
                note("No jams.");
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    /**
     * Open-loop voltage steps (2 V, 4 V, ... RampMaxVolts). From speed-vs-volts we get the real
     * free speed and suggested kS / kV. Run it with NO game piece, mechanism free to spin.
     */
    TestCommand speedRamp() {
        return new TestCommand(roller, "Speed Ramp + Free Speed Check") {
            double step, hold, max, volts, stepStart;
            double sumX, sumY, sumXX, sumXY, accRps, accA;
            int points, accN;

            @Override protected void start() {
                step = Math.max(0.5, rampStepVolts.get());
                hold = Math.max(0.5, rampStepHoldS.get());
                max = Math.min(rampMaxVolts.get(), roller.getMaxVolts());
                volts = Math.min(step, max);
                stepStart = 0; points = 0;
                sumX = sumY = sumXX = sumXY = 0;
                accRps = accA = 0; accN = 0;
            }

            @Override protected boolean loop(double t) {
                roller.runVolts(volts);
                double inStep = t - stepStart;
                if (inStep > hold - 0.4) { // average the last 0.4 s of each step (steady state)
                    accRps += Math.abs(roller.getVelocityRPM()) / 60.0;
                    accA += roller.getStatorCurrentA();
                    accN++;
                }
                if (inStep >= hold) {
                    double rps = accRps / Math.max(1, accN);
                    result(String.format("Step %.1fV RPM", volts), rps * 60.0);
                    result(String.format("Step %.1fV StatorA", volts), accA / Math.max(1, accN));
                    // Line fit: volts = kS + kV * rps
                    sumX += rps; sumY += volts; sumXX += rps * rps; sumXY += rps * volts; points++;
                    accRps = accA = 0; accN = 0;
                    stepStart = t;
                    volts += step;
                    return volts > max + 1e-6;
                }
                return false;
            }

            @Override protected Verdict report() {
                if (points < 2) { note("Not enough steps (cancelled early?)."); return Verdict.WARN; }
                double denom = points * sumXX - sumX * sumX;
                double kv = denom == 0 ? 0 : (points * sumXY - sumX * sumY) / denom;
                double ks = (sumY - kv * sumX) / points;
                double freeRpm = kv > 0 ? (12.0 - ks) / kv * 60.0 : 0;
                double expected = roller.getConfig().mechanismFreeSpeedRPM();
                double pct = expected > 0 ? freeRpm / expected * 100.0 : 0;
                result("Suggested kS", ks);
                result("Suggested kV", kv);
                result("Measured FreeSpeed RPM (12V)", freeRpm);
                result("Expected FreeSpeed RPM", expected);
                result("FreeSpeed PctOfExpected", pct);
                String gains = String.format(" Suggested kS=%.3f kV=%.4f.", ks, kv);
                if (pct < 70) { note(String.format("Only %.0f%% of expected free speed: heavy friction or wrong gear ratio in config.", pct) + gains); return Verdict.FAIL; }
                if (pct < 85) { note(String.format("%.0f%% of expected free speed: some friction / rubbing.", pct) + gains); return Verdict.WARN; }
                note(String.format("%.0f%% of expected free speed - spins freely.", pct) + gains);
                return Verdict.PASS;
            }
        };
    }

    // ═════════════════════════════════════════════════════════════════════════
    /** Spin up to TargetRPM, then measure every dip (a shot / a piece passing) and its recovery. */
    TestCommand flywheel() {
        return new TestCommand(roller, "Flywheel Spin-up + Recovery") {
            double spinUpAt, dipStart, dipMin, sumDip, maxDip, sumRec, maxRec;
            int shots;
            boolean inDip;

            @Override protected void start() {
                spinUpAt = -1; inDip = false; shots = 0;
                sumDip = maxDip = sumRec = maxRec = 0;
            }

            @Override protected boolean loop(double t) {
                double target = roller.getTargetRPM();
                roller.runRPM(target);
                double rpm = roller.getVelocityRPM();
                double tol = roller.getAtSpeedTolRPM();
                if (spinUpAt < 0) {
                    if (Math.abs(target - rpm) < tol) spinUpAt = t;
                    return t > timeoutS.get(); // never got there
                }
                if (!inDip && rpm < target - tol) { inDip = true; dipStart = t; dipMin = rpm; }
                if (inDip) {
                    dipMin = Math.min(dipMin, rpm);
                    if (Math.abs(target - rpm) < tol) {
                        inDip = false; shots++;
                        double dip = target - dipMin, rec = t - dipStart;
                        sumDip += dip; maxDip = Math.max(maxDip, dip);
                        sumRec += rec; maxRec = Math.max(maxRec, rec);
                    }
                }
                return t > timeoutS.get() + spinUpAt; // run TimeoutS after spin-up (or press B)
            }

            @Override protected Verdict report() {
                result("SpinUpS", spinUpAt);
                result("Shots", shots);
                result("AvgDipRPM", shots > 0 ? sumDip / shots : 0);
                result("MaxDipRPM", maxDip);
                result("AvgRecoveryS", shots > 0 ? sumRec / shots : 0);
                result("MaxRecoveryS", maxRec);
                if (spinUpAt < 0) {
                    note("Never reached target. Raise kV (run Speed Ramp), raise MaxVolts, or lower TargetRPM.");
                    return Verdict.FAIL;
                }
                note(String.format("Spin-up %.2f s, %d shot(s), avg recovery %.2f s.", spinUpAt, shots,
                        shots > 0 ? sumRec / shots : 0));
                return Verdict.PASS;
            }
        };
    }
}
