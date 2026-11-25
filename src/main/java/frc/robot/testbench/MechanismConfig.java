package frc.robot.testbench;

import java.util.ArrayList;
import java.util.List;

/**
 * Everything about ONE mechanism, written once in {@link MechanismPresets}.
 *
 * <h2>Units (the one rule to remember)</h2>
 * <ul>
 *   <li><b>Speed</b> is always <b>RPM of the mechanism output</b> (the roller / wheel / shaft),
 *       NOT the motor. Gear ratio converts it. Leave gearRatio = 1 and "RPM" = motor RPM.</li>
 *   <li><b>Position</b> is in <b>degrees of the mechanism</b> by default, or any unit you pick
 *       with {@link #units(String, double)} (e.g. inches for an elevator).</li>
 *   <li><b>Gains</b> (kP, kS, kV, kG...) are in <b>volts</b> per mechanism rotation (or rps).
 *       The motor does the conversion using gearRatio, so the code never converts by hand.</li>
 * </ul>
 *
 * <p>Every value here is only a STARTING value. Most of them become live-tunable in Elastic.
 */
public class MechanismConfig {

    /** One follower motor and whether it spins opposite to the leader. */
    public record FollowerSpec(int canId, boolean opposed) {}

    // ── Identity / wiring ───────────────────────────────────────────────────────
    public final String name;
    public int leaderId = -1;
    public final List<FollowerSpec> followers = new ArrayList<>();
    public String canBus = "rio";

    // ── Motor output ────────────────────────────────────────────────────────────
    /** false = CounterClockwise_Positive, true = Clockwise_Positive (same as Phoenix Tuner). */
    public boolean inverted = false;
    public boolean brakeMode = false;
    /** Max volts the mechanism may use (12 = full). Start low on a brand-new mechanism! */
    public double maxVolts = 12.0;

    // ── Gearing ─────────────────────────────────────────────────────────────────
    /** Motor rotations per ONE mechanism rotation. e.g. 25:1 gearbox = 25. */
    public double gearRatio = 1.0;
    /** Free speed of the motor (Kraken X60 = 6000, Falcon 500 = 6380, Kraken X44 = 7530). */
    public double motorFreeSpeedRPM = 6000.0;

    // ── Current limits ──────────────────────────────────────────────────────────
    public double statorLimitA = 80.0;
    public double supplyLimitA = 40.0;

    // ── Gains (volts) ───────────────────────────────────────────────────────────
    public double kP = 0, kI = 0, kD = 0, kS = 0, kA = 0, kG = 0;
    /** NaN = "work it out from motor free speed and gear ratio" (a good first guess). */
    public double kV = Double.NaN;

    // ── Position-only settings (ignored by rollers) ─────────────────────────────
    public String unitName = "deg";
    /** How many position units ONE mechanism rotation is. 360 for degrees. */
    public double unitsPerRotation = 360.0;
    public double minPosition = 0.0;
    public double maxPosition = 90.0;
    /** Where the mechanism sits when the robot turns on (the encoder is set to this). */
    public double startPosition = 0.0;
    /** Positions for the A and Y buttons (NaN = pick something inside the range). */
    public double presetA = Double.NaN, presetB = Double.NaN;
    /** Motion Magic speeds in position units per second (and per second^2, ^3). */
    public double cruiseVelocity = 90.0;
    public double acceleration = 180.0;
    public double jerk = 0.0; // 0 = off
    /**
     * How kG is used. NONE = no gravity help. ELEVATOR = constant kG (elevator, telescope).
     * ARM = kG * cos(angle). For ARM, position 0 MUST be "arm horizontal" (set startPosition so).
     */
    public Gravity gravity = Gravity.NONE;

    public enum Gravity { NONE, ELEVATOR, ARM }

    // ── Roller-only settings (ignored by position mechanisms) ───────────────────
    /** Speed used when you press A (live-tunable afterwards). Mechanism RPM. */
    public double targetRPM = 1000.0;

    private MechanismConfig(String name) {
        this.name = name;
    }

    /** Start a new config. The name is what you see in Elastic. */
    public static MechanismConfig named(String name) {
        return new MechanismConfig(name);
    }

    // ── Builder methods (each returns this, so you can chain them) ──────────────

    public MechanismConfig leader(int canId) { this.leaderId = canId; return this; }
    /** Add a follower. opposed = true if it is mounted facing the other way. */
    public MechanismConfig follower(int canId, boolean opposed) {
        followers.add(new FollowerSpec(canId, opposed));
        return this;
    }
    public MechanismConfig canBus(String bus) { this.canBus = bus; return this; }
    public MechanismConfig inverted(boolean clockwisePositive) { this.inverted = clockwisePositive; return this; }
    public MechanismConfig brake(boolean brake) { this.brakeMode = brake; return this; }
    public MechanismConfig maxVolts(double volts) { this.maxVolts = volts; return this; }
    public MechanismConfig gearRatio(double motorRotationsPerMechanismRotation) { this.gearRatio = motorRotationsPerMechanismRotation; return this; }
    public MechanismConfig motorFreeSpeedRPM(double rpm) { this.motorFreeSpeedRPM = rpm; return this; }
    public MechanismConfig currentLimits(double statorA, double supplyA) { this.statorLimitA = statorA; this.supplyLimitA = supplyA; return this; }
    public MechanismConfig pid(double kP, double kI, double kD) { this.kP = kP; this.kI = kI; this.kD = kD; return this; }
    public MechanismConfig feedforward(double kS, double kV, double kA) { this.kS = kS; this.kV = kV; this.kA = kA; return this; }
    public MechanismConfig kS(double kS) { this.kS = kS; return this; }

    /** e.g. units("in", Math.PI * 1.5) for an elevator on a 1.5" pitch-diameter sprocket. */
    public MechanismConfig units(String unitName, double unitsPerMechanismRotation) {
        this.unitName = unitName;
        this.unitsPerRotation = unitsPerMechanismRotation;
        return this;
    }
    public MechanismConfig range(double min, double max) { this.minPosition = min; this.maxPosition = max; return this; }
    public MechanismConfig startPosition(double position) { this.startPosition = position; return this; }
    /** What the A and Y buttons go to (live-tunable afterwards). */
    public MechanismConfig presets(double a, double b) { this.presetA = a; this.presetB = b; return this; }
    public MechanismConfig motionMagic(double cruisePerSec, double accelPerSec2, double jerkPerSec3) {
        this.cruiseVelocity = cruisePerSec;
        this.acceleration = accelPerSec2;
        this.jerk = jerkPerSec3;
        return this;
    }
    public MechanismConfig gravity(Gravity type, double kG) { this.gravity = type; this.kG = kG; return this; }
    public MechanismConfig targetRPM(double rpm) { this.targetRPM = rpm; return this; }

    // ── Derived values ──────────────────────────────────────────────────────────

    /** Expected free speed of the mechanism output, RPM. */
    public double mechanismFreeSpeedRPM() {
        return motorFreeSpeedRPM / gearRatio;
    }

    /** kV if given, else 12 V / free speed (volts per mechanism rotation-per-second). */
    public double kVOrEstimate() {
        if (!Double.isNaN(kV)) return kV;
        double freeRps = mechanismFreeSpeedRPM() / 60.0;
        return 12.0 / freeRps;
    }
}
