package frc.robot.testbench;

import edu.wpi.first.wpilibj.Preferences;
import java.util.ArrayList;
import java.util.List;

/**
 * A number you can change from Elastic WHILE THE ROBOT IS ENABLED. No redeploy.
 *
 * <p>Where to find it in Elastic: the "Preferences" table, e.g. Preferences/Intake Rollers/TargetRPM.
 * Drag it onto your layout as a "Text Field" / "Number Slider" widget and type a new value.
 *
 * <p>The value is saved on the roboRIO (WPILib Preferences), so it is still there after a reboot.
 * That also means: if you later change the default here in code, the SAVED value still wins.
 * Press "TestBench/Print All Tunables" to see which values differ from code, and
 * "TestBench/Reset Tunables To Code" to throw the saved values away.
 */
public class TunableNumber {

    private static final List<TunableNumber> ALL = new ArrayList<>();

    private final String key;
    private final double codeDefault;
    private double lastSeen;

    /** @param key shown in Elastic as Preferences/&lt;key&gt;, e.g. "Intake Rollers/TargetRPM" */
    public TunableNumber(String key, double codeDefault) {
        this.key = key;
        this.codeDefault = codeDefault;
        Preferences.initDouble(key, codeDefault); // only writes if no saved value exists yet
        this.lastSeen = get();
        ALL.add(this);
    }

    /** Current value (what is in Elastic right now). */
    public double get() {
        return Preferences.getDouble(key, codeDefault);
    }

    /** Write a new value (also used by the controller, e.g. POV up = +100 RPM). */
    public void set(double value) {
        Preferences.setDouble(key, value);
    }

    /** True once each time the value changes. Used to push new gains/limits to the motor. */
    public boolean hasChanged() {
        double now = get();
        if (now != lastSeen) {
            lastSeen = now;
            return true;
        }
        return false;
    }

    public String getKey() {
        return key;
    }

    // ── Helpers for the whole list (used by the dashboard buttons) ─────────────────

    /** Prints every tunable to the console so you can copy final values into code. */
    public static void printAll() {
        System.out.println("=========== TUNABLE VALUES (copy final ones into code) ===========");
        int changed = 0;
        for (TunableNumber t : ALL) {
            boolean differs = t.get() != t.codeDefault;
            if (differs) changed++;
            System.out.printf("%-45s = %10.4f   %s%n", t.key, t.get(),
                    differs ? "<-- CHANGED (code default " + t.codeDefault + ")" : "");
        }
        System.out.println("=========== " + changed + " value(s) differ from code ===========");
    }

    /**
     * Only the values that differ from code, one per line, e.g.
     * "Intake Rollers/TargetRPM = 2200 (code: 1800)". Shown in Elastic as TestBench/Tunables Report.
     */
    public static String changedReport() {
        StringBuilder sb = new StringBuilder();
        for (TunableNumber t : ALL) {
            if (t.get() != t.codeDefault) {
                sb.append(t.key).append(" = ").append(format(t.get()))
                  .append("   (code: ").append(format(t.codeDefault)).append(")\n");
            }
        }
        return sb.length() == 0 ? "All values match the code." : sb.toString().trim();
    }

    /** 2200.0 -> "2200", 0.12345 -> "0.1235". */
    private static String format(double v) {
        return v == Math.rint(v) ? String.valueOf((long) v) : String.format("%.4f", v);
    }

    /** Throws away every saved value and goes back to the defaults written in code. */
    public static void resetAllToCode() {
        for (TunableNumber t : ALL) {
            t.set(t.codeDefault);
        }
        System.out.println("[TestBench] All tunables reset to code defaults.");
    }

    /** How many values are currently different from code (shown on the dashboard as a reminder). */
    public static int countChangedFromCode() {
        int n = 0;
        for (TunableNumber t : ALL) {
            if (t.get() != t.codeDefault) n++;
        }
        return n;
    }
}
