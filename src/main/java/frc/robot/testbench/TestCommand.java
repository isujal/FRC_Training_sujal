package frc.robot.testbench;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base class for every automatic test. A test only has to say:
 * <ol>
 *   <li>{@link #start()}  - what to do once at the beginning</li>
 *   <li>{@link #loop(double)} - what to do every 20 ms; return true when finished</li>
 *   <li>{@link #report()} - publish its numbers with {@link #result} and return PASS / WARN / FAIL</li>
 * </ol>
 *
 * This class handles the rest: timing, stopping safely at the end, FAIL if a safety cut-off
 * tripped, CANCELLED if you pressed B / clicked the button again, and publishing the results.
 *
 * <p>Results appear in Elastic under TestBench/&lt;mechanism&gt;/Results/&lt;test&gt;/..., a one-line summary
 * under TestBench/&lt;mechanism&gt;/Last Result, and in the console + log file (for AdvantageScope).
 */
public abstract class TestCommand extends Command {

    public enum Verdict { PASS, WARN, FAIL, CANCELLED }

    protected final TestMechanism mech;
    private final String testName;
    private final String resultPath;
    private final StringBuilder summary = new StringBuilder();
    private double startTime;
    private boolean finished;
    private String note = "";

    protected TestCommand(TestMechanism mech, String testName) {
        this.mech = mech;
        this.testName = testName;
        this.resultPath = "TestBench/" + mech.getName() + "/Results/" + testName + "/";
        setName(testName);
        addRequirements(mech);
    }

    // ── What each test fills in ────────────────────────────────────────────────

    protected abstract void start();

    /** @param t seconds since the test started. @return true when the test is done. */
    protected abstract boolean loop(double t);

    /** Publish numbers with result(...), set a note(...), and return the verdict. */
    protected abstract Verdict report();

    /** Optional clean-up (e.g. turn soft limits back on). */
    protected void cleanUp() {}

    /** Return true if pressing B is the NORMAL way this test ends (e.g. "hold until B"). */
    protected boolean stopByUserIsNormal() { return false; }

    // ── Helpers for tests ──────────────────────────────────────────────────────

    /** Publish one number of the result. */
    protected final void result(String key, double value) {
        SmartDashboard.putNumber(resultPath + key, value);
        summary.append(String.format("  %s=%.3f", key, value));
    }

    /** One sentence explaining the verdict, written for the mechanical team. */
    protected final void note(String text) {
        note = text;
    }

    // ── Command plumbing ───────────────────────────────────────────────────────

    @Override
    public final void initialize() {
        summary.setLength(0);
        note = "";
        finished = false;
        startTime = Timer.getFPGATimestamp();
        SmartDashboard.putString(resultPath + "Verdict", "RUNNING");
        SmartDashboard.putString(resultPath + "Note", "");
        SmartDashboard.putString("TestBench/" + mech.getName() + "/Last Result", testName + ": RUNNING...");
        start();
    }

    @Override
    public final void execute() {
        if (!finished) {
            finished = loop(Timer.getFPGATimestamp() - startTime);
        }
    }

    @Override
    public final boolean isFinished() {
        return finished;
    }

    @Override
    public final void end(boolean interrupted) {
        cleanUp();
        mech.rest();

        Verdict v = report();
        if (mech.isFaulted()) {
            v = Verdict.FAIL;
            note("Safety cut-off: " + mech.getFault());
        } else if (interrupted && !stopByUserIsNormal()) {
            v = Verdict.CANCELLED;
            note("Stopped by user (numbers above are partial)");
        }
        result("DurationS", Timer.getFPGATimestamp() - startTime);

        SmartDashboard.putString(resultPath + "Verdict", v.name());
        SmartDashboard.putString(resultPath + "Note", note);
        String line = testName + ": " + v + (note.isEmpty() ? "" : " - " + note);
        SmartDashboard.putString("TestBench/" + mech.getName() + "/Last Result", line);

        // Console + log file: one line you can paste straight into the team's test sheet.
        DataLogManager.log("[TEST] " + mech.getName() + " | " + line + " |" + summary);
    }

    /** Small helper: tracks how long a condition has been continuously true. */
    protected static final class Debounce {
        private final double seconds;
        private double since = Double.NaN;

        Debounce(double seconds) { this.seconds = seconds; }

        boolean update(boolean condition, double t) {
            if (!condition) {
                since = Double.NaN;
                return false;
            }
            if (Double.isNaN(since)) since = t;
            return t - since >= seconds;
        }
    }
}
