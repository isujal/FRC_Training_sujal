package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake = two rollers (17 leader, 61 follower) + the elbow/pivot (59).
 *
 * UNITS (same rule everywhere in this repo):
 *   - Roller speed is RPM of the roller MOTOR shaft (no gear ratio is set for the rollers).
 *     Phoenix velocity is rotations per SECOND, so every RPM is divided by 60 before it is sent.
 *   - Elbow position is DEGREES of the elbow (converted with kElbowGearRatio).
 */
public class Intake extends SubsystemBase {

    private double m_targetPos = 0.0;
        // ─── Constants (measure these on your robot!) ─────────────────────────────
    public static final double kElbowGearRatio = 208.28;  // motor rotations per elbow rotation
    public static final double kMinAngleDeg    = 0.0;     // where the elbow sits at power-on
    public static final double kMaxAngleDeg    = 140.0;   // fully deployed
    public static final double kDeployAngleDeg = 120.0;
    public static final double testAngle = 135;
    public static final double randomAngle = -20;
    public static final double kStowAngleDeg   = 5.0;     // a few degrees off the hard stop
    public static final double kRollerVolts    = 9.0;     // peak duty 0.8 caps output at 9.6 V

    /** Roller speed for teleop intaking (IntakeCommand), motor RPM. */
    public static final double kRollerIntakeRPM = 1800.0;
    /** Roller speed used by the auto named commands, motor RPM. ~3000 is the most the 0.5 peak duty allows. */
    public static final double kRollerAutoRPM   = 3000.0;

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX leftRoller = new TalonFX(17, "rio");   // roller leader, VelocityVoltage
    private final TalonFX rightRoller = new TalonFX(61, "rio");   // roller follower (opposed)
    private final TalonFX intakeMotor = new TalonFX(59, "rio");   // elbow/pivot, MotionMagic position
    
    // ─── Control Requests ─────────────────────────────────────────────────────
    private final MotionMagicDutyCycle m_motionMagicRequest =
            new MotionMagicDutyCycle(0);

    private final VelocityVoltage m_velocityVoltageRequest =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(true);


    // ─── Constructor ──────────────────────────────────────────────────────────
    public Intake() {
        configureLR();
        configureIM();
        intakeMotor.setPosition(0);
    }

    // ─── Roller Config (Velocity Voltage ) ────────────────────────────────
    private void configureLR() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted             = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  0.5;
        cfg.MotorOutput.PeakReverseDutyCycle = -0.5;

        // Velocity gains are in volts per (motor rotation per second).
        cfg.Slot0.kP = 0.2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.12;  // 12 V / 100 rps (Kraken free speed). Without kV it never reaches the target RPM.
        cfg.Slot0.kS = 0.0;

        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.CurrentLimits.SupplyCurrentLimit       = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightRoller.setControl(new Follower(leftRoller.getDeviceID(), MotorAlignmentValue.Opposed));
        rightRoller.getConfigurator().apply(cfg, 0.050);
        leftRoller.getConfigurator().apply(cfg, 0.050);

   }

    
    // ─── Elbow Config (MotionMagicDutyCycle) ────────────────────────────────
    private void configureIM() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted             = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  1;
        cfg.MotorOutput.PeakReverseDutyCycle = -1;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kG = 0.0;

        cfg.MotionMagic.withMotionMagicAcceleration(200)
                       .withMotionMagicCruiseVelocity(200)
                       .withMotionMagicJerk(500);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.CurrentLimits.StatorCurrentLimit       = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(cfg, 0.050);
    }



    
    // ─── Telemetry ────────────────────────────────────────────────────────────
    @Override
    public void periodic() {

        // leftRoller
        SmartDashboard.putNumber("leftRoller/Position (rot)",    leftRoller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("leftRoller/Velocity (rps)",    leftRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("leftRoller/StatorCurrent (A)", leftRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("leftRoller/Temp (C)",          leftRoller.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("leftRoller/DutyCycle",         leftRoller.getDutyCycle().getValueAsDouble());

        // rightRoller
        SmartDashboard.putNumber("rightRoller/Velocity (rps)",    rightRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("rightRoller/Velocity (rpm)",    rightRoller.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("rightRoller/StatorCurrent (A)", rightRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("rightRoller/Temp (C)",          rightRoller.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("rightRoller/DutyCycle",         rightRoller.getDutyCycle().getValueAsDouble());

        // intake motor
        SmartDashboard.putNumber("intakeMotor/Velocity (rps)",    intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intakeMotor/Velocity (rpm)",    intakeMotor.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("intakeMotor/StatorCurrent (A)", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intakeMotor/Temp (C)",          intakeMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("intakeMotor/DutyCycle",         intakeMotor.getDutyCycle().getValueAsDouble());

        SmartDashboard.putNumber("Intake/ElbowAngle (deg)",  getIntakeAngleDeg());
        SmartDashboard.putNumber("Intake/ElbowTarget (deg)", m_targetPos * 360.0 / kElbowGearRatio);
        SmartDashboard.putNumber("Intake/RollerRPM",         getIntakeRollerRPM());

    }

    // ─── Elbow Actions (MotionMagic Position) ───────────────────────────────

    public void setIntakePosition(double degrees) {
        degrees = MathUtil.clamp(degrees, kMinAngleDeg, kMaxAngleDeg);   // safety limits
        m_targetPos = degToRot(degrees);                                  // degrees → motor rotations
        intakeMotor.setControl(m_motionMagicRequest.withPosition(m_targetPos).withSlot(0));
    }

    public void incrementPosition(double degrees) { setIntakePosition(m_targetPos * 360.0 / kElbowGearRatio + degrees); }
    public void decrementPosition(double degrees) { setIntakePosition(m_targetPos * 360.0 / kElbowGearRatio - degrees); }


    // public void incrementPosition(double rotations) {
    //     double currentPOS    = intakeMotor.getPosition().getValueAsDouble();
    //     double incrementPOS  = currentPOS + rotations;
    //     intakeMotor.setControl(m_motionMagicRequest.withPosition(incrementPOS).withSlot(0));
    // }

    // public void decrementPosition(double rotations) {
    //     double currentPOS   = intakeMotor.getPosition().getValueAsDouble();
    //     double decrementPOS = currentPOS - rotations;
    //     intakeMotor.setControl(m_motionMagicRequest.withPosition(decrementPOS).withSlot(0));
    // }

    public void intakeMotorZero() {
        intakeMotor.setPosition(0);
    }

    public double getintakePosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    public double getIntakeAngleDeg() {
        return getintakePosition() * 360.0 / kElbowGearRatio;
    }

    private static double degToRot(double deg) { return deg * kElbowGearRatio / 360.0; }

    // ─── Roller Actions (VelocityVoltage) ─────────────────────────────────────

    /** @param rpm roller motor RPM (e.g. {@link #kRollerIntakeRPM}). */
    public void setIntakeRollerRPM(double rpm) {
        leftRoller.setControl(m_velocityVoltageRequest.withVelocity(rpm / 60.0).withSlot(0)); // RPM -> rps
    }

    /** Roller motor RPM. */
    public double getIntakeRollerRPM() {
        return leftRoller.getVelocity().getValueAsDouble() * 60.0; // rps -> RPM
    }

    public void stopMotor() {
        leftRoller.setVoltage(0);
    }

    public void getIntakeRollerCurrent() {
  double current = leftRoller.getStatorCurrent().getValueAsDouble() + rightRoller.getStatorCurrent().getValueAsDouble();
  SmartDashboard.putNumber("IntakeRoller_Current", current);
}

}