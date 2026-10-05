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

public class Intake extends SubsystemBase {

    private double m_targetPos = 0.0;
        // ─── Constants (measure these on your robot!) ─────────────────────────────
    public static final double kElbowGearRatio = 208.28;  // motor rotations per elbow rotation
    public static final double kMinAngleDeg    = 0.0;     // where the elbow sits at power-on
    public static final double kMaxAngleDeg    = 120.0;   // fully deployed
    public static final double kDeployAngleDeg = 120.0;
    public static final double testAngle = 10;
    public static final double kStowAngleDeg   = 5.0;     // a few degrees off the hard stop
    public static final double kRollerVolts    = 9.0;     // peak duty 0.8 caps output at 9.6 V

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX leftRoller = new TalonFX(12, "rio");   // MotionMagic position
    private final TalonFX rightRoller = new TalonFX(11, "rio");   // VelocityVoltage
    private final TalonFX intakeMotor = new TalonFX(59, "rio");   // VelocityVoltage
    
    // ─── Control Requests ─────────────────────────────────────────────────────
    private final MotionMagicDutyCycle m_motionMagicRequest =
            new MotionMagicDutyCycle(0);

    private final VelocityVoltage m_velocityVoltageRequest =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    public double IntakeGearRatio = 208.28;//162
    private boolean calibratingMin = false;
    private boolean calibratingMax = false;

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
        cfg.MotorOutput.PeakForwardDutyCycle =  0.8;
        cfg.MotorOutput.PeakReverseDutyCycle = -0.8;

        cfg.Slot0.kP = 0.5;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
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

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted             = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  1;
        cfg.MotorOutput.PeakReverseDutyCycle = -1;

        cfg.Slot0.kP = 1.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kG = 0.0;

        cfg.MotionMagic.withMotionMagicAcceleration(100)
                       .withMotionMagicCruiseVelocity(100)
                       .withMotionMagicJerk(3000);

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
        SmartDashboard.putNumber("Intake/RollerRPM",         leftRoller.getVelocity().getValueAsDouble() * 60.0);

    }

    // ─── Elbow Actions (MotionMagic Position) ───────────────────────────────


    public void setIntakePosition(double position) {
        m_targetPos = position;
        intakeMotor.setControl(m_motionMagicRequest.withPosition(m_targetPos).withSlot(0));
    }

    public void incrementPosition(double rotations) {
        double currentPOS    = intakeMotor.getPosition().getValueAsDouble();
        double incrementPOS  = currentPOS + rotations;
        intakeMotor.setControl(m_motionMagicRequest.withPosition(incrementPOS).withSlot(0));
    }

    public void decrementPosition(double rotations) {
        double currentPOS   = intakeMotor.getPosition().getValueAsDouble();
        double decrementPOS = currentPOS - rotations;
        intakeMotor.setControl(m_motionMagicRequest.withPosition(decrementPOS).withSlot(0));
    }

    public void intakeMotorZero() {
        intakeMotor.setPosition(0);
    }

    public double getintakePosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

      public double getIntakeAngleDeg() {
    return getintakePosition() * 360 / IntakeGearRatio;
    }

    private static double degToRot(double deg) { return deg * kElbowGearRatio / 360.0; }

    // ─── Motor 2 Actions (VelocityVoltage) ────────────────────────────────────

    public void IntakeRollerRPM(double RPM) {
        leftRoller.setControl(m_velocityVoltageRequest.withVelocity(RPM).withSlot(0));
    }

    public void stopMotor() {
        leftRoller.setVoltage(0);
    }

    public void getIntakeRollerCurrent() {
  double current = leftRoller.getStatorCurrent().getValueAsDouble() + rightRoller.getStatorCurrent().getValueAsDouble();
  SmartDashboard.putNumber("IntakeRoller_Current", current);
}

}