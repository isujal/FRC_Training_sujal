package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Shooter extends SubsystemBase {

    public final TalonFX LShooterUp = new TalonFX(55, "rio");
    public final TalonFX LShooterDown = new TalonFX(56, "rio");
    public final TalonFX RShooterUp = new TalonFX(54, "rio");
    public final TalonFX RShooterDown = new TalonFX(53, "rio");
    public final TalonFX Hood = new TalonFX(52, "rio");

    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final MotionMagicDutyCycle hMotionMagicDutyCycle = new MotionMagicDutyCycle(0);


    private double leftTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;

    public double shooterRPM = 500;
    public double testHoodPos = 10;

    private double HoodGearRatio = 60.8417;
    public boolean HoodReset = false;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 38.0;


    public Shooter (){
        configureShooter();
        configureHood();
    }


    private void configureShooter(){

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted             = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  0.8;
        cfg.MotorOutput.PeakReverseDutyCycle = -0.8;

        cfg.Slot0.kP = 0.4;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.15;
        cfg.Slot0.kS = 0.25;
        cfg.Slot0.kA = 0.09;

        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.CurrentLimits.SupplyCurrentLimit       = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotionMagic.withMotionMagicAcceleration(200)
                       .withMotionMagicCruiseVelocity(500)
                       .withMotionMagicJerk(0);

        RShooterUp.getConfigurator().apply(cfg, 0.050);
        RShooterDown.getConfigurator().apply(cfg, 0.050);
        LShooterDown.getConfigurator().apply(cfg, 0.050);
        LShooterUp.getConfigurator().apply(cfg, 0.050);
        
        enableFollowers();


    }

    private void configureHood(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted             = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  1;
        cfg.MotorOutput.PeakReverseDutyCycle = -1;

        cfg.Slot0.kP = 0.3;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kG = 0.0;

        cfg.MotionMagic.withMotionMagicAcceleration(300)
                       .withMotionMagicCruiseVelocity(100)
                       .withMotionMagicJerk(0);

        double minRot = (HOOD_MIN_DEG * HoodGearRatio) / 360.0;
        double maxRot = (HOOD_MAX_DEG * HoodGearRatio) / 360.0;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRot;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRot;

        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.CurrentLimits.StatorCurrentLimit       = 80 ;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        Hood.getConfigurator().apply(cfg, 0.050);
    
    }

    public void enableFollowers(){
            LShooterUp.setControl(
        new Follower(RShooterUp.getDeviceID(), MotorAlignmentValue.Opposed));

    LShooterDown.setControl(
        new Follower(RShooterUp.getDeviceID(), MotorAlignmentValue.Opposed));

    RShooterDown.setControl(
        new Follower(RShooterUp.getDeviceID(), MotorAlignmentValue.Aligned));
    }

public void setShooterRPM(double rpm) {
    rightTargetRPM = rpm;                                   // remember it for printing/at-speed checks
    RShooterUp.setControl(
        shooterVelocityVoltage.withVelocity(rpm / 60.0).withSlot(0));   // RPM -> rps
}
    public void stopShooter() {
        RShooterUp.setVoltage(0);
    }

private double hoodTargetDeg = HOOD_MIN_DEG;

public void setHoodPosition(double angle) {
    angle = MathUtil.clamp(angle, HOOD_MIN_DEG, HOOD_MAX_DEG);
    hoodTargetDeg = angle;                               
    Hood.setControl(
        hMotionMagicDutyCycle.withPosition((angle * HoodGearRatio) / 360.0).withSlot(0));
}

public double getHoodAngleDeg() {
    return Hood.getPosition().getValueAsDouble() * 360.0 / HoodGearRatio;
}

public double getHoodTargetDeg() {
    return hoodTargetDeg;
}


public void incrementHood(double deg) {
    setHoodPosition(hoodTargetDeg + deg);
}

public void decrementHood(double deg) {
    setHoodPosition(hoodTargetDeg - deg);
}


private static double rpmOf(TalonFX motor) {
    return motor.getVelocity().getValueAsDouble() * 60.0;   // rps -> RPM
}

public double getShooterRPM() {
    return rpmOf(RShooterUp);
}

public double getTargetRPM() {
    return rightTargetRPM;
}

public boolean isAtSpeed(double toleranceRpm) {
    return Math.abs(getTargetRPM() - getShooterRPM()) < toleranceRpm;
}

public boolean isHoodAtTarget(double toleranceDeg) {
    return Math.abs(getHoodTargetDeg() - getHoodAngleDeg()) < toleranceDeg;
}

@Override
public void periodic() {
    publishTelemetry();
}

private void publishTelemetry() {
    // ---- Flywheel (leader) ----
    SmartDashboard.putNumber("Shooter/RPM", getShooterRPM());
    SmartDashboard.putNumber("Shooter/TargetRPM", getTargetRPM());
    SmartDashboard.putNumber("Shooter/ErrorRPM", getTargetRPM() - getShooterRPM());
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed(100));
    SmartDashboard.putNumber("Shooter/AppliedVolts", RShooterUp.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/StatorAmps", RShooterUp.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/SupplyAmps", RShooterUp.getSupplyCurrent().getValueAsDouble());

    // ---- All four flywheel motors, to confirm the followers are really spinning ----
    SmartDashboard.putNumber("Shooter/RShooterUp RPM", rpmOf(RShooterUp));
    SmartDashboard.putNumber("Shooter/RShooterDown RPM", rpmOf(RShooterDown));
    SmartDashboard.putNumber("Shooter/LShooterUp RPM", rpmOf(LShooterUp));
    SmartDashboard.putNumber("Shooter/LShooterDown RPM", rpmOf(LShooterDown));

    // ---- Temperatures (a hot motor means too much current) ----
    SmartDashboard.putNumber("Shooter/RShooterUp Temp", RShooterUp.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/LShooterUp Temp", LShooterUp.getDeviceTemp().getValueAsDouble());

    // ---- Hood ----
    SmartDashboard.putNumber("Hood/AngleDeg", getHoodAngleDeg());
    SmartDashboard.putNumber("Hood/TargetDeg", getHoodTargetDeg());
    SmartDashboard.putNumber("Hood/ErrorDeg", getHoodTargetDeg() - getHoodAngleDeg());
    SmartDashboard.putBoolean("Hood/AtTarget", isHoodAtTarget(1.0));
    SmartDashboard.putNumber("Hood/StatorAmps", Hood.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hood/MotorRotations", Hood.getPosition().getValueAsDouble());
}

    

    
}
