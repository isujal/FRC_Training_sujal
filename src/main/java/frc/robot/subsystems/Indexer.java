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
public class Indexer extends SubsystemBase {


    public final TalonFX Indexer = new TalonFX(10, "rio");
    public final TalonFX leftFeeder = new TalonFX(57, "rio");
    public final TalonFX rightFeeder  = new TalonFX(51,"rio");

    private final VelocityVoltage feederRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage indexerRequest = new VelocityVoltage(0.0);

private double indexerTargetRPM = 0.0;
private double feederTargetRPM = 0.0;

    public Indexer(){
        configureIndexer();
        configureFeeder();
    }


    private void configureIndexer(){

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted             = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  0.8;
        cfg.MotorOutput.PeakReverseDutyCycle = -0.8;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kA = 0.0;

        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.CurrentLimits.SupplyCurrentLimit       = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotionMagic.withMotionMagicAcceleration(1000)
                       .withMotionMagicCruiseVelocity(1000)
                       .withMotionMagicJerk(0);

        Indexer.getConfigurator().apply(cfg, 0.050);
        
    }

    private void configureFeeder(){

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode          = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted             = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.PeakForwardDutyCycle =  0.8;
        cfg.MotorOutput.PeakReverseDutyCycle = -0.8;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kA = 0.0;

        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.CurrentLimits.SupplyCurrentLimit       = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotionMagic.withMotionMagicAcceleration(1000)
                       .withMotionMagicCruiseVelocity(1000)
                       .withMotionMagicJerk(0);

        rightFeeder.setControl(new Follower(leftFeeder.getDeviceID(), MotorAlignmentValue.Opposed)); 
        leftFeeder.getConfigurator().apply(cfg, 0.050); 
        
    
    }


      public double FeederVoltage() {
    return leftFeeder.getMotorVoltage().getValueAsDouble() + rightFeeder.getMotorVoltage().getValueAsDouble();
  }

  public double FeederCurrent() {
    return leftFeeder.getStatorCurrent().getValueAsDouble() + rightFeeder.getStatorCurrent().getValueAsDouble();
  }

  public double FeederRPM() {
    return leftFeeder.getVelocity().getValueAsDouble() * 60; //+ LShooterDown.getVelocity().getValueAsDouble();
}



public void setFeederRPM(double rpm) {
    feederTargetRPM = rpm;
    leftFeeder.setControl(feederRequest.withVelocity(rpm / 60.0).withSlot(0));   // RPM -> rps
}

public void setIndexerRPM(double rpm) {
    indexerTargetRPM = rpm;
    Indexer.setControl(indexerRequest.withVelocity(rpm / 60.0).withSlot(0));
}

public void stopIndexer() {
    indexerTargetRPM = 0.0;
    Indexer.setVoltage(0);
}

public void stopFeeder() {
    feederTargetRPM = 0.0;
    leftFeeder.setVoltage(0);
}





private static double rpmOf(TalonFX motor) {
    return motor.getVelocity().getValueAsDouble() * 60.0;      // rps -> RPM
}

// ---- Indexer ----
public double getIndexerRPM()        { return rpmOf(Indexer); }
public double getIndexerTargetRPM()  { return indexerTargetRPM; }
public double getIndexerVolts()      { return Indexer.getMotorVoltage().getValueAsDouble(); }
public double getIndexerCurrent()    { return Indexer.getStatorCurrent().getValueAsDouble(); }
public double getIndexerTemp()       { return Indexer.getDeviceTemp().getValueAsDouble(); }

// ---- Feeder (left = leader, right = follower) ----
public double getFeederRPM()         { return rpmOf(leftFeeder); }
public double getFeederTargetRPM()   { return feederTargetRPM; }
public double getFeederVolts()       { return leftFeeder.getMotorVoltage().getValueAsDouble()
                                            + rightFeeder.getMotorVoltage().getValueAsDouble(); }
public double getFeederCurrent()     { return leftFeeder.getStatorCurrent().getValueAsDouble()
                                            + rightFeeder.getStatorCurrent().getValueAsDouble(); }
public double getFeederTemp()        { return leftFeeder.getDeviceTemp().getValueAsDouble(); }

// ---- Status checks ----
public boolean isIndexerRunning()    { return indexerTargetRPM != 0.0; }
public boolean isFeederRunning()     { return feederTargetRPM != 0.0; }

public boolean isIndexerAtSpeed(double toleranceRpm) {
    return Math.abs(indexerTargetRPM - getIndexerRPM()) < toleranceRpm;
}

public boolean isFeederAtSpeed(double toleranceRpm) {
    return Math.abs(feederTargetRPM - getFeederRPM()) < toleranceRpm;
}



@Override
public void periodic() {
    publishTelemetry();
}

private void publishTelemetry() {
    // ---- Indexer ----
    SmartDashboard.putNumber("Indexer/RPM", getIndexerRPM());
    SmartDashboard.putNumber("Indexer/TargetRPM", getIndexerTargetRPM());
    SmartDashboard.putNumber("Indexer/ErrorRPM", getIndexerTargetRPM() - getIndexerRPM());
    SmartDashboard.putBoolean("Indexer/AtSpeed", isIndexerAtSpeed(150));
    SmartDashboard.putNumber("Indexer/Volts", getIndexerVolts());
    SmartDashboard.putNumber("Indexer/StatorAmps", getIndexerCurrent());
    SmartDashboard.putNumber("Indexer/SupplyAmps", Indexer.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Indexer/Temp", getIndexerTemp());

    // ---- Feeder (both motors, to confirm the follower really follows) ----
    SmartDashboard.putNumber("Feeder/RPM", getFeederRPM());
    SmartDashboard.putNumber("Feeder/TargetRPM", getFeederTargetRPM());
    SmartDashboard.putNumber("Feeder/ErrorRPM", getFeederTargetRPM() - getFeederRPM());
    SmartDashboard.putBoolean("Feeder/AtSpeed", isFeederAtSpeed(150));
    SmartDashboard.putNumber("Feeder/Left RPM", rpmOf(leftFeeder));
    SmartDashboard.putNumber("Feeder/Right RPM", rpmOf(rightFeeder));
    SmartDashboard.putNumber("Feeder/Left Amps", leftFeeder.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Right Amps", rightFeeder.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Volts", getFeederVolts());
    SmartDashboard.putNumber("Feeder/Temp", getFeederTemp());
    SmartDashboard.putNumber("Feeder/Left Position", leftFeeder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Feeder/Right Position", rightFeeder.getPosition().getValueAsDouble());
}
    

    
}
