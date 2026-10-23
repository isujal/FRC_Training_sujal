package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-centre");

    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTa() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public void setPipeline(int index) {
        limelightTable.getEntry("pipeline").setNumber(index);
    }
}
