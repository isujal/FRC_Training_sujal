package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShotTable;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoShooterCommand extends Command {
    private final Shooter shooter;
     private final Vision vision;
    public AutoShooterCommand(Shooter s, Vision v){ 
        shooter = s;
        vision = v;
        addRequirements(s); 
    }

    @Override 
    public void execute() {
        double d = vision.getDistanceToGoal();
        shooter.setHoodPosition(ShotTable.hoodDeg.get(d));
        shooter.setShooterRPM(ShotTable.rpm.get(d));
    }
    @Override
    public void end(boolean i) { 
        shooter.stopShooter(); 
        shooter.setHoodPosition(0); 
    }
}