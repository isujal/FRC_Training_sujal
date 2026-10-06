package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

    private final Shooter shooter;

    public ShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterRPM(shooter.shooterRPM);
        shooter.setHoodPosition(shooter.testHoodPos);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.setHoodPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}