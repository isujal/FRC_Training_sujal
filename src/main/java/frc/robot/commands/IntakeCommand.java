package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    public final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.setIntakePosition(Intake.testAngle);
        intake.setIntakeRollerRPM(80);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotor();
        intake.setIntakePosition(Intake.kStowAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}