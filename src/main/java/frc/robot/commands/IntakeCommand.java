package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake m_intake;

    public IntakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        m_intake.setIntakePosition(Intake.testAngle);
        m_intake.IntakeRollerRPM(20);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopMotor();
        m_intake.setIntakePosition(Intake.kStowAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}