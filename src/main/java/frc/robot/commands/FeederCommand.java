package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class FeederCommand extends Command {

    public final Indexer indexer;
    private final double indexerRpm;
    private final double feederRpm;

    public FeederCommand(Indexer indexer, double indexerRpm, double feederRpm) {
        this.indexer = indexer;
        this.indexerRpm = indexerRpm;
        this.feederRpm = feederRpm;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIndexerRPM(indexerRpm);
        indexer.setFeederRPM(feederRpm);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
        indexer.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}