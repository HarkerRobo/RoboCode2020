package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Runs the Indexer continuously to feed all balls into the shooter. 
 * The shooter should already be spinning as this command begins.
 * 
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * 
 * @since January 23, 2020
 */
public class MoveBallsToShooter extends IndefiniteCommand {
    private static final double INDEX_PERCENT_OUTPUT = 0.8; //0.89

    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance()); 
    }

    @Override
    public void initialize() {
        Indexer.getInstance().getSolenoid().set(false);
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        if (currentTime % Indexer.AGITATOR_CYCLE_DUR > Indexer.AGITATOR_ON_DURATION) {
            Indexer.getInstance().spinAgitator(Indexer.AGITATOR_DEFAULT_OUTPUT);
        }
        else {
            Indexer.getInstance().spinAgitator(0);
        }
        Indexer.getInstance().spinSpine(INDEX_PERCENT_OUTPUT);  
    }
    
    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinSpine(0);
        Indexer.getInstance().getSolenoid().set(true);
    }
}