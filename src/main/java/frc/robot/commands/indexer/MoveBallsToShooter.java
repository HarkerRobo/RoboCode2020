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
 * @author Anirudh 
 * 
 * @since January 23, 2020
 */
public class MoveBallsToShooter extends IndefiniteCommand {
    private static final double INDEX_PERCENT_OUTPUT = 0.8;

    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }

    @Override
    public void execute() {
        Indexer.getInstance().spinIndexer(INDEX_PERCENT_OUTPUT);  
    }
    
    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinIndexer(0);
    }
}