package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
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
 * @author Angela Jia
 * 
 * @since January 23, 2020
 */
public class MoveBallsToShooter extends IndefiniteCommand {
    private static final double INDEX_PERCENT_OUTPUT = 0.9; 
    private static final long MIN_TIME = 100;

    private boolean backwards;

    private long startTime;

    public MoveBallsToShooter(boolean backwards) {
        addRequirements(Indexer.getInstance()); 
        this.backwards = backwards;
    }

    @Override
    public void initialize() {
        Indexer.getInstance().getAgitator().setNeutralMode(NeutralMode.Coast);

        Indexer.getInstance().getSolenoid().set(Indexer.OPEN);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();

        if (!Shooter.isPercentOutput && currentTime - startTime > MIN_TIME) {
            if (currentTime % Indexer.AGITATOR_CYCLE_DUR < Indexer.AGITATOR_ON_DURATION)
                Indexer.getInstance().spinAgitator(Indexer.AGITATOR_DEFAULT_OUTPUT);
            else
                Indexer.getInstance().spinAgitator(backwards ? -Indexer.AGITATOR_DEFAULT_OUTPUT : 0);

            Indexer.getInstance().spinSpine(INDEX_PERCENT_OUTPUT);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinSpine(0);
        Indexer.getInstance().spinAgitator(0);

        Indexer.getInstance().getSolenoid().set(Indexer.CLOSED);
        Shooter.isPercentOutput = true;
    }
}