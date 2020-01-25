package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Spins the indexer as balls are intaked
 * 
 * @author Shahzeb Lakhani
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @since January 23, 2020
 */
public class SpinIndexer extends IndefiniteCommand {
    private static final double INDEX_SPEED = 0.2;

    private boolean intakeFlag; // If the intake sensor just detected a ball
    private boolean indexerFlag; // If the ball in front of the indexer sensor has evacuated

    public SpinIndexer() {
        addRequirements(Indexer.getInstance());
        intakeFlag = false;
        indexerFlag = false;
    }

    @Override
    public void execute() {
        boolean intakeDetected = !Indexer.getInstance().getIntakeSensor().get();
        boolean indexerDetected = !Indexer.getInstance().getHopperSensor().get();
        boolean shooterDetected = !Indexer.getInstance().getShooterSensor().get();
        
        // If the indexer is full, never move
        if(!shooterDetected) {
            // If something is currently next to the intake sensor, say that a ball has passed through this sensor.
            if(intakeDetected && !intakeFlag) {
                intakeFlag = true;
                // Indexer.getInstance().numPowerCells++;
            }
            // Once the ball in front of the indexer has left, wait for the intake ball to reach the indexer
            if(intakeFlag && !indexerDetected) {
                indexerFlag = true;
                // Indexer.getInstance().numPowerCells++;
            }
            //If there is a ball that needs to be indexed - i.e one that has passed through the intake.
            if(intakeFlag) {
                Indexer.getInstance().spinIndexer(INDEX_SPEED);
            }
            //If there is a ball that needed to be indexed and there is something new in front of the indexer,
            //then we know that the ball that needed to be index reached its place and can reset our flags.
            if(intakeFlag && indexerFlag && indexerDetected) {
                Indexer.getInstance().spinIndexer(0); 
                intakeFlag = false;
                indexerFlag = false;
                // Indexer.getInstance().numPowerCells--;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinIndexer(INDEX_SPEED);
    }
}