package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BottomIntake;
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
    private static final double INDEX_SPEED = 1;
    private boolean backwards;
    // private boolean intakeFlag; // If the intake sensor just detected a ball
    private boolean indexerFlag; // If the ball in front of the indexer sensor has evacuated

    public SpinIndexer(boolean backwards) {
        addRequirements(Indexer.getInstance());
        // intakeFlag = false;
        indexerFlag = false;
        this.backwards = backwards;
    }

    @Override
    public void execute() {
        // boolean indexerDetected = !Indexer.getInstance().getIndexerSensor().get();
        boolean shooterDetected = !Indexer.getInstance().getShooterSensor().get();
        long currentTime = System.currentTimeMillis();

        SmartDashboard.putBoolean("Shooter Detected", shooterDetected);

        if(!shooterDetected || backwards)
            Indexer.getInstance().spinSpine(backwards? -INDEX_SPEED : INDEX_SPEED);
        else
            Indexer.getInstance().spinSpine(0);

        if (currentTime % Indexer.AGITATOR_CYCLE_DUR < Indexer.AGITATOR_ON_DURATION)
            Indexer.getInstance().spinAgitator(Indexer.AGITATOR_DEFAULT_OUTPUT);
        else
            Indexer.getInstance().spinAgitator(backwards ? -Indexer.AGITATOR_DEFAULT_OUTPUT : 0);

        

        // // If the indexer is full, never move
        // if(!shooterDetected) {
        //     // If something is currently next to the intake sensor, say that a ball has passed through this sensor.
        //     if(intakeDetected && !intakeFlag) {
        //         intakeFlag = true;
        //         // Indexer.getInstance().numPowerCells++;
        //     }
        //     // Once the ball in front of the indexer has left, wait for the intake ball to reach the indexer
        //     if(intakeFlag && !indexerDetected) {
        //         indexerFlag = true;
        //         // Indexer.getInstance().numPowerCells++;
        //     }
        //     //If there is a ball that needs to be indexed - i.e one that has passed through the intake.
        //     if(intakeFlag) {
        //         Indexer.getInstance().spinIndexer(INDEX_SPEED);
        //     }
        //     //If there is a ball that needed to be indexed and there is something new in front of the indexer,
        //     //then we know that the ball that needed to be index reached its place and can reset our flags.
        //     if(intakeFlag && indexerFlag && indexerDetected) {
        //         Indexer.getInstance().spinIndexer(0); 
        //         intakeFlag = false;
        //         indexerFlag = false;
        //         // Indexer.getInstance().numPowerCells--;
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinSpine(0);
        Indexer.getInstance().spinAgitator(0);
    }
}