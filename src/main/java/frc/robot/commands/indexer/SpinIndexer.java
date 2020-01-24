package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * 
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * @since January 23, 2020
 */
public class SpinIndexer extends IndefiniteCommand {
    private static final double INDEX_SPEED = 0.2;

    private boolean intakeFlag; // If the intake sensor just detected a ball
    private boolean hopperFlag; // If the ball in front of the hopper sensor has evacuated

    public SpinIndexer() {
        addRequirements(Indexer.getInstance());
        intakeFlag = false;
        hopperFlag = false;
    }
    
    @Override
    public void execute() {

        boolean intakeDetected = Indexer.getInstance().getIntakeSensor().get();
        boolean hopperDetected = Indexer.getInstance().getHopperSensor().get();
        boolean shooterDetected = Indexer.getInstance().getShooterSensor().get();
        
        //Check if the shooter currently does not have any ball.
        if(!shooterDetected) {
            
            if(intakeDetected) {
                intakeFlag = true;
            }
            if(intakeFlag && !hopperDetected) {
                hopperFlag = true;
            }
            if(intakeFlag) {
                Indexer.getInstance().spinIndexer(INDEX_SPEED);
            }
            if(intakeFlag && hopperFlag && hopperDetected) {
                Indexer.getInstance().spinIndexer(0);
                intakeFlag = false;
                hopperFlag = false;
            }
        }
    }
}