package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 * @author Angela Jia
 * @since January 23, 2020
 */
public class SpinIndexer extends IndefiniteCommand {
    private static final double INDEX_SPEED = 1;
    private boolean backwards;
    private double output;

    public SpinIndexer(double output, boolean backwards) {
        addRequirements(Indexer.getInstance());

        this.backwards = backwards;
        this.output = output;
    }

    @Override
    public void initialize() {
        Indexer.getInstance().getAgitator().setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void execute() {
        boolean indexerDetected = !Indexer.getInstance().getIndexerSensor().get();
        boolean hopperDetected = !Indexer.getInstance().getShooterSensor().get();

        long currentTime = System.currentTimeMillis();

        SmartDashboard.putBoolean("Shooter Detected", hopperDetected);
        SmartDashboard.putBoolean("Indexer detected", indexerDetected);
        // if (prevDetection == true && hopperDetected == false && !backwards) {
        //     indexerFlag = true;
        // }

        if(!(hopperDetected && indexerDetected) || backwards)
            Indexer.getInstance().spinSpine(output * (backwards ? -INDEX_SPEED : INDEX_SPEED));
        else
            Indexer.getInstance().spinSpine(0);

        if (currentTime % Indexer.AGITATOR_CYCLE_DUR < Indexer.AGITATOR_ON_DURATION)
            Indexer.getInstance().spinAgitator(output * Indexer.AGITATOR_DEFAULT_OUTPUT);
        else
            Indexer.getInstance().spinAgitator(output * (backwards ? -Indexer.AGITATOR_DEFAULT_OUTPUT : 0));
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().spinSpine(0);
        Indexer.getInstance().spinAgitator(0);
    }
}