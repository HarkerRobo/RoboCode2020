package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Spins the spine
 * 
 * @author Arjun Dixit
 * @since April 9, 2020
 */
public class SpinSpine extends IndefiniteCommand {
    private double output;
    private boolean isPercentOutput;

    public SpinSpine(double output, boolean isPercentOutput) {
        addRequirements(Indexer.getInstance());

        this.output = output;
        this.isPercentOutput = isPercentOutput;
    }

    @Override
    public void execute() {
        if(isPercentOutput)
            Indexer.getInstance().spinSpinePercentOutput(output);
        else
            Indexer.getInstance().spinSpineVelocity(output);
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().getSpine().set(ControlMode.Disabled, 0);
    }
}