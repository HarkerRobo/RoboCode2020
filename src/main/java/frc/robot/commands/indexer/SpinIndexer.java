package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SpinIndexer extends CommandBase {
    private static final double SPEED_MULTIPLIER = 1;

    public double outputMagnitude;
    
    public SpinIndexer(double outputMagnitude) {
        addRequirements(Indexer.getInstance());
        this.outputMagnitude = outputMagnitude;
    }
    
    public void execute() { 
        Indexer.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, outputMagnitude * SPEED_MULTIPLIER);
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().getFalcon().set(TalonFXControlMode.Disabled, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}