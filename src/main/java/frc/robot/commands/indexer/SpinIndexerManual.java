package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SpinIndexerManual extends CommandBase {
    public static double magnitude = 0;
    
    public SpinIndexerManual(double outputMagnitude) {
        addRequirements(Indexer.getInstance());
        magnitude = outputMagnitude;
    }
    
    public void execute() { 
        Indexer.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, magnitude);
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