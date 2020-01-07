package frc.robot.commands.indexer;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.OI;
import frc.robot.subsystem.Indexer;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SpinIndexerManual implements Command {

    private Set<Subsystem> subsystems;

    public static double indexerMagnitude = 0;
    
    public SpinIndexerManual() {
        subsystems = new HashSet<Subsystem>();
        subsystems.add(Indexer.getInstance());
    }
    
    public void execute() { 
        boolean isPressed = OI.getInstance().getDriverGamepad().getButtonXState();
        if(isPressed)
        {
            indexerMagnitude = 1;
        }
        else
        {
            indexerMagnitude = 0;
        }
        Indexer.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, indexerMagnitude);
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().getFalcon().set(TalonFXControlMode.Disabled, 0);
    }


    
    @Override
    public Set<Subsystem> getRequirements() {
        return subsystems;
    }
}