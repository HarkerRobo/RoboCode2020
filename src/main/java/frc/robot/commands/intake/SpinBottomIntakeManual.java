package frc.robot.commands.intake;

import java.util.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.OI;
import frc.robot.subsystem.BottomIntake;

/**
 * Spins the Bottom Intake 
 */
public class SpinBottomIntakeManual implements Command {

    private double bottomIntakeMagnitude = 0;
    private Set<Subsystem> subsystems;
    

    public SpinBottomIntakeManual() {
        subsystems = new HashSet<Subsystem>();
        subsystems.add(BottomIntake.getInstance());   
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return subsystems;
    }

    @Override
    public void execute() {
        boolean isPressed = OI.getInstance().getDriverGamepad().getButtonAState();
        if(isPressed)
        {
            bottomIntakeMagnitude = 1;
        }
        else
        {
            bottomIntakeMagnitude = 0;
        }
        BottomIntake.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, bottomIntakeMagnitude);
    }

    @Override
    public void end(boolean interrupted) {
        BottomIntake.getInstance().getFalcon().set(TalonFXControlMode.Disabled, 0);
    }  
}