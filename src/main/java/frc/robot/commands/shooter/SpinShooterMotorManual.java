package frc.robot.commands.shooter;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystem.Shooter;
import frc.robot.OI;

public class SpinShooterMotorManual implements Command {
    
    private Set<Subsystem> subsystems;

    public static final int SPEED_MULTIPLIER = 1;

    public SpinShooterMotorManual() 
    {
        subsystems = new HashSet<Subsystem>();
        subsystems.add(Shooter.getInstance());
        
    }

    public void execute() {

        double rightTriggerValue = OI.getInstance().getOperatorGamepad().getRightTrigger();

        Shooter.getInstance().getMaster().set(TalonFXControlMode.PercentOutput, rightTriggerValue * SPEED_MULTIPLIER);

    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return subsystems;
    }
    
}