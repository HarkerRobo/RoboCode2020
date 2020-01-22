package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

/**
 * Spins the shooter at a certain velocity
 * 
 * @param
 */
public class SpinShooter extends CommandBase {
    private static final double SPEED_MULTIPLIER = 1;
    
    private double outputMagnitude;

    public SpinShooter(double outputMagnitude) {
        addRequirements(Shooter.getInstance());
        this.outputMagnitude = outputMagnitude;
    }

    public void execute() {
        Shooter.getInstance().getMaster().set(TalonFXControlMode.PercentOutput, outputMagnitude * SPEED_MULTIPLIER);
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(TalonFXControlMode.Disabled, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}