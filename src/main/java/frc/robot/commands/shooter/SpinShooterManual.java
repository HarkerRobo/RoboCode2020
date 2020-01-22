package frc.robot.commands.shooter;

import frc.robot.OI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

/**
 * Spins the shooter at a velocity determined by the driver right y value
 * 
 */
public class SpinShooterManual extends CommandBase {
    private static final double SPEED_MULTIPLIER = 1;

    public SpinShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        double rightY = OI.getInstance().getDriverGamepad().getRightY();

        Shooter.getInstance().getMaster().set(TalonFXControlMode.PercentOutput, rightY * SPEED_MULTIPLIER);
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(TalonFXControlMode.Disabled, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}