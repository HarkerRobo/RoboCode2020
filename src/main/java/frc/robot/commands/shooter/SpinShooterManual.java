package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.OI;

/**
 * Set
 */
public class SpinShooterManual extends CommandBase {
    public static final int SPEED_MULTIPLIER = 1;

    public SpinShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        double rightTriggerValue = OI.getInstance().getOperatorGamepad().getRightTrigger();

        Shooter.getInstance().getMaster().set(TalonFXControlMode.PercentOutput, rightTriggerValue * SPEED_MULTIPLIER);

    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}