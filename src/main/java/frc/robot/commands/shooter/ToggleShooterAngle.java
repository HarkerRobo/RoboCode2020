package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

/**
 * Toggles the Shooter Solenoids
 * 
 * @since 1/6/20
 */
public class ToggleShooterAngle extends InstantCommand {

    public ToggleShooterAngle() {
        addRequirements(Shooter.getInstance());
    }

    public void initialize() {
        Shooter.getInstance().getAngleSolenoid().set(Shooter.getInstance().getAngleSolenoid().get() == Shooter.HIGH_ANGLE ? Shooter.LOW_ANGLE : Shooter.HIGH_ANGLE);
    }
}