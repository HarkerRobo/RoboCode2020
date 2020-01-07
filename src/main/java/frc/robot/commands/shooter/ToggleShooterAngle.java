package frc.robot.commands.shooter;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.Shooter;

/**
 * Toggles the Shooter Solenoids
 * 
 * @since 1/6/20
 */
public class ToggleShooterAngle extends InstantCommand {

    private Set<Subsystem> subsystems;

    @Override
    public Set<Subsystem> getRequirements() {
        return subsystems;
    }

    public ToggleShooterAngle() {
        subsystems = new HashSet<Subsystem>();
        subsystems.add(Shooter.getInstance());
    }

    public void initialize() {
        Shooter.getInstance().getAngleSolenoid().set(Shooter.getInstance().getAngleSolenoid().get() == Shooter.HIGH_ANGLE ? Shooter.LOW_ANGLE : Shooter.HIGH_ANGLE);
    }
}