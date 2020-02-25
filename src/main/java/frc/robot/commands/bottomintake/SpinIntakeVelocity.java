package frc.robot.commands.bottomintake;

import frc.robot.subsystems.BottomIntake;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Spins the bottom intake at a specific percent output.
 * 
 * @author Chirag Kaushik
 * @since February 12, 2020
 */
public class SpinIntakeVelocity extends IndefiniteCommand {
    private double magnitude;

    public SpinIntakeVelocity(double magnitude) {
        addRequirements(BottomIntake.getInstance());
        
        this.magnitude = magnitude;
    }

    @Override
    public void execute() {
        BottomIntake.getInstance().spinIntake(magnitude);
    }

    @Override
    public void end(boolean interrupted) {
        BottomIntake.getInstance().spinIntake(0);
    }
}