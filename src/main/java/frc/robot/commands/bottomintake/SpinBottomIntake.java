package frc.robot.commands.bottomintake;

import frc.robot.OI;
import frc.robot.subsystems.BottomIntake;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Spins the Bottom Intake 
 */
public class SpinBottomIntake extends IndefiniteCommand {

    public SpinBottomIntake() {
        addRequirements(BottomIntake.getInstance());
    }

    @Override
    public void execute() {
        // bottomIntakeMagnitude = 1;
        if (OI.getInstance().getDriverGamepad().getButtonX().get()) {
            BottomIntake.getInstance().spinIntake(1);
        } 
        else if (OI.getInstance().getDriverGamepad().getButtonA().get()) {
            BottomIntake.getInstance().spinIntake(-1); 
        } 
        else {
            BottomIntake.getInstance().spinIntake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        BottomIntake.getInstance().spinIntake(0);
    }
}