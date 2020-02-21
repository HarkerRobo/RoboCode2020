package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.OI;
import frc.robot.subsystems.Climber;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Sets the climber to a specific position and feedforward.
 * 
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @author Angela Jia
 * @author Chirag Kaushik
 * @since February 6, 2020
 */
public class MoveClimberManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 0.3;

    public MoveClimberManual() {
        addRequirements(Climber.getInstance());
    }

    @Override
    public void execute() {
        double output = OUTPUT_MULTIPLIER * ((OI.getInstance().getDriverGamepad().getUpDPadButton().get() ? 1 : 0) - (OI.getInstance().getDriverGamepad().getDownDPadButton().get() ? 1 : 0));
        Climber.getInstance().getMaster().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, Climber.FEED_FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Climber.FEED_FORWARD);
    }
}