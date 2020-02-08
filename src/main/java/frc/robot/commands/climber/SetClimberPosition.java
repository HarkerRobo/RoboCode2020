package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
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
public class SetClimberPosition extends IndefiniteCommand {
    private int position;
    private double feedForward;

    public SetClimberPosition(int position, double feedForward) {
        addRequirements(Climber.getInstance());
        this.position = position;
        this.feedForward = feedForward;
    }

    @Override
    public void initialize() {
        Climber.getInstance().getMaster().selectProfileSlot(Climber.CLIMBER_POSITION_SLOT, RobotMap.PRIMARY_INDEX);
    }

    @Override
    public void execute() {
        Climber.getInstance().getMaster().set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}