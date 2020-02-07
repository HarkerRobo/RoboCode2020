package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

public class ExtendClimber extends CommandBase {
    private static final int CLIMBER_CLIMB_POSITION = 0;

    public ExtendClimber() {
        addRequirements(Climber.getInstance());
        
    }

    @Override
    public void initialize() {
        Climber.getInstance().getMaster().selectProfileSlot(Climber.CLIMBER_POSITION_SLOT, RobotMap.PRIMARY_INDEX);
    }

    @Override
    public void execute() {
        Climber.getInstance().getMaster().set(ControlMode.Position, CLIMBER_CLIMB_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}