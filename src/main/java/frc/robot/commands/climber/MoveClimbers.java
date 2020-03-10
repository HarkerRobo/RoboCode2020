package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Climber;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Moves the climber to one of its two soft limits
 * 
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @author Angela Jia
 * @author Chirag Kaushik
 * @since February 6, 2020
 */
public class MoveClimbers extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 1;
    public static final double FEED_FORWARD = 0;

    private static final int EPSILON = 10;

    private boolean down;

    public MoveClimbers(boolean down) {
        addRequirements(Climber.getInstance());

        this.down = down;
    }

    @Override
    public void initialize() {
        Climber.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Climber.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }

    @Override
    public void execute() {
        double output = OUTPUT_MULTIPLIER * (down ? -1 : 1);
        SmartDashboard.putNumber("climber manual output", output);

        Climber.getInstance().getMaster().set(ControlMode.PercentOutput, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Climber.getInstance().getMaster().getSelectedSensorVelocity()) < EPSILON;
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, FEED_FORWARD);

        Climber.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Climber.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }
}