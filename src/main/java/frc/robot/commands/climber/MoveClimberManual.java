package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final double OUTPUT_MULTIPLIER = 1;

    private boolean inputFlag;

    public MoveClimberManual() {
        addRequirements(Climber.getInstance());
    }

    @Override
    public void initialize() {
        inputFlag = false;

        Climber.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Climber.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }

    @Override
    public void execute() {
        boolean upDpad = OI.getInstance().getDriverGamepad().getUpDPadButton().get();
        boolean downDpad = OI.getInstance().getDriverGamepad().getDownDPadButton().get();
        double output = OUTPUT_MULTIPLIER * ((upDpad ? 1 : 0) - (downDpad ? 1 : 0));

        SmartDashboard.putNumber("climber manual output", output);

        if (upDpad || downDpad)
            inputFlag = true;

        if (inputFlag)
            Climber.getInstance().getMaster().set(ControlMode.PercentOutput, output);
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, MoveClimbers.FEED_FORWARD);
    }
}