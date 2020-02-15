package frc.robot.commands.spinner;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Spinner;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the Spinner motor based on operator right joystick x input.
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Arjun Dixit
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * 
 * @since Februrary 14, 2020
 */
public class SpinnerManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 0.7;

    public SpinnerManual() {
        addRequirements(Spinner.getInstance());
    }

    @Override
    public void execute() {
        double output = MathUtil.mapJoystickOutput(OI.getInstance().getOperatorGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, OUTPUT_MULTIPLIER * output);
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);
    }
}