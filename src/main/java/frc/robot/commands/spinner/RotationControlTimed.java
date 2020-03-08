package frc.robot.commands.spinner;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

/**
 * Spins the Spinner motor for a set certain time using percent output.
 * 
 * @author Jatin Kohli
 * @author Arjun Dixit
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Anirudh Kotamraju
 * @author Shahzeb Lakhani
 * 
 * @since Februrary 14, 2020
 */
public class RotationControlTimed extends CommandBase {
    private static final double OUTPUT_MULTIPLIER = 0.8;
    private static double currentTime;
    private static final double TIMEOUT = 2200; //ms

    public RotationControlTimed() {
        addRequirements(Spinner.getInstance());
    }

    @Override
    public void initialize() {
        currentTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, OUTPUT_MULTIPLIER);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - currentTime > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);
    }
}