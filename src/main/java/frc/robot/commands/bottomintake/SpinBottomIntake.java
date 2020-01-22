package frc.robot.commands.bottomintake;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomIntake;

/**
 * Spins the Bottom Intake
 * 
 * @author Shahzeb Lakhani
 * @version 1/21/20
 */
public class SpinBottomIntake extends CommandBase {
    private static final double SPEED_MULTIPLIER = 1;

    private double outputMagnitude;

    public SpinBottomIntake(double outputMagnitude) {
        addRequirements(BottomIntake.getInstance());
        this.outputMagnitude = outputMagnitude;
    }

    @Override
    public void execute() {
        // bottomIntakeMagnitude = 1;
        BottomIntake.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, outputMagnitude * SPEED_MULTIPLIER);
    }

    @Override
    public void end(boolean interrupted) {
        BottomIntake.getInstance().getFalcon().set(TalonFXControlMode.Disabled, 0);
    }  

    @Override
    public boolean isFinished() {
        return false;
    }
}