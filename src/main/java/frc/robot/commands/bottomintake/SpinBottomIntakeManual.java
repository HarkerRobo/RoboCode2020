package frc.robot.commands.bottomintake;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomIntake;

/**
 * Spins the Bottom Intake 
 */
public class SpinBottomIntakeManual extends CommandBase {

    private double bottomIntakeMagnitude;

    public SpinBottomIntakeManual(double magnitude) {
        addRequirements(BottomIntake.getInstance());
        bottomIntakeMagnitude = magnitude;
    }

    @Override
    public void execute() {
        // bottomIntakeMagnitude = 1;
        BottomIntake.getInstance().getFalcon().set(TalonFXControlMode.PercentOutput, bottomIntakeMagnitude);
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