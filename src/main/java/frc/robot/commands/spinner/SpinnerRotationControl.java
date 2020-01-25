package frc.robot.commands.spinner;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Spinner;

/**
 * Obtains rotational control on the control panel.
 * 
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * 
 * @since January 25, 2020
 */
public class SpinnerRotationControl extends CommandBase {
    private static final int ROTATION_SETPOINT = Spinner.COLOR_OFFSET * (8 * 3 + 4); // 3.5 rotations of the wheel

    public SpinnerRotationControl() {
        addRequirements(Spinner.getInstance());
    }

    @Override
    public void initialize() {
        Spinner.getInstance().getSpinnerMotor().selectProfileSlot(Spinner.SPINNER_POSITION_SLOT, RobotMap.PRIMARY_INDEX);
        Spinner.getInstance().getSpinnerMotor().setSelectedSensorPosition(0);
    }

    @Override
    public void execute() {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Position, ROTATION_SETPOINT);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Spinner.getInstance().getSpinnerMotor().getClosedLoopError()) <= Spinner.ALLOWABLE_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);
    }
}