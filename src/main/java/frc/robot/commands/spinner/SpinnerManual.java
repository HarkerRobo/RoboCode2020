package frc.robot.commands.spinner;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Spinner;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SpinnerManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 0.4;

    public SpinnerManual() {
        addRequirements(Spinner.getInstance());
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
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