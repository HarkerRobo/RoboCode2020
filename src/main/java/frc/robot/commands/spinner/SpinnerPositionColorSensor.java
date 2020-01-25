package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Spinner.ColorValue;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Obtains position control on the control panel using the color sensor readings.
 * 
 * @author Angela Jia
 * @author Dennis Garvey
 * 
 * @since January 25, 2020
 */
public class SpinnerPositionColorSensor extends CommandBase {
    private ColorValue stationVal;

    public static final double OUTPUT = 0.3;

    public SpinnerPositionColorSensor() {
        addRequirements(Spinner.getInstance());
    }

    @Override
    public void initialize() {
        String info = DriverStation.getInstance().getGameSpecificMessage();
        if (info.equals("B")) 
            stationVal = ColorValue.RED; // our sensor and the field sensor are not in the same location
        else if(info.equals("R")) 
          stationVal = ColorValue.BLUE;
        else if (info.equals("Y")) 
          stationVal = ColorValue.GREEN;
        else if (info.equals("G")) 
          stationVal = ColorValue.YELLOW;
        else 
            CommandScheduler.getInstance().cancel(this);
    }

    @Override
    public void execute() {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, OUTPUT);
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);
    }

    @Override
    public boolean isFinished() {
        return Spinner.getInstance().getCurrentColor().getVal() == stationVal.getVal();
    }

}