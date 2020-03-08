package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Spinner.ColorValue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Obtains position control on the control panel using the color sensor readings.
 * 
 * @author Angela Jia
 * 
 * @since January 25, 2020
 */
public class SpinnerPositionColorSensor extends CommandBase {
    private ColorValue stationVal;

    public static double INIT_FAST_OUTPUT = 0.2;
    public static double END_FAST_OUTPUT = 0.15;
    public static double SLOW_OUTPUT = 0.3;

    private int initDiff;
    
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

        Spinner.getInstance().getSpinnerMotor().setNeutralMode(NeutralMode.Brake);
        Spinner.getInstance().getSpinnerMotor().configOpenloopRamp(0);
        
        initDiff = Spinner.getInstance().getCurrentColor().getVal() - stationVal.getVal();
    }

    @Override
    public void execute() {
        if (initDiff == 1) { //We started 1 wedge away
            Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput,SLOW_OUTPUT);
        }
        else {
            if (Spinner.getInstance().getCurrentColor().getVal() - stationVal.getVal() == 1) { //We are 1 wedge away
                Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, END_FAST_OUTPUT);
            }
            else { //We are 2 or more wedges away
                Spinner.getInstance().getSpinnerMotor().set(ControlMode.PercentOutput, INIT_FAST_OUTPUT);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return Spinner.getInstance().getCurrentColor().getVal() == stationVal.getVal();
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);

        Spinner.getInstance().getSpinnerMotor().configOpenloopRamp(Indexer.RAMP_RATE);
    }
}