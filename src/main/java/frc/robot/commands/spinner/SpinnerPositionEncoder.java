package frc.robot.commands.spinner;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Spinner.ColorValue;

/**
 * Obtains position control on the control panel using the color' sensor's initial color and the encoder's position.
 * We closed loop to a position calculated using the difference in colors between our current color wedge and the desired color wedge.
 * 
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * 
 * @since January 25, 2020
 */
public class SpinnerPositionEncoder extends CommandBase {
    private int positionSetpoint;

    public SpinnerPositionEncoder() {
        addRequirements(Spinner.getInstance()); 
    }

    @Override
    public void initialize() {
        String info = DriverStation.getInstance().getGameSpecificMessage();
        ColorValue stationVal = null;
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
        
        ColorValue actualVal = Spinner.getInstance().getCurrentColor();

        int deltaWedges = (stationVal.getVal() - actualVal.getVal() + 4) % 4; 
        if(deltaWedges == 3) deltaWedges -= 4;
        positionSetpoint = Spinner.COLOR_OFFSET * deltaWedges;
        Spinner.getInstance().getSpinnerMotor().setSelectedSensorPosition(0);
    }

    @Override
    public void execute() {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Position, positionSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        Spinner.getInstance().getSpinnerMotor().set(ControlMode.Disabled, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Spinner.getInstance().getSpinnerMotor().getClosedLoopError()) <= Spinner.ALLOWABLE_ERROR;
    }

}