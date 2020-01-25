package frc.robot.subsystems;

import harkerrobolib.wrappers.HSTalon;

import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * The Spinner manipulates the control panel to obtain position and rotation control.
 * 
 * @author Anirudh Kotamraju
 * @author Jatin Kohli
 * @author Shahzeb Lakhani 
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @author Aimee Wang
 * @since January 22, 2020
 */
public class Spinner extends SubsystemBase {
    private static final int COLOR_OFFSET = 160;//Tune for amount of distance the motor needs to turn to change color by one

    private static Spinner instance;

    private HSTalon bagMotor;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();
    
    private final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color redTarget = ColorMatch.makeColor(0.480, 0.350, 0.114);
    private final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public enum ColorValue {
        RED(0),BLUE(1),GREEN(2),YELLOW(3);//Tune values for offset
        int val;
        
        private ColorValue(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }

    }

    public Spinner() {
        //Same motor controller being used for spinner and indexer
        bagMotor = new HSTalon(RobotMap.CAN_IDS.SPINNER_ID);
        colorSensor = new ColorSensorV3(i2cPort);
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(yellowTarget); 
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();

        String colorString;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == blueTarget) {
            colorString = "Blue";
        } else if (match.color == redTarget) {
            colorString = "Red";
        } else if (match.color == greenTarget) {
            colorString = "Green";
        } else if (match.color == yellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("red", detectedColor.red);
        SmartDashboard.putNumber("green", detectedColor.green);
        SmartDashboard.putNumber("blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        SmartDashboard.putNumber("Desired red", match.color.red);
        SmartDashboard.putNumber("Desired blue", match.color.blue);
        SmartDashboard.putNumber("Desired green", match.color.green);
    }

    public HSTalon getBagMotor() {
        return bagMotor;
    }

    //Don't need to re-initialize hstalon configs for bagMotor

    public static Spinner getInstance() {
        if(instance == null)
            instance = new Spinner();
        return instance;
    }

    public void spinToColor() {
        String info = DriverStation.getInstance().getGameSpecificMessage();
        ColorValue stationVal = null;
        if (info.equals("B")) {
            stationVal = ColorValue.BLUE;
        } else if(info.equals("R")) {
            stationVal = ColorValue.RED;
        } else if (info.equals("Y")) {
            stationVal = ColorValue.YELLOW;
        } else if (info.equals("G")) {
            stationVal = ColorValue.GREEN;
        }

        Color curColor = colorSensor.getColor();

        ColorMatchResult match = colorMatcher.matchClosestColor(curColor);
        ColorValue curVal = null;
        if (match.color == blueTarget) {
            curVal = ColorValue.BLUE;
        } else if (match.color == redTarget) {
            curVal = ColorValue.RED;
        } else if (match.color == greenTarget) {
            curVal = ColorValue.GREEN;
        } else if (match.color == yellowTarget) {
            curVal = ColorValue.YELLOW;
        }
        int position = 3 * 8 * COLOR_OFFSET + COLOR_OFFSET * (curVal.getVal() - stationVal.getVal());
        bagMotor.set(ControlMode.Position, position);
        // bagMotor.set(ControlMode.Position, 3 * 8 * COLOR_OFFSET + COLOR_OFFSET * ());
    }
}