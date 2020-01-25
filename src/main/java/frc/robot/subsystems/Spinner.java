package frc.robot.subsystems;

import harkerrobolib.wrappers.HSTalon;

import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    private static Spinner instance;

    private HSTalon spinnerMotor;
    private DoubleSolenoid solenoid;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();
    /**
     * Distance (in encoder ticks) the motor needs to turn to rotate the wheel by one wedge
     */
    public static final int COLOR_OFFSET = 160; 
    
    private final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color redTarget = ColorMatch.makeColor(0.480, 0.350, 0.114);
    private final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static final boolean SPINNER_INVERT = false;
    private static final boolean SPINNER_SENSOR_PHASE = false;

    public static final int SPINNER_POSITION_SLOT = 0;
    private static final double SPINNER_POSITION_KP = 0.7; 
    private static final double SPINNER_POSITION_KI = 0;
    private static final double SPINNER_POSITION_KD = 0;

    public static final int ALLOWABLE_ERROR = 100;

    public enum ColorValue {
        RED(0), GREEN(1), BLUE(2), YELLOW(3);//Tune values for offset
        private int val;
        
        private ColorValue(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    }

    public Spinner() {
        //Same motor controller being used for spinner and indexer
        spinnerMotor = new HSTalon(RobotMap.CAN_IDS.SPINNER_ID);
        colorSensor = new ColorSensorV3(i2cPort);
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.SPINNER_SOLENOID_FORWARD, RobotMap.CAN_IDS.SPINNER_SOLENOID_REVERSE);

        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(yellowTarget); 

        spinnerMotor.setInverted(SPINNER_INVERT);
        spinnerMotor.setSensorPhase(SPINNER_SENSOR_PHASE);   
             
        setUpPositionPID();
    }

    public void setUpPositionPID() {
        spinnerMotor.config_kP(SPINNER_POSITION_SLOT, SPINNER_POSITION_KP);
        spinnerMotor.config_kI(SPINNER_POSITION_SLOT, SPINNER_POSITION_KI);
        spinnerMotor.config_kD(SPINNER_POSITION_SLOT, SPINNER_POSITION_KD);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("red", detectedColor.red);
        // SmartDashboard.putNumber("green", detectedColor.green);
        // SmartDashboard.putNumber("blue", detectedColor.blue);
        // SmartDashboard.putNumber("Confidence", match.confidence);
        // SmartDashboard.putString("Detected Color", currentColorString);
        // SmartDashboard.putNumber("Desired red", match.color.red);
        // SmartDashboard.putNumber("Desired blue", match.color.blue);
        // SmartDashboard.putNumber("Desired green", match.color.green);
    }

    public HSTalon getSpinnerMotor() {
        return spinnerMotor;
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    // public void spinToColor(ColorValue stationValue, ColorValue actualValue) {
    //     int error = stationValue.getVal() - actualValue.getVal(); //Number of color wedges away
    //     int multiplierVal = error < 0 ? 3 : 4; 

    //     int position = multiplierVal * 8 * COLOR_OFFSET + COLOR_OFFSET * (error);
    //     spinnerMotor.set(ControlMode.Position, position);
    //     // bagMotor.set(ControlMode.Position, 3 * 8 * COLOR_OFFSET + COLOR_OFFSET * ());
    // }

    public ColorValue getCurrentColor() {
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

        return curVal; 
    }

    public static Spinner getInstance() {
        if(instance == null)
            instance = new Spinner();
        return instance;
    }
}