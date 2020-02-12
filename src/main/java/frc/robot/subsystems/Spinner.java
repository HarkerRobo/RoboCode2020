package frc.robot.subsystems;

import harkerrobolib.wrappers.HSTalon;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The Spinner manipulates the control panel to obtain position and rotation control.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani 
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @author Aimee Wang
 * @since January 22, 2020
 */
public class Spinner extends SubsystemBase {
    private static Spinner instance;

    private HSTalon spinnerMotor;
    private DoubleSolenoid solenoid;
    // private CANCoder cancoder; 

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();

    public static final int SPINNER_POSITION_SLOT = 0;
    private static final double SPINNER_POSITION_KP = 0.7; 
    private static final double SPINNER_POSITION_KI = 0;
    private static final double SPINNER_POSITION_KD = 0;
   
    /**
     * Distance (in encoder ticks) the motor needs to turn to rotate the wheel by one wedge
     */
    public static final int COLOR_OFFSET = 160; 
    
    private final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color redTarget = ColorMatch.makeColor(0.480, 0.350, 0.114);
    private final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static final boolean SPINNER_INVERT = false; // Fix
    private static final boolean SPINNER_SENSOR_PHASE = false; //Fix

    public static final int ALLOWABLE_ERROR = 100;

    public static final int PEAK_DURATION = 10; // Fix
    public static final int PEAK_LIMIT = 20; // Fix
    public static final int CONT_LIMIT = 15; // Fix
    
    public static final double VOLTAGE_COMPENSATION = 0; // Fix

    public enum ColorValue {
        RED(0), GREEN(1), BLUE(2), YELLOW(3); //Tune values for offset
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
        // cancoder = new CANCoder(RobotMap.CAN_IDS.CANCODER_ID);

        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(yellowTarget); 

        spinnerMotor.setInverted(SPINNER_INVERT);
        spinnerMotor.setSensorPhase(SPINNER_SENSOR_PHASE);   
             
        setupTalons();
    }

    private void setupTalons() {
        spinnerMotor.configFactoryDefault();
                
        spinnerMotor.setInverted(SPINNER_INVERT);

        spinnerMotor.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        spinnerMotor.enableVoltageCompensation(true);
        
        spinnerMotor.setNeutralMode(NeutralMode.Coast);
      
        spinnerMotor.setSensorPhase(SPINNER_SENSOR_PHASE);

        spinnerMotor.configForwardSoftLimitEnable(false);
        spinnerMotor.configReverseSoftLimitEnable(false);

        spinnerMotor.configContinuousCurrentLimit(CONT_LIMIT);
        spinnerMotor.configPeakCurrentDuration(PEAK_DURATION);
        spinnerMotor.configPeakCurrentLimit(PEAK_LIMIT);
        spinnerMotor.enableCurrentLimit(true);
        // spinnerMotor.configRemoteFeedbackFilter(RobotMap.CAN_IDS.CANCODER_ID, RemoteSensorSource.CANCoder, RobotMap.SPINNER_REMOTE_ORDINAL);
        
        setupPositionPID();
    } 
    
    public void setupPositionPID() {
        spinnerMotor.config_kP(SPINNER_POSITION_SLOT, SPINNER_POSITION_KP);
        spinnerMotor.config_kI(SPINNER_POSITION_SLOT, SPINNER_POSITION_KI);
        spinnerMotor.config_kD(SPINNER_POSITION_SLOT, SPINNER_POSITION_KD);
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
        /*
        switch(colorMatcher.matchClosestColor(colorSensor.getColor())) {
            case(blueTarget): return ColorValue.BLUE;
            case(redTarget): return ColorValue.RED;
            case(greenTarget): return ColorValue.GREEN;
            case(yellowTarget): return ColorValue.YELLOW;
            default: return null;   
        }
        */
    }

    public static Spinner getInstance() {
        if(instance == null)
            instance = new Spinner();
        return instance;
    }
}