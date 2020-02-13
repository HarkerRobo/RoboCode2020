package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

/**
 * Represents the Bottom Intake Subsystem, controlled by 1 Falcon.
 * 
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since January 6, 2020
 */
public class BottomIntake extends SubsystemBase {

    static {
        if(RobotMap.IS_PRACTICE) {
            MOTOR_INVERT = false; //Change accordingly
        } else {
            MOTOR_INVERT = false; //Change accordingly
        }
    }  

    private static BottomIntake instance;

    private HSTalon talon;
    
    private static boolean MOTOR_INVERT;

    private static final double VOLTAGE_COMPENSATION = 10;
    
    public static final double DEFAULT_ROLLER_MAGNITUDE_INTAKE = 0;

    private static final int INTAKE_CURRENT_CONTINUOUS = 40;

    private static final int INTAKE_CURRENT_PEAK = 50;

    private static final int INTAKE_CURRENT_PEAK_DUR = 100;

    private static final double OUTPUT_MULTIPLIER = 0.5;

    private DoubleSolenoid solenoid; 

    private BottomIntake() {
        talon = new HSTalon(RobotMap.CAN_IDS.INTAKE_MOTOR_ID);

        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.INTAKE_SOLENOID_FORWARD, RobotMap.CAN_IDS.INTAKE_SOLENOID_REVERSE);

        setupTalons();
    }
    
    private void setupTalons() {
        talon.configFactoryDefault();

        talon.setInverted(MOTOR_INVERT);

        talon.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        talon.enableVoltageCompensation(true);
        
        talon.setNeutralMode(NeutralMode.Coast);

        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        talon.configContinuousCurrentLimit(INTAKE_CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(INTAKE_CURRENT_PEAK);
        talon.configPeakCurrentDuration(INTAKE_CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    
        talon.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        talon.enableVoltageCompensation(true);
    }

    public HSTalon getTalon() {
        return talon;
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public void spinIntake(double magnitude) {
        if(magnitude == 0) 
            talon.set(ControlMode.Disabled, 0);
        else
            talon.set(ControlMode.PercentOutput, OUTPUT_MULTIPLIER * magnitude);
    }

    public static BottomIntake getInstance() {
        if(instance == null)
            instance = new BottomIntake();
        return instance;
    }
}