package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

/**
 * The indexer stores and transfers power cells from the intake to the shooter. It consists
 * of a single motor that spins a belt. 
 * 
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Aimee Wang
 * @author Shahzeb Lakhani
 * @since January 23, 2020
 */
public class Indexer implements Subsystem {
    static {
        if(RobotMap.IS_PRACTICE) {
            MASTER_INVERT = false; //Change accordingly
            BAG_INVERT = false;
        } else {
            MASTER_INVERT = false; //Change accordingly
            BAG_INVERT = false;
        }
    } 
    
    private static Indexer instance;
    
    private static boolean MASTER_INVERT;
    private static boolean BAG_INVERT;
    
    private HSTalon master;
    private HSTalon bagMotor;
    
    public static final double OUTPUT_MULTIPLIER = 0.5;

    public static final int VOLTAGE_COMPENSATION = 10;

    private static final int INTAKE_CURRENT_PEAK_DUR = 100;
    private static final int INTAKE_CURRENT_CONTINUOUS = 40;
    private static final int INTAKE_CURRENT_PEAK = 50;

    // private DigitalInput[] sensors;
    // private int[] sensorIDS = {0, 0, 0};

    private DigitalInput intakeSensor;  // The first sensor when a ball is intaked
    private DigitalInput hopperSensor;  // The second sensor in the indexer
    private DigitalInput shooterSensor; // Determines if the hopper is full
    
    private static final int INTAKE_SENSOR_ID = 0;
    private static final int HOPPER_SENSOR_ID = 0;
    private static final int SHOOTER_SENSOR_ID = 0;

    private Indexer() {
        master = new HSTalon(RobotMap.CAN_IDS.HOPPER_TALON_ID);
        bagMotor = new HSTalon(RobotMap.CAN_IDS.BAG_TALON_ID);
        initializeSensors();
        setupTalons();
    }

    public void initializeSensors() {
        intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
        hopperSensor = new DigitalInput(HOPPER_SENSOR_ID);
        shooterSensor = new DigitalInput(SHOOTER_SENSOR_ID);
    }
    
    public void setupTalons() {
        master.configFactoryDefault();
        bagMotor.configFactoryDefault();

        //Setup master
        master.setInverted(MASTER_INVERT);

        master.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        master.enableVoltageCompensation(true);
        
        master.setNeutralMode(NeutralMode.Brake);

        master.configForwardSoftLimitEnable(false);
        master.configReverseSoftLimitEnable(false);
        master.overrideLimitSwitchesEnable(false);

        master.configContinuousCurrentLimit(INTAKE_CURRENT_CONTINUOUS);
        master.configPeakCurrentLimit(INTAKE_CURRENT_PEAK);
        master.configPeakCurrentDuration(INTAKE_CURRENT_PEAK_DUR);
        master.enableCurrentLimit(true);

        //Setup bagmotor
        bagMotor.follow(master);
        
        bagMotor.setInverted(BAG_INVERT);

        bagMotor.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        bagMotor.enableVoltageCompensation(true);
        
        bagMotor.setNeutralMode(NeutralMode.Brake);

        bagMotor.configForwardSoftLimitEnable(false);
        bagMotor.configReverseSoftLimitEnable(false);
        bagMotor.overrideLimitSwitchesEnable(false);

        bagMotor.configContinuousCurrentLimit(INTAKE_CURRENT_CONTINUOUS);
        bagMotor.configPeakCurrentLimit(INTAKE_CURRENT_PEAK);
        bagMotor.configPeakCurrentDuration(INTAKE_CURRENT_PEAK_DUR);
        bagMotor.enableCurrentLimit(true);
    
        bagMotor.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        bagMotor.enableVoltageCompensation(true);
    }

    public void spinIndexer(double percentOutput) {
        master.set(ControlMode.PercentOutput, percentOutput * OUTPUT_MULTIPLIER);
    }

    public DigitalInput getShooterSensor() {
        return shooterSensor;
    }

    public DigitalInput getIntakeSensor() {
        return intakeSensor;
    }

    public DigitalInput getHopperSensor() {
        return hopperSensor;
    }
    public HSTalon getTalon() {
        return master;
    }

    public HSTalon getBagMotor() {
        return bagMotor;
    }

    public static Indexer getInstance() {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }
}