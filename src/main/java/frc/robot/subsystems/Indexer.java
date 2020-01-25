package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

/**
 * The indexer stores and transfers power cells from the intake to the shooter. It consists
 * of a single motor that spins a belt. 
 * 
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Aimee Wang
 * @since January 23, 2020
 */
public class Indexer implements Subsystem {
    static {
        if(RobotMap.IS_PRACTICE) {
            MASTER_INVERT = false; //Change accordingly
            FOLLOWER_INVERT = false;
        } else {
            MASTER_INVERT = false; //Change accordingly
            FOLLOWER_INVERT = false;
        }
    } 
    
    private static Indexer instance;
    
    private static boolean MASTER_INVERT;
    private static boolean FOLLOWER_INVERT;
    
    private HSTalon master;
    private HSTalon follower;
    
    public static final double OUTPUT_MULTIPLIER = 0.5;

    public static final int VOLTAGE_COMPENSATION = 10;

    private static final int INTAKE_CURRENT_PEAK_DUR = 100;
    private static final int INTAKE_CURRENT_CONTINUOUS = 40;
    private static final int INTAKE_CURRENT_PEAK = 50;

    private DigitalInput intakeSensor;  // The first sensor when a ball is intaked
    private DigitalInput indexerSensor;  // The second sensor in the indexer
    private DigitalInput shooterSensor; // Determines if the indexer is full
    
    private static final int INTAKE_SENSOR_ID = 0;
    private static final int INDEXER_SENSOR_ID = 0;
    private static final int SHOOTER_SENSOR_ID = 0;

    private Solenoid solenoid;

    // public int numPowerCells = 0;

    private Indexer() {
        master = new HSTalon(RobotMap.CAN_IDS.INDEXER_TALON_ID);
        follower = new HSTalon(RobotMap.CAN_IDS.INDEXER_FOLLOWER_TALON_ID);
        initializeSensors();
        setupTalons();
        // numPowerCells = 0;
        solenoid = new Solenoid(RobotMap.CAN_IDS.INDEXER_SOLENOID);
    }

    public void initializeSensors() {
        intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
        indexerSensor = new DigitalInput(INDEXER_SENSOR_ID);
        shooterSensor = new DigitalInput(SHOOTER_SENSOR_ID);
    }
    
    public void setupTalons() {
        master.configFactoryDefault();
        follower.configFactoryDefault();
        
        follower.follow(master);
        follower.setInverted(FOLLOWER_INVERT);
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
    }

    public void spinIndexer(double percentOutput) {
        if(percentOutput == 0)
            master.set(ControlMode.Disabled, 0);
        else
            master.set(ControlMode.PercentOutput, percentOutput * OUTPUT_MULTIPLIER);
    }

    public Solenoid getSolenoid() {
        return solenoid;
    }

    public DigitalInput getShooterSensor() {
        return shooterSensor;
    }

    public DigitalInput getIntakeSensor() {
        return intakeSensor;
    }

    public DigitalInput getHopperSensor() {
        return indexerSensor;
    }

    public HSTalon getMaster() {
        return master;
    }

    public HSTalon getFollower() {
        return follower;
    }

    public static Indexer getInstance() {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }
}