package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
 * @author Angela Jia
 * @author Aimee Wang
 * @since January 23, 2020
 */
public class Indexer implements Subsystem {
    static {
        if(RobotMap.IS_PRACTICE) {
            SPINE_INVERT = false; //Change accordingly
            AGITATOR_INVERT = false;
        } else {
            SPINE_INVERT = false; //Change accordingly
            AGITATOR_INVERT = false;
        }
    } 
    
    private static Indexer instance;
    
    private static boolean SPINE_INVERT;
    private static boolean AGITATOR_INVERT;
    
    private HSTalon spine;
    private HSTalon agitator;
    
    public static final double OUTPUT_MULTIPLIER = 0.5;

    public static final int VOLTAGE_COMPENSATION = 10;

    private static final int INTAKE_CURRENT_PEAK_DUR = 100;
    private static final int INTAKE_CURRENT_CONTINUOUS = 40;
    private static final int INTAKE_CURRENT_PEAK = 50;

    //private DigitalInput intakeSensor;  // The first sensor when a ball is intaked
    private DigitalInput indexerSensor;  // The second sensor in the indexer
    private DigitalInput shooterSensor; // Determines if the indexer is full
    
    // private static final int INTAKE_SENSOR_ID = 0;
    private static final int INDEXER_SENSOR_ID = 0;
    private static final int SHOOTER_SENSOR_ID = 0;

    private static final double SPINE_DEFAULT_OUTPUT = 0;

    private static final double AGITATOR_DEFAULT_OUTPUT = 0;

    private Solenoid solenoid;

    // public int numPowerCells = 0;

    private Indexer() {
        spine = new HSTalon(RobotMap.CAN_IDS.SPINE_TALON_ID);
        agitator = new HSTalon(RobotMap.CAN_IDS.AGITATOR_TALON_ID);
        initializeSensors();
        setupTalons();
        // numPowerCells = 0;
        solenoid = new Solenoid(RobotMap.CAN_IDS.INDEXER_SOLENOID);
    }

    public void initializeSensors() {
        //intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
        indexerSensor = new DigitalInput(INDEXER_SENSOR_ID);
        shooterSensor = new DigitalInput(SHOOTER_SENSOR_ID);
    }
    
    public void setupTalons() {
        spine.configFactoryDefault();
        agitator.configFactoryDefault();
        
        agitator.setInverted(AGITATOR_INVERT);
        spine.setInverted(SPINE_INVERT);

        spine.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        spine.enableVoltageCompensation(true);
        agitator.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        agitator.enableVoltageCompensation(true);
        
        spine.setNeutralMode(NeutralMode.Brake);
        agitator.setNeutralMode(NeutralMode.Brake);

        spine.configForwardSoftLimitEnable(false);
        spine.configReverseSoftLimitEnable(false);
        spine.overrideLimitSwitchesEnable(false);
        agitator.configForwardSoftLimitEnable(false);
        agitator.configReverseSoftLimitEnable(false);
        agitator.overrideLimitSwitchesEnable(false);

        spine.configContinuousCurrentLimit(INTAKE_CURRENT_CONTINUOUS);
        spine.configPeakCurrentLimit(INTAKE_CURRENT_PEAK);
        spine.configPeakCurrentDuration(INTAKE_CURRENT_PEAK_DUR);
        spine.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        boolean indexerDetected = !Indexer.getInstance().getIndexerSensor().get();
        boolean shooterDetected = !Indexer.getInstance().getShooterSensor().get();

        if(BottomIntake.getInstance().getTalon().getStatorCurrent() > 0)
            spinAgitator(AGITATOR_DEFAULT_OUTPUT);

        if(!indexerDetected && !shooterDetected) {
            spinSpine(SPINE_DEFAULT_OUTPUT);
            spinAgitator(AGITATOR_DEFAULT_OUTPUT);
        }
    }

    public void spinSpine(double percentOutput) {
        if(percentOutput == 0)
            spine.set(ControlMode.Disabled, 0);
        else
            spine.set(ControlMode.PercentOutput, percentOutput * OUTPUT_MULTIPLIER);
    }

    public void spinAgitator(double percentOutput) {
        if(percentOutput == 0)
            agitator.set(ControlMode.Disabled, 0);
        else
            agitator.set(ControlMode.PercentOutput, percentOutput * OUTPUT_MULTIPLIER);
    }

    public Solenoid getSolenoid() {
        return solenoid;
    }

    public DigitalInput getShooterSensor() {
        return shooterSensor;
    }

    // public DigitalInput getIntakeSensor() {
    //     return intakeSensor;
    // }

    public DigitalInput getIndexerSensor() {
        return indexerSensor;
    }

    public HSTalon getMaster() {
        return spine;
    }

    public HSTalon getFollower() {
        return agitator;
    }

    public static Indexer getInstance() {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }
}