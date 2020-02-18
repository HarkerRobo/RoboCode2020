package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

/**
 * The indexer stores and transfers power cells from the intake to the shooter. It consists
 * of a single motor that spins a belt. 
 * 
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Aimee Wang
 * @since January 23, 2020
 */
public class Indexer extends SubsystemBase {
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
    private VictorSPX agitator;
    
    public static final double SPINE_OUTPUT_MULTIPLIER = 1;
    public static final double AGITATOR_OUTPUT_MULTIPLIER = 1;

    public static final int VOLTAGE_COMPENSATION = 10;
    public static final double RAMP_RATE = 0.04;

    private static final int INDEXER_CURRENT_PEAK_DUR = 50;
    private static final int INDEXER_CURRENT_CONTINUOUS = 30;
    private static final int INDEXER_CURRENT_PEAK = 40;

    //private DigitalInput intakeSensor;  // The first sensor when a ball is intaked
    private DigitalInput indexerSensor;  // The second sensor in the indexer
    private DigitalInput shooterSensor; // Determines if the indexer is full
    
    // private static final int INTAKE_SENSOR_ID = 0;
    private static final int INDEXER_SENSOR_ID = 6;
    private static final int SHOOTER_SENSOR_ID = 7;

    public static final double AGITATOR_DEFAULT_OUTPUT = 1;

    public static final double AGITATOR_CYCLE_DUR = 700;//millis

    public static final double AGITATOR_ON_DURATION = 500;//millis

    public static final DoubleSolenoid.Value CLOSED = Value.kForward;
    public static final DoubleSolenoid.Value OPEN = Value.kReverse;

    private DoubleSolenoid solenoid;

    private Indexer() {
        spine = new HSTalon(RobotMap.CAN_IDS.SPINE_TALON_ID);
        agitator = new VictorSPX(RobotMap.CAN_IDS.AGITATOR_TALON_ID);
        initializeSensors();
        setupTalons();
        // numPowerCells = 0;
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.INDEXER_SOLENOID_FORWARD, RobotMap.CAN_IDS.INDEXER_SOLENOID_REVERSE);
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
        spine.configOpenloopRamp(RAMP_RATE);
        spine.enableVoltageCompensation(true);

        agitator.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        agitator.configOpenloopRamp(RAMP_RATE);
        agitator.enableVoltageCompensation(true);
        
        spine.setNeutralMode(NeutralMode.Brake);
        agitator.setNeutralMode(NeutralMode.Brake);

        spine.configForwardSoftLimitEnable(false);
        spine.configReverseSoftLimitEnable(false);
        spine.overrideLimitSwitchesEnable(false);
        
        agitator.configForwardSoftLimitEnable(false);
        agitator.configReverseSoftLimitEnable(false);
        agitator.overrideLimitSwitchesEnable(false);

        spine.configContinuousCurrentLimit(INDEXER_CURRENT_CONTINUOUS);
        spine.configPeakCurrentLimit(INDEXER_CURRENT_PEAK);
        spine.configPeakCurrentDuration(INDEXER_CURRENT_PEAK_DUR);
        spine.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        // boolean indexerDetected = !Indexer.getInstance().getIndexerSensor().get();
        // boolean shooterDetected = !Indexer.getInstance().getShooterSensor().get();

        // if(BottomIntake.getInstance().getTalon().getMotorOutputPercent() > 0) {
        //     if (currentTime % AGITATOR_CYCLE_DUR < AGITATOR_ON_DURATION) {
        //         spinAgitator(AGITATOR_DEFAULT_OUTPUT);
        //     }
        //     spinAgitator(0);
        // }

        // if(!indexerDetected && !shooterDetected) {
        //     spinSpine(SPINE_DEFAULT_OUTPUT);
        //     spinAgitator(AGITATOR_DEFAULT_OUTPUT);
        // }
    }

    public void spinSpine(double percentOutput) {
        if(percentOutput == 0)
            spine.set(ControlMode.Disabled, 0);
        else
            spine.set(ControlMode.PercentOutput, percentOutput * SPINE_OUTPUT_MULTIPLIER);
    }

    public void spinAgitator(double percentOutput) {
        if(percentOutput == 0)
            agitator.set(ControlMode.Disabled, 0);
        else
            agitator.set(ControlMode.PercentOutput, percentOutput * AGITATOR_OUTPUT_MULTIPLIER);
    }

    public DoubleSolenoid getSolenoid() {
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

    public HSTalon getSpine() {
        return spine;
    }

    public VictorSPX getAgitator() {
        return agitator;
    }

    public static Indexer getInstance() {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }

    public void toggleSolenoid() {
        solenoid.set(solenoid.get() == Value.kReverse ? Value.kForward : Value.kReverse);
    }
}