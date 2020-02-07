package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the climber on the robot.
 * 
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @since January 18, 2020
 */
public class Climber extends SubsystemBase {
    private static Climber instance;
    
    private static TalonFX master;
    private static TalonFX follower; 

    private static final boolean LEFT_SENSOR_PHASE;
    private static final boolean RIGHT_SENSOR_PHASE;

    private static final TalonFXInvertType LEFT_INVERTED;
    private static final TalonFXInvertType RIGHT_INVERTED;

    public static final int CLIMBER_POSITION_SLOT = 0;
    private static final double CLIMBER_POSITION_KP;
    private static final double CLIMBER_POSITION_KI;
    private static final double CLIMBER_POSITION_KD;
    private static final double CLIMBER_RAMP_RATE = 0.1;
    private static final int CLIMBER_VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 50;
    private static final int CURRENT_PEAK = 50;
    private static final int CURRENT_PEAK_DURATION = 50;

    static {
        if (RobotMap.IS_PRACTICE) {
            LEFT_SENSOR_PHASE = false;
            RIGHT_SENSOR_PHASE = false;

            LEFT_INVERTED = TalonFXInvertType.Clockwise;
            RIGHT_INVERTED = TalonFXInvertType.Clockwise;

            CLIMBER_POSITION_KP = 0.0; // tune
            CLIMBER_POSITION_KI = 0.0; // tune
            CLIMBER_POSITION_KD = 0.0; // tune
        } else {
            LEFT_SENSOR_PHASE = false;
            RIGHT_SENSOR_PHASE = false;
            
            LEFT_INVERTED = TalonFXInvertType.Clockwise;
            RIGHT_INVERTED = TalonFXInvertType.Clockwise;

            CLIMBER_POSITION_KP = 0.0; // tune
            CLIMBER_POSITION_KI = 0.0; // tune
            CLIMBER_POSITION_KD = 0.0; // tune
        }
    }

    private Climber() {
        master = new TalonFX(RobotMap.CAN_IDS.CLIMBER_MASTER_ID); 
        follower = new TalonFX(RobotMap.CAN_IDS.CLIMBER_FOLLOWER_ID);
        
        setupTalons();
        setupPositionPID();
    }
    
    private void setupTalons() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);
        follower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);

        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        master.configForwardSoftLimitEnable(false);
        master.configReverseSoftLimitEnable(false);
        master.overrideLimitSwitchesEnable(false);
        
        follower.configForwardSoftLimitEnable(false);
        follower.configReverseSoftLimitEnable(false);
        follower.overrideLimitSwitchesEnable(false);

        master.setSelectedSensorPosition(0);
        follower.setSelectedSensorPosition(0);

        master.follow(follower);

        master.setInverted(LEFT_INVERTED);
        master.setSensorPhase(LEFT_SENSOR_PHASE);
        
        follower.setInverted(RIGHT_INVERTED);
        follower.setSensorPhase(RIGHT_SENSOR_PHASE);

        master.configVoltageCompSaturation(CLIMBER_VOLTAGE_COMP);
        follower.configVoltageCompSaturation(CLIMBER_VOLTAGE_COMP);
        master.enableVoltageCompensation(true);
        follower.enableVoltageCompensation(true);

        master.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DURATION));
        follower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DURATION));
    }
    
    private void setupPositionPID() {
        master.configClosedloopRamp(CLIMBER_RAMP_RATE);
        master.config_kP(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KP);
        master.config_kI(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KI);
        master.config_kD(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KD);
    }

    public TalonFX getMaster() {
        return master;
    }

    public TalonFX getFollower() {
        return follower;
    }

    public static Climber getInstance() {
        if (instance == null) 
            return new Climber();
        return instance;
    }
}