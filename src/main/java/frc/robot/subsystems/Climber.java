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
    
    private static TalonFX leftClimber;
    private static TalonFX rightClimber;

    private static final boolean LEFT_SENSOR_PHASE;
    private static final boolean RIGHT_SENSOR_PHASE;

    private static final TalonFXInvertType LEFT_INVERTED;
    private static final TalonFXInvertType RIGHT_INVERTED;

    private static final int CLIMBER_POSITION_SLOT = 0;
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
        leftClimber = new TalonFX(RobotMap.CAN_IDS.LEFT_CLIMBER_ID); 
        rightClimber = new TalonFX(RobotMap.CAN_IDS.RIGHT_CLIMBER_ID);

        setupTalons();
        setupPositionPID();
    }
    
    private void setupTalons() {
        leftClimber.configFactoryDefault();
        rightClimber.configFactoryDefault();

        leftClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);
        rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);

        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);

        leftClimber.configForwardSoftLimitEnable(false);
        leftClimber.configReverseSoftLimitEnable(false);
        leftClimber.overrideLimitSwitchesEnable(false);
        
        rightClimber.configForwardSoftLimitEnable(false);
        rightClimber.configReverseSoftLimitEnable(false);
        rightClimber.overrideLimitSwitchesEnable(false);

        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);

        leftClimber.follow(rightClimber);

        leftClimber.setInverted(LEFT_INVERTED);
        leftClimber.setSensorPhase(LEFT_SENSOR_PHASE);
        
        rightClimber.setInverted(RIGHT_INVERTED);
        rightClimber.setSensorPhase(RIGHT_SENSOR_PHASE);

        leftClimber.configVoltageCompSaturation(CLIMBER_VOLTAGE_COMP);
        rightClimber.configVoltageCompSaturation(CLIMBER_VOLTAGE_COMP);
        leftClimber.enableVoltageCompensation(true);
        rightClimber.enableVoltageCompensation(true);

        leftClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DURATION));
        rightClimber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DURATION));

    }
    
    private void setupPositionPID() {
        leftClimber.selectProfileSlot(CLIMBER_POSITION_SLOT, RobotMap.PRIMARY_INDEX);
        leftClimber.config_kP(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KP);
        leftClimber.config_kI(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KI);
        leftClimber.config_kD(CLIMBER_POSITION_SLOT, CLIMBER_POSITION_KD);
    }

    public TalonFX getLeftClimber() {
        return leftClimber;
    }

    public TalonFX getRightClimber() {
        return rightClimber;
    }
    
    public static Climber getInstance() {
        if (instance == null) {
            return new Climber();
        }
        return instance;
    }

}