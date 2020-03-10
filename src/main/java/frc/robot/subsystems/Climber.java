package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the climber on the robot.
 * 
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @since January 18, 2020
 */
public class Climber extends SubsystemBase {
    private static Climber instance;
    
    private static TalonFX master;
    private static TalonFX follower; 

    private static final boolean MASTER_SENSOR_PHASE;
    private static final boolean FOLLOWER_SENSOR_PHASE;

    private static final TalonFXInvertType MASTER_INVERTED;
    private static final TalonFXInvertType FOLLOWER_INVERTED;

    public static final int CLIMBER_POSITION_SLOT = 0;
    private static final double CLIMBER_POSITION_KP;
    private static final double CLIMBER_POSITION_KI;
    private static final double CLIMBER_POSITION_KD;
    private static final double CLIMBER_RAMP_RATE = 0.1;
    private static final int VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 60;
    private static final int CURRENT_PEAK = 70;
    private static final int CURRENT_PEAK_DURATION = 100;
    
    public static final int MAX_POSITION = 15800;
    public static final int MIN_POSITION = 200;
    private static final int FORWARD_SOFT_LIMIT = 300000;
    private static final int REVERSE_SOFT_LIMIT = 0;

    private static boolean isSoftLimiting; 

    static {
        if (RobotMap.IS_COMP) {
            MASTER_SENSOR_PHASE = true;
            FOLLOWER_SENSOR_PHASE = false;

            MASTER_INVERTED = TalonFXInvertType.CounterClockwise;
            FOLLOWER_INVERTED = TalonFXInvertType.Clockwise;

            CLIMBER_POSITION_KP = 0.0; // tune
            CLIMBER_POSITION_KI = 0.0; // tune
            CLIMBER_POSITION_KD = 0.0; // tune
        } else {
            MASTER_SENSOR_PHASE = false;
            FOLLOWER_SENSOR_PHASE = false;
            
            MASTER_INVERTED = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERTED = TalonFXInvertType.Clockwise;

            CLIMBER_POSITION_KP = 0.0; // tune
            CLIMBER_POSITION_KI = 0.0; // tune
            CLIMBER_POSITION_KD = 0.0; // tune
        }
    }

    private Climber() {
        master = new TalonFX(RobotMap.CAN_IDS.CLIMBER_MASTER_ID); 
        follower = new TalonFX(RobotMap.CAN_IDS.CLIMBER_FOLLOWER_ID);
        
        setupTalons();
        isSoftLimiting = true;
        setupPositionPID();
    }
    
    private void setupTalons() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, RobotMap.DEFAULT_TIMEOUT);

        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        master.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
        master.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitEnable(true);
        master.overrideLimitSwitchesEnable(false);
        
        follower.configForwardSoftLimitEnable(false);
        follower.configReverseSoftLimitEnable(false);
        follower.overrideLimitSwitchesEnable(false);

        master.setSelectedSensorPosition(0);

        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        master.setSensorPhase(MASTER_SENSOR_PHASE);
        
        follower.setInverted(FOLLOWER_INVERTED);
        follower.setSensorPhase(FOLLOWER_SENSOR_PHASE);

        master.configVoltageCompSaturation(VOLTAGE_COMP);
        follower.configVoltageCompSaturation(VOLTAGE_COMP);

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

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("climber pos", master.getSelectedSensorPosition());
        // SmartDashboard.putNumber("climber current", master.getOutputCurrent());
    }

    public void toggleSoftLimits() {
        isSoftLimiting = !isSoftLimiting;

        master.configForwardSoftLimitEnable(isSoftLimiting);
        master.configReverseSoftLimitEnable(isSoftLimiting);
    }

    public TalonFX getMaster() {
        return master;
    }

    public TalonFX getFollower() {
        return follower;
    }

    public static Climber getInstance() {
        if (instance == null) 
            instance = new Climber();
        return instance;
    }
}