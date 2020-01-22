package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

public class Shooter implements Subsystem {

    static {
        if(RobotMap.IS_PRACTICE) {
            FLYWHEEL_KP = 0.0; // tune;
            FLYWHEEL_KF = 0.0; // tune;

            SHOOTER_HIGH_ANGLE = Value.kForward;
            SHOOTER_LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERT = TalonFXInvertType.FollowMaster;
        } else {
            FLYWHEEL_KP = 0.0; // tune;
            FLYWHEEL_KF = 0.0; // tune;

            SHOOTER_HIGH_ANGLE = Value.kForward;
            SHOOTER_LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERT = TalonFXInvertType.FollowMaster;
        }
    }

    private static Shooter instance;

    private static TalonFX flywheelMaster;
    private static TalonFX flywheelFollower;

    private static final int FLYWHEEL_VELOCITY_SLOT = 0;

    private static double FLYWHEEL_KF;
    private static double FLYWHEEL_KP;

    private static DoubleSolenoid hoodSolenoid;

    public static DoubleSolenoid.Value SHOOTER_HIGH_ANGLE; 
    public static DoubleSolenoid.Value SHOOTER_LOW_ANGLE; 

    public static final int FLYWHEEL_VOLTAGE_COMP = 10;
    
    private static final int CURRENT_CONTINUOUS = 50;
    private static final int CURRENT_PEAK = 70;
    private static final int CURRENT_PEAK_DUR = 50;
    
    public static TalonFXInvertType MASTER_INVERT;
    public static TalonFXInvertType FOLLOWER_INVERT;
    
    public Shooter() {
        flywheelMaster = new TalonFX(RobotMap.CAN_IDS.SHOOTER_MASTER_ID);
        flywheelFollower = new TalonFX(RobotMap.CAN_IDS.SHOOTER_FOLLOWER_ID);
        hoodSolenoid = new DoubleSolenoid(RobotMap.CAN_IDS.SHOOTER_SOLENOID_FORWARD, RobotMap.CAN_IDS.SHOOTER_SOLENOID_REVERSE);

        setupFlywheel();
    }

    public void setupFlywheel() {
        flywheelFollower.follow(flywheelMaster);

        flywheelMaster.setInverted(MASTER_INVERT);
        flywheelFollower.setInverted(FOLLOWER_INVERT);

        flywheelMaster.setNeutralMode(NeutralMode.Coast);

        flywheelMaster.selectProfileSlot(FLYWHEEL_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX);

        flywheelMaster.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR));

        flywheelMaster.configVoltageCompSaturation(FLYWHEEL_VOLTAGE_COMP);
        flywheelMaster.enableVoltageCompensation(true);

        flywheelMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);

        setupVelocityPID();
    }

    public void setupVelocityPID() {
        flywheelMaster.config_kP(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KP);
        flywheelMaster.config_kF(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KF);
    }

    public static void toggleAngle() {
        hoodSolenoid.set(hoodSolenoid.get() == SHOOTER_HIGH_ANGLE ? SHOOTER_LOW_ANGLE : SHOOTER_HIGH_ANGLE);
    }

    public static void spinShooter(double magnitude) {
        flywheelMaster.set(ControlMode.Velocity, magnitude);
    }

    public TalonFX getMaster() {
        return flywheelMaster;
    }

    public TalonFX getFollower() {
        return flywheelFollower;
    }

    public DoubleSolenoid getAngleSolenoid() {
        return hoodSolenoid;
    }

    public static Shooter getInstance() {
        if(instance == null)
            instance = new Shooter();
        return instance;
    }
}