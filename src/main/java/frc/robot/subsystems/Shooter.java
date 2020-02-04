package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Represents the Shooter Subsystem, controlled by two Falcon 500s.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani 
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * @author Aimee Wang
 * @author Rohan Bhowmik
 * @since 1/22/20
 */
public class Shooter implements Subsystem {
    static {
        if(RobotMap.IS_PRACTICE) {
            FLYWHEEL_KF = 0.003; // tune;
            FLYWHEEL_KP = 0.01; // tune;

            SHOOTER_HIGH_ANGLE = Value.kForward;
            SHOOTER_LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERT = TalonFXInvertType.Clockwise;

            SENSOR_PHASE = false;
        } else {
            FLYWHEEL_KF = 0.058;
            FLYWHEEL_KP = 0.1; //2

            SHOOTER_HIGH_ANGLE = Value.kForward;
            SHOOTER_LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.CounterClockwise;
            FOLLOWER_INVERT = TalonFXInvertType.Clockwise;

            SENSOR_PHASE = false;
        }
    }

    //Instance variables
    private static Shooter instance;
    
    private static TalonFX flywheelMaster;
    private static TalonFX flywheelFollower;

    private static boolean SENSOR_PHASE;
    
    public static final double MAX_VELOCITY = 114.3; //127 //Other value : 148 // TODO: Ask Aditi for a better value
    public static final int FLYWHEEL_VELOCITY_SLOT = 0;
    
    private static double FLYWHEEL_KF;
    private static double FLYWHEEL_KP;
    
    private static DoubleSolenoid solenoid;
    
    public static DoubleSolenoid.Value SHOOTER_HIGH_ANGLE; 
    public static DoubleSolenoid.Value SHOOTER_LOW_ANGLE; 
    
    public static final int FLYWHEEL_VOLTAGE_COMP = 10;
    
    private static final int FLYWHEEL_CURRENT_CONTINUOUS = 50;
    private static final int FLYWHEEL_CURRENT_PEAK = 60;
    private static final int FLYWHEEL_CURRENT_PEAK_DUR = 50;

    private static final double VOLTAGE_COMPENSATION = 0;

    public static TalonFXInvertType MASTER_INVERT;
    public static TalonFXInvertType FOLLOWER_INVERT;

    public static final double GEAR_RATIO = 0.675;

    public static final double WHEEL_DIAMETER = 4;

    public static final int TICKS_PER_REV = 2048;
    /**
     * Constructs a Shooter.
     */
    public Shooter() {
        flywheelMaster = new TalonFX(RobotMap.CAN_IDS.SHOOTER_MASTER_ID);
        flywheelFollower = new TalonFX(RobotMap.CAN_IDS.SHOOTER_FOLLOWER_ID);
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.SHOOTER_SOLENOID_FORWARD, RobotMap.CAN_IDS.SHOOTER_BACKWARD);
        
        setupFlywheel();
    }


    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter % error", flywheelMaster.getClosedLoopError() / (1flywheelMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter error", flywheelMaster.getClosedLoopError());
        SmartDashboard.putNumber("Shooter current", flywheelMaster.getStatorCurrent());
        SmartDashboard.putNumber("Shooter % output", flywheelMaster.getMotorOutputPercent());   
    }
    /**
     * Sets up the master and follower talons.
     */
    public void setupFlywheel() {
        flywheelMaster.configFactoryDefault();
        flywheelFollower.configFactoryDefault();
        
        flywheelFollower.follow(flywheelMaster);
        
        flywheelMaster.setInverted(MASTER_INVERT);
        flywheelFollower.setInverted(FOLLOWER_INVERT);

        flywheelMaster.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        flywheelMaster.enableVoltageCompensation(true);
        
        flywheelMaster.setNeutralMode(NeutralMode.Coast);
      
        flywheelMaster.setSensorPhase(SENSOR_PHASE);

        flywheelMaster.configForwardSoftLimitEnable(false);
        flywheelMaster.configReverseSoftLimitEnable(false);
        flywheelMaster.overrideLimitSwitchesEnable(false);

        flywheelMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, FLYWHEEL_CURRENT_CONTINUOUS, FLYWHEEL_CURRENT_PEAK, FLYWHEEL_CURRENT_PEAK_DUR));
    
        setupVelocityPID();
    }

    /**
     * Set up for Velocity PID.
     */
    public void setupVelocityPID() {
        flywheelMaster.config_kP(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KP);
        flywheelMaster.config_kF(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KF);
    }

    /**
     * Sets the hood angle to opposite of its current position. If it
     * is currently at high angle then it switches it to low, if it is
     * at low it switches the position to high.
     */
    public void toggleAngle() {
        solenoid.set(solenoid.get() == SHOOTER_HIGH_ANGLE ? SHOOTER_LOW_ANGLE : SHOOTER_HIGH_ANGLE);
    }
    
    /**
     * Spins the shooter flywheel at a certain percent output
     */
    public void spinShooterPercentOutput(double percentOutput) {
        flywheelMaster.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * Spins the shooter flywheel at a certain velocity (in feet/second)
     */
    public void spinShooterVelocity(double velocity) {
        double velocityInTicks = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, velocity * GEAR_RATIO, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, TICKS_PER_REV);
        flywheelMaster.set(ControlMode.Velocity, velocityInTicks);
    }

    /**
     * Gets the master falcon.
     * @return the master falcon
     */
    public TalonFX getMaster() {
        return flywheelMaster;
    }

    /**
     * Gets the follower falcon.
     * @return the follower falcon
     */
    public TalonFX getFollower() {
        return flywheelFollower;
    }

    /**
     * Gets the angle solenoid.
     * @return the angle solenoid.
     */
    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    /**
     * Returns an instance of this Shooter object.
     * @return an instance of this Shooter object.
     */
    public static Shooter getInstance() {
        if(instance == null)
            instance = new Shooter();
        return instance;
    }
}