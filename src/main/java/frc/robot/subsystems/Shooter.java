package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.util.Limelight;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Represents the Shooter Subsystem, controlled by two Falcon 500s.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani 
 * @author Arjun Dixit
 * @author Anirudh Kotamraju
 * @author Aimee Wang
 * @author Rohan Bhowmik
 * @author Chirag Kaushik
 * @since January 22, 2020
 */
public class Shooter extends SubsystemBase {
    static {
        if(RobotMap.IS_COMP) {
            FLYWHEEL_KF = 0.064; // tune;
            FLYWHEEL_KP = 0.05; // tune;
            FLYWHEEL_KI = 0.001;
            FLYWHEEL_KD = 0.9;
            FLYWHEEL_IZONE = 150;

            HIGH_ANGLE = Value.kForward;
            LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERT = TalonFXInvertType.CounterClockwise;

            SENSOR_PHASE = false;
        } else {
            FLYWHEEL_KF = 0.064;//0.058;
            FLYWHEEL_KP = 0.05; 
            FLYWHEEL_KI = 0.001;
            FLYWHEEL_KD = 0.7;
            FLYWHEEL_IZONE = 150;

            HIGH_ANGLE = Value.kForward;
            LOW_ANGLE = Value.kReverse;

            MASTER_INVERT = TalonFXInvertType.Clockwise;
            FOLLOWER_INVERT = TalonFXInvertType.CounterClockwise;

            SENSOR_PHASE = false;
        }
    }

    //Instance variables
    private static Shooter instance;
    
    private static TalonFX flywheelMaster;
    private static TalonFX flywheelFollower;

    private static boolean SENSOR_PHASE;
    
    public static final double MAX_VELOCITY = 127;//114.3; 
    public static final int FLYWHEEL_VELOCITY_SLOT = 0;
    
    public static double FLYWHEEL_KF;
    public static double FLYWHEEL_KP;
    public static double FLYWHEEL_KI;
    public static double FLYWHEEL_KD;
    public static int FLYWHEEL_IZONE;
    
    private static DoubleSolenoid solenoid;
    
    public static DoubleSolenoid.Value HIGH_ANGLE; 
    public static DoubleSolenoid.Value LOW_ANGLE; 
    
    private static final int FLYWHEEL_CURRENT_CONTINUOUS = 50;
    private static final int FLYWHEEL_CURRENT_PEAK = 60;
    private static final int FLYWHEEL_CURRENT_PEAK_DUR = 200;

    private static final double VOLTAGE_COMPENSATION = 9.5;

    public static TalonFXInvertType MASTER_INVERT;
    public static TalonFXInvertType FOLLOWER_INVERT;

    public static final double GEAR_RATIO = 0.675;
    public static final double WHEEL_DIAMETER = 4;
    public static final int TICKS_PER_REV = 2048;

    private static final double DAY_FAR_DISTANCE_THRESHOLD = 26.706;    
    private static final double DAY_MEDIUM_DISTANCE_THRESHOLD = 11.753;
    private static final double NIGHT_THRESHOLD = 18.4;  // choosing between far and close pipelines for night

    private static final int CURRENT_DRAW_MIN = 10;
    private static final int STALL_VELOCITY = 100;

    public static boolean isPercentOutput = true;
    
    /**
     * Constructs a Shooter.
     */
    public Shooter() {
        flywheelMaster = new TalonFX(RobotMap.CAN_IDS.SHOOTER_MASTER_ID);
        flywheelFollower = new TalonFX(RobotMap.CAN_IDS.SHOOTER_FOLLOWER_ID);
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.SHOOTER_SOLENOID_FORWARD, RobotMap.CAN_IDS.SHOOTER_SOLENOID_BACKWARD);
        
        setupFlywheel();
        // SmartDashboard.putNumber("flywheel kd", FLYWHEEL_KD);
        // SmartDashboard.putNumber("flywheel kp", FLYWHEEL_KP);
    }

    @Override
    public void periodic() {
        double distance = getLimelightDistance();
        // SmartDashboard.putBoolean("isPercentOutput", isPercentOutput);
        
        if (distance != 0.0) {
            if (RobotMap.IS_NIGHT) {
                if (distance > NIGHT_THRESHOLD)
                    Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR);
                else
                    Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_CLOSE);
            } else {
                if (distance > DAY_FAR_DISTANCE_THRESHOLD) 
                    Limelight.setPipeline(RobotMap.PIPELINES.DAY_FAR);
                else if (distance > DAY_MEDIUM_DISTANCE_THRESHOLD) 
                    Limelight.setPipeline(RobotMap.PIPELINES.DAY_MEDIUM);
                else 
                    Limelight.setPipeline(RobotMap.PIPELINES.DAY_CLOSE);
            }
        } 
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

    public boolean isStalling() {
        return (flywheelMaster.getStatorCurrent() > CURRENT_DRAW_MIN && flywheelMaster.getSelectedSensorVelocity() < STALL_VELOCITY) || 
                (flywheelFollower.getStatorCurrent() > CURRENT_DRAW_MIN && flywheelFollower.getSelectedSensorVelocity() < STALL_VELOCITY);
    }

    /**
     * Set up for Velocity PID.
     */
    public void setupVelocityPID() {
        flywheelMaster.config_kF(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KF);
        flywheelMaster.config_kP(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KP);
        flywheelMaster.config_kI(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KI);
        flywheelMaster.config_kD(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_KD);
        flywheelMaster.config_IntegralZone(FLYWHEEL_VELOCITY_SLOT, FLYWHEEL_IZONE);
    }

    /**
     * Sets the hood angle to opposite of its current position. If it
     * is currently at high angle then it switches it to low, if it is
     * at low it switches the position to high.
     */
    public void toggleHoodAngle() {
        solenoid.set(solenoid.get() == HIGH_ANGLE ? LOW_ANGLE : HIGH_ANGLE);
    }
    
    /**
     * Gets the distance from the limelight to the power port (or 0 if it is not visible)
     */
    public double getLimelightDistance() {
        if(Limelight.isTargetVisible())
            return (SpinShooterLimelight.TARGET_HEIGHT - SpinShooterLimelight.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.getTy() + SpinShooterLimelight.LIMELIGHT_ANGLE));
        else
            return 0.0;
    }

    /**
     * Spins the shooter flywheel at a certain percent output
     */
    public void spinShooterPercentOutput(double percentOutput) {
        if(percentOutput == 0) 
            flywheelMaster.set(ControlMode.Disabled, 0);
        else
            flywheelMaster.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * Spins the shooter flywheel at a certain velocity (in feet/second)
     */
    public void spinShooterVelocity(double velocity) {
        double velocityInTicks = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, velocity * GEAR_RATIO, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, TICKS_PER_REV);
        if(0.95 * velocityInTicks > flywheelMaster.getSelectedSensorVelocity()) {
            flywheelMaster.set(ControlMode.PercentOutput, 1);
            isPercentOutput = true;
        }
        else {
            flywheelMaster.set(ControlMode.Velocity, velocityInTicks);
            isPercentOutput = false;
        }
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