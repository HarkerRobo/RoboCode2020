package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSTalon;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * A swerve module on the drivetrain.
 * A swerve module contains one motor to spin the wheel and another to change the wheel's angle.
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @since 11/1/19
 */
public class SwerveModule {
    public static final int ENCODER_TICKS = 4096;

    //Voltage/Current Constants
    private static final double VOLTAGE_COMP = 10;

    private static final int DRIVE_CURRENT_CONTINUOUS = 40;
    private static final int DRIVE_CURRENT_PEAK = 60;
    private static final int ANGLE_CURRENT_CONTINUOUS = 15;
    private static final int ANGLE_CURRENT_PEAK = 15;
    private static final int CURRENT_PEAK_DUR = 20;

    // Motor inversions
    private final TalonFXInvertType DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;
    
    private TalonFX driveMotor;
    private HSTalon angleMotor;

    public SwerveModule(int driveId, TalonFXInvertType invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        driveMotor = new TalonFX(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;
        
        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;
        
        driveFalconInit(driveMotor);
        angleTalonInit(angleMotor);
    }
    
    public void driveFalconInit(TalonFX falcon) {
        falcon.configFactoryDefault();

        falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PRIMARY_INDEX, 10);

        falcon.setNeutralMode(NeutralMode.Brake);

        falcon.setInverted(DRIVE_INVERTED);
        falcon.setSensorPhase(DRIVE_SENSOR_PHASE);
        
        falcon.configForwardSoftLimitEnable(false);
        falcon.configReverseSoftLimitEnable(false);
        falcon.overrideLimitSwitchesEnable(false);

        falcon.setSelectedSensorPosition(0);
        
        falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DRIVE_CURRENT_CONTINUOUS, DRIVE_CURRENT_PEAK, CURRENT_PEAK_DUR));

        falcon.configVoltageCompSaturation(VOLTAGE_COMP);
        falcon.enableVoltageCompensation(true);
    }

    public void angleTalonInit(HSTalon talon) {
        talon.configFactoryDefault();

        talon.setNeutralMode(NeutralMode.Brake);

        talon.setInverted(ANGLE_INVERTED);
        talon.setSensorPhase(ANGLE_SENSOR_PHASE);

        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        talon.configContinuousCurrentLimit(ANGLE_CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(ANGLE_CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
        
        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
    }
 
    /**
     * Sets the drive output of the swerve module in either percent output or velocity in feet per second.
     * 
     * @param output the output of the swerve module
     * @param isPercentOutput true if the output is in percent output, false if it is in feet per second.
     */
    public void setDriveOutput(double output, boolean isPercentOutput) {
        if(isPercentOutput) {
            driveMotor.set(TalonFXControlMode.PercentOutput, output);
        } else {
            driveMotor.set(TalonFXControlMode.Velocity, Conversions.convert(SpeedUnit.FEET_PER_SECOND, output * Drivetrain.FEET_PER_METER, SpeedUnit.ENCODER_UNITS) * Drivetrain.GEAR_RATIO);
        }
    }
    
    public void setAngleAndDriveVelocity(double targetAngle, double output, boolean isPercentOutput, boolean isMotionProfile) {
        boolean shouldReverse = !isMotionProfile && Math.abs(targetAngle - getAngleDegrees()) > 90;
        
        if (shouldReverse) {
            setDriveOutput(-output, isPercentOutput);
            if (targetAngle - getAngleDegrees() > 90) {
                targetAngle -= 180;
            }
            else {
                targetAngle += 180;
            }
        } else {
            setDriveOutput(output, isPercentOutput);
        }
        
        int targetPos = (int)((targetAngle / 360) * 4096);

        angleMotor.set(ControlMode.Position, targetPos);
    }

    // public void setAngleAndDrivePosition(double targetAngle, double position, double feedForward) {
    //     boolean shouldReverse = Math.abs(targetAngle - getAngleDegrees()) > 90;
        
    //     if (shouldReverse) {
    //         position = driveMotor.getSelectedSensorPosition() - Math.signum(position) * (position - driveMotor.getSelectedSensorPosition());
    //         if (targetAngle - getAngleDegrees() > 90) {
    //             targetAngle -= 180;
    //         }
    //         else {
    //             targetAngle += 180;
    //         }
    //     }
        
    //     int targetPos = (int)((targetAngle / 360) * 4096);

    //     angleMotor.set(ControlMode.Position, targetPos);
    //     driveMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
    // }

    /**
     * Returns the current angle in degrees
     */
    public double getAngleDegrees() {
        return angleMotor.getSelectedSensorPosition() * 360.0 / SwerveModule.ENCODER_TICKS; //Convert encoder ticks to degrees
    }

    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

     /**
      * Returns the current state of the module.
      *
      * @return The current state of the module.
      */
    public SwerveModuleState getState() {
        // System.out.println(angleMotor.getSelectedSensorPosition() * 360 / 4096 % 360);
        return new SwerveModuleState(Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, driveMotor.getSelectedSensorVelocity() / Drivetrain.GEAR_RATIO, SpeedUnit.FEET_PER_SECOND) * Drivetrain.METERS_PER_FOOT, 
            Rotation2d.fromDegrees(angleMotor.getSelectedSensorPosition() * 360 / 4096));
    }
}