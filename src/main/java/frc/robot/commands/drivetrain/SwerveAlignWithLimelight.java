package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import harkerrobolib.util.MathUtil;
/**
 * Aligns the Robot with the target, using the Limelight's tX to turn towards the target
 * and its thor to get close enough to the target
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * 
 * @since 8/14/19
 */
public class SwerveAlignWithLimelight extends CommandBase {
     
    public static final double PID_CONTROLLER_PERIOD = 0.01; //In seconds

    private static final double OUTPUT_MULTIPLIER = 0.5;

	private static final int TX_VELOCITY_MULTIPLIER = 0;

    private PIDController txController;
    private PIDController thorController;

    public SwerveAlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());

        txController = new PIDController(Drivetrain.TX_kP, Drivetrain.TX_kI, Drivetrain.TX_kD, PID_CONTROLLER_PERIOD);
        thorController = new PIDController(Drivetrain.THOR_kP, Drivetrain.THOR_kI, Drivetrain.THOR_kD, PID_CONTROLLER_PERIOD);
        txController.setSetpoint(Drivetrain.TX_SETPOINT);
        thorController.setSetpoint(Drivetrain.THOR_SETPOINT);
        txController.setTolerance(Drivetrain.TX_ALLOWABLE_ERROR);
        thorController.setTolerance(Drivetrain.THOR_ALLOWABLE_ERROR);
    }
    
    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.setNeutralMode(NeutralMode.Brake));
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.configClosedloopRamp(Drivetrain.DRIVE_RAMP_RATE));
        Drivetrain.getInstance().applyToAllAngle((angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.setNeutralMode(NeutralMode.Brake));
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.configClosedloopRamp(Drivetrain.ANGLE_RAMP_RATE));
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();
    }

    @Override
    public void execute() {
        //double speed = -thorController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_DRIVE_VELOCITY;
        double turn = txController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_ROTATION_VELOCITY;

        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);

        translateX *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        translateY *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;

        turn -= ((Math.cos(Drivetrain.getInstance().getTopLeft().getAngleDegrees()) * Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorVelocity() +
                Math.cos(Drivetrain.getInstance().getTopRight().getAngleDegrees()) * Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorVelocity() +
                Math.cos(Drivetrain.getInstance().getBackLeft().getAngleDegrees()) * Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorVelocity() +
                Math.cos(Drivetrain.getInstance().getBackRight().getAngleDegrees()) * Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorVelocity()) / 4) * TX_VELOCITY_MULTIPLIER;
        
        if (Math.abs(Limelight.getTx()) < Drivetrain.TX_ALLOWABLE_ERROR)
            turn = 0;

        // turn += (Drivetrain.getInstance().getTopLeft().getState().speedMetersPerSecond) * TX_VELOCITY_MULTIPLIER;
    

        // SwerveModuleState tlState = Drivetrain.getInstance().getTopLeft().getState();
        // SwerveModuleState trState = Drivetrain.getInstance().getTopRight().getState();
        // SwerveModuleState blState = Drivetrain.getInstance().getBackLeft().getState();
        // SwerveModuleState brState = Drivetrain.getInstance().getBackRight().getState();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turn, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], 0, false, false);

        SmartDashboard.putNumber("Limelight tx error", txController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        txController.reset();
        thorController.reset();
        Drivetrain.getInstance().stopAllDrive();
        Limelight.setLEDS(true);
    }
}