package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 * @since August 14, 2020
 */
public class SwerveAlignWithLimelight extends CommandBase {
    private static final double OUTPUT_MULTIPLIER = 1;

    private static final int TX_VELOCITY_MULTIPLIER = 0;
    // private static final double OFFSET = 0;

    private PIDController txController;

    private double prevXPos;
    private double prevTime;

    public SwerveAlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());

        txController = new PIDController(Drivetrain.TX_kP, Drivetrain.TX_kI, Drivetrain.TX_kD);

        SmartDashboard.putNumber("TX", Drivetrain.TX_SETPOINT);
    }
    
    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.configClosedloopRamp(Drivetrain.DRIVE_RAMP_RATE));
        
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.configClosedloopRamp(Drivetrain.ANGLE_RAMP_RATE));
        
        Limelight.setCamModeVision();
        Limelight.setLEDS(true);

        prevXPos = Drivetrain.getInstance().getOdometry().getPoseMeters().getTranslation().getX();
        prevTime = System.currentTimeMillis();
    }
    
    @Override
    public void execute() {
        Limelight.setLEDS(true);
        //double speed = -thorController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_DRIVE_VELOCITY;
        txController.setSetpoint(SmartDashboard.getNumber("TX", Drivetrain.TX_SETPOINT));
        SmartDashboard.putNumber("TX setpoint", txController.getSetpoint());
        // SmartDashboard.putNumber("TX error", Limelight.getTx());

        double turn = txController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_ROTATION_VELOCITY;

        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        double turnManual = -3 * MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        
        translateX *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        translateY *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;

        // turn -= ((Math.cos(Math.toRadians(Drivetrain.getInstance().getTopLeft().getAngleDegrees())) * Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorVelocity() +
        //         Math.cos(Math.toRadians(Drivetrain.getInstance().getTopRight().getAngleDegrees())) * Drivetrain.getInstance().getTopRight().getDriveMotor().getSelectedSensorVelocity() +
        //         Math.cos(Math.toRadians(Drivetrain.getInstance().getBackLeft().getAngleDegrees())) * Drivetrain.getInstance().getBackLeft().getDriveMotor().getSelectedSensorVelocity() +
        //         Math.cos(Math.toRadians(Drivetrain.getInstance().getBackRight().getAngleDegrees())) * Drivetrain.getInstance().getBackRight().getDriveMotor().getSelectedSensorVelocity()) / 4) * TX_VELOCITY_MULTIPLIER;
        
        double xPosition = Drivetrain.getInstance().getOdometry().getPoseMeters().getTranslation().getX();
        double curTime = System.currentTimeMillis();

        double xVel = (xPosition - prevXPos) / ((curTime - prevTime) / 1000); //in m/s

        prevTime = curTime;
        prevXPos = xPosition;

        turn -= xVel * TX_VELOCITY_MULTIPLIER;
        // turn -= OFFSET;
        
        if (Math.abs(Limelight.getTx()) < Drivetrain.TX_ALLOWABLE_ERROR)
            turn = 0;

        // turn += (Drivetrain.getInstance().getTopLeft().getState().speedMetersPerSecond) * TX_VELOCITY_MULTIPLIER;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, Limelight.isTargetVisible() ? turn : turnManual, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], false, false);
    }

    @Override
    public void end(boolean interrupted) {
        txController.reset();
        Drivetrain.getInstance().stopAllDrive();
        Limelight.setLEDS(false);
    }
}