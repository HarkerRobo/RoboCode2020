package frc.robot.commands.drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Copy of SwerveAlignWithLimelight; only change is addition of an isFinished method
 * 
 * @author Ada Praun-Petrovic
 * @author Shahzeb Lakhani
 */
public class SwerveAutonAlignWithLimelight extends CommandBase {
    private PIDController txController;

    public SwerveAutonAlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());

        txController = new PIDController(Drivetrain.TX_kP, Drivetrain.TX_kI, Drivetrain.TX_kD);
        txController.setSetpoint(Drivetrain.TX_SETPOINT);
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.configClosedloopRamp(Drivetrain.DRIVE_RAMP_RATE));

        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.configClosedloopRamp(Drivetrain.ANGLE_RAMP_RATE));

        Limelight.setCamModeVision();
        Limelight.setLEDS(true);
    }

    @Override
    public void execute() {
        double turn = txController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_ROTATION_VELOCITY;

        //No translation
        double translateX = 0;
        double translateY = 0; 

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turn, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], false, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Limelight.getTx()) < Drivetrain.TX_ALLOWABLE_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        txController.reset();
        Drivetrain.getInstance().stopAllDrive();

        Limelight.setLEDS(true);
    }
}