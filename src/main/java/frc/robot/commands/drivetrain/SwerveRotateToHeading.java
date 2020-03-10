package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveRotateToHeading extends CommandBase {
    private double heading;
    private static final double ALLOWABLE_ERROR = 1;
    private double turnMagnitude;
    private PIDController headingController;
    
    public SwerveRotateToHeading(double heading) {
        this.heading = heading;
        headingController = new PIDController(Drivetrain.HEADING_KP, Drivetrain.HEADING_KI, Drivetrain.HEADING_KD);
    }

    @Override
    public void initialize() {
        while (Drivetrain.getInstance().getPigeon().getFusedHeading() - heading > 180) {
            heading += 360;
        }

        while (Drivetrain.getInstance().getPigeon().getFusedHeading() - heading <- 180) {
            heading -= 360;
        }
        turnMagnitude = 0;
    }

    @Override
    public void execute() {
        turnMagnitude = headingController.calculate(Drivetrain.getInstance().getPigeon().getFusedHeading(), heading);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, turnMagnitude, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], false, false);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(Drivetrain.getInstance().getPigeon().getFusedHeading() - heading) <= ALLOWABLE_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().stopAllDrive();
        SwerveManualHeadingControl.joystickFlag = false;
        SwerveManualHeadingControl.headingFlag = false;
    }

}