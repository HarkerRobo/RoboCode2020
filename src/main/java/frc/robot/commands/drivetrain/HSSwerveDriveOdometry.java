/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an array. The desired wheel and module
 * rotation velocities should be taken from those and used in velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes to the angle given in the final state of the trajectory.
 */

@SuppressWarnings("MemberName")
public class HSSwerveDriveOdometry extends CommandBase {
    private final Timer m_timer = new Timer();
    private Pose2d m_finalPose;

    private final Trajectory m_trajectory;
    private final Rotation2d heading;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;

    /**
     * Direct copy of WPILib's SwerveControllerCommand, except this command takes in a Rotation2d object
     * that represents the gyroscope heading for the robot to hold at all points in the profile.
     * In other words, the drivetrain continuously faces in the direction of the desired heading and ignores
     * the Trajectory's heading.
     * 
     * Constructs a new SwerveControllerCommand that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * raw module states from the position controllers which need to be put into a
     * velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path- this is left to the user, since it is not appropriate for paths
     * with nonstationary endstates.
     *
     * <p>
     * Note 2: The rotation controller will calculate the rotation based on the
     * final pose in the trajectory, not the poses at each time step.
     *
     * @param trajectory         The trajectory to follow.
     * @param heading            The heading for the swerve robot to have throughout
     *                           the entire path
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param thetaController    The Trajectory Tracker PID controller for angle for
     *                           the robot.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public HSSwerveDriveOdometry(Trajectory trajectory, Rotation2d heading, Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
            ProfiledPIDController thetaController,

            Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        this.heading = heading;
        m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

        m_xController = requireNonNullParam(xController, "xController", "SwerveControllerCommand");
        m_yController = requireNonNullParam(yController, "xController", "SwerveControllerCommand");
        m_thetaController = requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand");

        m_outputModuleStates = requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveControllerCommand");
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        // Sample final pose to get robot rotation
        m_finalPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

        m_timer.reset();
        m_timer.start();
    }

    public void resetPID() {
        m_xController.setPID(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD);
        m_yController.setPID(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD);
        m_thetaController.setPID(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_THETA_KD);
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();

        var desiredState = m_trajectory.sample(curTime);
        var desiredPose = desiredState.poseMeters;

        var poseError = desiredPose.relativeTo(m_pose.get());

        double targetXVel = m_xController.calculate(m_pose.get().getTranslation().getX(),
                desiredPose.getTranslation().getX());

        double targetYVel = m_yController.calculate(m_pose.get().getTranslation().getY(),
                desiredPose.getTranslation().getY());

        // The robot will go to the desired rotation of the final pose in the
        // trajectory,
        // not following the poses at individual states.
        double targetAngularVel = m_thetaController.calculate(Math.toRadians(-Drivetrain.getInstance().getPigeon().getFusedHeading()),
                heading.getRadians());

        double vRef = desiredState.velocityMetersPerSecond;
        targetXVel += vRef * poseError.getRotation().getCos();
        targetYVel += vRef * poseError.getRotation().getSin();

        var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);

        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        SmartDashboard.putNumber("Target X Pos", desiredPose.getTranslation().getX());
        SmartDashboard.putNumber("Target Y Pos", desiredPose.getTranslation().getY());
        SmartDashboard.putNumber("Trajectory Angle", heading.getDegrees());

        SmartDashboard.putNumber("Trajectory X Error", m_xController.getPositionError());
        SmartDashboard.putNumber("Trajectory Y Error", m_yController.getPositionError());
        SmartDashboard.putNumber("Trajectory Angle Error", Math.toDegrees(m_thetaController.getPositionError()));

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}