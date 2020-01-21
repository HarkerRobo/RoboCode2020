package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import harkerrobolib.wrappers.XboxGamepad;
import jaci.pathfinder.Waypoint;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since January 6, 2020
 */
public class OI
{
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    private static final double SHOOTER_SPEED = 2;
    private static SwerveDriveKinematicsConstraint constraint;
    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;
    

    private OI() {
        driverGamepad = new XboxGamepad(RobotMap.DRIVER_PORT);
        operatorGamepad = new XboxGamepad(RobotMap.OPERATOR_PORT);

        initBindings();

    }

    private void initBindings() {
        // operatorGamepad.getButtonB().whilePressed(new ToggleShooterAngle());
        // operatorGamepad.getButtonA().whilePressed(new SpinBottomIntakeManual(1));
        // operatorGamepad.getButtonX().whilePressed(new SpinIndexerManual(1));
        // driverGamepad.getButtonY().whenPressed(new InstantCommand(Shooter::toggleAngle, Shooter.getInstance()));
        // driverGamepad.getButtonA().whilePressed(() -> Shooter.spinShooter(SHOOTER_SPEED));
        constraint = new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.MAX_DRIVE_VELOCITY);

        TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MAX_DRIVE_VELOCITY, Drivetrain.MAX_DRIVE_ACCELERATION)
                .setKinematics(Drivetrain.getInstance().getKinematics())
                .addConstraint(constraint);

        Trajectory linearTrajectory = TrajectoryGenerator.generateTrajectory(List.of( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0))), config);

        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());

        driverGamepad.getButtonA().whenPressed(new SwerveDriveWithOdometryProfiling(linearTrajectory));
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}