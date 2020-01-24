package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since 01/06/20
 */
public class OI
{
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static SwerveDriveKinematicsConstraint constraint;
    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;
    
    private OI() {
        driverGamepad = new XboxGamepad(RobotMap.DRIVER_PORT);
        operatorGamepad = new XboxGamepad(RobotMap.OPERATOR_PORT);

        initBindings();
    }

    /**
     * Sets up button bindings on the driver and operator controllers
     */
    private void initBindings() {
        driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
                                                            new SwerveAlignWithLimelight(), 
                                                            new SpinShooterLimelight()));
        driverGamepad.getButtonBumperLeft().whilePressed(new MoveBallsToShooter());

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

    /**
     * Returns the driver Xbox controller
     */
    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    /**
     * Returns the operator Xbox controller
     */
    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    /**
     * Returns the global instance of OI
     */
    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}