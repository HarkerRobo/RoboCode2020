package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.CallMethodCommand;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since January 6, 2020
 */
public class OI
{
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static SwerveDriveKinematicsConstraint constraint;
    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    

    // //Go from auton left starting position to shooting position
    // private static final Trajectory leftStartingToShooting = TrajectoryGenerator.generateTrajectory(List.of( 
    //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(0, 10, Rotation2d.fromDegrees(180))), config);
            
    // private static final Trajectory middleStartingToShooting = TrajectoryGenerator.generateTrajectory(List.of( 
    //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(0, 10, Rotation2d.fromDegrees(0)),
    //     new Pose2d(-9.72, 0, Rotation2d.fromDegrees(180))), config);

    // private static final Trajectory rightStartingToShooting = TrajectoryGenerator.generateTrajectory(List.of( 
    //     new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
    //     new Pose2d(1.676, -6.447, Rotation2d.fromDegrees(-90))), config);

    
    
    private OI() {
        driverGamepad = new XboxGamepad(RobotMap.DRIVER_PORT);
        operatorGamepad = new XboxGamepad(RobotMap.OPERATOR_PORT);

        initBindings();
    }

    /**
     * Sets up button bindings on the driver and operator controllers
     */
    private void initBindings() {
        // driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
        //                                                     new SwerveAlignWithLimelight(), 
        //                                                     new SpinShooterLimelight()));
        // driverGamepad.getButtonBumperLeft().whilePressed(new MoveBallsToShooter());
        // driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
        //     Drivetrain.getInstance().updatePositionPID();
        // }));
        driverGamepad.getButtonBumperLeft().whenPressed(new InstantCommand(Shooter.getInstance()::toggleHoodAngle, Shooter.getInstance()));
        driverGamepad.getButtonY().whilePressed(new SpinShooterLimelight());
        constraint = new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.MAX_DRIVE_VELOCITY);

        TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY, Drivetrain.MP_MAX_DRIVE_ACCELERATION)
                .setKinematics(Drivetrain.getInstance().getKinematics())
                .addConstraint(constraint);

        Trajectory horizontalTrajectory = TrajectoryGenerator.generateTrajectory(List.of( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0))), config);
        Trajectory verticalTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
            new Pose2d(0, 1, Rotation2d.fromDegrees(90))
        ), config);

        Trajectory initiationToBackTest = TrajectoryGenerator.generateTrajectory(List.of( 
         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
         new Pose2d(2, 0, Rotation2d.fromDegrees(0))), config);

        Rotation2d heading = Rotation2d.fromDegrees(0);
        driverGamepad.getButtonX().whilePressed(new SpinShooterVelocity(90));
        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());
        driverGamepad.getButtonA().whenPressed(new SwerveDriveWithOdometryProfiling(initiationToBackTest, heading));
        
        driverGamepad.getDownDPadButton().whenPressed(
            new ConditionalCommand(
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.DAY_FAR); }), 
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR); }), 
                () -> !RobotMap.IS_NIGHT));
        driverGamepad.getRightDPadButton().whenPressed(
            new ConditionalCommand(
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.DAY_MEDIUM); }), 
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR); }), 
                () -> !RobotMap.IS_NIGHT));
        driverGamepad.getUpDPadButton().whenPressed(
            new ConditionalCommand(
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.DAY_CLOSE); }), 
                new CallMethodCommand(() -> { Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_CLOSE); }), 
                () -> !RobotMap.IS_NIGHT));

        // driverGamepad.getButtonBumperLeft().whenPressed(new InstantCommand(
        //     () -> {
        //         Drivetrain.getInstance().getTopLeft().getAngleMotor().setSelectedSensorPosition(0);
        //         Drivetrain.getInstance().getTopRight().getAngleMotor().setSelectedSensorPosition(0);
        //         System.out.println(Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //         System.out.println(Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //     }, 
        //     Drivetrain.getInstance()));

        // driverGamepad.getButtonBumperRight().whenPressed(new InstantCommand(
        //     () -> {
        //         Drivetrain.getInstance().getBackLeft().getAngleMotor().setSelectedSensorPosition(0);
        //         Drivetrain.getInstance().getBackRight().getAngleMotor().setSelectedSensorPosition(0);
        //         System.out.println(Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //         System.out.println(Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //     }, 
        //     Drivetrain.getInstance()));

        // driverGamepad.getButtonA().whenPressed(new SwerveDriveWithOdometryProfiling(horizontalTrajectory, heading));
        driverGamepad.getButtonStart().whenPressed(
                new InstantCommand(() -> { Drivetrain.getInstance().getPigeon().setFusedHeading(0); }));
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