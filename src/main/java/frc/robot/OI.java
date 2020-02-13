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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.bottomintake.SpinBottomIntake;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.auton.SwerveAutonAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterLimelightAuton;
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

    ;
    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    private StartingPositionEight startingPositionEight = StartingPositionEight.LEFT;
    private StartingPositionTen startingPositionTen = StartingPositionTen.LEFT;

    private static SwerveDriveKinematicsConstraint constraint = new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.MAX_DRIVE_VELOCITY);

    private static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY, Drivetrain.MP_MAX_DRIVE_ACCELERATION)
            .setKinematics(Drivetrain.getInstance().getKinematics())
            .addConstraint(constraint);

    private static Trajectory leftStartingAutonEight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(2, 5, Rotation2d.fromDegrees(180))
    ), config);

    private static Trajectory middleStartingAutonEight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        new Pose2d(0, 5, Rotation2d.fromDegrees(180))
    ), config);

    private static Trajectory rightStartingAutonEight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        new Pose2d(-2, 5, Rotation2d.fromDegrees(180))
    ), config);

    private static Trajectory leftStartingAutonTen= TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        new Pose2d(5, 0, Rotation2d.fromDegrees(270))
    ), config);

    private static Trajectory middleStartingAutonTen= TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(270))
    ), config);

    private static Trajectory rightStartingAutonTen = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        new Pose2d(1, 0, Rotation2d.fromDegrees(270))
    ), config);

    private static Trajectory autonToShootingPositionTenBall = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(-2, 5, Rotation2d.fromDegrees(180))
    ), config);

    private static Trajectory horizontalTrajectory = TrajectoryGenerator.generateTrajectory(List.of( 
            new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(180))), config);

    private static Trajectory autonPartTwo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(2, 0, Rotation2d.fromDegrees(0))
    ), config);

    private static Trajectory autonPartThree = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(2, 0, Rotation2d.fromDegrees(180))
        ), config);
        
    private static Rotation2d heading = Rotation2d.fromDegrees(0);
    //Change heading to what it needs to be
    SequentialCommandGroup eightBallAuton = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(startingPositionEight.getValue(), heading),
        new SwerveAutonAlignWithLimelight(),
        new SpinShooterLimelightAuton().raceWith(
            new MoveBallsToShooter()),
        new SwerveDriveWithOdometryProfiling(autonPartTwo, heading).raceWith(
            new SpinBottomIntake(1), 
            new SpinIndexer()),
        new SwerveDriveWithOdometryProfiling(autonPartThree, heading),
        new SwerveAutonAlignWithLimelight(),
        new SpinShooterLimelightAuton().raceWith(
            new MoveBallsToShooter())
    );

    SequentialCommandGroup tenBallAuton = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(startingPositionTen.getValue(), heading).raceWith(
            new SpinBottomIntake(1),
            new SpinIndexer()),
        new SwerveDriveWithOdometryProfiling(autonToShootingPositionTenBall, heading),
        new SwerveAutonAlignWithLimelight(),
        new SpinShooterLimelightAuton().raceWith(
            new MoveBallsToShooter()),
        new SwerveDriveWithOdometryProfiling(autonPartTwo, heading).raceWith(
            new SpinBottomIntake(1), 
            new SpinIndexer()),
        new SwerveDriveWithOdometryProfiling(autonPartThree, heading),
        new SwerveAutonAlignWithLimelight(),
        new SpinShooterLimelightAuton().raceWith(
            new MoveBallsToShooter())
    );
    
    private enum StartingPositionEight {
        LEFT(leftStartingAutonEight), MIDDLE(middleStartingAutonEight), RIGHT(rightStartingAutonEight);
        private Trajectory value;
        private StartingPositionEight(Trajectory value) {
            this.value = value;
        }
        public Trajectory getValue() {
            return value;
        }
    }
    
    private enum StartingPositionTen {
        LEFT(leftStartingAutonTen), MIDDLE(middleStartingAutonTen), RIGHT(rightStartingAutonTen);
        private Trajectory value;
        private StartingPositionTen(Trajectory value) {
            this.value = value;
        }
        public Trajectory getValue() {
            return value;
        }
    }    
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
        

        
        
        driverGamepad.getButtonX().whilePressed(new SpinShooterVelocity(90));
        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());


        driverGamepad.getButtonB().whilePressed(new ParallelCommandGroup(new SpinBottomIntake(1), new SpinIndexer()));

        driverGamepad.getButtonY().whilePressed(new ParallelCommandGroup(new SpinShooterLimelight(), new MoveBallsToShooter()));

        driverGamepad.getButtonSelect().whenPressed(
            eightBallAuton
        );
            

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