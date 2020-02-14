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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.bottomintake.SpinBottomIntake;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.auton.SwerveAutonAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterLimelightAuton;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import frc.robot.auto.Trajectories;
import harkerrobolib.commands.CallMethodCommand;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since January 6, 2020
 */
public class OI {
    private static OI instance;

    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    public static TrajectoryConfig TRAJ_CONFIG = Trajectories.config;

    private StartingPositionEight startingPositionEight = StartingPositionEight.LEFT;
    private StartingPositionTen startingPositionTen = StartingPositionTen.LEFT;

    public static Rotation2d forwardHeading = Rotation2d.fromDegrees(180);
    public static Rotation2d pickupFiveHeading = Rotation2d.fromDegrees(120);

    private enum StartingPositionEight {
        LEFT(Trajectories.Eight.leftStartingAutonEight), MIDDLE(Trajectories.Eight.middleStartingAutonEight),
        RIGHT(Trajectories.Eight.rightStartingAutonEight);

        public Trajectory value;

        private StartingPositionEight(Trajectory value) {
            this.value = value;
        }
    }

    private enum StartingPositionTen {
        LEFT(Trajectories.Ten.leftStartingAutonTen), MIDDLE(Trajectories.Ten.middleStartingAutonTen),
        RIGHT(Trajectories.Ten.rightStartingAutonTen);

        public Trajectory value;

        private StartingPositionTen(Trajectory value) {
            this.value = value;
        }
    }

    // Change heading to what it needs to be
    // SequentialCommandGroup eightBallAuton = new SequentialCommandGroup(
    // new SwerveDriveWithOdometryProfiling(startingPositionEight.getValue(),
    // heading),
    // new SwerveAutonAlignWithLimelight(),
    // new SpinShooterLimelightAuton().raceWith(
    // new MoveBallsToShooter()),
    // new SwerveDriveWithOdometryProfiling(autonPartTwo, heading).raceWith(
    // new SpinBottomIntake(1),
    // new SpinIndexer()),
    // new SwerveDriveWithOdometryProfiling(autonPartThree, heading),
    // new SwerveAutonAlignWithLimelight(),
    // new SpinShooterLimelightAuton().raceWith(
    // new MoveBallsToShooter())
    // );

    // SequentialCommandGroup tenBallAuton = new SequentialCommandGroup(
    // new SwerveDriveWithOdometryProfiling(startingPositionTen.getValue(),
    // heading).raceWith(
    // new SpinBottomIntake(1),
    // new SpinIndexer()),
    // new SwerveDriveWithOdometryProfiling(autonToShootingPositionTenBall,
    // heading),
    // new SwerveAutonAlignWithLimelight(),
    // new SpinShooterLimelightAuton().raceWith(
    // new MoveBallsToShooter()),
    // new SwerveDriveWithOdometryProfiling(autonPartTwo, heading).raceWith(
    // new SpinBottomIntake(1),
    // new SpinIndexer()),
    // new SwerveDriveWithOdometryProfiling(autonPartThree, heading),
    // new SwerveAutonAlignWithLimelight(),
    // new SpinShooterLimelightAuton().raceWith(
    // new MoveBallsToShooter())
    // );

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
        // new SwerveAlignWithLimelight(),
        // new SpinShooterLimelight()));
        // driverGamepad.getButtonBumperLeft().whilePressed(new MoveBallsToShooter());
        // driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
        // Drivetrain.getInstance().updatePositionPID();
        // }));
        // driverGamepad.getButtonBumperLeft().whenPressed(new
        // InstantCommand(Shooter.getInstance()::toggleHoodAngle,
        // Shooter.getInstance()));
        // driverGamepad.getButtonY().whilePressed(new SpinShooterLimelight());
        // constraint = new
        // SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(),
        // Drivetrain.MAX_DRIVE_VELOCITY);

        // driverGamepad.getButtonX().whilePressed(new SpinShooterVelocity(90));
        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());
        
        // driverGamepad.getButtonY().whenPressed(new SpinIndexer());
        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
            Indexer.getInstance().toggleSolenoid();
        }));

        driverGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            BottomIntake.getInstance().toggleSolenoid();
        }));

        driverGamepad.getButtonB().whilePressed(
            new ParallelCommandGroup(new SpinBottomIntake(0.3), new SpinIndexer(false)));
        // driverGamepad.getButtonB().whilePressed(new SpinBottomIntake(1));

        driverGamepad.getButtonY().whilePressed(new ParallelCommandGroup(
            new SpinShooterLimelight(),  new SequentialCommandGroup(new WaitCommand(2), new MoveBallsToShooter(false))));

        // driverGamepad.getDownDPadButton().whilePressed(new SpinnerManual());
        // driverGamepad.getDownDPadButton().whilePressed(
        //     new ParallelCommandGroup(new SpinBottomIntake(0.3), new SpinIndexer(true)));
        
        // driverGamepad.getUpDPadButton().whilePressed(
        //      new SpinIndexer(true));
        // driverGamepad.getButtonSelect().whenPressed(
        // eightBallAuton
        // );

        driverGamepad.getDownDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_FAR);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR);
        }), () -> !RobotMap.IS_NIGHT));

        driverGamepad.getRightDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_MEDIUM);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR);
        }), () -> !RobotMap.IS_NIGHT));

        driverGamepad.getUpDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_CLOSE);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_CLOSE);
        }), () -> !RobotMap.IS_NIGHT));

        // driverGamepad.getButtonBumperLeft().whenPressed(new InstantCommand(
        // () -> {
        // Drivetrain.getInstance().getTopLeft().getAngleMotor().setSelectedSensorPosition(0);
        // Drivetrain.getInstance().getTopRight().getAngleMotor().setSelectedSensorPosition(0);
        // System.out.println(Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // System.out.println(Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // },
        // Drivetrain.getInstance()));

        // driverGamepad.getButtonBumperRight().whenPressed(new InstantCommand(
        // () -> {
        // Drivetrain.getInstance().getBackLeft().getAngleMotor().setSelectedSensorPosition(0);
        // Drivetrain.getInstance().getBackRight().getAngleMotor().setSelectedSensorPosition(0);
        // System.out.println(Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // System.out.println(Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // },
        // Drivetrain.getInstance()));

        // driverGamepad.getButtonA().whenPressed(
        //         new SwerveDriveWithOdometryProfiling(Trajectories.Test.heart, Rotation2d.fromDegrees(0)));

        // driverGamepad.getButtonStart().whenPressed(new InstantCommand(() -> {
        //     Drivetrain.getInstance().getPigeon().setFusedHeading(0);
        // }));
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
        if (instance == null)
            instance = new OI();
        return instance;
    }
}