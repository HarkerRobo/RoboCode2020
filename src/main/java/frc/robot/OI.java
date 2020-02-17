package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.bottomintake.SpinBottomIntake;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.commands.spinner.RotationControlTimed;
import frc.robot.commands.spinner.SpinnerPositionColorSensor;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
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

    public static Rotation2d forwardHeading = Rotation2d.fromDegrees(180);
    public static Rotation2d pickupFiveHeading = Rotation2d.fromDegrees(120);

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
        //         new SwerveAlignWithLimelight(),
        //         new SpinShooterLimelight()));

        // driverGamepad.getButtonBumperLeft().whilePressed(new MoveBallsToShooter(false));
        // driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> Drivetrain.getInstance().updatePositionPID()));
        // driverGamepad.getButtonY().whilePressed(new SpinShooterLimelight());

        driverGamepad.getButtonX().whilePressed(new SpinShooterVelocity(90));
        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());
        
        // driverGamepad.getButtonY().whenPressed(new SpinIndexer());
        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
            Indexer.getInstance().toggleSolenoid();
        }));

        driverGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            BottomIntake.getInstance().toggleSolenoid();
        }));

        driverGamepad.getButtonB().whilePressed(
            new ParallelCommandGroup(new SpinBottomIntake(0.5), new SpinIndexer(false)));

        // driverGamepad.getButtonY().whilePressed(new ParallelCommandGroup(
        //     new SpinShooterLimelight(), new SequentialCommandGroup(new WaitCommand(2), new MoveBallsToShooter(false))));
        driverGamepad.getLeftDPadButton().whilePressed(
            new ParallelCommandGroup(
                new SpinShooterLimelight(), 
                new SequentialCommandGroup(
                    new WaitCommand(1), 
                    new InstantCommand(() -> Indexer.getInstance().getSolenoid().set(Indexer.OPEN)), 
                    new MoveBallsToShooter(false)))
            );

        driverGamepad.getButtonY().whilePressed(new SpinShooterVelocity(90));

        driverGamepad.getButtonStart().whilePressed(new MoveBallsToShooter(false));
        driverGamepad.getButtonBumperLeft().whilePressed(new ParallelCommandGroup(new SpinIndexer(true), new SpinBottomIntake(-0.3)));
        // driverGamepad.getDownDPadButton().whilePressed(new SpinnerManual());
        // driverGamepad.getDownDPadButton().whilePressed(
        //     new ParallelCommandGroup(new SpinBottomIntake(0.3), new SpinIndexer(true)));
        
        // driverGamepad.getUpDPadButton().whilePressed(
        //      new SpinIndexer(true));
        // driverGamepad.getButtonSelect().whenPressed(
        // eightBallAuton
        // );
        // driverGamepad.getButtonSelect().whenPressed(new InstantCommand(() -> Spinner.getInstance().toggleSolenoid()));

        operatorGamepad.getDownDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_FAR);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR);
        }), () -> !RobotMap.IS_NIGHT));

        operatorGamepad.getRightDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_MEDIUM);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR);
        }), () -> !RobotMap.IS_NIGHT));

        operatorGamepad.getUpDPadButton().whenPressed(new ConditionalCommand(new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.DAY_CLOSE);
        }), new CallMethodCommand(() -> {
            Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_CLOSE);
        }), () -> !RobotMap.IS_NIGHT));

        // driverGamepad.getLeftDPadButton().whenPressed(new SpinnerPositionColorSensor());
        // driverGamepad.getButtonStart().whenPressed(new RotationControlTimed());
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