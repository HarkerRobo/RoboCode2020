package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.bottomintake.SpinIntakeVelocity;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.drivetrain.SwerveManualHeadingControl;
import frc.robot.commands.drivetrain.SwerveRotateToHeading;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.commands.spinner.RotationControlTimed;
import frc.robot.commands.spinner.SpinnerPositionColorSensor;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Climber;
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

    public static final double XBOX_JOYSTICK_DEADBAND = 0.05;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static final double SHOOTER_REV_TIME = 2;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    public static TrajectoryConfig TRAJ_CONFIG = Trajectories.config;

    public static Rotation2d forwardHeading = Rotation2d.fromDegrees(0);
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
        // ParallelCommandGroup shootLimelight = ;

        // ParallelCommandGroup shootManual = ;

        // ParallelCommandGroup intakeAgitate = new ParallelCommandGroup( //Intaking while spinning spine forward
        //     new SpinIntakeVelocity(0.5), 
        //     new SpinIndexer(1, false));

        // ParallelCommandGroup outtakeAgitate = new ParallelCommandGroup( //Outaking while reversing spine 
        //     new SpinIntakeVelocity(-0.5), 
        //     new SpinIndexer(1, true));

        ParallelCommandGroup jumble = new ParallelCommandGroup( //Intaking while reversing spine
            new SpinIntakeVelocity(0.5), 
            new SpinIndexer(1, true));
            
        /*
            https://docs.google.com/drawings/d/1EcjV5vskFA2VKJZLO42NMxbFo7ZGpU-gMVATEqj9jZ0/edit

            Default Command Controls:
            Driver left joystick x and y: Translation
            Driver right joystick x: Rotation
            Driver up/down dpad: Climber Manual
            Driver/Operator left trigger: Outtake
            Driver/Operator right trigger: Intake
            Operator right x: Spinner manual
        */
        // driverGamepad.getButtonSelect().whenPressed(new InstantCommand(() -> Climber.getInstance().toggleSoftLimits()));
        
        // driverGamepad.getButtonA().whenPressed(new InstantCommand(() -> { 
        //         System.out.println(Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //         System.out.println(Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //         System.out.println(Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        //         System.out.println(Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // }));

        driverGamepad.getButtonStart().whenPressed(new InstantCommand(() -> SwerveManualHeadingControl.isNotOptimized = !SwerveManualHeadingControl.isNotOptimized));
        driverGamepad.getButtonB().whilePressed(new StartEndCommand(() -> Shooter.getInstance().spinShooterPercentOutput(0.15), () -> Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0), Shooter.getInstance()));
        // driverGamepad.getButtonY().whenPressed(new SwerveDriveWithOdometryProfiling(Trajectories.Test.circle, Rotation2d.fromDegrees(0)));
        driverGamepad.getRightDPadButton().whenPressed(command)
        driverGamepad.getButtonSelect().whenPressed(new InstantCommand(() -> {Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * Drivetrain.getInstance().getPigeon().getFusedHeading()); 
            SwerveManualHeadingControl.headingFlag = false;
            SwerveManualHeadingControl.flag = false;}));
        // driverGamepad.getButtonX().whilePressed(new SpinIndexer(0.7, true));

        //Shoot from Target Zone Command
        // driverGamepad.getButtonY().whilePressed(new ParallelCommandGroup(new SpinShooterVelocity(65), new MoveBallsToShooter(false)));
        // driverGamepad.getButtonY().whenPressed(new SwerveRotateToHeading(180));
        driverGamepad.getButtonBumperRight().whilePressed(new SwerveAlignWithLimelight());
        // driverGamepad.getButtonX().whilePressed(new SpinShooterVelocity(90));

        // driverGamepad.getButtonSelect().whilePressed(new ParallelCommandGroup(
        //     new SpinShooterVelocity(90), 
        //     new MoveBallsToShooter(false)));

        // driverGamepad.getButtonStart().whilePressed(new MoveBallsToShooter(false));
        // driverGamepad.getButtonX().whenPressed(
        //     new InstantCommand(
        //         () -> Drivetrain.getInstance().setDefaultCommand(Drivetrain.getInstance().getDefaultCommand().getName().equals("SwerveManual") ? new SwerveManualHeadingControl() : new SwerveManual()))
        //         );

        // operatorGamepad.getLeftDPadButton().whilePressed(new SpinIndexer(0.7, false));

        // driverGamepad.getLeftDPadButton().whenPressed(new SetClimberPosition(Climber.MIN_POSITION, Climber.FEED_FORWARD));
        // driverGamepad.getRightDPadButton().whenPressed(new SetClimberPosition(Climber.MAX_POSITION, Climber.FEED_FORWARD));

        operatorGamepad.getButtonBumperLeft().whilePressed(new ParallelCommandGroup(
            new SpinShooterLimelight(), 
            new MoveBallsToShooter(false)));
        
        operatorGamepad.getButtonBumperRight().whilePressed(new SpinShooterVelocity(90));
        operatorGamepad.getButtonB().whenPressed(new InstantCommand(() -> BottomIntake.getInstance().toggleSolenoid()));
        operatorGamepad.getButtonA().whilePressed(new MoveBallsToShooter(false));
        operatorGamepad.getButtonX().whenPressed(new InstantCommand(() -> Indexer.getInstance().toggleSolenoid()));
        operatorGamepad.getLeftDPadButton().whilePressed(jumble);
        operatorGamepad.getButtonSelect().whilePressed(new SpinIntakeVelocity(0.3));
        operatorGamepad.getButtonStart().whenPressed(new InstantCommand(() -> Shooter.getInstance().toggleHoodAngle()));
        // operatorGamepad.getLeftDPadButton().whilePressed(jumble);
        // operatorGamepad.getLeftDPadButton().whilePressed(new SpinIntakeVelocity(0.5));
        operatorGamepad.getButtonY().whenPressed(new InstantCommand(() -> Spinner.getInstance().toggleSolenoid()));
        operatorGamepad.getUpDPadButton().whenPressed(new RotationControlTimed());
        // operatorGamepad.getDownDPadButton().whenPressed(new SpinnerPositionColorSensor());
        operatorGamepad.getRightDPadButton().whilePressed(new SpinIndexer(0.7, false));
        // operatorGamepad.getDownDPadButton().whenPressed(new ConditionalCommand(
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.DAY_FAR)), 
      
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR)), 
        //     () -> !RobotMap.IS_NIGHT));

        // operatorGamepad.getRightDPadButton().whenPressed(new ConditionalCommand(
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.DAY_MEDIUM)), 
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_FAR)), 
        //     () -> !RobotMap.IS_NIGHT));

        // operatorGamepad.getUpDPadButton().whenPressed(new ConditionalCommand(
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.DAY_CLOSE)), 
        //     new CallMethodCommand(() -> Limelight.setPipeline(RobotMap.PIPELINES.NIGHT_CLOSE)), 
        //     () -> !RobotMap.IS_NIGHT));
        // 
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