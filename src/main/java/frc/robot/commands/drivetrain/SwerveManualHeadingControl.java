package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.MathUtil;

/**
 * Controls the Swerve Modules using PercentOutput or Velociy for the drive motors and 
 * Position PID for the angle motors.
 * The left joystick controls translation (velocity direction and magnitude)
 * The right joystick's X axis controls rotation (angular velocity magnitude)
 * 
 * 'back' is defined as closest to the battery
 * 'left' is defined as left when standing at the back and looking forward
 * 
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @author Angela Jia
 * @author Chirag Kaushik
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @author Rohan Bhowmik
 * @since 2/16/20
 */
public class SwerveManualHeadingControl extends CommandBase {

    static {
        if (RobotMap.IS_PRACTICE) {
            HIGH_VELOCITY_HEADING_MULTIPLIER = 0.16;
            LOW_VELOCITY_HEADING_MULTIPLIER = 0.09;
        } else {
            HIGH_VELOCITY_HEADING_MULTIPLIER = 0.17;
            LOW_VELOCITY_HEADING_MULTIPLIER = 0.17;
        }
    }
    
    private static final double OUTPUT_MULTIPLIER = 1;
    private static final boolean IS_PERCENT_OUTPUT = false;
    private static double HIGH_VELOCITY_HEADING_MULTIPLIER;
    private static double LOW_VELOCITY_HEADING_MULTIPLIER;
    private static final double ACCELERATION_HEADING_MULTIPLIER = 0;
    private static final double TURN_VEL_THRESHOLD = 160;
    
    private double translateX, translateY, headingX, headingY, headingAngle, turnMagnitude;
    
    private static double prevPigeonHeading;
    private static double prevTime;
    private static double prevVel;
    private static boolean pigeonFlag; //True if the Driver Right X input is non-zero
    private static double pigeonAngle;
    
    private static double lastPigeonUpdateTime; // seconds
    private static double turnVel;
    private static double turnAccel;
    
    private static boolean joystickFlag;
    private PIDController headingController;
    
    public SwerveManualHeadingControl() {
        addRequirements(Drivetrain.getInstance());
        headingController = new PIDController(Drivetrain.HEADING_KP, Drivetrain.HEADING_KI, Drivetrain.HEADING_KD);
        pigeonFlag = false;

        // pigeonAngle = 90;
        // prevPigeonHeading = 90;
        prevTime = Timer.getFPGATimestamp();
        lastPigeonUpdateTime = Timer.getFPGATimestamp();
        prevPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();
        prevVel = 0;
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive(
            (driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX)
        );

        Drivetrain.getInstance().applyToAllAngle(
            (angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX)
        );
        pigeonFlag = true;
        prevPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();
        pigeonAngle = prevPigeonHeading;

        joystickFlag = false;
    }

    @Override
    public void execute() {
        translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        headingX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        headingY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), OI.XBOX_JOYSTICK_DEADBAND);

        if (headingY != 0 && headingX != 0) {
            headingAngle = Math.toDegrees(Math.atan2(headingY, headingX));
            while (Drivetrain.getInstance().getPigeon().getFusedHeading() - headingAngle > 180) {
                headingAngle += 360;
            }
            while (Drivetrain.getInstance().getPigeon().getFusedHeading() - headingAngle < 180) {
                headingAngle -= 360;
            }
        }
        turnMagnitude = headingController.calculate(Drivetrain.getInstance().getPigeon().getFusedHeading(), headingAngle);

        if (Math.abs(translateX) > 0 || Math.abs(translateY) > 0 || Math.abs(headingX) > 0 || Math.abs(headingY) > 0) {
            joystickFlag = true;
        }

        //scale input from joysticks
        translateX *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        translateY *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        turnMagnitude *= -1 * OUTPUT_MULTIPLIER * Drivetrain.MAX_ROTATION_VELOCITY;

        // double currentPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();

        // if(pigeonFlag && turnMagnitude == 0) { //If there was joystick input but now there is not
        //     double velocityHeadingMultiplier = Math.abs(turnVel) > TURN_VEL_THRESHOLD ? HIGH_VELOCITY_HEADING_MULTIPLIER : LOW_VELOCITY_HEADING_MULTIPLIER;

        //     // account for momentum when turning
        //     pigeonAngle = currentPigeonHeading + turnVel * velocityHeadingMultiplier + turnAccel * ACCELERATION_HEADING_MULTIPLIER;
        // }

        // pigeonFlag = Math.abs(turnMagnitude) > 0; //Update pigeon flag

        // if(!pigeonFlag) { //If there is no joystick input currently
        //     // turnMagnitude = !RobotMap.IS_PRACTICE ? Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading) : turnMagnitude;
        //     turnMagnitude = Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading);
        // }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turnMagnitude, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        if (joystickFlag)
            Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], IS_PERCENT_OUTPUT, false);

        // if(Timer.getFPGATimestamp() - lastPigeonUpdateTime > 0.01) {
        //     double currentTime = Timer.getFPGATimestamp();
        //     double deltaTime = (double)(currentTime - prevTime);

        //     turnVel = (currentPigeonHeading - prevPigeonHeading) / deltaTime;
            
        //     turnAccel = (turnVel - prevVel) / deltaTime;

        //     prevPigeonHeading = currentPigeonHeading;
        //     prevVel = turnVel;
        //     prevTime = currentTime;
        //     lastPigeonUpdateTime = Timer.getFPGATimestamp();
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().stopAllDrive();
    }
}