package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
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
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @author Rohan Bhowmik
 * @since 11/4/19
 */
public class SwerveManual extends CommandBase {
    private static final double OUTPUT_MULTIPLIER = 0.7;
    private static final boolean IS_PERCENT_OUTPUT = false;
    private static final double HIGH_VELOCITY_HEADING_MULTIPLIER = 0.5;//0.25
    private static final double LOW_VELOCITY_HEADING_MULTIPLIER = 0.1;
    private static final double ACCELERATION_HEADING_MULTIPLIER = 0;
    private static final double TURN_VEL_THRESHOLD = 160;

    private double translateX, translateY, turnMagnitude;
    
    private static double prevPigeonHeading;
    private static double prevTime;
    private static double prevVel;
    private static boolean pigeonFlag; //True if the Driver Right X input is non-zero
    private static double pigeonAngle;

    private static double lastPigeonUpdateTime; // seconds
    private static double turnVel;
    private static double turnAccel;
    
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());

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
    }

    @Override
    public void execute() {
        translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        turnMagnitude = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);

        SmartDashboard.putNumber("translate x", translateX);
        SmartDashboard.putNumber("translate y", translateY);
        SmartDashboard.putNumber("turn mag", turnMagnitude);
        //scale input from joysticks
        translateX *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        translateY *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VELOCITY;
        turnMagnitude *= -1 * OUTPUT_MULTIPLIER * Drivetrain.MAX_ROTATION_VELOCITY;

        double currentPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();

        SmartDashboard.putNumber("Turn acceleration", turnAccel);
        SmartDashboard.putNumber("Turn Vel", turnVel);
        SmartDashboard.putNumber("dtetha", currentPigeonHeading - prevPigeonHeading);

        if(pigeonFlag && turnMagnitude == 0) { //If there was joystick input but now there is not
            double velocityHeadingMultiplier = Math.abs(turnVel) > TURN_VEL_THRESHOLD ? HIGH_VELOCITY_HEADING_MULTIPLIER : LOW_VELOCITY_HEADING_MULTIPLIER;

            // account for momentum when turning
            pigeonAngle = currentPigeonHeading + turnVel * velocityHeadingMultiplier + turnAccel * ACCELERATION_HEADING_MULTIPLIER;
        }

        pigeonFlag = Math.abs(turnMagnitude) > 0; //Update pigeon flag

        if(!pigeonFlag) { //If there is no joystick input currently
            // turnMagnitude = !RobotMap.IS_PRACTICE ? Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading) : turnMagnitude;
            turnMagnitude = Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading);
            SmartDashboard.putNumber("Pigeon Error", pigeonAngle - currentPigeonHeading);
        }

        SmartDashboard.putNumber("turn mag", turnMagnitude);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turnMagnitude, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        SmartDashboard.putNumber("Speed x", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Speed y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Speed rot", speeds.omegaRadiansPerSecond);

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], 0, IS_PERCENT_OUTPUT, false);

        if(Timer.getFPGATimestamp() - lastPigeonUpdateTime > 0.01) {
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = (double)(currentTime - prevTime);

            turnVel = (currentPigeonHeading - prevPigeonHeading) / deltaTime;
            
            turnAccel = (turnVel - prevVel) / deltaTime;

            prevPigeonHeading = currentPigeonHeading;
            prevVel = turnVel;
            prevTime = currentTime;
            lastPigeonUpdateTime = Timer.getFPGATimestamp();
        }
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