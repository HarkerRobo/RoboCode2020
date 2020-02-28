package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveModule;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the drivetrain using WPILib's SwerveControllerCommand with 
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 */
public class SwerveDriveWithOdometryProfiling extends HSSwerveDriveOdometry {
    
    private Timer timer;
    private Trajectory trajectory;

    private final int ALLOWABLE_ERROR = 5;
    private final int TIMEOUT = 800;
        
    public SwerveDriveWithOdometryProfiling(Trajectory trajectory, Rotation2d heading) {
        super(trajectory,
            heading,
            Drivetrain.getInstance()::getPose,
            Drivetrain.getInstance().getKinematics(), 
            new PIDController(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD), 
            new PIDController(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD), 
            new ProfiledPIDController(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_THETA_KD,
                                    Drivetrain.THETA_CONSTRAINTS),
            Drivetrain.getInstance()::setDrivetrainModuleStates,
            Drivetrain.getInstance()
        );
        
        this.trajectory = trajectory;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().updateMPPID();
        resetPID();

        //Set to x and y from starting Pose2d of path but keep current rotation value from odometry
        Pose2d initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), 
                Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));

        Rotation2d currentRot = Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading());

        Drivetrain.getInstance().getOdometry().resetPosition(initialPose, currentRot);

        long initialTime = System.currentTimeMillis();
        boolean isAtSepoint = false;
        Rotation2d initialRotation = initialPose.getRotation().plus(Rotation2d.fromDegrees(90));
        SmartDashboard.putNumber("initialRotation", initialRotation.getDegrees());
        //Perhaps add some functionality to rotate robot to the heading as well
        while (System.currentTimeMillis() - initialTime < TIMEOUT && !isAtSepoint) {
            isAtSepoint = Math.abs(Drivetrain.getInstance().getTopLeft().getAngleErrorDegrees(initialRotation.getDegrees())) < ALLOWABLE_ERROR
                && Math.abs(Drivetrain.getInstance().getTopRight().getAngleErrorDegrees(initialRotation.getDegrees())) < ALLOWABLE_ERROR
                && Math.abs(Drivetrain.getInstance().getBackLeft().getAngleErrorDegrees(initialRotation.getDegrees())) < ALLOWABLE_ERROR
                && Math.abs(Drivetrain.getInstance().getBackRight().getAngleErrorDegrees(initialRotation.getDegrees())) < ALLOWABLE_ERROR;

            Drivetrain.getInstance().setDrivetrainVelocity(
                new SwerveModuleState(0, initialRotation), 
                new SwerveModuleState(0, initialRotation), 
                new SwerveModuleState(0, initialRotation), 
                new SwerveModuleState(0, initialRotation), false, true);
        }
        
        super.initialize();
        timer.start();
    }   
    
    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.reset();
    }
}