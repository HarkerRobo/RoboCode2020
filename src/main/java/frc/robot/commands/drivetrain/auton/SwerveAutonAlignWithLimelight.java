package frc.robot.commands.drivetrain.auton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * copy of SwerveAlignWithLimelight; only change is addition of an isFinished method
 * @author Ada Praun-Petrovic
 * @author Shahzeb Lakhani
 */
public class SwerveAutonAlignWithLimelight extends CommandBase {

    public static final double PID_CONTROLLER_PERIOD = 0.01; //In seconds

    private PIDController txController;
    private PIDController thorController;
    private final double ALLOWABLE_ERROR = 0.1;

    public SwerveAutonAlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());

        txController = new PIDController(Drivetrain.TX_kP, Drivetrain.TX_kI, Drivetrain.TX_kD, PID_CONTROLLER_PERIOD);
        thorController = new PIDController(Drivetrain.THOR_kP, Drivetrain.THOR_kI, Drivetrain.THOR_kD, PID_CONTROLLER_PERIOD);
        txController.setSetpoint(Drivetrain.TX_SETPOINT);
        thorController.setSetpoint(Drivetrain.THOR_SETPOINT);
        txController.setTolerance(Drivetrain.TX_ALLOWABLE_ERROR);
        thorController.setTolerance(Drivetrain.THOR_ALLOWABLE_ERROR);
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.setNeutralMode(NeutralMode.Brake));
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.configClosedloopRamp(Drivetrain.DRIVE_RAMP_RATE));
        Drivetrain.getInstance().applyToAllAngle((angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.PRIMARY_INDEX));
        Drivetrain.getInstance().applyToAllDrive((falcon) -> falcon.setNeutralMode(NeutralMode.Brake));
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.configClosedloopRamp(Drivetrain.ANGLE_RAMP_RATE));

        Limelight.setCamModeVision();
        Limelight.setLEDS(true);
    }

    @Override
    public void execute() {
        double turn = txController.calculate(Limelight.getTx(), Drivetrain.TX_SETPOINT) * Drivetrain.MAX_ROTATION_VELOCITY;

        //No translation
        double translateX = 0;
        double translateY = 0; 

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX, translateY, turn, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
        );

        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);

        Drivetrain.getInstance().setDrivetrainVelocity(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3], false, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Limelight.getTx()) < ALLOWABLE_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        txController.reset();
        thorController.reset();
        Drivetrain.getInstance().stopAllDrive();

        Limelight.setLEDS(true);
    }
}