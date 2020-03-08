package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * 64.111 distance is break point for far
 * 17.643 distance is break point for middle
 * 
 * Aligns the drivetrain to a target using limelight and revs up the shooter to be prepared to shoot.
 */
public class SpinShooterLimelight extends IndefiniteCommand {
    public static double LIMELIGHT_ANGLE = 18;
    public static final double LIMELIGHT_HEIGHT = 1.54; // tune
    public static final double TARGET_HEIGHT = 7.5625; // tune

    public static final double SHOOTER_HIGH_ANGLE_DEGREES = 50;
    public static final double SHOOTER_LOW_ANGLE_DEGREES = 21;
    private static final double SCALE_A = 0.375576;//0.139165;
    private static final double SCALE_B = -13.3005;//-4.15045;
    private static final double SCALE_C = 217.474;//131.773;

    private static final int NUM_SAMPLES = 30;
    public static MedianFilter medianFilter = new MedianFilter(NUM_SAMPLES);

    public SpinShooterLimelight() {
        addRequirements(Shooter.getInstance());

        SmartDashboard.putNumber("Set shooter speed", 100);
    }

    @Override
    public void initialize() {
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();

        medianFilter.reset();

        // Shooter.getInstance().getMaster().config_kP(Shooter.FLYWHEEL_VELOCITY_SLOT, SmartDashboard.getNumber("flywheel kp", Shooter.FLYWHEEL_KP));
        // Shooter.getInstance().getMaster().config_kD(Shooter.FLYWHEEL_VELOCITY_SLOT, SmartDashboard.getNumber("flywheel kd", Shooter.FLYWHEEL_KD));
    }
    
    public void execute() {
        Limelight.setLEDS(true);

        double distance = Shooter.getInstance().getLimelightDistance();
        LIMELIGHT_ANGLE = SmartDashboard.getNumber("Limelight angle", LIMELIGHT_ANGLE);
       
        double averageDistance = medianFilter.calculate(distance);
        SmartDashboard.putNumber("distance", averageDistance);
        
        // linear scale
        // double desiredVel = averageDistance * SmartDashboard.getNumber("scale", SCALE);

        // quadratic
        double desiredVel = 0.98 * (averageDistance * averageDistance * SCALE_A + averageDistance * SCALE_B + SCALE_C);

        Shooter.getInstance().spinShooterVelocity(desiredVel);

        SmartDashboard.putNumber("Shooter set velocity", desiredVel);
        SmartDashboard.putNumber("Shooter actual velocity", Shooter.getInstance().getMaster().getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter velocity error", Shooter.getInstance().getMaster().getClosedLoopError());        
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
        
        Limelight.setLEDS(false);
    }
}
