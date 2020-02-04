package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Aligns the drivetrain to a target using limelight and revs up the shooter to be prepared to shoot.
 */
public class SpinShooterLimelight extends IndefiniteCommand {
    private static final double LIMELIGHT_ANGLE = 10; // tune
    private static final double LIMELIGHT_HEIGHT = 1.54; // tune
    private static final double TARGET_HEIGHT = 7.5625; // tune
    private static final double THRESHOLD = 0;

    public static final double SHOOTER_HIGH_ANGLE_DEGREES = 50;
    public static final double SHOOTER_LOW_ANGLE_DEGREES = 21;

    public static final double GRAVITATIONAL_FIELD_STRENGTH = 9.8;

    public static final double DISTANCE_Y = 0;
    
    public static final double SCALE_A = 0.00212;//LIN_SCALE 2.32;
    public static final double SCALE_B = 1.914;
    public static final double SCALE_C = 18.77;

    private static final int SAMPLING_SIZE = 75;
    private LinearFilter filter = LinearFilter.movingAverage(SAMPLING_SIZE);
    public SpinShooterLimelight() {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();
    }
    
    public void execute() {
        
        double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.getTy() + LIMELIGHT_ANGLE));
        // double yDisplacement = Limelight.getCamtranY();
        // double xDisplacement = Limelight.getCamtranX();

        // double initialVelocityY = Math.sqrt(2 * 9.8 * yDisplacement);
        // double angle = Limelight.getTy() - LIMELIGHT_ANGLE; //Y angle
        // double initialVelocityX = Math.tan(90 - angle) * yDisplacement; //90-angle is complement

        // double initialVelocity = Math.sqrt(Math.pow(initialVelocityY, 2) , Math.pow(initialVelocityX, 2));
        
        // Shooter.getInstance().spinShooter(initialVelocity + MULTIPLIER * distance);
        double averageDistance = filter.calculate(distance);
        // double distance = Math.sqrt(Math.pow(Limelight.getCamtranZ(), 2) + Math.pow(Limelight.getCamtranX(), 2)) / 12.0;
        SmartDashboard.putNumber("Distance", averageDistance);
        SmartDashboard.putNumber("Shooter velocity Limelight", Math.pow(averageDistance, 2) * SCALE_A + averageDistance * SCALE_B + SCALE_C);
        Shooter.getInstance().spinShooterVelocity(Math.pow(averageDistance, 2) * SCALE_A + averageDistance * SCALE_B + SCALE_C); //Distance will be proportional to our velocity (use the SCALE to tune)
        // Shooter.getInstance().getMaster().set(ControlMode.Velocity, 11000);
    }
}