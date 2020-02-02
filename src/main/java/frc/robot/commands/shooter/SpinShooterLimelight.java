package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Aligns the drivetrain to a target using limelight and revs up the shooter to be prepared to shoot.
 */
public class SpinShooterLimelight extends IndefiniteCommand {
    private static final double LIMELIGHT_ANGLE = 20; // tune
    private static final double LIMELIGHT_HEIGHT = 10; // tune
    private static final double TARGET_HEIGHT = 50; // tune
    private static final double THRESHOLD = 0;

    public static final double SHOOTER_HIGH_ANGLE_DEGREES = 50;
    public static final double SHOOTER_LOW_ANGLE_DEGREES = 21;

    public static final double GRAVITATIONAL_FIELD_STRENGTH = 9.8;

    public static final double DISTANCE_Y = 0;
    
    public static final double SCALE = 0.7;

    public SpinShooterLimelight() {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize() {
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();
    }
    
    public void execute() {
        
        // double yDisplacement = Limelight.getCamtranY();
        // double xDisplacement = Limelight.getCamtranX();

        // double initialVelocityY = Math.sqrt(2 * 9.8 * yDisplacement);
        // double angle = Limelight.getTy(); //Y angle
        // double initialVelocityX = Math.tan(90 - angle) * yDisplacement; //90-angle is complement

        // double initialVelocity = Math.sqrt(Math.pow(initialVelocityY, 2) , Math.pow(initialVelocityX, 2));
        
        // Shooter.getInstance().spinShooter(initialVelocity + MULTIPLIER * distance);

        double distance = Math.sqrt(Math.pow(Limelight.getCamtranZ(), 2) + Math.pow(Limelight.getCamtranX(), 2)) / 12.0;
        SmartDashboard.putNumber("Distance", distance);
        
        Shooter.getInstance().spinShooterVelocity(distance * SCALE); //Distance will be proportional to our velocity (use the SCALE to tune)
        
        //If we try to optimize use this code:
        // Shooter.getInstance().getAngleSolenoid().set(distance > THRESHOLD ? Shooter.SHOOTER_LOW_ANGLE : Shooter.SHOOTER_HIGH_ANGLE);
        // //vf^2 - vi^2 = 2aΔd
        // double angle = distance > THRESHOLD ? SHOOTER_HIGH_ANGLE_DEGREES : SHOOTER_LOW_ANGLE_DEGREES;
        // double velocity = Math.sqrt(
        //     (GRAVITY * Drivetrain.FEET_PER_METER * Math.pow(DISTANCE_Y,2)) / 
        //     (2 * (Math.tan(Math.toRadians(angle)) * DISTANCE_Y - DISTANCE_Y) * Math.pow(Math.cos(Math.toRadians(angle)), 2))
        // );
    }
}