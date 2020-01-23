package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class SpinShooterLimelight extends IndefiniteCommand {
    private static final double LIMELIGHT_ANGLE = 20; // tune
    private static final double LIMELIGHT_HEIGHT = 10; // tune
    private static final double TARGET_HEIGHT = 50; // tune
    private static final double THRESHOLD = 0;

    
    public static final double SHOOTER_HIGH_ANGLE_DEGREES = 50;
    public static final double SHOOTER_LOW_ANGLE_DEGREES = 21;

    public static final double GRAVITY = 9.8;

    public static final double DISTANCE_Y = 0;
    
    public static final double SCALE = 0.7;

    public SpinShooterLimelight() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        double distance = Math.sqrt(Math.pow(Limelight.getCamtranZ(), 2) + Math.pow(Limelight.getCamtranX(), 2)) / 12.0;
        Shooter.getInstance().spinShooter(distance * SCALE);
        
        
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