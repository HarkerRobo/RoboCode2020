package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Aligns the drivetrain to a target using limelight and revs up the shooter to be prepared to shoot.
 */
public class SpinShooterLimelight extends IndefiniteCommand {
    public static final double LIMELIGHT_ANGLE = 10; // tune
    public static final double LIMELIGHT_HEIGHT = 1.54; // tune
    public static final double TARGET_HEIGHT = 7.5625; // tune

    public static final double SHOOTER_HIGH_ANGLE_DEGREES = 50;
    public static final double SHOOTER_LOW_ANGLE_DEGREES = 21;
    
    private static final double SCALE = 1.6;//1.6
    private static final double SCALE_A = 0.00867972;//LIN_SCALE 2.32;
    private static final double SCALE_B = -0.207790;
    private static final double SCALE_C = 92.998;

    private static final double BB_BELOW_OUTPUT = 0.9;
    private static final double BB_ABOVE_OUTPUT = 0;

    private static final int NUM_SAMPLES = 180;
    private static final double DISTANCE_SCALE = 0.99; //Accounts for offset in limelight
    private LinearFilter averageFilter = LinearFilter.movingAverage(NUM_SAMPLES);
    private MedianFilter medianFilter = new MedianFilter(NUM_SAMPLES);
    public SpinShooterLimelight() {
        addRequirements(Shooter.getInstance());
        SmartDashboard.putNumber("scale", SCALE);
    }

    @Override
    public void initialize() {
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();
        medianFilter.reset();
        averageFilter.reset();
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
       
        double averageDistance = medianFilter.calculate(distance);
        SmartDashboard.putNumber("Distance", averageDistance);
        
        // linear scale
        // double desiredVel = averageDistance * SmartDashboard.getNumber("scale", SCALE);

        // quadratic
        double desiredVel = DISTANCE_SCALE * (averageDistance * averageDistance * SCALE_A + averageDistance * SCALE_B + SCALE_C);

        // 90% Optimization
        // double currentVel = Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, Shooter.getInstance().getMaster().getSelectedSensorVelocity() / Shooter.GEAR_RATIO, SpeedUnit.FEET_PER_SECOND, Shooter.WHEEL_DIAMETER, Shooter.TICKS_PER_REV);
        // if(currentVel < 0.9 * desiredVel) {
        //     Shooter.getInstance().spinShooterPercentOutput(0.9);
        //     SmartDashboard.putNumber("Shooter velocity Limelight", 1072);
        // }
        // else {
        //     Shooter.getInstance().spinShooterVelocity(averageDistance * SCALE);
        //     SmartDashboard.putNumber("Shooter velocity Limelight", desiredVel);
        // }
 // if(currentVel < desiredVel) {
        //     Shooter.getInstance().spinShooterPercentOutput(BB_BELOW_OUTPUT);
        //     SmartDashboard.putNumber("Shooter %output Limelight", BB_BELOW_OUTPUT);
        // } else {
        //     Shooter.getInstance().spinShooterPercentOutput(BB_ABOVE_OUTPUT);
        //     SmartDashboard.putNumber("Shooter %output Limelight", BB_ABOVE_OUTPUT);
        // }
        // Bang-bang
        // double output = currentVel < desiredVel 
        //     ? Shooter.FLYWHEEL_KP * (desiredVel - currentVel) + Shooter.FLYWHEEL_KF * desiredVel 
        //     : Shooter.FLYWHEEL_KF * desiredVel;
        // Shooter.getInstance().getMaster().set(ControlMode.PercentOutput, output);
        Shooter.getInstance().spinShooterVelocity(desiredVel);
        SmartDashboard.putNumber("Shooter sent velocity Limelight", desiredVel);
        SmartDashboard.putNumber("Shooter velocity Limelight", Shooter.getInstance().getMaster().getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter error", Shooter.getInstance().getMaster().getClosedLoopError());
            
        }
    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}
