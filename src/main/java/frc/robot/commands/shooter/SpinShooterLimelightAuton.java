package frc.robot.commands.shooter;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.LinearFilter;
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
public class SpinShooterLimelightAuton extends IndefiniteCommand {
    public static double LIMELIGHT_ANGLE = 18;
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
    private static double DISTANCE_SCALE; //Accounts for offset in limelight
    private LinearFilter averageFilter = LinearFilter.movingAverage(NUM_SAMPLES);
    private MedianFilter medianFilter = new MedianFilter(NUM_SAMPLES);
    private static long startTime;
    private static long timeout;

    public SpinShooterLimelightAuton(long timeout) {
        addRequirements(Shooter.getInstance());
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        Limelight.setLEDS(true);
        Limelight.setCamModeVision();
        medianFilter.reset();
        averageFilter.reset();
        startTime = System.currentTimeMillis();
    }
    
    public void execute() {
        
        double distance = Shooter.getInstance().getLimelightDistance();
        
        // Shooter.getInstance().spinShooter(initialVelocity + MULTIPLIER * distance);
       
        double averageDistance = medianFilter.calculate(distance);
        
        // linear scale
        // double desiredVel = averageDistance * SmartDashboard.getNumber("scale", SCALE);

        // quadratic
        double desiredVel = (averageDistance * averageDistance * SCALE_A + averageDistance * SCALE_B + SCALE_C);

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

        SmartDashboard.putNumber("Shooter set velocity", desiredVel);
        SmartDashboard.putNumber("Shooter actual velocity", Shooter.getInstance().getMaster().getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter velocity error", Shooter.getInstance().getMaster().getClosedLoopError());
            
        }
    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }

    public boolean isFinished() { //TODO: actually make this 
        return System.currentTimeMillis() - startTime > timeout;
    }
}
