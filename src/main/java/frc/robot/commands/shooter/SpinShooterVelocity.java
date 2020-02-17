package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * 69.82 max distance 32-33 ft max shot distance
 * 
 * 64.706 full send 114.3
 * 
 * 49.234, 103.7
 * 46, 102.5
 * 43.868,101.3
 * 40.684, 101
 * 36.374, 99
 * 34.155 , 98
 * 27.473, 95.6
 * 22.075,93
 * 17.314, 92.3
 * 14.783, 103 (THRESHOLD)
 * 
 */
public class SpinShooterVelocity extends IndefiniteCommand {
    private double velocity;
    private MedianFilter medianFilter = new MedianFilter(400);

    public SpinShooterVelocity(double velocity) {
        addRequirements(Shooter.getInstance());
        this.velocity = velocity;

        SmartDashboard.putNumber("Set shooter speed", velocity);
    }

    public void execute() {
        velocity = SmartDashboard.getNumber("Set shooter speed", velocity);

        Shooter.getInstance().spinShooterVelocity(velocity);

        double distance = Shooter.getInstance().getLimelightDistance();
        double averageDistance = medianFilter.calculate(distance);

        SmartDashboard.putNumber("Shooter limelight distance", averageDistance);
        SmartDashboard.putNumber("Shooter velocity error", Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, Shooter.getInstance().getMaster().getClosedLoopError(), SpeedUnit.FEET_PER_SECOND, Shooter.WHEEL_DIAMETER, Shooter.TICKS_PER_REV));
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }

}