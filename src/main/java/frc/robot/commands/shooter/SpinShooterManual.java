package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the shooter at a velocity determined by the driver right trigger.
 * 
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since January 22, 2020
 */
public class SpinShooterManual extends IndefiniteCommand {
    private static final double SPEED_MULTIPLIER = 1;

    public SpinShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void initialize() {  
        Shooter.getInstance().getMaster().selectProfileSlot(Shooter.FLYWHEEL_VELOCITY_SLOT, RobotMap.PRIMARY_INDEX);
    }

    public void execute() {
        double rightTrigger = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.XBOX_TRIGGER_DEADBAND);
        double leftTrigger = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.XBOX_TRIGGER_DEADBAND);
        
        //Checks which trigger has more output and picks between them.
        //Left trigger means reject the current ball.
        double output = rightTrigger - leftTrigger; // From [-1, 1]
        
        Shooter.getInstance().spinShooter(output * SPEED_MULTIPLIER * Shooter.MAX_VELOCITY);
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().getMaster().set(TalonFXControlMode.Disabled, 0);
    }
}