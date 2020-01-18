package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since January 6, 2020
 */
public class OI
{
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    private static final double SHOOTER_SPEED = 2;

    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    private OI() {
        driverGamepad = new XboxGamepad(RobotMap.DRIVER_PORT);
        operatorGamepad = new XboxGamepad(RobotMap.OPERATOR_PORT);

        initBindings();
    }

    private void initBindings() {
        // operatorGamepad.getButtonB().whilePressed(new ToggleShooterAngle());
        // operatorGamepad.getButtonA().whilePressed(new SpinBottomIntakeManual(1));
        // operatorGamepad.getButtonX().whilePressed(new SpinIndexerManual(1));
        // driverGamepad.getButtonY().whenPressed(new InstantCommand(Shooter::toggleAngle, Shooter.getInstance()));
        // driverGamepad.getButtonA().whilePressed(() -> Shooter.spinShooter(SHOOTER_SPEED));
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public static OI getInstance() {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}