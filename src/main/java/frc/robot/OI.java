package frc.robot;

import frc.robot.commands.bottomintake.SpinBottomIntakeManual;
import frc.robot.commands.indexer.SpinIndexerManual;
import frc.robot.commands.shooter.ToggleShooterAngle;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * The OI class reads inputs from the joysticks and binds them to commands.
 * 
 * @since January 6, 2020
 */
public class OI
{
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;

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