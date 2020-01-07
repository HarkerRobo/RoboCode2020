package frc.robot;

import harkerrobolib.wrappers.XboxGamepad;

/**
 * OI class to hold controllers and bindings
 * 
 * @since 1/6/20
 */
public class OI
{
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