package frc.robot;

import frc.robot.*;
import harkerrobolib.wrappers.XboxGamepad;

public class OI
{
    private static OI instance;

    private XboxGamepad driverGamepad;
    private XboxGamepad operatorGamepad;

    private OI()
    {
        driverGamepad = new XboxGamepad(RobotMap.DRIVER_PORT);
        operatorGamepad = new XboxGamepad(RobotMap.OPERATOR_PORT);

        initBindings();
    }

    private void initBindings()
    {
    }

    public XboxGamepad getDriverGamepad()
    {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad()
    {
        return operatorGamepad;
    }

    public static OI getInstance()
    {
        if(instance == null)
            instance = new OI();
        return instance;
    }
}