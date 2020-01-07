package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Represents the Bottom Intake Subsystem, controlled by 1 Falcon.
 * 
 * @since 1/6/20
 */
public class BottomIntake implements Subsystem {
    private static BottomIntake instance;
    
    private static final TalonFXInvertType MOTOR_INVERT = TalonFXInvertType.Clockwise; //Change accordingly

    private TalonFX Falcon;

    private static final double VOLTAGE_COMPENSATION = 10;
    
    public static final double DEFAULT_ROLLER_MAGNITUDE_INTAKE = 0;
    
    private BottomIntake() {
        Falcon = new TalonFX(RobotMap.CAN_IDS.BOTTOM_INTAKE_MOTOR_ID);
        setupMotors();
    }
    
    private void setupMotors() {
        invert();
        enableVoltageComp();
        enableNeutralMode();
    }

    private void invert() {
        Falcon.setInverted(MOTOR_INVERT);
    }

    public void enableVoltageComp() {
        Falcon.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        Falcon.enableVoltageCompensation(true);
    }

    public void enableNeutralMode() {
        Falcon.setNeutralMode(NeutralMode.Brake);
    }

    public TalonFX getFalcon() {
        return Falcon;
    }

    public static BottomIntake getInstance() {
        if(instance == null)
        {
            instance = new BottomIntake();
        }
        return instance;
    }
}