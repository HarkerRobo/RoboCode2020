package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

public class Indexer implements Subsystem {

    private static Indexer instance;
    
    private TalonFX falcon;

    private static final TalonFXInvertType INVERT = TalonFXInvertType.Clockwise;

    public static final int VOLTAGE_COMP = 10;

    private Indexer()
    {
        falcon = new TalonFX(RobotMap.CAN_IDS.HOPPER_TALON_ID);
    }
    
    public void setupTalons() {
        
        falcon.setInverted(INVERT);

        falcon.setNeutralMode(NeutralMode.Coast);

        falcon.configVoltageCompSaturation(VOLTAGE_COMP);
        falcon.enableVoltageCompensation(true);
    }

    public TalonFX getFalcon() {
        return falcon;
    }

    public static Indexer getInstance()
    {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }
}