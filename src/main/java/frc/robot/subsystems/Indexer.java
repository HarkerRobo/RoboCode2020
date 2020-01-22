package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * The indexer stores and transfers power cells from the intake to the shooter. It consists
 * of a single motor that spins a belt. 
 * 
 * @author Chirag Kaushik
 * @author Shahzeb Lakhani
 * @since January 6, 2020
 */
public class Indexer implements Subsystem {
    private static Indexer instance;
    
    private static TalonFXInvertType FALCON_INVERT;
    static {
        if(RobotMap.IS_PRACTICE) {
            FALCON_INVERT = TalonFXInvertType.Clockwise; //Change accordingly
        } else {
            FALCON_INVERT = TalonFXInvertType.Clockwise; //Change accordingly
        }
    }  
    
    private TalonFX falcon;
    public static final int VOLTAGE_COMP = 10;

    private Indexer() {
        falcon = new TalonFX(RobotMap.CAN_IDS.HOPPER_TALON_ID);
    }
    
    public void setupTalons() {
        falcon.setInverted(FALCON_INVERT);
        falcon.setNeutralMode(NeutralMode.Coast);

        falcon.configVoltageCompSaturation(VOLTAGE_COMP);
        falcon.enableVoltageCompensation(true);
    }

    public TalonFX getFalcon() {
        return falcon;
    }

    public static Indexer getInstance() {
        if(instance == null)
            instance = new Indexer();
        return instance;
    }
}