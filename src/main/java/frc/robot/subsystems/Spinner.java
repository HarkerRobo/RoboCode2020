package frc.robot.subsystems;

import harkerrobolib.wrappers.HSTalon;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * The Spinner manipulates the control panel to obtain position and rotation control.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani 
 * @author Chirag Kaushik
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @author Aimee Wang
 * @since January 22, 2020
 */
public class Spinner extends SubsystemBase {
    private static Spinner instance;

    private HSTalon bagMotor;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    
    public Spinner() {
        //Same motor controllerbeing used for spinner and indexer
        bagMotor = Indexer.getInstance().getBagMotor();
        colorSensor = new ColorSensorV3(i2cPort);
    }

    public HSTalon getBagMotor() {
        return bagMotor;
    }

    //Don't need to re-initialize hstalon configs for bagMotor

    public static Spinner getInstance() {
        if(instance == null)
            instance = new Spinner();
        return instance;
    }
}