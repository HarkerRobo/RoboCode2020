package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
/**
 * Spins the control panel based on encoder position, read Driver station color,
 * and color sensor 
 * 
 * @author Shahzeb Lakhani
 * @version 1/21/20
 */
public class Spinner extends SubsystemBase {
    private static TalonSRX spinnerTalon;
    public Spinner() {
        spinnerTalon = new TalonSRX(RobotMap.CAN_IDS.SPINNER_TALON);
        // setupTalon();
    }
    
    // private void setupTalon() {
    //     spinnerTalon.work();
    // }


}