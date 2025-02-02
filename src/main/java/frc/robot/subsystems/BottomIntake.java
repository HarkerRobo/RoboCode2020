package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.bottomintake.SpinIntakeVelocity;
import frc.robot.commands.indexer.SpinIndexer;
import harkerrobolib.wrappers.HSTalon;

/**
 * Represents the Bottom Intake Subsystem, controlled by 1 Falcon.
 * 
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Angela Jia
 * @since January 6, 2020
 */
public class BottomIntake extends SubsystemBase {

    public static final Value IN = Value.kReverse;
    public static final Value OUT = Value.kForward;

    static {
        if(RobotMap.IS_COMP) {
            MOTOR_INVERT = true;
        } else {
            MOTOR_INVERT = true;
        }
    }

    private static BottomIntake instance;

    private HSTalon talon;
    
    private static boolean MOTOR_INVERT;

    private static final double VOLTAGE_COMPENSATION = 10;
    
    private static final int INTAKE_CURRENT_CONTINUOUS = 40;
    private static final int INTAKE_CURRENT_PEAK = 50;
    private static final int INTAKE_CURRENT_PEAK_DUR = 100;

    private static final double OUTPUT_MULTIPLIER = 0.7;

    private static final boolean SENSOR_PHASE = true;
    private static final double CURRENT_DRAW_MIN = 10;
    private static final int JAMMED_VELOCITY = 100;

    private DoubleSolenoid solenoid; 

    private static boolean jamFlag = false;
    
    private BottomIntake() {
        talon = new HSTalon(RobotMap.CAN_IDS.INTAKE_MOTOR_ID);
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.INTAKE_SOLENOID_FORWARD, RobotMap.CAN_IDS.INTAKE_SOLENOID_REVERSE);

        setupTalons();
    }
    
    private void setupTalons() {
        talon.configFactoryDefault();

        talon.setInverted(MOTOR_INVERT);

        talon.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        talon.enableVoltageCompensation(true);
        
        talon.setNeutralMode(NeutralMode.Coast);

        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        talon.configContinuousCurrentLimit(INTAKE_CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(INTAKE_CURRENT_PEAK);
        talon.configPeakCurrentDuration(INTAKE_CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    
        talon.configVoltageCompSaturation(VOLTAGE_COMPENSATION);
        talon.enableVoltageCompensation(true);
        talon.setSensorPhase(SENSOR_PHASE);
    }

    public void spinIntake(double magnitude) {
        if(magnitude == 0) 
            talon.set(ControlMode.Disabled, 0);
        else
            talon.set(ControlMode.PercentOutput, OUTPUT_MULTIPLIER * magnitude);
    }

    public void toggleSolenoid() {
        solenoid.set(solenoid.get() == Value.kReverse ? Value.kForward : Value.kReverse);
    }

    public boolean isStalling() {
        return talon.getStatorCurrent() > CURRENT_DRAW_MIN && talon.getSelectedSensorVelocity() < JAMMED_VELOCITY;
    }
            
    public HSTalon getTalon() {
        return talon;
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public static BottomIntake getInstance() {
        if(instance == null)
            instance = new BottomIntake();
        return instance;
    }
}