package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

public class Shooter implements Subsystem {

    public TalonFX master;
    public TalonFX follower;

    public DoubleSolenoid angleSolenoid;

    public static final TalonFXInvertType MASTER_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType FOLLOWER_INVERT = TalonFXInvertType.FollowMaster;
    
    public static final int VOLTAGE_COMP = 10;

    private static Shooter instance;
    
    public Shooter() {
        master = new TalonFX(RobotMap.CAN_IDS.SHOOTER_MASTER_ID);
        follower = new TalonFX(RobotMap.CAN_IDS.SHOOTER_FOLLOWER_ID);
        angleSolenoid = new DoubleSolenoid(RobotMap.CAN_IDS.SHOOTER_SOLENOID_FORWARD, RobotMap.CAN_IDS.SHOOTER_SOLENOID_REVERSE);
        setupTalons();
    }

    public void setupTalons() {
        follower.follow(master);
        invert();
        setNeutralMode();
        setupVoltageComp();
    }

    

    private void setupVoltageComp() {
        master.configVoltageCompSaturation(VOLTAGE_COMP);
        master.enableVoltageCompensation(true);
    }

    private void setNeutralMode() {
        master.setNeutralMode(NeutralMode.Coast);
    }

    private void invert() {
        master.setInverted(MASTER_INVERT);
        follower.setInverted(FOLLOWER_INVERT);
    }
    
    public TalonFX getMaster() {
        return master;
    }

    public TalonFX getFollower() {
        return follower;
    }

    public static Shooter getInstance()
    {
        if(instance == null)
            instance = new Shooter();
        return instance;
    }
}