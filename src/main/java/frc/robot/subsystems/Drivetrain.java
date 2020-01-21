package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSPigeon;
import harkerrobolib.wrappers.HSTalon;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Simulates the drivetrain subsystem on the robot. A Swerve Drivetrain contains
 * four SwerveModules.
 * 
 * 'back' is defined as closest to the battery 'left' is defined as left when
 * standing at the back and looking forward
 * 
 * Acronyms: TL: Top Left TR: Top Right BL: Back Left BR: Back Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Arjun Dixit
 * @since 11/1/19
 */
public class Drivetrain extends SubsystemBase {
    public static Drivetrain instance;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private HSPigeon pigeon;
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;

    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(-Drivetrain.DT_WIDTH/2, Drivetrain.DT_LENGTH/2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(Drivetrain.DT_WIDTH/2, Drivetrain.DT_LENGTH/2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-Drivetrain.DT_WIDTH/2, -Drivetrain.DT_LENGTH/2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(Drivetrain.DT_WIDTH/2, -Drivetrain.DT_LENGTH/2);
    
    private static TalonFXInvertType TL_DRIVE_INVERTED;
    private static TalonFXInvertType TR_DRIVE_INVERTED;
    private static TalonFXInvertType BL_DRIVE_INVERTED;
    private static TalonFXInvertType BR_DRIVE_INVERTED;

    private static boolean TL_ANGLE_INVERTED;
    private static boolean TR_ANGLE_INVERTED;
    private static boolean BL_ANGLE_INVERTED;
    private static boolean BR_ANGLE_INVERTED;

    private static boolean TL_DRIVE_SENSOR_PHASE;
    private static boolean TR_DRIVE_SENSOR_PHASE;
    private static boolean BL_DRIVE_SENSOR_PHASE;
    private static boolean BR_DRIVE_SENSOR_PHASE;

    private static boolean TL_ANGLE_SENSOR_PHASE;
    private static boolean TR_ANGLE_SENSOR_PHASE;
    private static boolean BL_ANGLE_SENSOR_PHASE;
    private static boolean BR_ANGLE_SENSOR_PHASE;

    public static final int TL_OFFSET;//15561;
    public static final int TR_OFFSET;//2492;
    public static final int BL_OFFSET;//15351;
    private static final int BR_OFFSET;//10413;

    public static final int ANGLE_POSITION_SLOT = 0;
    private static final double ANGLE_POSITION_KP;

    private static final double ANGLE_POSITION_KI;
    private static final double ANGLE_POSITION_KD;

    public static final int DRIVE_VELOCITY_SLOT = 0;
    private static final double DRIVE_VELOCITY_KP;
    private static final double DRIVE_VELOCITY_KI;
    private static final double DRIVE_VELOCITY_KD;
    private static final double DRIVE_VELOCITY_KF; // theoretical: 0.034;


    public static final double PIGEON_kP;

    public static final double MP_X_KP;//2.6;
    public static final double MP_X_KI;
    public static final double MP_X_KD;//15;

    public static final double MP_Y_KP;//0.7;
    public static final double MP_Y_KI;
    public static final double MP_Y_KD;

    public static final double MP_THETA_KP;//3.1;
    public static final double MP_THETA_KI;
    public static final double MP_THETA_KD;

    public static final double DRIVE_RAMP_RATE;
    public static final double ANGLE_RAMP_RATE;

    static {
        //practice constants
        if (RobotMap.IS_PRACTICE) {
            TL_DRIVE_INVERTED = TalonFXInvertType.Clockwise;
            TR_DRIVE_INVERTED = TalonFXInvertType.Clockwise;
            BL_DRIVE_INVERTED = TalonFXInvertType.Clockwise;
            BR_DRIVE_INVERTED = TalonFXInvertType.Clockwise;

            TL_ANGLE_INVERTED = true;
            TR_ANGLE_INVERTED = true;
            BL_ANGLE_INVERTED = true;
            BR_ANGLE_INVERTED = false;

            TL_DRIVE_SENSOR_PHASE = true;
            TR_DRIVE_SENSOR_PHASE = true;
            BL_DRIVE_SENSOR_PHASE = false;
            BR_DRIVE_SENSOR_PHASE = false;

            TL_ANGLE_SENSOR_PHASE = true;
            TR_ANGLE_SENSOR_PHASE = true;
            BL_ANGLE_SENSOR_PHASE = true;
            BR_ANGLE_SENSOR_PHASE = false;

            TL_OFFSET = 9154;
            TR_OFFSET = 5915;
            BL_OFFSET = 1604;
            BR_OFFSET = 5724;
            
            ANGLE_POSITION_KP = 1.1;
            ANGLE_POSITION_KI = 0.0;
            ANGLE_POSITION_KD = 11;
            
            DRIVE_VELOCITY_KP = 0.5;
            DRIVE_VELOCITY_KI = 0.0;
            DRIVE_VELOCITY_KD = 5;
            DRIVE_VELOCITY_KF = 0.034; // theoretical: 0.034;

            DRIVE_RAMP_RATE = 0.1;
            ANGLE_RAMP_RATE = 0.2;  
            
            PIGEON_kP = 0.05;//sh

            MP_X_KP = 0;//2.6;
            MP_X_KI = 0;
            MP_X_KD = 0;//15;

            MP_Y_KP = 0;//0.7;
            MP_Y_KI = 0;
            MP_Y_KD = 0;

            MP_THETA_KP = 0;//3.1;
            MP_THETA_KI = 0;
            MP_THETA_KD = 0;
        
        } else { // competitive bot constants
            TL_DRIVE_INVERTED = TalonFXInvertType.CounterClockwise;
            TR_DRIVE_INVERTED = TalonFXInvertType.CounterClockwise;
            BL_DRIVE_INVERTED = TalonFXInvertType.CounterClockwise;
            BR_DRIVE_INVERTED = TalonFXInvertType.CounterClockwise;

            TL_ANGLE_INVERTED = false;
            TR_ANGLE_INVERTED = true;
            BL_ANGLE_INVERTED = false;
            BR_ANGLE_INVERTED = true;

            TL_DRIVE_SENSOR_PHASE = true;
            TR_DRIVE_SENSOR_PHASE = true;
            BL_DRIVE_SENSOR_PHASE = true;
            BR_DRIVE_SENSOR_PHASE = true;
            
            TL_ANGLE_SENSOR_PHASE = false;
            TR_ANGLE_SENSOR_PHASE = true;
            BL_ANGLE_SENSOR_PHASE = false;
            BR_ANGLE_SENSOR_PHASE = true;

            TL_OFFSET = 11575;//15561;
            TR_OFFSET = 14161;//2492;
            BL_OFFSET = 11400;//15351;
            BR_OFFSET = 6447;//10413;
            
            ANGLE_POSITION_KP = 1.1;
            ANGLE_POSITION_KI = 0.0;
            ANGLE_POSITION_KD = 11;
            
            DRIVE_VELOCITY_KP = 0.5;//0.5
            DRIVE_VELOCITY_KI = 0.0;
            DRIVE_VELOCITY_KD = 5;
            DRIVE_VELOCITY_KF = 0.034; // theoretical: 0.034;

            DRIVE_RAMP_RATE = 0.1;
            ANGLE_RAMP_RATE = 0.2;  
            
            PIGEON_kP = 0.02;

            MP_X_KP = 0;//2.6;
            MP_X_KI = 0;
            MP_X_KD = 0;//15;

            MP_Y_KP = 0;//0.7;
            MP_Y_KI = 0;
            MP_Y_KD = 0;

            MP_THETA_KP = 0;//3.1;
            MP_THETA_KI = 0;
            MP_THETA_KD = 0;
        }
    }

    

    public static final double MAX_DRIVE_VELOCITY = 2;
    public static final double MAX_ROTATION_VELOCITY = (2 * Math.PI);
    public static final double MAX_ROTATION_ACCELERATION = 2 * (2 * Math.PI);
    public static final double MAX_DRIVE_ACCELERATION = 3;

   
    public static final double GEAR_RATIO = 6;

    //conversions
    public static final double METERS_PER_FOOT = 0.3048;
    public static final double FEET_PER_METER = 3.28084;

    /**
     * Feet between both of the wheels on the front or back
     */
    public static final double DT_WIDTH = 0.66; //25.5 x 32
    /**
     * Feet between both of the wheels on the left or right
     */
    public static final double DT_LENGTH = 0.535; //20.6 feet;


    
    public static final double WHEEL_DIAMETER = 4;
    
    public static final Constraints THETA_CONSTRAINTS = new Constraints(MAX_ROTATION_VELOCITY, MAX_ROTATION_ACCELERATION);

	public static final double TX_kP = 0.015;

	public static final double TX_kI = 0;

    public static final double TX_kD = 0.;
    
	public static final double THOR_kP = 0;

	public static final double THOR_kI = 0;

	public static final double THOR_kD = 0;

	public static final double TX_SETPOINT = 0.0;

    public static final double THOR_SETPOINT = 0;
    
    public static final double TX_ALLOWABLE_ERROR = 0.4;
    
    public static final double THOR_ALLOWABLE_ERROR = 0;
    private static int index = 0;

    /**
     * Default constructor for Drivetrain
     * Initializes SwerveModules with inverts for drive and angle motors 
     * and sensor phases, resets the encoders to the offset position and 
     * zeroes the angle motors so that all modules are facing forward at
     * 90 degrees, zeroes the pigeon to 90 degrees, and initializes 
     * SwerveDriveKinematics and Odometry.
     */
    private Drivetrain() {
        topLeft = new SwerveModule(RobotMap.CAN_IDS.TL_DRIVE_ID, TL_DRIVE_INVERTED, TL_DRIVE_SENSOR_PHASE, 
                RobotMap.CAN_IDS.TL_ANGLE_ID, TL_ANGLE_INVERTED, TL_ANGLE_SENSOR_PHASE);
        topRight = new SwerveModule(RobotMap.CAN_IDS.TR_DRIVE_ID, TR_DRIVE_INVERTED, TR_DRIVE_SENSOR_PHASE,
                RobotMap.CAN_IDS.TR_ANGLE_ID, TR_ANGLE_INVERTED, TR_ANGLE_SENSOR_PHASE);
        backLeft = new SwerveModule(RobotMap.CAN_IDS.BL_DRIVE_ID, BL_DRIVE_INVERTED, BL_DRIVE_SENSOR_PHASE,
                RobotMap.CAN_IDS.BL_ANGLE_ID, BL_ANGLE_INVERTED, BL_ANGLE_SENSOR_PHASE);
        backRight = new SwerveModule(RobotMap.CAN_IDS.BR_DRIVE_ID, BR_DRIVE_INVERTED, BR_DRIVE_SENSOR_PHASE,
                RobotMap.CAN_IDS.BR_ANGLE_ID, BR_ANGLE_INVERTED, BR_ANGLE_SENSOR_PHASE);

        // topLeft.getCANCoder().setPosition(topLeft.getCANCoder().getAbsolutePosition() - TL_ABSOLUTE_OFFSET);
        // topRight.getCANCoder().setPosition(topRight.getCANCoder().getAbsolutePosition() - TR_ABSOLUTE_OFFSET);
        // backLeft.getCANCoder().setPosition(backLeft.getCANCoder().getAbsolutePosition() - BL_ABSOLUTE_OFFSET);
        // backRight.getCANCoder().setPosition(backRight.getCANCoder().getAbsolutePosition() - BR_ABSOLUTE_OFFSET);

        topLeft.getAngleMotor().setSelectedSensorPosition((topLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TL_OFFSET) / 4);
        topRight.getAngleMotor().setSelectedSensorPosition((topRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - TR_OFFSET) / 4);
        backLeft.getAngleMotor().setSelectedSensorPosition((backLeft.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BL_OFFSET) / 4);
        backRight.getAngleMotor().setSelectedSensorPosition((backRight.getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs() - BR_OFFSET) / 4);

        applyToAllDrive((motor) -> motor.setSelectedSensorPosition(0));

        setupPositionPID();
        setupVelocityPID();

        pigeon = new HSPigeon(RobotMap.CAN_IDS.PIGEON_ID);
        pigeon.configFactoryDefault();
        pigeon.zero();
        pigeon.setFusedHeading(90);

        Conversions.setWheelDiameter(WHEEL_DIAMETER);

        kinematics = new SwerveDriveKinematics(Drivetrain.FRONT_LEFT_LOCATION, Drivetrain.FRONT_RIGHT_LOCATION,
                Drivetrain.BACK_LEFT_LOCATION, Drivetrain.BACK_RIGHT_LOCATION);


        
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(pigeon.getFusedHeading()), new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        Pose2d initialPose = new Pose2d(new Translation2d(), 
                Rotation2d.fromDegrees(pigeon.getFusedHeading()));

        Rotation2d currentRot = Rotation2d.fromDegrees(pigeon.getFusedHeading());

        odometry.resetPosition(initialPose, currentRot);

    }

    /**
     * Updates odometry periodically based on the pigeon heading and
     * the states of the SwerveModules.
     */
    @Override
    public void periodic() {
        if(index  <= 1)
        {
            System.out.println(getPose().getRotation().getDegrees());
            System.out.println(pigeon.getFusedHeading());
            // Drivetrain.().
            System.out.println(topLeft.getState().angle.getDegrees());
            System.out.println(topRight.getState().angle.getDegrees());
            System.out.println(backLeft.getState().angle.getDegrees());
            System.out.println(backRight.getState().angle.getDegrees());
            
            index++;
        }

        odometry.update(Rotation2d.fromDegrees(pigeon.getFusedHeading() - 90),
                topLeft.getState(), topRight.getState(),
                backLeft.getState(), backRight.getState());

        SmartDashboard.putNumber("Current X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Current Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Current Rot", odometry.getPoseMeters().getRotation().getDegrees());
    }

    /**
     * Returns the currently-estimated pose of the robot based on odometry.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();

        // return new Pose2d(new Translation2d(-rawPose.getTranslation().getY(), rawPose.getTranslation().getX()), 
        //         rawPose.getRotation());
    }

    /**
     * Binds PID slots and ramp rate for angle motors.
     */
    public void setupPositionPID() {
        applyToAllAngle((angleMotor) -> angleMotor.config_kP(ANGLE_POSITION_SLOT, ANGLE_POSITION_KP));
        applyToAllAngle((angleMotor) -> angleMotor.config_kI(ANGLE_POSITION_SLOT, ANGLE_POSITION_KI));
        applyToAllAngle((angleMotor) -> angleMotor.config_kD(ANGLE_POSITION_SLOT, ANGLE_POSITION_KD));
        applyToAllAngle((angleMotor) -> angleMotor.configClosedloopRamp(ANGLE_RAMP_RATE));
    }

    /**
     * Binds PID slots and ramp rate for drive motors.
     */
    public void setupVelocityPID() {
        applyToAllDrive((driveMotor) -> driveMotor.config_kF(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KF));
        applyToAllDrive((driveMotor) -> driveMotor.config_kP(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KP));
        applyToAllDrive((driveMotor) -> driveMotor.config_kI(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KI));
        applyToAllDrive((driveMotor) -> driveMotor.config_kD(DRIVE_VELOCITY_SLOT, DRIVE_VELOCITY_KD));
        applyToAllDrive((driveMotor) -> driveMotor.configClosedloopRamp(DRIVE_RAMP_RATE));
    }

    /**
     * Sets the velocity and angle of the drivetrain motors based on the properties 
     * of the passed SwerveModuleStates, with feedforward set to 0, percent output 
     * as false, and motion profile as true.
     */
    public void setDrivetrainModuleStates(SwerveModuleState[] states) {
        setDrivetrainVelocity(states[0], states[1], states[2], states[3], 0, false, true);
    }

    /**
     * Sets the output of the drivetrain based on desired output vectors for each
     * swerve module
     */
    public void setDrivetrainVelocity(SwerveModuleState tl, SwerveModuleState tr, SwerveModuleState bl,
            SwerveModuleState br, double feedForward, boolean isPercentOutput, boolean isMotionProfile) {
        double tlOutput = tl.speedMetersPerSecond;
        double trOutput = tr.speedMetersPerSecond;
        double blOutput = bl.speedMetersPerSecond;
        double brOutput = br.speedMetersPerSecond;

        setSwerveModuleVelocity(topLeft, tlOutput, convertAngle(topLeft, tl.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(topRight, trOutput, convertAngle(topRight, tr.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(backLeft, blOutput, convertAngle(backLeft, bl.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
        setSwerveModuleVelocity(backRight, brOutput, convertAngle(backRight, br.angle.getDegrees()), isPercentOutput,
                isMotionProfile);
    }

    public void setSwerveModuleVelocity(SwerveModule module, double output, double angle, boolean isPercentOutput,
            boolean isMotionProfile) {
        module.setAngleAndDriveVelocity(angle, output, isPercentOutput, isMotionProfile);
    }

    /**
     * Converts the target angle from the desired Vectors into the actual angle for
     * the motors to hold. Angle modification for Field sensitive drive should have
     * already been handled.
     * 
     * Steps:
     * 
     * 1. Subtract 90 degrees. 0 degrees on the Joysticks and desired Vectors points
     * to the right (positive x axis) while 0 degrees on the robot points forward
     * (positive y axis). The subtraction deals with this offset. 2.
     * Increase/Decrease the targetAngle by 360 degrees until it is within +- 180
     * degrees of the current angle
     * 
     * @return The desired angle after all modifications
     */
    public static double convertAngle(SwerveModule module, double targetAngle) {
        // Step 1
        // targetAngle += 90;

        double currDegrees = module.getAngleDegrees();

        // Step 2
        while (currDegrees - targetAngle > 180) {
            targetAngle += 360;
        }
        while (currDegrees - targetAngle < -180) {
            targetAngle -= 360;
        }

        return targetAngle;
    }

    /**
     * Stops all drive motors while holding the current angle
     */
    public void stopAllDrive() {
        setSwerveModuleVelocity(topLeft, 0, topLeft.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(topRight, 0, topRight.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(backLeft, 0, backLeft.getAngleDegrees(), true, false);
        setSwerveModuleVelocity(backRight, 0, backRight.getAngleDegrees(), true, false);
    }

    /**
     * Calls a method on all swerve modules on the drivetrain. For example:
     * Drivetrain.getInstance().applyToAll((motor) -> motor.doSomething());
     */
    public void applyToAll(Consumer<SwerveModule> consumer) {
        consumer.accept(topLeft);
        consumer.accept(topRight);
        consumer.accept(backLeft);
        consumer.accept(backRight);
    }

    /**
     * Calls a method on the angle motor of each swerve module.
     */
    public void applyToAllAngle(Consumer<HSTalon> consumer) {
        consumer.accept(topLeft.getAngleMotor());
        consumer.accept(topRight.getAngleMotor());
        consumer.accept(backLeft.getAngleMotor());
        consumer.accept(backRight.getAngleMotor());
    }

    /**
     * Calls a method on the drive motor of each swerve module.
     */
    public void applyToAllDrive(Consumer<TalonFX> consumer) {
        consumer.accept(topLeft.getDriveMotor());
        consumer.accept(topRight.getDriveMotor());
        consumer.accept(backLeft.getDriveMotor());
        consumer.accept(backRight.getDriveMotor());
    }

    public SwerveModule getTopLeft() {
        return topLeft;
    }

    public SwerveModule getTopRight() {
        return topRight;
    }

    public SwerveModule getBackLeft() {
        return backLeft;
    }

    public SwerveModule getBackRight() {
        return backRight;
    }

    public HSPigeon getPigeon() {
        return pigeon;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
}
