package frc.robot;

/**
 * Stores all project-wide constants, such as CAN Ids
 * 
 * @since 01/06/20
 */
public class RobotMap {
	public static final boolean IS_PRACTICE = false; 

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    public static final int PRIMARY_INDEX = 0;
    public static final int AUXILIARY_INDEX = 1;
    
    public static final int DEFAULT_TIMEOUT = 10; //ms

    /**
     * Stores CAN Id constants.
     */
	public static class CAN_IDS {
        public static final int INTAKE_MOTOR_ID = -1;
        public static final int INTAKE_SOLENOID_FORWARD = -1;
		public static final int INTAKE_SOLENOID_REVERSE = -1;
        
        public static final int SPINNER_ID = 0;
		public static final int SPINNER_SOLENOID_FORWARD = -1;
		public static final int SPINNER_SOLENOID_REVERSE = -1;
        		
        public static final int INDEXER_TALON_ID = -1;
        public static final int INDEXER_FOLLOWER_TALON_ID = -1; //For Color Spinner and one indexer motor
        public static final int INDEXER_SOLENOID = -1;

		public static final int SHOOTER_MASTER_ID = -1;
		public static final int SHOOTER_FOLLOWER_ID = -1;
		public static final int SHOOTER_SOLENOID = -1;

		public static final int LEFT_CLIMBER_ID = -1;
		public static final int RIGHT_CLIMBER_ID = -1;
		public static final int CLIMBER_SOLENOID_FORWARD = -1;
		public static final int CLIMBER_SOLENOID_REVERSE = -1;
		
		public static final int TL_DRIVE_ID = 0;
		public static final int TL_ANGLE_ID = 1;

		public static final int TR_DRIVE_ID = 2;
		public static final int TR_ANGLE_ID = 3;

		public static final int BL_DRIVE_ID = 4;
		public static final int BL_ANGLE_ID = 5;

		public static final int BR_DRIVE_ID = 6;
		public static final int BR_ANGLE_ID = 7;

		public static final int PIGEON_ID = 0;
	}
}