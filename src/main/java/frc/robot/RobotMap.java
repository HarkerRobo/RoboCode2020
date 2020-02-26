package frc.robot;

/**
 * Stores all project-wide constants, such as CAN Ids
 * 
 * @since January 6, 2020
 */
public class RobotMap {
	public static final boolean IS_COMP = true; 
	public static final boolean IS_NIGHT = false;

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    public static final int PRIMARY_INDEX = 0;
    public static final int AUXILIARY_INDEX = 1;
    
    public static final int DEFAULT_TIMEOUT = 10; //ms

    /**
     * Stores CAN Id constants.
     */
	public static class CAN_IDS {

		static {
			if(IS_COMP) {
				//THESE ARE IDS NOT CORRECT. FIX THEM.
				INTAKE_SOLENOID_FORWARD = 3;
				INTAKE_SOLENOID_REVERSE = 4;

				SPINNER_SOLENOID_FORWARD = 7;
				SPINNER_SOLENOID_REVERSE = 0;

				INDEXER_SOLENOID_FORWARD = 5;
				INDEXER_SOLENOID_REVERSE = 2;

				SHOOTER_SOLENOID_FORWARD = 6;
				SHOOTER_BACKWARD = 1;
			}
			else {
				INTAKE_SOLENOID_FORWARD = 3;
				INTAKE_SOLENOID_REVERSE = 4;

				SPINNER_SOLENOID_FORWARD = 7;
				SPINNER_SOLENOID_REVERSE = 0;

				INDEXER_SOLENOID_FORWARD = 5;
				INDEXER_SOLENOID_REVERSE = 2;

				SHOOTER_SOLENOID_FORWARD = 6;
				SHOOTER_BACKWARD = 1;
			}
		}
        public static final int INTAKE_MOTOR_ID = 10;
        public static final int INTAKE_SOLENOID_FORWARD;
		public static final int INTAKE_SOLENOID_REVERSE;
        
        public static final int SPINNER_ID = 13;
		public static final int SPINNER_SOLENOID_FORWARD;
        public static final int SPINNER_SOLENOID_REVERSE;
        public static final int SPINNER_CANCODER = 0;
        		
        public static final int SPINE_TALON_ID = 12;
        public static final int AGITATOR_TALON_ID = 13; //For Color Spinner and one indexer motor
		public static final int INDEXER_SOLENOID_FORWARD;
		public static final int INDEXER_SOLENOID_REVERSE;
        
		public static final int SHOOTER_MASTER_ID = 8;
		public static final int SHOOTER_FOLLOWER_ID = 9;
		public static final int SHOOTER_SOLENOID_FORWARD;
		public static final int SHOOTER_BACKWARD;

		public static final int CLIMBER_MASTER_ID = 14;
		public static final int CLIMBER_FOLLOWER_ID = 15;
		
		public static final int TL_DRIVE_ID = 0;
		public static final int TL_ANGLE_ID = 1;

		public static final int TR_DRIVE_ID = 2;
		public static final int TR_ANGLE_ID = 3;

		public static final int BL_DRIVE_ID = 4;
		public static final int BL_ANGLE_ID = 5;

		public static final int BR_DRIVE_ID = 6;
		public static final int BR_ANGLE_ID = 7;

		public static final int PIGEON_ID = 0;

		public static final int CANCODER_ID = 0;
	}

	public static class PIPELINES {
		public static final int DAY_CLOSE = 0;
		public static final int DAY_MEDIUM = 1;
		public static final int DAY_FAR = 2;
		public static final int NIGHT_FAR = 3;
		public static final int NIGHT_CLOSE = 4;
	}
}