package frc.robot;

public class RobotMap {

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    public static final int PRIMARY_INDEX = 0;
    public static final int AUXILIARY_INDEX = 1;

	public static class CAN_IDS {
		public static final int BOTTOM_INTAKE_MOTOR_ID = -1;
		
		public static final int SHOOTER_MASTER_ID = -1;
		public static final int SHOOTER_FOLLOWER_ID = -1;

		public static final int SHOOTER_SOLENOID_FORWARD = -1;
		public static final int SHOOTER_SOLENOID_REVERSE = -1;

        public static final int HOPPER_TALON_ID = -1;
        
        //Drivetrain CAN ids.
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