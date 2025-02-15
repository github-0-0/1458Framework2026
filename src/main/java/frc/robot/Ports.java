package frc.robot;

import frc.robot.lib.drivers.CanDeviceId;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 *
	 * Swerve Modules go:
	 * 0 1
	 * 2 3
	 *
	 * spotless:off
	 */

	/* DRIVETRAIN CAN DEVICE IDS */
//	public static final CanDeviceId FL_DRIVE = new CanDeviceId(0, "CV");
//	public static final CanDeviceId FL_ROTATION = new CanDeviceId(1, "CV");
	public static final CanDeviceId FL_CANCODER = new CanDeviceId(7, "CV");

//	public static final CanDeviceId FR_DRIVE = new CanDeviceId(2, "CV");
//	public static final CanDeviceId FR_ROTATION = new CanDeviceId(3, "CV");
	public static final CanDeviceId FR_CANCODER = new CanDeviceId(6, "CV");

//	public static final CanDeviceId BL_DRIVE = new CanDeviceId(4, "CV");
//	public static final CanDeviceId BL_ROTATION = new CanDeviceId(5, "CV");
	public static final CanDeviceId BL_CANCODER = new CanDeviceId(14, "CV");//dc.2.15.25, changed id to debug lasercan

//	public static final CanDeviceId BR_DRIVE = new CanDeviceId(6, "CV");
//	public static final CanDeviceId BR_ROTATION = new CanDeviceId(7, "CV");
	public static final CanDeviceId BR_CANCODER = new CanDeviceId(1, "CV");

//	public static final CanDeviceId EL_CANCODER = new CanDeviceId(13, "CV");
//	public static final CanDeviceId ER_CANCODER = new CanDeviceId(14, "CV");

	
	/* SUBSYSTEM CAN DEVICE IDS */
/*
	public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(8, "CV");
	public static final CanDeviceId INTAKE_ROLLER = new CanDeviceId(9, "rio");

	public static final CanDeviceId SERIALIZER = new CanDeviceId(10, "CV");
	public static final CanDeviceId FEEDER = new CanDeviceId(11, "CV");

	public static final CanDeviceId AMP_ROLLER = new CanDeviceId(12, "rio");

	public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(13, "CV");
	public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(14, "CV");

	public static final CanDeviceId SHOOTER_TOP = new CanDeviceId(15, "CV");
	public static final CanDeviceId SHOOTER_BOTTOM = new CanDeviceId(16, "CV");

	public static final CanDeviceId HOOD = new CanDeviceId(17, "CV");
	public static final CanDeviceId HOOD_CANCODER = new CanDeviceId(5, "CV");

	public static final CanDeviceId CLIMBER_MAIN = new CanDeviceId(18, "CV");
	public static final CanDeviceId CLIMBER_FOLLOWER = new CanDeviceId(19, "CV");
 */
	public static final int PIGEON = 60;//20
	
	public static final CanDeviceId LEDS = new CanDeviceId(21, "CV");

	public static final CanDeviceId LaserCanIDCoralBack = new CanDeviceId(30, "CV");
	public static final CanDeviceId LaserCanIDCoralFront = new CanDeviceId(31, "CV");
	public static final CanDeviceId LaserCanIDAlgae = new CanDeviceId(32, "CV");

	/* BEAM BREAK DIO CHANNELS*/
	public static final int SERIALIZER_BREAK = Constants.isEpsilon ? 7 : 8;
	public static final int FEEDER_BREAK = Constants.isEpsilon ? 8 : 7;
	public static final int AMP_BREAK = 9; 

	/* LINEAR SERVO PWM CHANNELS */
	public static final int CLIMBER_LINEAR_ACTUATOR = 9;
	public static final int ELEVATOR_LINEAR_ACTUATOR = 0;

	// spotless:on
}
