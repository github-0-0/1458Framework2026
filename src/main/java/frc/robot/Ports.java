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
	public static final CanDeviceId FL_CANCODER = new CanDeviceId(7, "CV");

	public static final CanDeviceId FR_CANCODER = new CanDeviceId(6, "CV");

	public static final CanDeviceId BL_CANCODER = new CanDeviceId(14, "CV");

	public static final CanDeviceId BR_CANCODER = new CanDeviceId(1, "CV");

	public static final int PIGEON = 60; // TODO: this must be tuned to the specific robot
	
	public static final CanDeviceId LEDS = new CanDeviceId(21, "CV");

	public static final CanDeviceId LaserCanIDCoralBack = new CanDeviceId(30, "");
	public static final CanDeviceId LaserCanIDCoralFront = new CanDeviceId(31, "");
	public static final CanDeviceId LaserCanIDAlgae = new CanDeviceId(32, "");

	/* BEAM BREAK DIO CHANNELS*/
	public static final int SERIALIZER_BREAK = Constants.isEpsilon ? 7 : 8;
	public static final int FEEDER_BREAK = Constants.isEpsilon ? 8 : 7;
	public static final int AMP_BREAK = 9; 

	/* LINEAR SERVO PWM CHANNELS */
	public static final int CLIMBER_LINEAR_ACTUATOR = 9;
	public static final int ELEVATOR_LINEAR_ACTUATOR = 0;

	// spotless:on
}
