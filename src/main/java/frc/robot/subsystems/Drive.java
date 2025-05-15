package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.interpolation.InterpolatingPose2d;
import frc.robot.lib.drivers.Pigeon;
import frc.robot.lib.loops.ILooper;
import frc.robot.lib.loops.Loop;
import frc.robot.lib.swerve.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.lib.trajectory.TrajectoryIterator;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.config.PIDConstants;

public class Drive extends Subsystem {
	private static Drive mInstance;

	public static Drive getInstance() {
		if (mInstance == null) {
			mInstance = new Drive();
		}
		return mInstance;
	}


	private PeriodicIO mPeriodicIO = new PeriodicIO();
	public static class PeriodicIO {
		// Inputs/Desired States
		double timestamp;
		ChassisSpeeds des_chassis_speeds = new ChassisSpeeds();
		Twist2d measured_velocity = new Twist2d();
		Rotation2d heading = new Rotation2d();
		Rotation2d pitch = new Rotation2d();

		// Outputs
		SwerveModuleState[] des_module_states = new SwerveModuleState[] {
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
		};
		Twist2d predicted_velocity = new Twist2d();
		Translation2d translational_error = new Translation2d();
		Rotation2d heading_error = new Rotation2d();
		Rotation2d heading_setpoint = new Rotation2d();
	}

	private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;

	public enum DriveControlState {
		FORCE_ORIENT,
		OPEN_LOOP,
		HEADING_CONTROL,
		VELOCITY,
		PATH_FOLLOWING
	}

	public WheelTracker mWheelTracker;
	private Pigeon mPigeon = Pigeon.getInstance();
	public final Field2d mField = new Field2d();
	public final Field2d mAdvScopeField = new Field2d();
	public Module[] mModules;

	private Rotation2d mTrackingAngle = new Rotation2d();

	private final DriveController mMotionPlanner;
	private final SwerveHeadingController mHeadingController;

	private boolean odometryReset = false;
	private Translation2d enableFieldToOdom = null;
	private boolean mOverrideTrajectory = false;
	private boolean mOverrideHeading = false;

	private KinematicLimits mKinematicLimits = Constants.Swerve.UNCAPPED_KINEMATIC_LIMITS;

	private final StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SmartDashboard/Drive/States_Desired", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> measuredStatesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SmartDashboard/Drive/States_Measured", SwerveModuleState.struct).publish();
    private final StructPublisher<Rotation2d> rotationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Drive/Rotation", Rotation2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Drive/ChassisSpeeds", ChassisSpeeds.struct).publish();

	private Drive() {
		mModules = new Module[] {
			new Module(0, Constants.Swerve.FrontLeftMod.CONSTANTS, Cancoders.getInstance().getFrontLeft()),
			new Module(1, Constants.Swerve.FrontRightMod.CONSTANTS, Cancoders.getInstance().getFrontRight()),
			new Module(2, Constants.Swerve.BackLeftMod.CONSTANTS, Cancoders.getInstance().getBackLeft()),
			new Module(3, Constants.Swerve.BackRightMod.CONSTANTS, Cancoders.getInstance().getBackRight())
		};

		mMotionPlanner = new PIDHolonomicDriveController(
			new PIDConstants(Constants.Auto.X_CONTROLLER, 0.0, 0.), 
			new PIDConstants(Constants.Auto.THETA_CONTROLLER, 0.0, 0.0),
			Constants.Auto.THETA_CONTROLLER_CONSTRAINTS
		);

		mHeadingController = new SwerveHeadingController();

		mWheelTracker = new WheelTracker(mModules);

		SmartDashboard.putData("Field", mField);
		SmartDashboard.putData("Field2", mAdvScopeField);
	}

	public void setKinematicLimits(KinematicLimits newLimits) {
		this.mKinematicLimits = newLimits;
	}

	/**
	 * Updates drivetrain with latest desired speeds from the joystick, and sets DriveControlState appropriately.
	 *
	 * @param speeds a robot-relative ChassisSpeeds object derived from joystick input
	 */
	public void feedTeleopSetpoint(ChassisSpeeds speeds) {
		if (mControlState == DriveControlState.PATH_FOLLOWING) {
			if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)> mKinematicLimits.kMaxDriveVelocity * 0.05 || 
				Math.abs(speeds.omegaRadiansPerSecond)> 0.1 * Constants.Swerve.MAX_ANGULAR_VELOCITY) { 
				mControlState = DriveControlState.OPEN_LOOP;
			} else {
				return;
			}
		} else if (mControlState == DriveControlState.HEADING_CONTROL) {
			if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0*Constants.Swerve.MAX_ANGULAR_VELOCITY) { //original value = 0.1
				mControlState = DriveControlState.OPEN_LOOP;
				System.out.println("back to Open_Loop");

			} else {
				double x = speeds.vxMetersPerSecond;
				double y = speeds.vyMetersPerSecond;
				double omega = mHeadingController.update(mPeriodicIO.heading.getRadians(), Timer.getFPGATimestamp());
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(x, y, omega);
				SmartDashboard.putNumber("Drive/feedTeleop/StabilizingOmega =",omega);
				return;
			}
		} else if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		} else {	

		}

		mPeriodicIO.des_chassis_speeds = speeds;
	}

	public void setOpenLoop(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		}
	}

	public void setVelocity(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.VELOCITY) {
			mControlState = DriveControlState.VELOCITY;
		}
	}

	/**
	 * Instructs the drivetrain to stabilize heading around target angle.
	 *
	 * @param angle Target angle to stabilize around.
	 */
	public void stabilizeHeading(Rotation2d angle) {
		if (mControlState != DriveControlState.HEADING_CONTROL && mControlState != DriveControlState.PATH_FOLLOWING) {
			mControlState = DriveControlState.HEADING_CONTROL;
		}
		if (mHeadingController.getTargetHeadingRadians() != angle.getRadians()) {
			mHeadingController.setStabilizeTarget(angle.getRadians());
		}
	}

	/**
	 * Enable/disables vision heading control.
	 *
	 * @param value Whether or not to override rotation joystick with vision target.
	 */
	public synchronized void overrideHeading(boolean value) {
		mOverrideHeading = value;
	}

	/**
	 * Updates needed angle to track a goal.
	 *
	 * @param angle Sets the wanted robot heading to track a goal.
	 */
	public synchronized void feedTrackingSetpoint(Rotation2d angle) {
		mTrackingAngle = angle;
	}

	/**
	 * Stops modules in place.
	 */
	public synchronized void stopModules() {
		List<Rotation2d> orientations = new ArrayList<>();
		for (SwerveModuleState SwerveModuleState : mPeriodicIO.des_module_states) {
			orientations.add(SwerveModuleState.angle);
		}
		orientModules(orientations);
	}

	/**
	 * Orients modules to the angles provided.
	 * @param orientations Rotation2d of target angles, indexed by module number.
	 */
	public synchronized void orientModules(List<Rotation2d> orientations) {
		if (mControlState != DriveControlState.FORCE_ORIENT) {
			mControlState = DriveControlState.FORCE_ORIENT;
		}
		for (int i = 0; i < mModules.length; ++i) {
			mPeriodicIO.des_module_states[i] = new SwerveModuleState(0.0, orientations.get(i));
		}
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.start();
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Drive.this) {
					switch (mControlState) {
						case PATH_FOLLOWING:
							updatePathFollower();
							break;
						case HEADING_CONTROL:
							break;
						case OPEN_LOOP:
						case VELOCITY:
						case FORCE_ORIENT:
							break;
						default:
							stop();
							break;
					}
					updateSetpoint();

					RobotState.addOdometryUpdate(
									timestamp,
									new InterpolatingPose2d(mWheelTracker.getRobotPose()),
									mPeriodicIO.measured_velocity,
									mPeriodicIO.predicted_velocity);
					mField.setRobotPose(mWheelTracker.getRobotPose());//it works i think but i really cant tell
					Pose2d advScopeRotation = mWheelTracker.getRobotPose();
					advScopeRotation = new Pose2d(advScopeRotation.getTranslation(), advScopeRotation.getRotation().times(Math.PI/180));
					mAdvScopeField.setRobotPose(advScopeRotation);
				}
			}

			@Override
			public void onStop(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.stop();
				enableFieldToOdom = null;
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		for (Module swerveModule : mModules) {
			swerveModule.readPeriodicInputs();
		}

		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		mPeriodicIO.heading = mPigeon.getYaw();
		mPeriodicIO.pitch = mPigeon.getPitch();

		SwerveModuleState[] moduleStates = getModuleStates();
		Twist2d twist_vel = toTwist2d(Constants.Swerve.KINEMATICS
				.toChassisSpeeds(moduleStates));
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(getHeading());
		mPeriodicIO.measured_velocity = new Twist2d(
				translation_vel.getX(),
				translation_vel.getY(),
				twist_vel.dtheta);

		measuredStatesPublisher.set(moduleStates);
	}

	public synchronized void setTrajectory(TrajectoryIterator trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);
			mControlState = DriveControlState.PATH_FOLLOWING;
		}
	}

	/**
	 * @param pid_enable Switches between using PID control or Pure Pursuit control to follow trajectories.
	 */
	public synchronized void setUsePIDControl(boolean pid_enable) {
		
	}

	public synchronized boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null || mControlState != DriveControlState.PATH_FOLLOWING) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory;
	}

	/**
	 * If in the Path Following state, updates the
	 * DriveMotionPlanner and PeriodicIO path setpoint/error.
	 */
	private void updatePathFollower() {
		if (mControlState == DriveControlState.PATH_FOLLOWING) {
			ChassisSpeeds output = mMotionPlanner.calculate();
			if (output != null) {
				mPeriodicIO.des_chassis_speeds = output;
			}
		} else {
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	/**
	 * Updates the wanted setpoint, including whether heading should
	 * be overridden to the tracking angle. Also includes
	 * updates for Path Following.
	 */
	private void updateSetpoint() {
		if (mControlState == DriveControlState.FORCE_ORIENT) return;

		SmartDashboard.putNumber("Drive/ChassisSpeeds/Target/vx", mPeriodicIO.des_chassis_speeds.vxMetersPerSecond);
		SmartDashboard.putNumber("Drive/ChassisSpeeds/Target/vy", mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
		SmartDashboard.putNumber("Drive/ChassisSpeeds/Target/omega", mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond);

		Pose2d robot_pose_vel = new Pose2d(
				mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.LOOPER_DT * 4.0,
				mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.LOOPER_DT * 4.0,
				Rotation2d.fromRadians(
						mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.LOOPER_DT * 4.0));
		Twist2d twist_vel = Util.scaledTwist2d(Util.logMap(robot_pose_vel), 1.0 / (4.0 * Constants.LOOPER_DT));

		ChassisSpeeds wanted_speeds;
		if (mOverrideHeading) {
			System.out.println("updateSetPoint(): override heading is TRUE;");
			stabilizeHeading(mTrackingAngle);
			double new_omega = mHeadingController.update(mPeriodicIO.heading.getRadians(), Timer.getFPGATimestamp());
			ChassisSpeeds speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);
			wanted_speeds = speeds;
		} else {
			wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
		}

		if (mControlState != DriveControlState.PATH_FOLLOWING) {
			// Limit rotational velocity
			wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
					* Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

			// Limit translational velocity
			double velocity_magnitude = Math.hypot(
					mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
			if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
				wanted_speeds.vxMetersPerSecond =
						(wanted_speeds.vxMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
				wanted_speeds.vyMetersPerSecond =
						(wanted_speeds.vyMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
			}

			SwerveModuleState[] prev_module_states =
					mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
			ChassisSpeeds prev_chassis_speeds = Constants.Swerve.KINEMATICS.toChassisSpeeds(prev_module_states);
			SwerveModuleState[] target_module_states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(wanted_speeds);

			// Zero the modules' speeds if want_speeds is less epsilon value
			if (Util.chassisSpeedsEpsilonEquals(wanted_speeds, new ChassisSpeeds(), Util.kEpsilon)) {
				for (int i = 0; i < target_module_states.length; i++) {
					target_module_states[i].speedMetersPerSecond = 0.0;
					target_module_states[i].angle = prev_module_states[i].angle;
				}
			}

			double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
			double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
			double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

			double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.LOOPER_DT;
			double min_translational_scalar = 1.0;

			if (max_velocity_step < Double.MAX_VALUE * Constants.LOOPER_DT) {
				// Check X
				double x_norm = Math.abs(dx / max_velocity_step);
				min_translational_scalar = Math.min(min_translational_scalar, x_norm);

				// Check Y
				double y_norm = Math.abs(dy / max_velocity_step);
				min_translational_scalar = Math.min(min_translational_scalar, y_norm);

				min_translational_scalar *= max_velocity_step;
			}

			double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.LOOPER_DT;
			double min_omega_scalar = 1.0;

			if (max_omega_step < Double.MAX_VALUE * Constants.LOOPER_DT) {
				double omega_norm = Math.abs(domega / max_omega_step);
				min_omega_scalar = Math.min(min_omega_scalar, omega_norm);

				min_omega_scalar *= max_omega_step;
			}

			wanted_speeds = new ChassisSpeeds(
					prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
					prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
					prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);
		}


		SwerveModuleState[] real_module_setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(wanted_speeds);
   		
		SmartDashboard.putNumber("Drive/ChassisSpeeds/ToModule/Omega)", wanted_speeds.omegaRadiansPerSecond);
		SmartDashboard.putNumber("Drive/ChassisSpeeds/ToModule/vx)", wanted_speeds.vxMetersPerSecond);
		SmartDashboard.putNumber("Drive/ChassisSpeeds/ToModule/vy)", wanted_speeds.vyMetersPerSecond);

		SwerveDriveKinematics.desaturateWheelSpeeds(real_module_setpoints, Constants.Swerve.MAX_SPEED);

		Twist2d pred_twist_vel= new Twist2d(wanted_speeds.vxMetersPerSecond,wanted_speeds.vyMetersPerSecond,wanted_speeds.omegaRadiansPerSecond);
		mPeriodicIO.predicted_velocity =
				Util.logMap(Util.expMap(pred_twist_vel).rotateBy(getHeading()));
		mPeriodicIO.des_module_states = real_module_setpoints;
	}

	public void resetModulesToAbsolute() {
		for (Module module : mModules) {
			module.resetToAbsolute();
		}
	}

	public void zeroGyro(double reset_deg) {
		mPigeon.setYaw(reset_deg);
		enableFieldToOdom = null;
	}

	/**
	 * Configs if module drive motors should brake when commanded neutral output.
	 * @param brake Enable brake
	 */
	public void setNeutralBrake(boolean brake) {
		for (Module swerveModule : mModules) {
			swerveModule.setDriveNeutralBrake(brake);
		}
	}

	@Override
	public void writePeriodicOutputs() {
		for (int i = 0; i < mModules.length; i++) {
			if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
				mModules[i].setOpenLoop(mPeriodicIO.des_module_states[i]);
			} else if (mControlState == DriveControlState.PATH_FOLLOWING
					|| mControlState == DriveControlState.VELOCITY
					|| mControlState == DriveControlState.FORCE_ORIENT) {
				mModules[i].setVelocity(mPeriodicIO.des_module_states[i]);
			}
		}

		for (Module swerveModule : mModules) {
			swerveModule.writePeriodicOutputs();
		}

		if (Robot.isSimulation()) {
			for (Module swerveModule : mModules) {
				swerveModule.updateSimPeriodic();
			}
			mPigeon.updateSimPeriodic(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond);
		}

		mField.setRobotPose(mWheelTracker.getRobotPose());

		// Publish swerve module states and rotaton to smartdashboard
		SwerveModuleState[] other = new SwerveModuleState[4];
		for (int i = 0; i < mPeriodicIO.des_module_states.length; i++) {
			other[i] = mPeriodicIO.des_module_states[i];
			other[i].angle = mPeriodicIO.des_module_states[i].angle.unaryMinus();
		}

		desiredStatesPublisher.set(other);

		chassisSpeedsPublisher.set(mPeriodicIO.des_chassis_speeds);

		Rotation2d rotation = mWheelTracker.getRobotPose().getRotation();
		rotationPublisher.set(rotation);
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (Module mod : mModules) {
			states[mod.moduleNumber()] = mod.getState();
		}
		return states;
	}

	public Pose2d getPose() {
		return RobotState.getLatestFieldToVehicle();
	}

	public void resetOdometry(Pose2d pose) {
		odometryReset = true;
		Pose2d wanted_pose = pose;
		mWheelTracker.resetPose(wanted_pose);
		RobotState.addOdometryUpdate(
			Timer.getFPGATimestamp(), 
			new InterpolatingPose2d(mWheelTracker.getRobotPose()),
			mPeriodicIO.measured_velocity,mPeriodicIO.predicted_velocity
		);
	}

	public Rotation2d getHeading() {
		return mPigeon.getYaw();
	}

	public DriveController getMotionPlanner() {
		return mMotionPlanner;
	}

	public KinematicLimits getKinematicLimits() {
		return mKinematicLimits;
	}



	@Override
	public void outputTelemetry() {

	}

	@Override
	public synchronized void stop() {
		mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
		mControlState = DriveControlState.OPEN_LOOP;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	public static class KinematicLimits {
		public double kMaxDriveVelocity = Constants.Swerve.MAX_SPEED;
		public double kMaxAccel = Double.MAX_VALUE;
		public double kMaxAngularVelocity = Constants.Swerve.MAX_ANGULAR_VELOCITY;
		public double kMaxAngularAccel = Double.MAX_VALUE;
	}

	public Twist2d toTwist2d(ChassisSpeeds chassisSpeeds) {
		return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
	}
}
