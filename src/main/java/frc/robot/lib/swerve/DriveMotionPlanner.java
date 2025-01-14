package frc.robot.lib.swerve;

import java.util.List;

//dc.10.21.2024, rewrite citrus code using wpilib Trajectory classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.Constants;
import frc.robot.lib.control.Lookahead;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.util.Util;

public class DriveMotionPlanner {
	private static final double kMaxDx = 0.0127; // m
	private static final double kMaxDy = 0.0127; // m
	private static final double kMaxDTheta = Math.toRadians(1.0);

	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
	public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019)
	public static final double kAdaptivePathMinLookaheadDistance = 0.15;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.61;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

    //follower type
	public enum FollowerType {
		FEEDFORWARD_ONLY,
		PID,
		PURE_PURSUIT,
		RAMSETE
	}

	FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}

	private double defaultCook = 0.5;
	private boolean useDefaultCook = true;

	public void setDefaultCook(double new_value) {
		defaultCook = new_value;
	}

    TrajectoryIterator mCurrentTrajectory;

    double mDt = 0.0;   //delta of time
    boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
/* 	public TimedState<Pose2dWithMotion> mLastSetpoint = null;*/ //dc.10.21.2024, mLastSetpoint seems only to appear on left and zero right-side reference
	public Trajectory.State mSetpoint = new Trajectory.State(0.,0.,0.,new Pose2d(0.,0.,new Rotation2d(0)),0.);
	Pose2d mError = new Pose2d(0, 0, new Rotation2d(0));
/*  ErrorTracker mErrorTracker = new ErrorTracker(15 * 100);   */ //dc.10.21.2024, mErrorTracker seems only to appear on left and its getty function has zero reference. 
	Translation2d mTranslationalError = new Translation2d(0, 0);
	Rotation2d mPrevHeadingError = new Rotation2d(0);
	Pose2d mCurrentState = new Pose2d(0, 0, new Rotation2d(0));
	
	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();

	Lookahead mSpeedLookahead = null;

	// PID controllers for path following
	SynchronousPIDF mXPIDF;
	SynchronousPIDF mYPIDF;
	SynchronousPIDF mHeadingPIDF;

    //constructor code 
    public DriveMotionPlanner() {}

    //set trajectory to traverse
    public void setTrajectory(final TrajectoryIterator trajectory) {
        mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getState();
//		mLastSetpoint = null;
		useDefaultCook = true;
		mSpeedLookahead = new Lookahead(
				kAdaptivePathMinLookaheadDistance,
				kAdaptivePathMaxLookaheadDistance,
				0.0,
				Constants.SwerveConstants.maxAutoSpeed);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.trajectory().getTotalTimeSeconds();	//dc.11.21.24, replace citrus code = .getLastPoint().state().t();
		
		//check if trajectory is reversed
		List<Trajectory.State> stateList = trajectory.trajectory().getStates();
		for (int i = 0; i < stateList.size(); ++i) {
			if (stateList.get(i).velocityMetersPerSecond > Util.kEpsilon) {
				mIsReversed = false;
				break; //dc.11.21.24, assume all states of the trajectory have the same direction
			} else if (stateList.get(i).velocityMetersPerSecond  < -Util.kEpsilon) {
				mIsReversed = true;
				break;
			}
		}
    }

	public void reset() {
//		mErrorTracker.reset();
		mTranslationalError = new Translation2d();
		mPrevHeadingError = new Rotation2d();
//		mLastSetpoint = null;
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}

    // update chassis speeds at the specified timestamp based on current Pose2d and Velocity
	// return a robot-relative chassis_speeds
	public ChassisSpeeds update(double timestamp, Pose2d current_pose, Translation2d current_velocity) {
		if (mCurrentTrajectory == null) return null;
		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		Trajectory.State sample_point;  //dc. replace citrus TrajectorySamplePoint with wpilib trajectory.state
		mCurrentState = current_pose;
		Twist2d pid_error;	//twist2d between actual and desired states. 


		if (!isDone()) {
			
			// Compute error in robot frame
			mPrevHeadingError = mError.getRotation();
//			mError = current_pose.relativeTo(mSetpoint.poseMeters);
			mError = mSetpoint.poseMeters.relativeTo(current_pose);	//delta = mSetpoint - current_pose, in robot's local frame, dc.12.7.2024 bugfix, error shall = target - current
			pid_error = current_pose.log(mSetpoint.poseMeters);//* calculate the Twist2d delta/error between actual pose and the desired pose.  citrus original code is //Pose2d.log(mError);			
//dc.10.21.2024			mErrorTracker.addObservation(mError);
			if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.velocityMetersPerSecond;
				// Field relative
				/* 
				 *dc.10.22.2024 replace with code based on wpilib Trajectory.State
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_pose.getRotation().inverse().rotateBy(motion_direction);
				*/
				Rotation2d motion_direction = mSetpoint.poseMeters.getRotation(); // Get the planned course of motion from the trajectory (the robot's desired rotation)
				motion_direction = current_pose.getRotation().unaryMinus().rotateBy(motion_direction);

				mOutput = new ChassisSpeeds(
						motion_direction.getCos() * velocity_m,
						motion_direction.getSin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.curvatureRadPerMeter); //state().getHeadingRate());
			} else if (mFollowerType == FollowerType.RAMSETE) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;
				// mOutput = updateRamsete(sample_point.state(), current_pose, current_velocity);
			} else if (mFollowerType == FollowerType.PID) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.velocityMetersPerSecond;
				// Field relative
				/* 
				 *dc.10.22.2024 replace with code based on wpilib Trajectory.State
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_pose.getRotation().inverse().rotateBy(motion_direction);
				*/
				Rotation2d motion_direction = mSetpoint.poseMeters.getRotation(); // Get the planned course of motion from the trajectory (the robot's desired rotation)
				motion_direction = current_pose.getRotation().unaryMinus().rotateBy(motion_direction);

				var chassis_speeds = new ChassisSpeeds(
						motion_direction.getCos() * velocity_m,
						motion_direction.getSin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.curvatureRadPerMeter);
				// PID is in robot frame
				mOutput = updatePIDChassis(chassis_speeds, pid_error);
			} else if (mFollowerType == FollowerType.PURE_PURSUIT) {
				
				double searchStepSize = 1.0;	// these search steps are temporal, in seconds 
				double previewQuantity = 0.0;
				double searchDirection = 1.0;
				double forwardDistance = distance(current_pose, previewQuantity + searchStepSize);
				double reverseDistance = distance(current_pose, previewQuantity - searchStepSize);
				searchDirection = Math.signum(reverseDistance - forwardDistance);
				while (searchStepSize > 0.001) {
					SmartDashboard.putNumber("PurePursuit/PreviewDist(m)", distance(current_pose, previewQuantity));
					if (Util.epsilonEquals(distance(current_pose, previewQuantity), 0.0, 0.0003937)) break;
					while (distance(current_pose, previewQuantity + searchStepSize * searchDirection) 
							< distance(current_pose, previewQuantity)) { 		/* next trajectory point is closer to current_pose than current trajectory point */ 
						previewQuantity += searchStepSize * searchDirection;	/* continue to next point */
					}
					searchStepSize /= 10.0;	//further refine search steps
					searchDirection *= -1;
				}
				SmartDashboard.putNumber("PurePursuit/PreviewQuantity(s)", previewQuantity);
				
				sample_point = mCurrentTrajectory.advance(previewQuantity);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;
				mOutput = updatePurePursuit(current_pose, 0.0);
			}
		} else {
			if (mCurrentTrajectory.getLastPoint().velocityMetersPerSecond == 0.0) {
				mOutput = new ChassisSpeeds();
			}
		}

		return mOutput;
	}

    // check if we complete the current trajectory
    public boolean isDone() {
		if (mCurrentTrajectory!=null){
		}
		if(mCurrentTrajectory.isDone()){
		}
		return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
	}

	//dc. add Twist2d pid_error as input parameter to remove dependency on class property in original citrus code
	protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds, Twist2d pid_error) {
		// dc.10.22.2024, TODO: tune the "K" constants of PID algo
		// Feedback on longitudinal error (distance).
		final double kPathk = 2.4; // 2.4;/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
		final double kPathKTheta = 3.0;
		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
		return chassisSpeeds;
	}

	//dc 11.21.2024. adapting citrus PurePursuit algo using WPILib math classes
	protected ChassisSpeeds updatePurePursuit(Pose2d current_pose, double feedforwardOmegaRadiansPerSecond) {
		double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		Trajectory.State lookahead_state = mCurrentTrajectory.preview(lookahead_time);
		double actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityMetersPerSecond);
				//+ kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().getNorm(); //TODO: TB restored, dc.12.7.24, turn off mError compensation
		SmartDashboard.putNumber("PurePursuit/Error.dx", mError.getTranslation().getX());
		SmartDashboard.putNumber("PurePursuit/Error.dy", mError.getTranslation().getY());
		SmartDashboard.putNumber("PurePursuit/Error.Radians", mError.getRotation().getRadians());
		
		// Find the Point on the Trajectory that is Lookahead Distance Away
		while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;	//+10ms
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		}

		// If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead
		// distance away
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			Transform2d transform = new Transform2d(
					new Translation2d((mIsReversed ? -1.0 : 1.0)* (kPathMinLookaheadDistance - actual_lookahead_distance),0.0), 
					new Rotation2d());
			lookahead_state = new Trajectory.State(
					lookahead_state.timeSeconds,
					lookahead_state.velocityMetersPerSecond,
					lookahead_state.accelerationMetersPerSecondSq, 
					lookahead_state.poseMeters.transformBy(transform),
					lookahead_state.curvatureRadPerMeter);
			System.out.println("PurePursuit().lookahead_distance actual < adaptive happened at lookahead_state.timeSeconds=" + lookahead_state.timeSeconds);

		}
		SmartDashboard.putNumber("PurePursuit/ActualLookaheadDist(m)", actual_lookahead_distance);
		SmartDashboard.putNumber("PurePursuit/AdaptiveLookahead(m)", adaptive_lookahead_distance);
//		LogUtil.recordPose2d(
//				"PurePursuit/LookaheadState", lookahead_state.state().getPose());
		SmartDashboard.putNumber("PurePursuit/RemainingProgress(s)", mCurrentTrajectory.getRemainingProgress());
		SmartDashboard.putNumber("PurePursuit/PathVelocity(m/s)", lookahead_state.velocityMetersPerSecond);

		if (lookahead_state.velocityMetersPerSecond == 0.0) {
			mCurrentTrajectory.advance(Double.POSITIVE_INFINITY); //advance to the end of Trajectory
			return new ChassisSpeeds();
		}

		// Find the vector between robot's current position and the lookahead state = lookahead_state - current_pose
//		Translation2d lookaheadTranslation = current_pose.getTranslation().minus(lookahead_state.poseMeters.getTranslation());
		Translation2d lookaheadTranslation = lookahead_state.poseMeters.getTranslation().minus(current_pose.getTranslation());//dc.12.7.24, bugfix, flip the vector
		SmartDashboard.putNumber("PurePursuit/lookAheadTranslation.dx", lookaheadTranslation.getX());
		SmartDashboard.putNumber("PurePursuit/lookAheadTranslation.dy", lookaheadTranslation.getY());
		/* original citrus code = "new Translation2d(
				current_pose.getTranslation(), lookahead_state.state().getTranslation());"*/		

		// Set the steering direction as the direction of the vector
		Rotation2d steeringDirection = lookaheadTranslation.getAngle();// original citrus code = "".direction();"

		// Convert from field-relative steering direction to robot-relative = steeringDirection - current_pose.getRotation()
		steeringDirection = steeringDirection.rotateBy(Util.inversePose2d(current_pose).getRotation());//.inverse().getRotation());

		// Use the Velocity Feedforward of the Closest Point on the Trajectory
		double normalizedSpeed = Math.abs(mSetpoint.velocityMetersPerSecond) / Constants.SwerveConstants.maxAutoSpeed; 

		// The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot
		// will drive at the defaultCook speed
		if (normalizedSpeed > defaultCook || mSetpoint.timeSeconds > (mCurrentTrajectoryLength / 2.0)) {
			useDefaultCook = false;
		}
		if (useDefaultCook) {
			normalizedSpeed = defaultCook;
		}
		SmartDashboard.putNumber("PurePursuit/NormalizedSpeed", normalizedSpeed);

		// Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
		final Translation2d steeringVector =
				new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.getX() * Constants.SwerveConstants.maxAutoSpeed,
				steeringVector.getY() * Constants.SwerveConstants.maxAutoSpeed,
				feedforwardOmegaRadiansPerSecond);

/* 
		// Use the PD-Controller for To Follow the Time-Parametrized Heading
		final double kThetakP = 0.0; //TODO: TB restored, dc.12.7.24, turn off PD controller, citrus orignal value = 3.5;
		final double kThetakD = 0.0;
		final double kPositionkP = 0.0; //TODO: TB restored, dc.12.7.24, turn off PD controller,  citrus orignal value = 2.0;

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
				+ kPositionkP * mError.getTranslation().getX();
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
				+ kPositionkP * mError.getTranslation().getY();
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
				+ (kThetakP * mError.getRotation().getRadians())
				+ kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
*/
		return chassisSpeeds;
	}

	//getty functions to access key properties 
	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().getX(), mError.getTranslation().getY());
	}
	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}


	//dc.10.24.24, calculate the curve distance to travel measured by robot's odometry
	private double distance(Pose2d current_pose, double additional_progress) {
		Trajectory.State previewState = mCurrentTrajectory.preview(additional_progress);
		return distance(previewState.poseMeters, current_pose);
	}

	//dc.11.21.24, calculate curvature distance (LOCAL) travelled by robot from GLOBAL startPose to endPose
	private double distance (Pose2d startPose, Pose2d endPose){
		Twist2d twist = startPose.log(endPose); //calculate curvature delta, per say "endPose - startPose"
		// get the norm of twist2d object
		if (twist.dy == 0.0)
			return Math.abs(twist.dx);
		return Math.hypot(twist.dx, twist.dy);
	}
}
