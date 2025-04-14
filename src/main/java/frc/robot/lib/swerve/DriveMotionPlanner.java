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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.control.Lookahead;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.util.Util;

public class DriveMotionPlanner implements DriveController {
	private static final double kMaxDx = 0.0127; // m
	private static final double kMaxDy = 0.0127; // m
	private static final double kMaxDTheta = Math.toRadians(1.0);

	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
	public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019), not used in team1458 code
	public static final double kAdaptivePathMinLookaheadDistance = 0.05;//0.05;//0.15;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.20;//0.2
	;//0.61;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;
	/*
	* dc.1.24.2025, bugfix: PurePursuit often stucks/oscilates around sharp turning point on trajectory
	* optimize search algo to avoid oscilation (cause by reverse advancement on trajectory) 
	* around sharp turning points of trajectory. Many methods have been tried. The most effective one is just to
	* reduce max search radius (searchStepSize initial value) for closest point on trajectory. 
	* Due to the significant impacts on PurePursuit bahavior, we introduce a constant value for it. 
	*/
	public static final double KPurePursuitMaxSearchRadius = 0.5;

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

	private double defaultCook = 0.2; //0.5;//bugfix: we need defaultCook to start robot on trajectory starting with zero speed
	private boolean useDefaultCook = true;

	public void setDefaultCook(double new_value) {
		defaultCook = new_value;
	}

    TrajectoryIterator mCurrentTrajectory;

    private double mDt = 0.0;   //delta of time
    private boolean mIsReversed = false;
	private double mLastTime = Double.POSITIVE_INFINITY;
	public Trajectory.State mSetpoint = new Trajectory.State(0.,0.,0.,new Pose2d(0.,0.,new Rotation2d(0)),0.);
	private Pose2d mError = new Pose2d(0, 0, new Rotation2d(0));
	private Translation2d mTranslationalError = new Translation2d(0, 0);
	private Rotation2d mPrevHeadingError = new Rotation2d(0);
	private Pose2d mCurrentState = new Pose2d(0, 0, new Rotation2d(0));
	
	private double mCurrentTrajectoryLength = 0.0;
	private double mTotalTime = Double.POSITIVE_INFINITY;
	private double mStartTime = Double.POSITIVE_INFINITY;
	private ChassisSpeeds mOutput = new ChassisSpeeds();

	private Lookahead mSpeedLookahead = null;

	// PID controllers for path following
	private SynchronousPIDF mXPIDF;
	private SynchronousPIDF mYPIDF;
	private SynchronousPIDF mHeadingPIDF;

    //constructor code 
    public DriveMotionPlanner() {}

    //set trajectory to traverse
    @Override
	public void setTrajectory(TrajectoryIterator trajectory) {
        mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getState();
		useDefaultCook = true;
		mSpeedLookahead = new Lookahead(
				kAdaptivePathMinLookaheadDistance,
				kAdaptivePathMaxLookaheadDistance,
				0.0,
				Constants.Swerve.maxAutoSpeed);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.trajectory().getTotalTimeSeconds();	//dc.11.21.24, replace citrus code = .getLastPoint().state().t();
		
		//check if trajectory is reversed
		mIsReversed = mCurrentTrajectory.isReversed();
    }

	@Override
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
	@Override
	public ChassisSpeeds calculate() {
		double timestamp = Timer.getFPGATimestamp();
		Pose2d current_pose = RobotState.getInstance().getLatestFieldToVehicle();
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
			SmartDashboard.putNumber("PurePursuit/mError.norm", mError.getTranslation().getNorm());
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
				//1. find the cloeset (optimal) point on trajectory to robot
				double searchStepSize =KPurePursuitMaxSearchRadius; // bugfix: reduce search radius to 0.5 from citrus code value =1.0, which causes lots of bugs in trajectory with sharp turning 
				double previewQuantity = 0.0;
				double searchDirection = 1.0;
				double forwardDistance = distanceToTrajectory(current_pose, previewQuantity + searchStepSize);
				double reverseDistance = distanceToTrajectory(current_pose, previewQuantity - searchStepSize);
				searchDirection = Math.signum(reverseDistance - forwardDistance);
				while (searchStepSize > 0.001) {
					if (Util.epsilonEquals(distanceToTrajectory(current_pose, previewQuantity), 0.0, 0.0003937)) break; //break if robot is right on the preview-pose of the trajectory
					while (distanceToTrajectory(current_pose, previewQuantity + searchStepSize * searchDirection) 
							< distanceToTrajectory(current_pose, previewQuantity)) { 		/* continue search if next step trajectory point is closer to current_pose than current trajectory point */ 
								previewQuantity += searchStepSize * searchDirection;	/* continue to next point */
					}					
					searchStepSize /= 10.0;	//reduce search step size by one factor
					searchDirection *= -1;	//reverse the search direction
				}				
				//2. advance trajectory sample, from which purepursuit algo will search for optimal look-ahead state
				SmartDashboard.putNumber("PurePursuit/PreviewQuantity(s)", previewQuantity);
				sample_point = mCurrentTrajectory.advance(previewQuantity);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				//3.calculate chassis speed for next movement via pure pursuit algo, 
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
	@Override
    public boolean isDone() {
		Pose2d currPose = RobotState.getInstance().getLatestFieldToVehicle();
		if (mCurrentTrajectory != null){
			if (mCurrentTrajectory.getRemainingProgress()<0.3){
				//check if currPose is really close the ending pose of traj for the last 300ms of the path 
				Trajectory.State endPoint = mCurrentTrajectory.getLastPoint();
				Pose2d delta = endPoint.poseMeters.relativeTo(currPose);	//delta = endPoint - current_pose, in robot's local frame, dc.12.7.2024 bugfix, error shall = target - current
				if (delta.getTranslation().getNorm() < 0.01 //less then 1cm
					&& Math.abs(delta.getRotation().getDegrees()) < 3){ //less than 3 degree
						return true;//consider traverse done, even if we have not reached the end of mCurrentTrajectory 
					}
			}
			return mCurrentTrajectory.isDone();
		}
		return false;
		// return mCurrentTrajectory != null && mCurrentTrajectory.isDone() && Math.abs(getHeadingError().getDegrees()) < 3;//TODO: put this magic number in constants, this could cause
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
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityMetersPerSecond); // (lookahead_range) * (speed - min_speed) / (speed_range) + min_lookahead
				//+ kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().getNorm(); //TODO: TB restored, dc.12.7.24, turn off mError compensation
		
		// try Points on Trajectory to approach max lookahead_distance (adaptive_lookahead_distance)
		while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;	//+10ms
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		}
		//dc.1.19.2025, i believe this is a bug in citrus code. When this happens, 
		//lookahead_time must >= trajectory.remainingProgress, .preview() shall return the last point to lookahead_state 
		//We DON'T need to translate its Pose, and robot shall converge to it
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			/*
			Transform2d transform = new Transform2d(
					new Translation2d((mIsReversed ? -1.0 : 1.0)* (kPathMinLookaheadDistance - actual_lookahead_distance),0.0), 
					new Rotation2d());
			lookahead_state = new Trajectory.State(
					lookahead_state.timeSeconds,
					lookahead_state.velocityMetersPerSecond,
					lookahead_state.accelerationMetersPerSecondSq, 
					lookahead_state.poseMeters.transformBy(transform),//do we need this? or just use the pose of end point on trajectory?
					lookahead_state.curvatureRadPerMeter);
			*/
		}
		SmartDashboard.putNumber("PurePursuit/ActualLookaheadDist(m)", actual_lookahead_distance);
		SmartDashboard.putNumber("PurePursuit/AdaptiveLookahead(m)", adaptive_lookahead_distance);
//		LogUtil.recordPose2d(
//				"PurePursuit/LookaheadState", lookahead_state.state().getPose());
		SmartDashboard.putNumber("PurePursuit/RemainingProgress(s)", mCurrentTrajectory.getRemainingProgress());

		//dc.1.19.2025, i believe this is a bug in citrus code, it causes path follower stop too earlier before the end of trajectory 
		//even when lookahead_state.velocity is zero, i believe robot shall continue forward from its current position to lookahead_state
		//it will take quite some cycles for robot actually reach the end of path.
		if (lookahead_state.velocityMetersPerSecond == 0.0) {
			/* 
			mCurrentTrajectory.advance(Double.POSITIVE_INFINITY); //advance to the end of Trajectory
			return new ChassisSpeeds(); 
			*/
		}


		// Find the vector between robot's current position and the lookahead state = lookahead_state - current_pose
		Translation2d lookaheadTranslation = lookahead_state.poseMeters.getTranslation().minus(current_pose.getTranslation());//dc.12.7.24, bugfix, flip the vector
		SmartDashboard.putNumber("PurePursuit/Pose/current.y",current_pose.getY());
		SmartDashboard.putNumber("PurePursuit/Pose/lookahead.y",lookahead_state.poseMeters.getY());
		/* original citrus code = "new Translation2d(
				current_pose.getTranslation(), lookahead_state.state().getTranslation());"*/		

		// Set the steering direction as the direction of the vector
		Rotation2d steeringDirection = lookaheadTranslation.getAngle();// original citrus code = "".direction();"

		// Convert from field-relative steering direction to robot-relative = steeringDirection - current_pose.getRotation()
		steeringDirection = steeringDirection.rotateBy(Util.inversePose2d(current_pose).getRotation());//.inverse().getRotation());
		SmartDashboard.putNumber("PurePursuit/SteeringDirection",steeringDirection.getDegrees());
		SmartDashboard.putNumber("PurePursuit/lookaheadTranslation.x",lookaheadTranslation.getX());
		SmartDashboard.putNumber("PurePursuit/lookaheadTranslation.y",lookaheadTranslation.getY());

		// use avergae of scalar speed of current and lookahead sample points as the driving speed. 
		// No directions used here as we just want to make sure the scalar value of the speed 
		// Assume scalar speed on the trajectory maintain continuity 
		double normalizedSpeed = Math.abs(mSetpoint.velocityMetersPerSecond + lookahead_state.velocityMetersPerSecond) /2.0/ Constants.Swerve.maxAutoSpeed; 

		// Use Default Cook at the begining of the trajectory until path speed exceeds it or robot progresses far enough 
		// It is also bugfix for zero speed at the beginning of trajectory
		// The applicable progress length is calculated based on that robot reached max speed no later than the middle point of path
		// For the rest of the path, use path predefined speed will provide smooth movement
		if (normalizedSpeed > defaultCook || mSetpoint.timeSeconds > mCurrentTrajectoryLength / 2) {
			useDefaultCook = false;
		}
		if (useDefaultCook) {normalizedSpeed = defaultCook;	}
				
		// Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
		final Translation2d steeringVector =
				new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);

		// dc.1.22.2025, finally let's 'compensate the rotation speed in pursuiting lookahead state
		// method1. compensate omega speed from current trajectory state with angle error of current pose 
//		Rotation2d currPoseRotationDelta = current_pose.getRotation().minus(mSetpoint.poseMeters.getRotation());//dc.1.22.2025, add code to support pose rotation on the trajectory
//		double deltaOmegaRadiansPerSecond = (lookaheadTranslation.getNorm() > Util.kEpsilon )? currPoseRotationDelta.getRadians() / lookaheadTranslation.getNorm() * Math.abs(mSetpoint.velocityMetersPerSecond) : 0.0;
//		double deltaOmegaRadiansPerSecond = (lookaheadTranslation.getNorm() > Util.kEpsilon )? currPoseRotationDelta.getRadians() / lookaheadTranslation.getNorm() * normalizedSpeed*Constants.Swerve.maxAutoSpeed: 0.0;

		// method2. calc omega directly from angle difference between current pose and lookahead state
		Rotation2d currPoseRotationDelta = lookahead_state.poseMeters.getRotation().minus(current_pose.getRotation());//dc.1.22.2025, 2nd methods for calc oemga
		SmartDashboard.putNumber("PurePursuit/CurrDelta",currPoseRotationDelta.getRadians());
		
		double trueOmegaRadiansPerSecond = (lookaheadTranslation.getNorm() > kAdaptivePathMinLookaheadDistance) ? 
			currPoseRotationDelta.getRadians() / lookaheadTranslation.getNorm() * normalizedSpeed * Constants.Swerve.maxAutoSpeed : 
			(mCurrentTrajectory.getRemainingProgress() > 0.0) ? currPoseRotationDelta.getRadians() / mCurrentTrajectory.getRemainingProgress() :
			mCurrentTrajectory.getLastPoint().poseMeters.getRotation().minus(current_pose.getRotation()).getRadians() * 3 ;//TODO: put magic number in constants
		trueOmegaRadiansPerSecond = Math.min(trueOmegaRadiansPerSecond, Constants.Swerve.maxAngularVelocity);		
		trueOmegaRadiansPerSecond = Math.max(trueOmegaRadiansPerSecond, -Constants.Swerve.maxAngularVelocity);
		SmartDashboard.putNumber("PurePursuit/OmegaRadiansPerSecond",trueOmegaRadiansPerSecond);
		SmartDashboard.putNumber("PurePursuit/Heading.Error", current_pose.getRotation().minus(mSetpoint.poseMeters.getRotation()).getDegrees());

		SmartDashboard.putNumber("PurePursuit/steeringVector.vx", steeringVector.getX());		
		SmartDashboard.putNumber("PurePursuit/steeringVector.vy", steeringVector.getY());		
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.getX() * Constants.Swerve.maxAutoSpeed,
				steeringVector.getY() * Constants.Swerve.maxAutoSpeed,
				// mSetpoint.curvatureRadPerMeter*mSetpoint.velocityMetersPerSecond);				
//				mSetpoint.curvatureRadPerMeter*mSetpoint.velocityMetersPerSecond - deltaOmegaRadiansPerSecond);		
				trueOmegaRadiansPerSecond);

		//output debug info as we get close to the end of path
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			Trajectory.State endPoint = mCurrentTrajectory.getLastPoint();
			System.out.println("PurePursuit() remaining (s) =" + mCurrentTrajectory.getRemainingProgress() 
				+ ", err.distance=" + current_pose.relativeTo(endPoint.poseMeters).getTranslation().getNorm()
				+ ", err.angle=" + currPoseRotationDelta.getDegrees() 
				// + ", lookahead=" + lookaheadTranslation.getNorm()
//				+ ", OmegaRPS orig=" + mSetpoint.curvatureRadPerMeter*mSetpoint.velocityMetersPerSecond 
//				+ ", OmegaRPS comp=" + deltaOmegaRadiansPerSecond
				// + ", OmegaRPS =" + trueOmegaRadiansPerSecond
				);

		
		}			
		
		
 
		// Use the PD-Controller for To Follow the Time-Parametrized Heading
		final double kThetakP = 0.0; //TODO: TB restored, dc.12.7.24, turn off PD controller, citrus orignal value = 3.5;
		final double kThetakD = 0.0;
		final double kPositionkP =1.75; //TODO: TB restored, dc.12.7.24, turn off PD controller,  citrus orignal value = 2.0;

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
				+ kPositionkP * mError.getTranslation().getX();
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
				+ kPositionkP * mError.getTranslation().getY();
/*		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
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


	//dc.10.24.24, calculate the curve distance from current pose (from PoseEstimator) to the desired pose of the preview position on the trajectory 
	private double distanceToTrajectory(Pose2d current_pose, double additional_progress) {
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
