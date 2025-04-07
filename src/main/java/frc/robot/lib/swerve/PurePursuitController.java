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
import frc.robot.lib.util.SynchronousTranslationPIDF;
import frc.robot.lib.util.Util;

public class PurePursuitController implements DriveController {
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
	private SynchronousTranslationPIDF mTranslationPIDF;
	private SynchronousPIDF mHeadingPIDF;

    //constructor code 
    public PurePursuitController() {}

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
				Constants.SwerveConstants.maxAutoSpeed);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.trajectory().getTotalTimeSeconds();	//dc.11.21.24, replace citrus code = .getLastPoint().state().t();
		mTranslationPIDF = new SynchronousTranslationPIDF(0.3, 0.00001, 0.01);
		mHeadingPIDF = new SynchronousPIDF(10, 0.0001, 0.1);
		mHeadingPIDF.setContinuous(true);
		mHeadingPIDF.setInputRange(-Math.PI, Math.PI);
		mHeadingPIDF.setOutputRange(-Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);
		
		mIsReversed = mCurrentTrajectory.isReversed();
    }

	public void reset() {
		mTranslationalError = new Translation2d();
		mPrevHeadingError = new Rotation2d();
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}

    /**
	 * Calculates the next ChassisSpeeds on the trajectory
	 * @return the new ChassisSpeeds
	 */
	@Override
	public ChassisSpeeds calculate() {
		double timestamp = Timer.getFPGATimestamp();
		Pose2d current_pose = RobotState.getInstance().getLatestFieldToVehicle();
		if (mCurrentTrajectory == null) return null;
		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		Trajectory.State sample_point;
		mCurrentState = current_pose;


		if (!isDone()) {
			mPrevHeadingError = mError.getRotation();
			mError = mSetpoint.poseMeters.relativeTo(current_pose);	
			SmartDashboard.putNumber("PurePursuit/mError.norm", mError.getTranslation().getNorm());
			SmartDashboard.putNumber("PurePursuit/mError.degrees", mError.getRotation().getDegrees());
			double searchStepSize = KPurePursuitMaxSearchRadius;
			double previewQuantity = 0.0;
			double searchDirection = 1.0;
			double forwardDistance = distanceToTrajectory(current_pose, previewQuantity + searchStepSize);
			double reverseDistance = distanceToTrajectory(current_pose, previewQuantity - searchStepSize);
			searchDirection = Math.signum(reverseDistance - forwardDistance);
			while (searchStepSize > 0.001) {
				if (Util.epsilonEquals(distanceToTrajectory(current_pose, previewQuantity), 0.0, 0.0003937)) break; 
				while (distanceToTrajectory(current_pose, previewQuantity + searchStepSize * searchDirection) 
						< distanceToTrajectory(current_pose, previewQuantity)) { 		
							previewQuantity += searchStepSize * searchDirection;	/* continue to next point */
				}					
				searchStepSize /= 10.0;	
				searchDirection *= -1;	
			}				
			sample_point = mCurrentTrajectory.advance(previewQuantity);
			mSetpoint = sample_point;

			mOutput = updatePurePursuit(current_pose, 0.0);
		}

		return mOutput;
	}

    // check if we complete the current trajectory
	@Override
    public boolean isDone() {
		Pose2d currPose = RobotState.getInstance().getLatestFieldToVehicle();
		if (mCurrentTrajectory != null){
			if (mCurrentTrajectory.getRemainingProgress()<0.3){
				Trajectory.State endPoint = mCurrentTrajectory.getLastPoint();
				Pose2d delta = endPoint.poseMeters.relativeTo(currPose);	
				if (delta.getTranslation().getNorm() < 0.01 
					&& Math.abs(delta.getRotation().getDegrees()) < 3){ 
						return true;
					}
			}
			return mCurrentTrajectory.isDone();
		}
		return false;
	}

	protected ChassisSpeeds updatePurePursuit(Pose2d current_pose, double feedforwardOmegaRadiansPerSecond) {
		double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		Trajectory.State lookahead_state = mCurrentTrajectory.preview(lookahead_time);
		double actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityMetersPerSecond); // (lookahead_range) * (speed - min_speed) / (speed_range) + min_lookahead
		while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;	//+10ms
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		}
		
		// Find the vector between robot's current position and the lookahead state = lookahead_state - current_pose
		Translation2d lookaheadTranslation = lookahead_state.poseMeters.getTranslation().minus(current_pose.getTranslation());//dc.12.7.24, bugfix, flip the vector
		// Set the steering direction as the direction of the vector
		Rotation2d steeringDirection = lookaheadTranslation.getAngle();// original citrus code = "".direction();"

		// Convert from field-relative steering direction to robot-relative = steeringDirection - current_pose.getRotation()
		steeringDirection = steeringDirection.rotateBy(Util.inversePose2d(current_pose).getRotation());//.inverse().getRotation());
	
		double normalizedSpeed = Math.abs(mSetpoint.velocityMetersPerSecond + lookahead_state.velocityMetersPerSecond) /2.0/ Constants.SwerveConstants.maxAutoSpeed; 

		// Use Default Cook at the begining of the trajectory until path speed exceeds it or robot progresses far enough 
		// It is also bugfix for zero speed at the beginning of trajectory
		// The applicable progress length is calculated based on that robot reached max speed no later than the middle point of path
		// For the rest of the path, use path predefined speed will provide smooth movement
		if (normalizedSpeed > defaultCook || mSetpoint.timeSeconds > mCurrentTrajectoryLength / 2) {
			useDefaultCook = false;
		}
		if (useDefaultCook) {normalizedSpeed = defaultCook;	}
				
		// Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
		Translation2d steeringVector =
				new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);
		mTranslationPIDF.setSetpoint(mSetpoint.poseMeters.getTranslation());
		steeringVector = steeringVector.plus(mTranslationPIDF.calculate(current_pose.getTranslation()));


		Rotation2d currPoseRotationDelta = lookahead_state.poseMeters.getRotation().minus(current_pose.getRotation());//dc.1.22.2025, 2nd methods for calc oemga
		double trueOmegaRadiansPerSecond = 0;
		mHeadingPIDF.setSetpoint(lookahead_state.poseMeters.getRotation().getRadians());
		trueOmegaRadiansPerSecond = mHeadingPIDF.calculate(current_pose.getRotation().getRadians());
		// if (lookaheadTranslation.getNorm() > kAdaptivePathMinLookaheadDistance) {
		// 	trueOmegaRadiansPerSecond = currPoseRotationDelta.getRadians() / lookaheadTranslation.getNorm() * normalizedSpeed * Constants.SwerveConstants.maxAutoSpeed;
		// } else { 
		// 	if (mCurrentTrajectory.getRemainingProgress() > 0.0) {
		// 		trueOmegaRadiansPerSecond = currPoseRotationDelta.getRadians() / mCurrentTrajectory.getRemainingProgress(); 
		// 	} else {
		// 		trueOmegaRadiansPerSecond = mCurrentTrajectory.getLastPoint().poseMeters.getRotation().minus(current_pose.getRotation()).getRadians() * 3;
		// 	} 
		// }
		
		trueOmegaRadiansPerSecond = Math.min(trueOmegaRadiansPerSecond, Constants.Swerve.maxAngularVelocity);		
		trueOmegaRadiansPerSecond = Math.max(trueOmegaRadiansPerSecond, -Constants.Swerve.maxAngularVelocity);
		
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.getX() * Constants.SwerveConstants.maxAutoSpeed,
				steeringVector.getY() * Constants.SwerveConstants.maxAutoSpeed,
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
		
        System.out.println("Using version 2.0");
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
