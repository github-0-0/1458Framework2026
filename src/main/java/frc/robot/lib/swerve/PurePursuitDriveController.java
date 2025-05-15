package frc.robot.lib.swerve;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.control.Lookahead;
import frc.robot.lib.util.Util;

public class PurePursuitDriveController implements DriveController {
	private static final double kMaxDx = 0.0127;
	private static final double kMaxDy = 0.0127;
	private static final double kMaxDTheta = Math.toRadians(1.0);

	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1;
	public static final double kPathMinLookaheadDistance = 0.3;
	public static final double kAdaptivePathMinLookaheadDistance = 0.05;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.20;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;
	public static final double KPurePursuitMaxSearchRadius = 0.5;
	public static final double kPositionkP = 1.75;

	private double defaultCook = 0.2; 
	private boolean useDefaultCook = true;

    private TrajectoryIterator mCurrentTrajectory;

    private double mDt = 0.0;   //delta of time
	private double mLastTime = Double.POSITIVE_INFINITY;
	public Trajectory.State mSetpoint = new Trajectory.State();
	private Pose2d mError = new Pose2d();
	
	private double mCurrentTrajectoryLength = 0.0;
	private ChassisSpeeds mOutput = new ChassisSpeeds();

	private Lookahead mSpeedLookahead = null;

    //constructor code 
    public PurePursuitDriveController() {}

	public void setDefaultCook(double new_value) {
		defaultCook = new_value;
	}

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
				Constants.Swerve.MAX_AUTO_SPEED);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.trajectory().getTotalTimeSeconds();
    }

	@Override
	public void reset() {
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}


	@Override    
	/** update chassis speeds at the specified timestamp based on current Pose2d and Velocity
	 *  return a robot-relative chassis_speeds
	 * 
	 */
	public ChassisSpeeds calculate() {
		double timestamp = Timer.getFPGATimestamp();
		Pose2d current_pose = RobotState.getLatestFieldToVehicle();
		if (mCurrentTrajectory == null) return null;
		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		Trajectory.State sample_point;

		if (!isDone()) {
			mError = mSetpoint.poseMeters.relativeTo(current_pose);	
			SmartDashboard.putNumber("PurePursuit/mError.norm", mError.getTranslation().getNorm());
			sample_point = mCurrentTrajectory.advance(mDt);
			mSetpoint = sample_point;

			double velocity_m = mSetpoint.velocityMetersPerSecond;
			Rotation2d motion_direction = mSetpoint.poseMeters.getRotation(); // Get the planned course of motion from the trajectory (the robot's desired rotation)
			motion_direction = current_pose.getRotation().unaryMinus().rotateBy(motion_direction);

			mOutput = new ChassisSpeeds(
					motion_direction.getCos() * velocity_m,
					motion_direction.getSin() * velocity_m,
					velocity_m * mSetpoint.curvatureRadPerMeter);
		

			double searchStepSize =KPurePursuitMaxSearchRadius; 
			double previewQuantity = 0.0;
			double searchDirection = 1.0;
			double forwardDistance = distanceToTrajectory(current_pose, previewQuantity + searchStepSize);
			double reverseDistance = distanceToTrajectory(current_pose, previewQuantity - searchStepSize);
			searchDirection = Math.signum(reverseDistance - forwardDistance);
			while (searchStepSize > 0.001) {
				if (Util.epsilonEquals(distanceToTrajectory(current_pose, previewQuantity), 0.0, 0.0003937)) break; 
				while (distanceToTrajectory(current_pose, previewQuantity + searchStepSize * searchDirection) 
						< distanceToTrajectory(current_pose, previewQuantity)) { 
							previewQuantity += searchStepSize * searchDirection;	
				}					
				searchStepSize /= 10.0;	
				searchDirection *= -1;	
			}				

			SmartDashboard.putNumber("PurePursuit/PreviewQuantity(s)", previewQuantity);
			sample_point = mCurrentTrajectory.advance(previewQuantity);
			mSetpoint = sample_point;

			mOutput = updatePurePursuit(current_pose, 0.0);
		}

		return mOutput;
	}

    @Override
    public boolean isDone() {
		Pose2d currPose = RobotState.getLatestFieldToVehicle();
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

	/**
	 * Updates pure pursuit
	 * @param current_pose the current pose in meters
	 * @param feedforwardOmegaRadiansPerSecond the rotation speed feedforward
	 * @return
	 */
	protected ChassisSpeeds updatePurePursuit(Pose2d current_pose, double feedforwardOmegaRadiansPerSecond) {
		double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		Trajectory.State lookahead_state = mCurrentTrajectory.preview(lookahead_time);
		double actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityMetersPerSecond); 

		while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;	//+10ms
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = distance(mSetpoint.poseMeters, lookahead_state.poseMeters);
		}

		Translation2d lookaheadTranslation = lookahead_state.poseMeters.getTranslation().minus(current_pose.getTranslation());
		Rotation2d steeringDirection = lookaheadTranslation.getAngle();


		steeringDirection = steeringDirection.rotateBy(Util.inversePose2d(current_pose).getRotation());

		double normalizedSpeed = Math.abs(mSetpoint.velocityMetersPerSecond + lookahead_state.velocityMetersPerSecond) /2.0/ Constants.Swerve.MAX_AUTO_SPEED; 

		if (normalizedSpeed > defaultCook || mSetpoint.timeSeconds > mCurrentTrajectoryLength / 2) {
			useDefaultCook = false;
		}
		if (useDefaultCook) normalizedSpeed = defaultCook;
				
		Translation2d steeringVector =
				new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);

		Rotation2d currPoseRotationDelta = lookahead_state.poseMeters.getRotation().minus(current_pose.getRotation());
		SmartDashboard.putNumber("PurePursuit/CurrDelta",currPoseRotationDelta.getRadians());
		
		double trueOmegaRadiansPerSecond = (lookaheadTranslation.getNorm() > kAdaptivePathMinLookaheadDistance) ? 
			currPoseRotationDelta.getRadians() / lookaheadTranslation.getNorm() * normalizedSpeed * Constants.Swerve.MAX_AUTO_SPEED : 
			(mCurrentTrajectory.getRemainingProgress() > 0.0) ? currPoseRotationDelta.getRadians() / mCurrentTrajectory.getRemainingProgress() :
			mCurrentTrajectory.getLastPoint().poseMeters.getRotation().minus(current_pose.getRotation()).getRadians() * 3;
		trueOmegaRadiansPerSecond = Math.min(trueOmegaRadiansPerSecond, Constants.Swerve.MAX_ANGULAR_VELOCITY);		
		trueOmegaRadiansPerSecond = Math.max(trueOmegaRadiansPerSecond, -Constants.Swerve.MAX_ANGULAR_VELOCITY);

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.getX() * Constants.Swerve.MAX_AUTO_SPEED,
				steeringVector.getY() * Constants.Swerve.MAX_AUTO_SPEED,
				trueOmegaRadiansPerSecond);

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
				+ kPositionkP * mError.getTranslation().getX();
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
				+ kPositionkP * mError.getTranslation().getY();
		return chassisSpeeds;
	}

	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().getX(), mError.getTranslation().getY());
	}

	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}


	private double distanceToTrajectory(Pose2d current_pose, double additional_progress) {
		Trajectory.State previewState = mCurrentTrajectory.preview(additional_progress);
		return distance(previewState.poseMeters, current_pose);
	}

	private double distance (Pose2d startPose, Pose2d endPose){
		Twist2d twist = startPose.log(endPose);
		if (twist.dy == 0.0)
			return Math.abs(twist.dx);
		return Math.hypot(twist.dx, twist.dy);
	}
}
