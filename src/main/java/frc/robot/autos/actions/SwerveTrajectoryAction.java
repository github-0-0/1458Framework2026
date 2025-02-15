package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.lib.trajectory.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
public class SwerveTrajectoryAction implements Action {
	private SwerveDrive mDrive = null;
	private TrajectoryIterator mTrajectory;
	private final PathPlannerTrajectory kTrajectory;
	private ResetWheelTracker mResetWheelTracker = ResetWheelTracker.NO;
	String name = null;
	public enum ResetWheelTracker {
		SET_TO_STARTING_POS,
		SET_TO_ZERO,
		NO
	};

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
		this(trajectory, ResetWheelTracker.NO);
	}

	public SwerveTrajectoryAction(String key, ResetWheelTracker resetPose) {
		this(TrajectoryGenerator.getInstance().getTrajectorySet().set.get(key),resetPose);
		name = key;
	}

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory, ResetWheelTracker resetPose) {
		kTrajectory = trajectory;
		mTrajectory = new TrajectoryIterator(trajectory);
		mDrive = SwerveDrive.getInstance();
		mResetWheelTracker = resetPose;
	}

	@Override
	public void start() {
		mTrajectory = new TrajectoryIterator(kTrajectory);
		switch(mResetWheelTracker){
			case SET_TO_ZERO:
				System.out.println("Reset to 0");
				mDrive.resetOdometry(new Pose2d());//use mDrive method instead of direct access mWheeltracker
				//mDrive.zeroGyro(0);
				break;
			case SET_TO_STARTING_POS:
				Pose2d newPose = mTrajectory.getState().poseMeters;
				System.out.println("Reset wheel tracker to pose: X: " + newPose.getX() + " Y: "+ newPose.getY()+ " Degrees: "+newPose.getRotation().getDegrees());
				mDrive.resetOdometry(newPose);
				/*
				double newRotation = mTrajectory.getState().poseMeters.getRotation().getDegrees();
				System.out.println("Reset gyro to " + newRotation);
				mDrive.zeroGyro(newRotation);
				*/
				
				break;
			case NO:
				break;
		}
		System.out.println("Swerve Trajectory Action Started! " + ((name != null) ? name : ""));
		mDrive.setTrajectory(mTrajectory);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTrajectory();
	}

	@Override
	public void done() {}
}
