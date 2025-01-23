package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WheelTracker;
import frc.robot.lib.trajectory.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.*;

public class SwerveTrajectoryAction implements Action {
	private SwerveDrive mDrive = null;
	private WheelTracker mWheelTracker = null;

	private final TrajectoryIterator mTrajectory;
	private ResetWheelTracker mResetWheelTracker = ResetWheelTracker.NO;
	public enum ResetWheelTracker {
		SET_TO_STARTING_POS,
		SET_TO_ZERO,
		NO
	};

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
		this(trajectory, ResetWheelTracker.NO);
	}

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory, ResetWheelTracker resetPose) {
		mTrajectory = new TrajectoryIterator(trajectory);
		mDrive = SwerveDrive.getInstance();
		mWheelTracker = mDrive.mWheelTracker;
		mResetWheelTracker = resetPose;
	}

	@Override
	public void start() {
		switch(mResetWheelTracker){
			case SET_TO_ZERO:
				System.out.println("Reset to 0");
				mWheelTracker.resetPose(new Pose2d());
				//mDrive.zeroGyro(0);
				break;
			case SET_TO_STARTING_POS:
				Pose2d newPose = mTrajectory.getState().poseMeters;
				System.out.println("Reset wheel tracker to pose: X: " + newPose.getX() + " Y: "+ newPose.getY()+ " Degrees: "+newPose.getRotation().getDegrees());
				mWheelTracker.resetPose(newPose);
				
				double newRotation = mTrajectory.getState().poseMeters.getRotation().getDegrees();
				System.out.println("Reset gyro to " + newRotation);
				mDrive.zeroGyro(newRotation);
				
				break;
			case NO:
				break;
		}
		System.out.println("Swerve Trajectory Action Started!");
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
