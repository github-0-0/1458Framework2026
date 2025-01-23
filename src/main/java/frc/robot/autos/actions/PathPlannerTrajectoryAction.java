package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.lib.trajectory.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.*;

public class PathPlannerTrajectoryAction implements Action {

	private SwerveDrive mDrive = null;

	private final PathPlannerTrajectoryIterator mTrajectory;
	private final boolean mResetGyro;

	public PathPlannerTrajectoryAction(PathPlannerTrajectory trajectory) {
		this(trajectory, false);
	}

	public PathPlannerTrajectoryAction(PathPlannerTrajectory trajectory, boolean resetPose) {
		mTrajectory = new PathPlannerTrajectoryIterator(trajectory);
		mDrive = SwerveDrive.getInstance();
		mResetGyro = resetPose;
	}

	@Override
	public void start() {
		if (mResetGyro) {
			double newRotation = mTrajectory.getState().poseMeters.getRotation().getDegrees();
			System.out.println("Reset gyro to " + newRotation);
			mDrive.zeroGyro(newRotation);
		}
		System.out.println("Swerve Trajectory Action Started!");
		System.out.println(mTrajectory);
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
