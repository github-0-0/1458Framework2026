package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.lib.trajectory.*;
import edu.wpi.first.math.trajectory.*;

public class SwerveTrajectoryAction implements Action {

	private SwerveDrive mDrive = null;

	private final TrajectoryIterator mTrajectory;
	private final boolean mResetGyro;

	public SwerveTrajectoryAction(Trajectory trajectory) {
		this(trajectory, false);
	}

	public SwerveTrajectoryAction(Trajectory trajectory, boolean resetPose) {
		mTrajectory = new TrajectoryIterator(trajectory);
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
