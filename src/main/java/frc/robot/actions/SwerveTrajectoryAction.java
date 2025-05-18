package frc.robot.actions;

import frc.robot.subsystems.Drive;
import frc.robot.RobotState;
import frc.robot.lib.trajectory.*;
import java.util.Optional;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveTrajectoryAction implements Action {
	private static final double K_EPSILON = 0.05; //Meters per second
	private static Drive mDrive;

	private TrajectoryIterator mTrajectoryIterator;
	private final PathPlannerTrajectory kTrajectory;

	private String name = null;

	public SwerveTrajectoryAction(String key) {
		PathPlannerTrajectory pTraj = TrajectoryGenerator
				.getInstance().getTrajectorySet().loadPathPlannerTrajectory(key);
		Optional<Alliance> ally = RobotState.getAlliance();
		if (ally.isPresent() && ally.get() == Alliance.Red) {
			kTrajectory = pTraj.flip();
		} else {
			kTrajectory = pTraj;
		}
		name = key;
	}

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
		kTrajectory = trajectory;
	}

	@Override
	public void start() {
		mTrajectoryIterator = new TrajectoryIterator(kTrajectory);
		System.out.println("Swerve Trajectory Action Started! " + ((name != null) ? name : ""));
		mDrive = Drive.getInstance();
		mDrive.setTrajectory(mTrajectoryIterator);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTrajectory();
	}

	@Override
	public void done() {
		if (mTrajectoryIterator.getLastPoint().velocityMetersPerSecond < K_EPSILON) {
			mDrive.stop();
		}
	}
}
