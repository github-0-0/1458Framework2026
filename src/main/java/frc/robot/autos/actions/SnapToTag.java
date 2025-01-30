package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.lib.util.Util;
public class SnapToTag implements Action {
	private PathPlannerPath generatedPath = null;

	private Translation2d initialPosition = new Translation2d();
	private Rotation2d initialRotation = new Rotation2d();
	private double initialSpeed = 0.0;
	
	private Translation2d finalPosition = new Translation2d();
	private Rotation2d finalRotation = new Rotation2d();
	private double kFinalSpeed = 0.0;

	private PathPlannerTrajectory mTrajectory = null;
	private Action mAction = null;
	private Translation2d offset;
	/**
	 * @param isLeft - if the robot is on the left side of the field
	 */
	public SnapToTag(boolean isLeft) {
		getInitialState();
		getTagPosition();
		generatedPath = new PathPlannerPath(
			List.of(
				new Waypoint(null,initialPosition,finalPosition),
				new Waypoint(initialPosition,finalPosition,null)
			),
			new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, Constants.Swerve.maxAngularVelocity, Constants.Swerve.kMaxAngularAcceleration),
			new IdealStartingState(initialSpeed,initialRotation),
			new GoalEndState(kFinalSpeed,finalRotation)
		);
		mTrajectory = generatedPath.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();
		mAction = new SwerveTrajectoryAction(mTrajectory);
	}


	@Override
	public void start() {
		mAction.start();
	}

	@Override
	public void update() {
		mAction.update();
	}

	@Override
	public boolean isFinished() {
		return mAction.isFinished();
	}

	@Override
	public void done() {
		mAction.done();
	}

	private void getInitialState() {
		initialSpeed = Util.twist2dMagnitude(RobotState.getInstance().getSmoothedVelocity()) / Constants.kLooperDt;
		initialRotation = RobotState.getInstance().getLatestFieldToVehicle().getRotation();
		initialPosition = RobotState.getInstance().getLatestFieldToVehicle().getTranslation();
	}
	
	private void getTagPosition() {
		Rotation2d aprilTagRotation = FieldLayout.getClosestTagPos(initialPosition).getRotation().toRotation2d();//TODO: check if flipped 180 deg
		finalRotation = aprilTagRotation.minus(new Rotation2d(Math.PI));
		finalPosition = FieldLayout.getClosestTagPos(initialPosition).getTranslation().toTranslation2d().plus(offset.rotateBy(aprilTagRotation));
	}
}
