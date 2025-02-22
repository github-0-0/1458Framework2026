package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.SwerveDrive;
public class SnapToTag implements Action {
	private PathPlannerPath generatedPath = null;
	public SwerveDrive mDrive = SwerveDrive.getInstance();

	private Translation2d initialPosition = new Translation2d();
	private Rotation2d initialRotation = new Rotation2d();
	private double initialSpeed = 0.0;
	
	private Translation2d finalPosition = new Translation2d();
	private Rotation2d finalRotation = new Rotation2d();
	private double kFinalSpeed = 0.0;

	private PathPlannerTrajectory mTrajectory = null;
	private Action mAction = null;
	private int tag = 0;
	private int mNum = 0;
	protected static boolean isRunning = false;
	/**
	 * @param isLeft - if the robot is on the left side of the field
	 */
	public SnapToTag(int num) {
		mNum = num;
	}

    @Override
	public void start() {
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
		if(generatedPath.getAllPathPoints().size() == 1) {
			mAction = null;
			System.out.println("Something goofy happened!");
			return;
		}
		mTrajectory = generatedPath.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();

		// Create the trajectory to follow in autonomous. It is best to initialize
		// trajectories here to avoid wasting time in autonomous.
	
		// Create and push Field2d to SmartDashboard.

		SmartDashboard.putData(mDrive.m_field);
	
		// Push the trajectory to Field2d.
		ArrayList<Trajectory.State> temp = new ArrayList<>();
		for (PathPlannerTrajectoryState s : mTrajectory.getStates()) {
			temp.add(fromPathPlannerTrajectoryState(s));
		}
		mDrive.m_field.getObject("traj").setTrajectory(new Trajectory(temp));
		
		mAction = new SwerveTrajectoryAction(mTrajectory);
		System.out.println("Snap to tag "+new Pose2d(initialPosition,initialRotation).toString()+ " -> " + new Pose2d(finalPosition,finalRotation).toString());
	
		mAction.start();
	}

	@Override
	public void update() {
		mAction.update();
	}

	@Override
	public boolean isFinished() {
		if (mAction == null) {
			return true;
		}
		return mAction.isFinished();
	}

	@Override
	public void done() {
		if (mAction == null) {
			return;
		}
		mAction.done();
	}

	private void getInitialState() {
		initialSpeed = Util.twist2dMagnitude(RobotState.getInstance().getSmoothedVelocity()) / Constants.kLooperDt;
		initialRotation = RobotState.getInstance().getLatestFieldToVehicle().getRotation();
		initialPosition = RobotState.getInstance().getLatestFieldToVehicle().getTranslation();
	}
	
	private void getTagPosition() {
		boolean shouldFlip = false;
		tag = FieldLayout.getClosestTag(initialPosition);
		for (int num : new int[] {1, 2, 12, 13}) {
			if (num == tag) {
				shouldFlip = true;
				break;
			}
		}
		Rotation2d aprilTagRotation = FieldLayout.getClosestTagPos(initialPosition).getRotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(0));//TODO: check if flipped 180 deg
		finalRotation = aprilTagRotation.minus(new Rotation2d(shouldFlip?0.0:Math.PI));
		finalPosition = FieldLayout.getClosestTagPos(initialPosition).getTranslation().toTranslation2d().plus(FieldLayout.offsets[mNum].rotateBy(aprilTagRotation));
		
	}
	public Trajectory.State fromPathPlannerTrajectoryState(PathPlannerTrajectoryState state) {
        Trajectory.State converted = new Trajectory.State(state.timeSeconds,state.linearVelocity,0,state.pose,state.fieldSpeeds.omegaRadiansPerSecond/state.linearVelocity);
        return converted;
    }
}
