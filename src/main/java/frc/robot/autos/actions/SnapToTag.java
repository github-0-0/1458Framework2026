package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.SwerveDrive;
public class SnapToTag implements Action {
    private PathPlannerPath generatedPath = null;
	private SwerveDrive mDrive = SwerveDrive.getInstance();

    private Pose2d initialPose = new Pose2d();
    private Twist2d initialSpeed = new Twist2d();
    
    private Pose2d finalPose = new Pose2d();
	private boolean shouldFlip = false;

    private PathPlannerTrajectory mTrajectory = null;
    private Action mAction = null;
    private int tag = 0;
    private String mOffset;
    protected static boolean isRunning = false;
    
    /**
     * @param offset LEFTBAR, RIGHTBAR, CENTER, CS, HANG
     */
    public SnapToTag(String offset) {
        mOffset = offset;
    }

    @Override
    public void start() {
        getInitialState();
        getTagPosition();
		Pose2d intermediatePose = poseBehind(
            finalPose,
            ((shouldFlip) ? -1 : 1) * 
            (Math.min(
                0.5,
                initialPose.minus(finalPose).
                getTranslation().getNorm() / 2)
            )
        );
		generatedPath = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				initialPose,
				//intermediatePose,
				finalPose
			), List.of(
				new RotationTarget(0.5, finalPose.getRotation())
			), List.of(), 
            List.of(
				new ConstraintsZone(0.5, 15, 
					new PathConstraints(Constants.Swerve.maxSpeed, 
                                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
                                        Constants.Swerve.maxAngularVelocity/4, 
                                        Constants.Swerve.kMaxAngularAcceleration/2)
				)
			), List.of(), 
			new PathConstraints(Constants.Swerve.maxSpeed, 
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
                                Constants.Swerve.maxAngularVelocity/2, 
                                Constants.Swerve.kMaxAngularAcceleration), 
			new IdealStartingState(Util.twist2dMagnitude(initialSpeed), initialPose.getRotation()), 
			new GoalEndState(0, finalPose.getRotation()), false);
        if(generatedPath.getAllPathPoints().size() == 1) {
            mAction = null;
            System.out.println("Something goofy happened!");
            return;
        }

        mTrajectory = generatedPath.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();

		// SmartDashboard.putData(mDrive.m_field);
		// ArrayList<Trajectory.State> temp = new ArrayList<>();
		// for (PathPlannerTrajectoryState s : mTrajectory.getStates()) {
		// 	temp.add(TrajectoryIterator.fromPathPlannerTrajectoryState(s));
		// }
		// mDrive.m_field.getObject("traj").setTrajectory(new Trajectory(temp));
		
        mAction = new SwerveTrajectoryAction(mTrajectory);
        System.out.println("Snap to tag "+toString()+ " -> " + toString());
    
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
        System.out.println("Done w snap");
        mAction.done();
    }

    private void getInitialState() {
        initialSpeed = RobotState.getInstance().getSmoothedVelocity();
        initialPose = RobotState.getInstance().getLatestFieldToVehicle();
    }
    
    private void getTagPosition() {
        tag = FieldLayout.getClosestTag(initialPose.getTranslation()).ID;

        for (int num : new int[] {1, 2, 12, 13}) {
            if (num == tag) {
                shouldFlip = true;
                break;
            }
        }

        Rotation2d aprilTagRotation = FieldLayout.getClosestTag(initialPose.getTranslation()).pose.getRotation().toRotation2d();//TODO: check if flipped 180 deg
        finalPose = new Pose2d(
            FieldLayout.getClosestTag(initialPose.getTranslation()).pose
            .getTranslation().toTranslation2d().
            plus(FieldLayout.offsets.get(mOffset).rotateBy(aprilTagRotation)),
            aprilTagRotation.minus(new Rotation2d(shouldFlip ? 0.0 : Math.PI)));

    }
    
    public Pose2d poseBehind(Pose2d pose, double n) { 
        return pose.plus(new Transform2d(new Translation2d(-n, 0), new Rotation2d())); 
    }

	public Waypoint withControls(Double prevControlLength, Double prevControlHeading, Translation2d anchor, Double nextControlLength, Double nextControlHeading) {
		Translation2d prevControlPoint = null;
		Translation2d nextControlPoint = null;

		if (prevControlLength != null && prevControlHeading != null) 
			prevControlPoint = anchor.plus(new Translation2d(prevControlLength, Rotation2d.fromDegrees(prevControlHeading)));
		if (nextControlLength != null && nextControlHeading != null)
			nextControlPoint = anchor.plus(new Translation2d(nextControlLength, Rotation2d.fromDegrees(nextControlHeading)));
		return new Waypoint(prevControlPoint, anchor, nextControlPoint);
	}
}