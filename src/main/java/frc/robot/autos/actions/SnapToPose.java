package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.*;

import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.util.Util;

public class SnapToPose implements Action {
    private PathPlannerPath generatedPath = null;

    private Pose2d initialPose = null;
    private Twist2d initialSpeed = null;
    
    private Pose2d finalPose = null;

    private PathPlannerTrajectory mTrajectory = null;
    private Action mAction = null;
    protected static boolean isRunning = false;    

    private boolean debug = true;

    /**
     * @param fieldRelativePose Field-relative pose to drive to
     */
    public SnapToPose(Pose2d fieldRelativePose) {
        finalPose = fieldRelativePose;
    }

    /**
     * @param fieldRelativePose Field-relative pose to drive to
     * @param initialPose Starting pose of the robot
     * @param initialSpeed Starting speeds of the robot
     */
    public SnapToPose(Pose2d fieldRelativePose, Pose2d initialPose, Twist2d initialSpeed) {
        finalPose = fieldRelativePose;
    }

    @Override
    public void start() {
        if (initialPose == null || initialSpeed == null) {
            getInitialState();
        }

		generatedPath = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				initialPose,
				finalPose
			), List.of(
				new RotationTarget(0.5, finalPose.getRotation())
			), List.of(), 
            List.of(
				new ConstraintsZone(0.5, Double.MAX_VALUE, 
					new PathConstraints(Constants.Swerve.maxSpeed, 
                                        Constants.Swerve.maxAcceleration / 4, 
                                        Constants.Swerve.maxAngularVelocity, 
                                        Constants.Swerve.kMaxAngularAcceleration)
				)
			), List.of(), 
			new PathConstraints(Constants.Swerve.maxSpeed, 
                                Constants.Swerve.maxAcceleration, 
                                Constants.Swerve.maxAngularVelocity, 
                                Constants.Swerve.kMaxAngularAcceleration), 
			new IdealStartingState(Util.twist2dMagnitude(initialSpeed), initialPose.getRotation()), 
			new GoalEndState(0, finalPose.getRotation()), false);

        if(generatedPath.getAllPathPoints().size() == 1) {
            mAction = null;
            System.out.println("Something goofy happened!");
            return;
        }

        mTrajectory = generatedPath.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();

        if (debug) {
            TrajectoryIterator debugIterator = new TrajectoryIterator(mTrajectory);
            debugIterator.visualizeTrajectory();
        }
		
        mAction = new SwerveTrajectoryAction(mTrajectory);
        System.out.println("Snapped to pose successfully");
    
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
        System.out.println("Done with snap");
        mAction.done();
    }

    private void getInitialState() {
        initialSpeed = RobotState.getInstance().getSmoothedVelocity();
        initialPose = RobotState.getInstance().getLatestFieldToVehicle();
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