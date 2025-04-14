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
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.Drive;
public class SnapToTag implements Action {
    private PathPlannerPath generatedPath = null;
	private Drive mDrive = Drive.getInstance();

    private Pose2d initialPose = new Pose2d();
    private Twist2d initialSpeed = new Twist2d();
    
    private Pose2d finalPose = new Pose2d();
	private boolean shouldFlip = false;

    private PathPlannerTrajectory mTrajectory = null;
    private Action mAction = null;
    private AprilTag tag = null;
    private String mOffset;
    private String mPreset;
    protected static boolean isRunning = false;    

    private boolean debug = true;
    
    public static HashMap<String, Translation2d> offsets = new HashMap<>();
	public static HashMap<String, int[]> presets = new HashMap<>();

    private static final int[] flipIDs = new int[] {};


	static {
        offsets.put("CENTER",
            new Translation2d(Constants.Swerve.trackWidth/2 + 0.20,0.0));
    
        presets.put("ANY", new int[] {});
	}
    
    /**
     * @param offset CENTER
     */
    public SnapToTag(String offset) {
        this(offset, "ANY");
    }

    /**
     * @param offset CENTER
     * @param preset ANY
     */
    public SnapToTag(String offset, String preset) {
        mOffset = offset;
        mPreset = preset;
    }

    @Override
    public void start() {
        getInitialState();
        getTagPosition();

		generatedPath = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				initialPose,
				finalPose
			), List.of(
				new RotationTarget(0.5, finalPose.getRotation())
			), List.of(), 
            List.of(
				new ConstraintsZone(0.5, 15, 
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
            SmartDashboard.putData(mDrive.m_field);
            ArrayList<Trajectory.State> temp = new ArrayList<>();
            for (PathPlannerTrajectoryState s : mTrajectory.getStates()) {
                temp.add(TrajectoryIterator.fromPathPlannerTrajectoryState(s));
            }
            mDrive.m_field.getObject("traj").setTrajectory(new Trajectory(temp));
        }
		
        mAction = new SwerveTrajectoryAction(mTrajectory);
        System.out.println("Snap to tag -> " + tag);
    
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
        tag = getClosestTag(initialPose.getTranslation(), mPreset);

        for (int num : flipIDs) {
            if (num == tag.ID) {
                shouldFlip = true;
                break;
            }
        }

        Rotation2d aprilTagRotation = tag.pose.toPose2d().getRotation();
        finalPose = new Pose2d(
            tag.pose.getTranslation().toTranslation2d().
            plus(offsets.get(mOffset).rotateBy(aprilTagRotation)),
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
    /**
	 * Default is any
	 * @param robot_position
	 * @return
	 */
	public static AprilTag getClosestTag(Translation2d robot_position) {
		return getClosestTag(robot_position, presets.get("ANY"));
	}
	/**
	 * 
	 * @param robot_position The current robot position
	 * @param ids Positive for whitelist, negative for blacklist
	 * @return The closest apriltag on the field
	 */
	public static AprilTag getClosestTag(Translation2d robot_position, int[] ids) {
		AprilTag closest_tag = null;
		double closest_distance = Double.MAX_VALUE;

		for (AprilTag tag : FieldLayout.kTagMap.getTags()) {
			double distance = robot_position.getDistance(tag.pose.getTranslation().toTranslation2d());

			for (int num : ids) {
				int absNum = Math.abs(num);
				if (tag.ID == absNum) {
					if (num == absNum) {
						distance -= 1000000000;
						break; //hopefully there is no situation where this is insufficient
					} else {
						distance = Double.MAX_VALUE;
						break;
					}
				}
			}

			if (distance < closest_distance) {
				closest_tag = tag;
				closest_distance = distance;
			}
		}

		return closest_tag;
	}

	public static AprilTag getClosestTag(Translation2d robot_position, String preset) {
		return getClosestTag(robot_position, presets.get(preset));
	}
}