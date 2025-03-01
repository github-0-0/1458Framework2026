package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.lib.util.Util;
public class SnapToTag implements Action {
    private PathPlannerPath generatedPath1 = null;
    private PathPlannerPath generatedPath2 = null;

    private Translation2d initialPosition = new Translation2d();
    private Rotation2d initialRotation = new Rotation2d();
    private double initialSpeed = 0.0;
    
    private Translation2d finalPosition = new Translation2d();
    private Rotation2d finalRotation = new Rotation2d();
    private double kFinalSpeed = 0.0;

    private PathPlannerTrajectory mTrajectory1 = null;
    private PathPlannerTrajectory mTrajectory2 = null;
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
        Translation2d intermediatePosition = poseBehind(new Pose2d(finalPosition, finalRotation), 0.5).getTranslation();
        generatedPath1 = new PathPlannerPath(
            List.of(
                new Waypoint(null,initialPosition,intermediatePosition),
                new Waypoint(initialPosition,intermediatePosition,null)
            ),
            new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, Constants.Swerve.maxAngularVelocity, Constants.Swerve.kMaxAngularAcceleration),
            new IdealStartingState(initialSpeed,initialRotation),
            new GoalEndState(kFinalSpeed,finalRotation)
        );      
        generatedPath2 = new PathPlannerPath(
            List.of(
                new Waypoint(null,intermediatePosition,finalPosition),
                new Waypoint(intermediatePosition,finalPosition,null)
            ),
            new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, Constants.Swerve.maxAngularVelocity, Constants.Swerve.kMaxAngularAcceleration),
            new IdealStartingState(initialSpeed,initialRotation),
            new GoalEndState(kFinalSpeed,finalRotation)
        );
        if(generatedPath1.getAllPathPoints().size() == 1) {
            mAction = null;
            System.out.println("Something goofy happened!");
            return;
        }
        mTrajectory1 = generatedPath1.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();
        mTrajectory2 = generatedPath2.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();
        mAction = new SeriesAction(new SwerveTrajectoryAction(mTrajectory1), new SwerveTrajectoryAction(mTrajectory2));
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
        System.out.println("Done w snap");
        mAction.done();
    }

    private void getInitialState() {
        initialSpeed = Util.twist2dMagnitude(RobotState.getInstance().getSmoothedVelocity()) / Constants.kLooperDt;
        initialRotation = RobotState.getInstance().getLatestFieldToVehicle().getRotation();
        initialPosition = RobotState.getInstance().getLatestFieldToVehicle().getTranslation();
    }
    
    private void getTagPosition() {
        boolean shouldFlip = false;
        tag = FieldLayout.getClosestTag(initialPosition).ID;
        for (int num : new int[] {1, 2, 12, 13}) {
            if (num == tag) {
                shouldFlip = true;
                break;
            }
        }
        Rotation2d aprilTagRotation = FieldLayout.getClosestTag(initialPosition).pose.getRotation().toRotation2d();//TODO: check if flipped 180 deg
        finalRotation = aprilTagRotation.minus(new Rotation2d(shouldFlip?0.0:Math.PI));
        finalPosition = FieldLayout.getClosestTag(initialPosition).pose.getTranslation().toTranslation2d().plus(FieldLayout.offsets[mNum].rotateBy(aprilTagRotation));
        
    }
    
    public Pose2d poseBehind(Pose2d pose, double n) { 
        return pose.transformBy(new Transform2d(new Translation2d(-n, 0), new Rotation2d())); 
    }

}