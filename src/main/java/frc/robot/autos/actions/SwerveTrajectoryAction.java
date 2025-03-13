package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.lib.trajectory.*;

import java.util.Optional;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
public class SwerveTrajectoryAction implements Action {
	private SwerveDrive mDrive = null;
	private TrajectoryIterator mTrajectory;
	private final PathPlannerTrajectory kTrajectory;
	private ResetWheelTracker mResetWheelTracker = ResetWheelTracker.NO;
	String name = null;
	public enum ResetWheelTracker {
		SET_TO_STARTING_POS,
		SET_TO_ZERO,
		NO
	};

	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
		this(trajectory, ResetWheelTracker.NO);
	}
	
	public SwerveTrajectoryAction(String key) {
		this(key, ResetWheelTracker.NO);
	}
/*
	public SwerveTrajectoryAction(String key, ResetWheelTracker resetPose) {
		this(TrajectoryGenerator.getInstance().getTrajectorySet().set.get(key),resetPose);
		name = key;
	}
*/
	public SwerveTrajectoryAction(String key, ResetWheelTracker resetPose) {
		PathPlannerTrajectory pTraj = TrajectoryGenerator.getInstance().getTrajectorySet().loadPathPlannerTrajectory(key);
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent() && ally.get() == Alliance.Red) {
			kTrajectory = pTraj.flip();
			System.out.println("flip for running red side trajectory");
		}else{
			kTrajectory = pTraj;
		}
		mDrive = SwerveDrive.getInstance();
		mResetWheelTracker = resetPose;
		name = key;
	}
	//
	//dc.2.23.25, all constructors MUST call this ultimate constructor.
	//1. we will flip the trajectory for red side alliance here. 
	//2. will init properties of the object
	//TODO: we shal remove mResetWheelTracker and related code.  WheelTracker is reset at proper time during mode initialization  
	//
	public SwerveTrajectoryAction(PathPlannerTrajectory trajectory, ResetWheelTracker resetPose) {
		kTrajectory = trajectory;
		mDrive = SwerveDrive.getInstance();
		mResetWheelTracker = resetPose;
	}

	@Override
	public void start() {
		mTrajectory = new TrajectoryIterator(kTrajectory);
		switch(mResetWheelTracker){
			case SET_TO_ZERO:
				if (Robot.isSimulation()) {
					System.out.println("Reset to 0");
					mDrive.resetOdometry(new Pose2d());//use mDrive method instead of direct access mWheeltracker
					mDrive.zeroGyro(0);
				}
				break;
			case SET_TO_STARTING_POS:
				if (Robot.isSimulation()) {
					Pose2d newPose = mTrajectory.getState().poseMeters;
					System.out.println("Reset wheel tracker to pose: X: " + newPose.getX() + " Y: "+ newPose.getY()+ " Degrees: "+newPose.getRotation().getDegrees());
					mDrive.resetOdometry(newPose);
				}

				/*
				double newRotation = mTrajectory.getState().poseMeters.getRotation().getDegrees();
				System.out.println("Reset gyro to " + newRotation);
				mDrive.zeroGyro(newRotation);
				*/
				
				break;
			case NO:
				break;
		}
		System.out.println("Swerve Trajectory Action Started! " + ((name != null) ? name : ""));
		mDrive.setTrajectory(mTrajectory);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTrajectory();
	}

	@Override
	public void done() {
		mDrive.stop();
	}
}
