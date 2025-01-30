package frc.robot.autos.actions;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.limelight.LimeLight;
import frc.robot.subsystems.vision.VisionDeviceManager;
import frc.robot.subsystems.vision.VisionPoseAcceptor;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.lib.trajectory.*;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;
public class SnapToLimelight implements Action {
	private SwerveDrive mDrive = null;
	private VisionDeviceManager mManager = null;
	private ResetWheelTracker mResetWheelTracker = ResetWheelTracker.NO;
	private PathPlannerPath generatedPath = null;
	public enum ResetWheelTracker {
		SET_TO_STARTING_POS,
		SET_TO_ZERO,
		NO
	};

	public SnapToLimelight(int tag, Pose2d relativePose) {
		mDrive = SwerveDrive.getInstance();
	}

	@Override
	public void start() {
		
	}

	@Override
	public void update() {

	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {}

	public Pose2d getPose(){
		return null;
	}
}
