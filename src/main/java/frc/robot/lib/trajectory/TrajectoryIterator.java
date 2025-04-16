package frc.robot.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.*;

import com.pathplanner.lib.trajectory.*;

public class TrajectoryIterator {
    protected double progress_ = 0.0;
    protected PathPlannerTrajectoryState current_sample_;
    protected PathPlannerTrajectory mCurrentTrajectory = null;
    public ChassisSpeeds current_chassis_speeds;

    /**
     * Makes an iterator for a PathPlannerTrajectory
     * @param curTrajectory the trajectory
     */
    public TrajectoryIterator(PathPlannerTrajectory curTrajectory) {
        mCurrentTrajectory = curTrajectory;
        current_sample_ = curTrajectory.getStates().get(0);
        current_chassis_speeds = current_sample_.fieldSpeeds;
    }

    /**
     * Advances along the trajectory
     * @param additional_progress the time to advance in seconds
     * @return the state after advancing
     */
    public Trajectory.State advance(double additional_progress) {
        progress_ = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        current_sample_ = mCurrentTrajectory.sample(progress_);
        current_chassis_speeds = current_sample_.fieldSpeeds;
        return fromPathPlannerTrajectoryState(current_sample_);
    }

    /**
     * Previews a point along the trajectory
     * @param additional_progress the amount of seconds ahead
     * @return the state
     */
    public Trajectory.State preview(double additional_progress) {
        final double progress = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        return fromPathPlannerTrajectoryState(mCurrentTrajectory.sample(progress));
    }

    /**
     * Checks if done
     * @return wheter it is done
     */
    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, mCurrentTrajectory.getTotalTimeSeconds() - progress_);
    }

    public Trajectory.State getLastPoint(){
        List<PathPlannerTrajectoryState> stateList = mCurrentTrajectory.getStates();
        return fromPathPlannerTrajectoryState(stateList.get(stateList.size()-1));
    }

    public Trajectory.State getInitialState(){
        List<PathPlannerTrajectoryState> stateList = mCurrentTrajectory.getStates();
        return fromPathPlannerTrajectoryState(stateList.get(0));
    }

    public PathPlannerTrajectory trajectory() {
        return mCurrentTrajectory;
    }

    public Trajectory.State getState() {
        return fromPathPlannerTrajectoryState(current_sample_);
    }

    public static Trajectory.State fromPathPlannerTrajectoryState(PathPlannerTrajectoryState state) {
        Trajectory.State converted = new Trajectory.State(state.timeSeconds,state.linearVelocity,0,state.pose,state.fieldSpeeds.omegaRadiansPerSecond/state.linearVelocity);
        return converted;
    }

    public boolean isReversed() {
		List<PathPlannerTrajectoryState> stateList = mCurrentTrajectory.getStates();
		for (int i = 0; i < stateList.size(); ++i) {
			if (stateList.get(i).linearVelocity > Util.kEpsilon) {
				return false;
			} else if (stateList.get(i).linearVelocity  < -Util.kEpsilon) {
				return true;
			}		
        }
        return false;
    }


    public void visualizeTrajectory() {
        Drive mDrive = Drive.getInstance();
        SmartDashboard.putData(mDrive.m_field);
        ArrayList<Trajectory.State> temp = new ArrayList<>();
        for (PathPlannerTrajectoryState s : mCurrentTrajectory.getStates()) {
            temp.add(TrajectoryIterator.fromPathPlannerTrajectoryState(s));
        }
        mDrive.m_field.getObject("traj").setTrajectory(new Trajectory(temp));
    }
}
