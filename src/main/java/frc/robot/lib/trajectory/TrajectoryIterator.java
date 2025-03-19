package frc.robot.lib.trajectory;

import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.*;
import frc.robot.lib.util.Util;

import com.pathplanner.lib.trajectory.*;
//dc.10.21.2024, rewrite the TrajectoryIterator class based on wpilib Trajectory package, main functions as following
//
//2. rewrite the advance () method which is used often by its callers
//
public class TrajectoryIterator {
    protected double progress_ = 0.0;
    protected PathPlannerTrajectoryState current_sample_;     //corresponding to TrajectorySamplePoint in citrus code
    protected PathPlannerTrajectory mCurrentTrajectory=null;
    public ChassisSpeeds current_chassis_speeds;

    //construtor code
    public TrajectoryIterator (PathPlannerTrajectory curTrajectory){
        mCurrentTrajectory = curTrajectory;
        current_sample_ = curTrajectory.getStates().get(0);
        current_chassis_speeds = current_sample_.fieldSpeeds;
        //progress_ = view_.first_interpolant();        //dc. is it not zero??
    }

    //advance by additional time on the trajectory
    public Trajectory.State advance (double additional_progress){
//        System.out.println("soup "+additional_progress);
        progress_ = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        current_sample_ = mCurrentTrajectory.sample(progress_);
        current_chassis_speeds = current_sample_.fieldSpeeds;
        return fromPathPlannerTrajectoryState(current_sample_);
    }

    //preview the trajectory
    public Trajectory.State preview (double additional_progress){
        final double progress = Math.max(0.0, Math.min(mCurrentTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        return fromPathPlannerTrajectoryState(mCurrentTrajectory.sample(progress));
    }

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

    //access to the current trajectory properties
    public PathPlannerTrajectory trajectory(){ return mCurrentTrajectory;}

    //get the current state
    public Trajectory.State getState(){
        return fromPathPlannerTrajectoryState(current_sample_);
    }

    public static Trajectory.State fromPathPlannerTrajectoryState(PathPlannerTrajectoryState state) {
        Trajectory.State converted = new Trajectory.State(state.timeSeconds,state.linearVelocity,0,state.pose,state.fieldSpeeds.omegaRadiansPerSecond/state.linearVelocity);
        return converted;
    }

    //check if trajectory is reversed
    public boolean isReversed(){
		List<PathPlannerTrajectoryState> stateList = mCurrentTrajectory.getStates();
		for (int i = 0; i < stateList.size(); ++i) {
			if (stateList.get(i).linearVelocity > Util.kEpsilon) {
				return false; //dc.11.21.24, assume all states of the trajectory have the same direction
			} else if (stateList.get(i).linearVelocity  < -Util.kEpsilon) {
				return true;
			}		
        }
        return false;//this code shall never reach here. 
    }
}
