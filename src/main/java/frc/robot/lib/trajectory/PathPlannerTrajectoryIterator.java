package frc.robot.lib.trajectory;

import java.util.List;
import edu.wpi.first.math.trajectory.*;

import edu.wpi.first.math.geometry.*;
import com.pathplanner.lib.trajectory.*;

//dc.10.21.2024, rewrite the TrajectoryIterator class based on wpilib Trajectory package, main functions as following
//
//2. rewrite the advance () method which is used often by its callers
//
public class PathPlannerTrajectoryIterator {
    protected double progress_ = 0.0;
    protected PathPlannerTrajectoryState current_PathPlannerSample_;     //corresponding to TrajectorySamplePoint in citrus code
    protected PathPlannerTrajectory mCurrentPathPlannerTrajectory=null;
    protected Trajectory.State current_sample_;
    protected PathPlannerTrajectoryState prev_PathPlannerSample_;
    //construtor code
    public PathPlannerTrajectoryIterator (PathPlannerTrajectory curTrajectory){
        mCurrentPathPlannerTrajectory=curTrajectory;
        current_PathPlannerSample_ = curTrajectory.getStates().get(0);
        prev_PathPlannerSample_ = current_PathPlannerSample_;
        //progress_ = view_.first_interpolant();        //dc. is it not zero??
    }

    //advance by additional time on the trajectory
    public Trajectory.State advance (double additional_progress){
        prev_PathPlannerSample_ = current_PathPlannerSample_;
        if(additional_progress == Double.POSITIVE_INFINITY){
        }
        progress_ = Math.max(0.0, Math.min(mCurrentPathPlannerTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        current_PathPlannerSample_ = mCurrentPathPlannerTrajectory.sample(progress_);
        current_sample_ = fromPathPlannerTrajectoryState(current_PathPlannerSample_,prev_PathPlannerSample_);
        System.out.println("advance");
        return current_sample_;
    }

    //preview the trajectory
    public Trajectory.State preview (double additional_progress){
        final double progress = Math.max(0.0, Math.min(mCurrentPathPlannerTrajectory.getTotalTimeSeconds(), progress_ + additional_progress));
        PathPlannerTrajectoryState future_PathPlannerSample_ = mCurrentPathPlannerTrajectory.sample(progress);
        Trajectory.State future_sample_ = fromPathPlannerTrajectoryState(future_PathPlannerSample_,oneBefore(future_PathPlannerSample_,progress));
        System.out.println("preview");
        return future_sample_;
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, mCurrentPathPlannerTrajectory.getTotalTimeSeconds() - progress_);
    }

    public Trajectory.State getLastPoint(){
        List<PathPlannerTrajectoryState> stateList = mCurrentPathPlannerTrajectory.getStates();
        PathPlannerTrajectoryState lastSample = stateList.get(stateList.size()-1);
        Trajectory.State lastPathPlannerSample = fromPathPlannerTrajectoryState(lastSample,oneBefore(lastSample,lastSample.timeSeconds));
        System.out.println("lastpoint");
        return lastPathPlannerSample;
    }

    //access to the current trajectory properties
    public PathPlannerTrajectory trajectory(){ return mCurrentPathPlannerTrajectory;}

    //get the current state
    public Trajectory.State getState(){
        System.out.println("getstate");
        return fromPathPlannerTrajectoryState(current_PathPlannerSample_,prev_PathPlannerSample_);
    }
    public Trajectory.State fromPathPlannerTrajectoryState(PathPlannerTrajectoryState first, PathPlannerTrajectoryState second) {
        double deltaTime = first.timeSeconds - second.timeSeconds;
        double curvature = first.heading.minus(second.heading).getRadians() / deltaTime;
        double acceleration = first.linearVelocity - second.linearVelocity;
        Trajectory.State converted = new Trajectory.State(first.timeSeconds,first.linearVelocity,acceleration,first.pose,curvature);
        return converted;
    }
    public PathPlannerTrajectoryState oneBefore(PathPlannerTrajectoryState state, double progress) {
        List<PathPlannerTrajectoryState> stateList = mCurrentPathPlannerTrajectory.getStates();//lobotomy time!
        int low = 1;
        int high = stateList.size() - 1;
        while (low != high) {
            int mid = (low + high) / 2;
            if (stateList.get(mid).timeSeconds < progress) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        int index = low;
        double deltaTime = state.timeSeconds - stateList.get(index).timeSeconds;
        PathPlannerTrajectoryState oneBefore = mCurrentPathPlannerTrajectory.sample(stateList.get(index - 1).timeSeconds + deltaTime);//basically, just 1 sample behind future pathplanner sample
        return oneBefore;
    }
}
