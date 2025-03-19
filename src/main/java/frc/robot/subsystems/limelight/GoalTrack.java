package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import java.util.TreeMap;

public class GoalTrack {
    final GoalTracker.Configuration mConfiguration;
    TreeMap<Double, Translation2d> mObservedPositions = new TreeMap<>();
    Translation2d mSmoothedPosition = null;
    int mId;

    private GoalTrack(GoalTracker.Configuration config) {
        mConfiguration = config;
    }

    public synchronized void emptyUpdate() {
        pruneByTime();
    }

    public synchronized boolean tryUpdate(double timestamp, Translation2d new_observation) {
        if (!isAlive()) {
            return false;
        }
        Translation2d distanceVector = mSmoothedPosition.minus(new_observation);
        double distance = distanceVector.getNorm(); 
        if (distance < mConfiguration.kMaxTrackerDistance) {
            mObservedPositions.put(timestamp, new_observation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
    }

    public synchronized boolean isAlive() {
        return !mObservedPositions.isEmpty();
    }

    synchronized void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - mConfiguration.kMaxGoalTrackAge;
        mObservedPositions.entrySet().removeIf(entry -> entry.getKey() < delete_before);
        if (mObservedPositions.isEmpty()) {
            mSmoothedPosition = null;
        } else {
            smooth();
        }
    }

    public static GoalTrack makeNewTrack(
            GoalTracker.Configuration config, double timestamp, Translation2d first_observation, int id) {
        GoalTrack rv = new GoalTrack(config);
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        rv.mId = id;
        return rv;
    }

    synchronized void smooth() {
        if (isAlive()) {
            double x = 0;
            double y = 0;
            double t_now = Timer.getFPGATimestamp();
            int num_samples = 0;
            for (Map.Entry<Double, Translation2d> entry : mObservedPositions.entrySet()) {
                if (t_now - entry.getKey() > mConfiguration.kMaxGoalTrackSmoothingTime) {
                    continue;
                }
                ++num_samples;
                x += entry.getValue().getX();
                y += entry.getValue().getY();
            }
            x /= num_samples;
            y /= num_samples;

            if (num_samples == 0) {
                mSmoothedPosition = mObservedPositions.lastEntry().getValue();
            } else {
                mSmoothedPosition = new Translation2d(x, y);
            }
        }
    }

    public synchronized Translation2d getSmoothedPosition() {
        return mSmoothedPosition;
    }

    public synchronized Translation2d getLatestPosition() {
        return mObservedPositions.lastEntry().getValue();
    }

    public synchronized double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public synchronized double getStability() {
        return Math.min(
                1.0, mObservedPositions.size() / (mConfiguration.kCameraFrameRate * mConfiguration.kMaxGoalTrackAge));
    }

    public synchronized int getId() {
        return mId;
    }
}
