package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * This is used in the event that multiple goals are detected to judge all goals
 * based on timestamp, stability, and
 * continuation of previous goals (i.e. if a goal was detected earlier and has
 * changed locations). This allows the robot
 * to make consistent decisions about which goal to aim at and to smooth out
 * jitter from vibration of the camera.
 */
public class GoalTracker {
    public static class TrackReport {
        public Translation2d field_to_target;
        public double latest_timestamp;
        public double stability;
        public int id;

        public TrackReport(GoalTrack track) {
            this.field_to_target = track.getSmoothedPosition();
            this.latest_timestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.id = track.getId();
        }
    }

    public static class TrackReportComparator implements Comparator<TrackReport> {
        final Configuration mConfiguration;
        double mCurrentTimestamp;
        int mLastTrackId;

        protected TrackReportComparator(Configuration config, int last_track_id, double current_timestamp) {
            this.mConfiguration = config;
            this.mLastTrackId = last_track_id;
            this.mCurrentTimestamp = current_timestamp;
        }

        double score(TrackReport report) {
            double stability_score = mConfiguration.kStabilityWeight * report.stability;
            double age_score = mConfiguration.kAgeWeight
                    * Math.max(
                            0,
                            (mConfiguration.kMaxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp))
                                    / mConfiguration.kMaxGoalTrackAge);
            double switching_score = (report.id == mLastTrackId ? mConfiguration.kSwitchingWeight : 0);
            return stability_score + age_score + switching_score;
        }

        @Override
        public int compare(TrackReport o1, TrackReport o2) {
            double diff = score(o1) - score(o2);
            return Double.compare(diff, 0);
        }
    }

    public static class Configuration {
        public double kMaxTrackerDistance = 0.0;
        public double kMaxGoalTrackAge = 1.0; 
        public double kMaxGoalTrackSmoothingTime = 0.1; 
        public double kCameraFrameRate = 90.0; 

        public double kStabilityWeight = 1.0;
        public double kAgeWeight = 1.0;
        public double kSwitchingWeight = 1.0;
    }

    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId = 0;
    final Configuration mConfiguration;

    public GoalTracker(Configuration config) {
        mConfiguration = config;
    }

    public synchronized void reset() {
        mCurrentTracks.clear();
    }

    public synchronized TrackReportComparator getComparator(int last_track_id, double timestamp) {
        return new TrackReportComparator(mConfiguration, last_track_id, timestamp);
    }

    public synchronized void update(double timestamp, List<Translation2d> field_to_goals) {
        for (Translation2d target : field_to_goals) {
            boolean hasUpdatedTrack = false;
            for (GoalTrack track : mCurrentTracks) {
                if (!hasUpdatedTrack) {
                    if (track.tryUpdate(timestamp, target)) {
                        hasUpdatedTrack = true;
                    }
                } else {
                    track.emptyUpdate();
                }
            }
            if (!hasUpdatedTrack) {
                mCurrentTracks.add(GoalTrack.makeNewTrack(mConfiguration, timestamp, target, mNextId));
                ++mNextId;
            }
        }

        maybePruneTracks();
    }

    public synchronized void maybePruneTracks() {
        mCurrentTracks.removeIf(track -> !track.isAlive());
    }

    public synchronized boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public synchronized List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }
}
