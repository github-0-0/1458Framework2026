package frc.robot.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {
    public double mStartTime = Double.POSITIVE_INFINITY;
    public double mPrevTime = Double.POSITIVE_INFINITY;
    public double mCurrentTime = Double.POSITIVE_INFINITY;

    public Stopwatch() {
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
        mCurrentTime = mStartTime;
    }

    public void update() {
        mPrevTime = mCurrentTime;
        mCurrentTime = Timer.getFPGATimestamp();
    }

    public void reset() {
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
        mCurrentTime = mStartTime;
    }

    public double getTimeElapsed() {
        return mCurrentTime - mStartTime;
    }

    public double getDeltaTime() {
        return mCurrentTime - mPrevTime;
    }
}
