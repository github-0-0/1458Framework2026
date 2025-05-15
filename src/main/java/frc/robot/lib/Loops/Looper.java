package frc.robot.lib.loops;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
	public final double kPeriod;

	private boolean mRunning;

	private final Notifier mNotifier;
	private final List<Loop> mLoops;
	private final Object mTaskRunningLock = new Object();
	private double mTimestamp = 0;
	private double mDt = 0;

	public double getDt() {
		return mDt;
	}

	private final CrashTrackingRunnable mRunnable = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (mTaskRunningLock) {
				if (mRunning) {
					double now = Timer.getFPGATimestamp();

					for (Loop loop : mLoops) {
						loop.onLoop(now);
					}

					mDt = now - mTimestamp;
					mTimestamp = now;
				}
			}
		}
	};

	public Looper(double loopTime) {
		mNotifier = new Notifier(mRunnable);
		mRunning = false;
		mLoops = new ArrayList<>();
		kPeriod = loopTime;
	}

	public Looper() {
		this(Constants.LOOPER_DT);
	}

	@Override
	public synchronized void register(Loop loop) {
		synchronized (mTaskRunningLock) {
			mLoops.add(loop);
		}
	}

	public synchronized void start() {
		if (!mRunning) {
			System.out.println("Starting loops");
			synchronized (mTaskRunningLock) {
				mTimestamp = Timer.getFPGATimestamp();
				for (Loop loop : mLoops) {
					loop.onStart(mTimestamp);
				}
				mRunning = true;
			}
			mNotifier.startPeriodic(kPeriod);
		}
	}

	public synchronized void stop() {
		if (mRunning) {
			System.out.println("Stopping loops");
			mNotifier.stop();
			synchronized (mTaskRunningLock) {
				mRunning = false;
				mTimestamp = Timer.getFPGATimestamp();
				for (Loop loop : mLoops) {
					System.out.println("Stopping " + loop);
					loop.onStop(mTimestamp);
				}
			}
		}
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("looper_dt", mDt);
	}
}
