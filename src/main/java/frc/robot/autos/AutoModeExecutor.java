package frc.robot.autos;

import frc.robot.lib.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
	private AutoModeBase mAutoMode;
	private Thread mThread = null;

	public synchronized void setAutoMode(AutoModeBase new_auto_mode) {
		mAutoMode = new_auto_mode;
	}

	public AutoModeBase getAutoMode() {
		return mAutoMode;
	}

	public void start() {
		if (mThread == null) {
			mThread = new Thread(new CrashTrackingRunnable() {
				@Override
				public void runCrashTracked() {
					if (mAutoMode != null) {
						mAutoMode.run();
						System.out.println("Auto Executor Running!");
					}
				}
			});

			mThread.start();
		}

		System.out.println("Auto Executor Started!");
	}

	public void stop() {
		if (mAutoMode != null) {
			mAutoMode.stop();
		}

		mThread = null;
		System.out.println("Auto Executor Stopped!");
	}
}
