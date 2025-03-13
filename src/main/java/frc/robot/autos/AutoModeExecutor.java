package frc.robot.autos;

import frc.robot.Loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
	private AutoModeBase m_auto_mode;
	private Thread m_thread = null;

	/* 
	* dc.1.24.2025,bugfix, make all public methods thread safe 
	*/

	public synchronized void setAutoMode(AutoModeBase new_auto_mode) {
		m_auto_mode = new_auto_mode;
	}

	public AutoModeBase getAutoMode() {
		return m_auto_mode;
	}

	public void start() {
		if (m_thread == null) {
			m_thread = new Thread(new CrashTrackingRunnable() {
				@Override
				public void runCrashTracked() {
					if (m_auto_mode != null) {
						m_auto_mode.run();
						System.out.println("Auto Executor Running!");
					}
				}
			});

			m_thread.start();
		}

		System.out.println("Auto Executor Started!");
	}

	public void stop() {
		if (m_auto_mode != null) {
			m_auto_mode.stop();
		}

		m_thread = null;
		System.out.println("Auto Executor Stopped!");
	}
}
