package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;

import frc.robot.autos.actions.Action;
import frc.robot.lib.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class TeleopActionExecutor {
	private Thread m_thread = null;
	private boolean mIsActive = false;

    private List<Action> mRunningActions = new ArrayList<Action>();

	public void start() {
		if (m_thread == null) {
			m_thread = new Thread(new CrashTrackingRunnable() {
				@Override
				public void runCrashTracked() {        
					try{
						while(mIsActive) {
							Iterator<Action> iterator = mRunningActions.iterator();
							while (iterator.hasNext()) {
								Action action = iterator.next();
								if (action.isFinished()) {
									action.done();
									iterator.remove();
								} else {
									action.update();
								}
							}
							long waitTime = (long) (Constants.LOOPER_DT * 1000.0);
							try {
								Thread.sleep(waitTime);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
						}
					} catch (Exception e) {}
					System.out.println("Auto Executor Running!");
				}
			});

			m_thread.start();
		}

		System.out.println("Auto Executor Started!");
	}

	public void stop() {

		m_thread = null;
		System.out.println("Auto Executor Stopped!");
	}

    // start a new action
    public synchronized void runAction(Action action) {
        if (action != null){
            action.start();
            mRunningActions.add(action);
        }
    }

    // abort the all active actions 
   	public synchronized void abort() {		
		if (mRunningActions.isEmpty()) {
			System.out.println("No running actions to abort");
			return;
		}

		for (Action action : mRunningActions) {
			action.done();
            System.out.println("Aborted " + action.getClass().toString() + " at time " + Timer.getFPGATimestamp());
		}

		mRunningActions.clear();
	}
}
