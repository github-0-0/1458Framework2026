package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import frc.robot.actions.Action;
import frc.robot.lib.loops.CrashTrackingRunnable;

/**
 * A class to run actions with.
 */
public class ActionExecutor {
    private Thread mThread = null;
    private volatile boolean mIsActive = false;
	private static final long WAIT_MS = (long) (Constants.LOOPER_DT * 1000);

    // Buffer for incoming actions
    private final BlockingQueue<Action> mActionBuffer = new LinkedBlockingQueue<>();
    // Active actions being executed
    private final List<Action> mRunningActions = new ArrayList<>();

    /**
     * Starts the executor thread.
     */
    public synchronized void start() {
        if (mThread == null) {
            mIsActive = true;
            mThread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    try {
                        while (mIsActive) {
                            // Drain buffered actions into running list
                            Action action;
                            while ((action = mActionBuffer.poll()) != null) {
                                action.start();
                                synchronized (mRunningActions) {
                                    mRunningActions.add(action);
                                }
                            }

                            // Update or finish running actions
                            synchronized (mRunningActions) {
                                Iterator<Action> iterator = mRunningActions.iterator();
                                while (iterator.hasNext()) {
                                    Action running = iterator.next();
                                    if (running.isFinished()) {
                                        running.done();
                                        iterator.remove();
                                    } else {
                                        running.update();
                                    }
                                }
                            }

                            // Sleep to maintain loop rate
                            try {
                                Thread.sleep(WAIT_MS);
                            } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                                break;
                            }
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    } finally {
                        System.out.println("Action Executor exiting loop.");
                    }
                }
            });
            mThread.start();
            System.out.println("Action Executor Started!");
        }
    }

    /**
     * Stops the executor thread cleanly.
     */
    public synchronized void stop() {
        mIsActive = false;
        if (mThread != null) {
            mThread.interrupt();
            try {
                mThread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            mThread = null;
        }
        System.out.println("Action Executor Stopped!");
    }

    /**
     * Queues a new action to run.
     */
    public void runAction(Action action) {
        if (action != null) {
            mActionBuffer.offer(action);
        }
    }

    /**
     * Aborts all active and buffered actions.
     */
    public synchronized void abort() {
        // Clear buffered actions
        mActionBuffer.clear();

        // Abort running actions
        synchronized (mRunningActions) {
            if (mRunningActions.isEmpty()) {
                System.out.println("No running actions to abort");
                return;
            }
            for (Action running : mRunningActions) {
                running.done();
                System.out.println("Aborted " + running.getClass().getSimpleName() + " at time " + Timer.getFPGATimestamp());
            }
            mRunningActions.clear();
        }
    }
}
