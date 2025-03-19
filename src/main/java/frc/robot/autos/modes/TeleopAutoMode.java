package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;

import frc.robot.autos.actions.*;

public class TeleopAutoMode extends AutoModeBase{
    private List<Action> m_runningActions = new ArrayList<Action>();

    //constructor
    public TeleopAutoMode() {}
    
    //callback from the executor thread 
    @Override
	protected void routine() throws AutoModeEndedException {
        while(isActiveWithThrow()) {
            Iterator<Action> iterator = m_runningActions.iterator();
            while (iterator.hasNext()) {
                Action action = iterator.next();
                if (action.isFinished()) {
                    action.done();
                    iterator.remove();
                } else {
                    action.update();
                }
            }
            long waitTime = (long) (m_update_rate * 1000.0);
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // start a new action
    public synchronized void runAction(Action action) {
        if (action != null){
            action.start();
            m_runningActions.add(action);
        }
    }

    // abort the all active actions 
   	public synchronized void abort() {		
		if (m_runningActions.isEmpty()) {
			System.out.println("No running actions to abort");
			return;
		}
		for (Action action : m_runningActions) {
			action.done();
            System.out.println("Aborted "+action.getClass().toString() + " at time "+Timer.getFPGATimestamp());
		}
		m_runningActions.clear();
	}
}
