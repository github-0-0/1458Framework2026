package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;

import frc.robot.autos.actions.*;

//dc.2.19.2025, special auto mode to provide one-click shortcut operation in teleop mode
public class TeleopAutoMode extends AutoModeBase{
    // following actions and their SerialActions could come into this mode in asynchronized fashion
    // {SnapToTag, ElevatorAction, CoralShooterAction, AlgaeShooterAction}
    // All of these actions's underlying subsystems is singleton, so they are safe for multi callings/re-entrance
    // However Coral and Algae actions are atomic, commpletion-after-fire is required.  
    private List<Action> m_runningActions = new ArrayList<Action>();

    //constructor
    public void TeleopAutoMode(){}
    
    //callback from the executor thread 
    @Override
	protected void routine() throws AutoModeEndedException {
        // update action progress, takes no time
		for (Action action : m_runningActions) {
			if (!action.isFinished()) {
				action.update();
			} else {
				action.done();
                m_runningActions.remove(action);//remove completed actions from the list
			}
		}
        //sleep for a cycle, in milli seconds
		long waitTime = (long) (m_update_rate * 1000.0);
		try {Thread.sleep(waitTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // start a new action
    public void runAction(Action action) {
        if (action !=null){
            action.start();
            m_runningActions.add(action);
        }
    }

    // abort the all active actions 
   	public void abort() {		
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
