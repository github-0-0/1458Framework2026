package frc.robot.actions;

public class ConditionalAction implements Action {
    public Action currentAction = null;

    public ConditionalAction(boolean isTrue, Action executes, Action elseAction) {
        currentAction = isTrue ? executes : elseAction;
    }
    
	@Override
	public void start() {
        if (currentAction != null) 
            currentAction.start(); 
    }

	@Override
	public void update() {
        if (currentAction != null) 
            currentAction.update();
    }

	@Override
	public boolean isFinished() {
		return currentAction == null ? true : currentAction.isFinished();
	}

	@Override
	public void done() {
        if (currentAction != null) 
            currentAction.done();
    }
}
