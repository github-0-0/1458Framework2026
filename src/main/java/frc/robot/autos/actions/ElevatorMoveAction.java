package frc.robot.autos.actions;

import frc.robot.subsystems.Elevator;

public class ElevatorMoveAction implements Action {
	private Elevator mElevator = null;
	

	
    public ElevatorMoveAction(Elevator E) {
		mElevator = E;
    }

	@Override
	public void start() {

	}

	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		return mElevator.goToTarget();

	}

	@Override
	public void done() {}
}
