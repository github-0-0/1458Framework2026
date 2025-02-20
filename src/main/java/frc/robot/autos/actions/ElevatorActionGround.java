package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorActionGround implements Action {
	private Elevator mElevator = null;

	@Override
	public void start() {
		mElevator = Elevator.getInstance();
		mElevator.goToElevatorGround();
	}

	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return true; //mElevator.getIsAtTarget();	/*dc.2.11.25. TODO: MUST revise when Elevator class is merged*/
	}

	@Override
	public void done() {}
}
