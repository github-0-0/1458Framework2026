package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorActionL4 implements Action {
	private Elevator mElevator = null;

	@Override
	public void start() {
		mElevator = Elevator.getInstance();
		mElevator.goToElevatorL4();
	}

	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return mElevator.isAtTarget();
	}

	@Override
	public void done() {
		mElevator.stop();
	}
}
