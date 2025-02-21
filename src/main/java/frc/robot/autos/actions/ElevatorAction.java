package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorAction implements Action {
	private Elevator mElevator = null;
	private String target = "";
	public ElevatorAction(String targ) {
		target = targ;
	}
	@Override
	public void start() {
		mElevator = Elevator.getInstance();
		mElevator.setTarget(target);
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
