package frc.robot.autos.actions;

import frc.robot.subsystems.Elevator;

public class ElevatorAction implements Action {
	private Elevator mElevator = null;
	private enum Levels {
		NONE,
		GROUND,
		L1,
		L2,
		L3,
		L4,
	}
	@Override
	public void start() {}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {}
}
