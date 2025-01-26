package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorAction implements Action {
	private Elevator mElevator = null;
	private int kLevel;
    public ElevatorAction(int level) {
		kLevel = level;
		System.out.println("Elevator going to " + level);
    }

	@Override
	public void start() {
		mElevator = Elevator.getInstance();
		mElevator.setTargetLevel(kLevel);
	}

	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return mElevator.goToTarget();
	}

	@Override
	public void done() {}
}
