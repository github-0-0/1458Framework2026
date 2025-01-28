package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

//NOTE: UNUSED

public class FunnelAction implements Action {
	private Intake mIntake = null;
	private boolean kStart = true;
    public FunnelAction(boolean start) {
		kStart = start;
    }

	@Override
	public void start() {
		mIntake = Intake.getInstance();
		if (kStart) {
			mIntake.goToStart();
		} else {
			mIntake.goToEnd();
		}
		System.out.println("Intake going to " + kStart);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return mIntake.isPivotAtTarget();
	}

	@Override
	public void done() {}
}
