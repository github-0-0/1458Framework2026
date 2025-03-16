package frc.robot.autos.actions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.Laser;

public class AlgaeIntakeAction implements Action {
    private AlgaeShooter mShooter = null;
	private Action wAction = null;
	public AlgaeIntakeAction() {
		wAction = new WaitAction(Constants.AlgaeShooter.kWaitTime);
	}
	@Override
	public void start() {
		mShooter = AlgaeShooter.getInstance();
		mShooter.intake();
		wAction.start();
	}

	@Override
	public void update() {
		wAction.update();
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return wAction.isFinished();
	}

	@Override
	public void done() {
		mShooter.stopAlgaeShooter();
		wAction.done();
	}
}
