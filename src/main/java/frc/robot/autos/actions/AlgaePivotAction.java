package frc.robot.autos.actions;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.Laser;

public class AlgaePivotAction implements Action {
    private AlgaeShooter mShooter = null;
	private String target = "";
	public AlgaePivotAction(String targ) {
		target = targ;
	}
	@Override
	public void start() {
		mShooter = AlgaeShooter.getInstance();
		mShooter.setTarget(target);
	}

	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return mShooter.isAtTarget();
	}

	@Override
	public void done() {
		mShooter.stop();
	}
}
