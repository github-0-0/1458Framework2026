package frc.robot.autos.actions;

import frc.robot.Robot;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Laser;

public class CoralShootAction implements Action {
	private CoralShooter mShooter = null;

	@Override
	public void start() {
		mShooter = CoralShooter.getInstance();
		mShooter.shoot();
	}
	
	@Override
	public void update() {
		
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
		return !Laser.inRangeShooter();
	}

	@Override
	public void done() {
		mShooter.intake();
	}
}
