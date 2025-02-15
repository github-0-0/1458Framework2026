package frc.robot.autos.actions;


import frc.robot.Robot;
import frc.robot.subsystems.Laser;
import frc.robot.subsystems.CoralShooter;

public class CoralShooterAction implements Action {
    private CoralShooter mShooter = CoralShooter.getInstance();
	
	public CoralShooterAction() {
		mShooter = CoralShooter.getInstance();
	}
	@Override
	public void start() {
		mShooter.shoot();
		System.out.println("Shooter shooting at phenomenally slow speeds!");
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
        return !Laser.inRangeShooter();
	}

	@Override
	public void done() {
		mShooter.stopShooter();
	}
}
