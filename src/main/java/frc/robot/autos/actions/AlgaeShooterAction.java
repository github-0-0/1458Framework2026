package frc.robot.autos.actions;


import frc.robot.Robot;
import frc.robot.subsystems.Laser;
import frc.robot.subsystems.AlgaeShooter;

public class AlgaeShooterAction implements Action {
    private AlgaeShooter mShooter = AlgaeShooter.getInstance();
	
	public AlgaeShooterAction() {
		mShooter = AlgaeShooter.getInstance();
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
		mShooter.stopAlgaeShooter();
	}
}
