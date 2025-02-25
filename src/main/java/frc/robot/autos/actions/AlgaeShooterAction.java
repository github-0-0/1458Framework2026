package frc.robot.autos.actions;


import frc.robot.Robot;
import frc.robot.subsystems.Laser;
import frc.robot.subsystems.AlgaeShooter;

public class AlgaeShooterAction implements Action {
    private AlgaeShooter mShooter;
	
	public AlgaeShooterAction() {
		mShooter = AlgaeShooter.getInstance();
	}
	@Override
	public void start() {
		mShooter.shoot();
		System.out.println("Shooter shooting");
	}

	@Override
	public void update() {
		System.out.println("Shooter shooting pt2");
	}

	@Override
	public boolean isFinished() {
		//if (Robot.isSimulation()) return true;

		if(Laser.getMeasurementAlgaeShooter() > 400) {
			return true;
			
		}
		else{
			System.out.println("Not finished");
			return false;
		}
	}

	@Override
	public void done() {
		System.out.println("Finished");
		mShooter.stopAlgaeShooter();
	}
}
