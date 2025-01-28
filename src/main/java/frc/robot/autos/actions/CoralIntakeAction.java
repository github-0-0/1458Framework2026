package frc.robot.autos.actions;
import frc.robot.Robot;
import frc.robot.subsystems.Laser;
import frc.robot.subsystems.Shooter;

public class CoralIntakeAction implements Action {
    private Shooter mShooter = Shooter.getInstance();
	
	public CoralIntakeAction() {
		mShooter = Shooter.getInstance();
	}
	@Override
	public void start() {
		mShooter.intake();
		System.out.println("Shooter shooting at phenomenally slow speeds!");
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;
        return !Laser.inRangeIntake();
	}

	@Override
	public void done() {
		mShooter.stopShooter();
	}
}
