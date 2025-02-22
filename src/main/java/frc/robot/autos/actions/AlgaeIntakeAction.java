package frc.robot.autos.actions;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.Laser;

public class AlgaeIntakeAction implements Action {
    private AlgaeShooter mShooter = AlgaeShooter.getInstance();
	
	public AlgaeIntakeAction() {
		mShooter = AlgaeShooter.getInstance();
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
        return Laser.inRangeIntake();
	}

	@Override
	public void done() {
		mShooter.stopAlgaeShooter();
	}
}
