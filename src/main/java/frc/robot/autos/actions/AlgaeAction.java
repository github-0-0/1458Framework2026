package frc.robot.autos.actions;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.Laser;

public class AlgaeAction implements Action {
    private AlgaeShooter mShooter=null;
	private String dir;
	
	public AlgaeAction(String dir) {
//		mShooter = AlgaeShooter.getInstance();
		this.dir = dir;
	}
	@Override
	public void start() {
		if(dir.equals("Intake")){
			mShooter.intake();
		}
		else {
			mShooter.shoot();
		}
		
		//System.out.println("Shooter shooting at phenomenally slow speeds!");
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation()) return true;

		if(dir.equals("Intake")){
			return Laser.inRangeAlgaeShooter();
		}
        else {
			return Laser.getMeasurementAlgaeShooter() > 300;
		}
	}

	@Override
	public void done() {
		mShooter.stopAlgaeShooter();
	}
}
