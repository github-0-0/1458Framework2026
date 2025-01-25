package frc.robot.autos.actions;


import frc.robot.subsystems.Laser;
import frc.robot.subsystems.Shooter;

public class CoralShootAction {
    private Shooter shooter = null;

    public CoralShootAction(Shooter S) {
        shooter = S;
    }

    public boolean isFinished(){
        return Laser.inRangeShooter();
    };

	/**
	 * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic lives in this
	 * method
	 */
	public void update(){

    }

	/**
	 * Run code once when the action finishes, usually for clean up
	 */
	public void done(){
		shooter.stop();
    }

	/**
	 * Run code once when the action is started, for set up
	 */
	public void start(){
        shooter.spin();
    }
}
