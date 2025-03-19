package frc.robot.autos.modes;


import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.*;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;

public class TestPathMode extends AutoModeBase {
	public TestPathMode() {}

	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("TestPathMode: Running test mode auto!");
		runAction(new SwerveTrajectoryAction("Example Path", ResetWheelTracker.SET_TO_STARTING_POS));
		System.out.println("Finished auto!");
	}
}
