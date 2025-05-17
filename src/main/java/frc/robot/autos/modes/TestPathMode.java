package frc.robot.autos.modes;


import frc.robot.actions.*;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;

public class TestPathMode extends AutoModeBase {
	public TestPathMode() {}

	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("TestPathMode: Running test mode auto!");
		runAction(new SwerveTrajectoryAction("Example Path"));
		System.out.println("Finished auto!");
	}
}
