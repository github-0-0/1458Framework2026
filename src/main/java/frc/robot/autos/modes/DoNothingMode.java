package frc.robot.autos.modes;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("doing nothing");
	}
}
