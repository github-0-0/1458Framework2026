package frc.robot.autos.modes;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.autos.actions.WaitAction;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;

public class TestPathMode extends AutoModeBase {
	private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
	private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
	public TestPathMode() {}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("TestPathMode: Running test mode auto!");
		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectorySmallLoop, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryZigzag, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectorySlowCurve, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryBackForth, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryOneCircle, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryTwoCircle, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryBeeDancing, true));
/*//TODO: chain other test actions here 
		System.out.println("TestPathMode: wait for 1 seconds!");
		runAction(new WaitAction(1));
		System.out.println("TestPathMode: start returning Trajectory!");
		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryBackForth_Return, false));
*/		System.out.println("Finished auto!");
	}
	// spotless:on
}
