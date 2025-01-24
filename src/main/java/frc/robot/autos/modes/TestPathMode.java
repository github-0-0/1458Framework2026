package frc.robot.autos.modes;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.autos.actions.WaitAction;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
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
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
		/*
		for(String trajectory:trajectorySet.set.keySet()){
			System.out.println(trajectory);
			runAction(new WaitAction(1));
			runAction(new SwerveTrajectoryAction(trajectorySet.set.get(trajectory), ResetWheelTracker.SET_TO_STARTING_POS));
			System.out.println("Done with "+trajectory);
		}*/
		
		//runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectorySmallLoop, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryZigzag, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectorySlowCurve, true));
//		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryBackForth, true)); //bug: rotation at ending
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
