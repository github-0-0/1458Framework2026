package frc.robot.autos.modes;


import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.*;
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

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-S1")));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R1")));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-S1")));
/*  	
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S1-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S2-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S3-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("S4-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R3"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R4"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R5"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS2-R6"), ResetWheelTracker.SET_TO_STARTING_POS));

 		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
 		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R2-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
 		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R3-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R4-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R5-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R6-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R2-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R3-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R4-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R5-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R6-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));


		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R1-P1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R2-P1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R3-P1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R4-P1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R5-P1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("R6-P1"), ResetWheelTracker.SET_TO_STARTING_POS));

		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("P1-CS1"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("P1-CS2"), ResetWheelTracker.SET_TO_STARTING_POS));
		runAction(new SwerveTrajectoryAction(trajectorySet.set.get("P1-CS1b"), ResetWheelTracker.SET_TO_STARTING_POS));
*/

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
/*//TOD: chain other test actions here 
		System.out.println("TestPathMode: wait for 1 seconds!");
		runAction(new WaitAction(1));
		System.out.println("TestPathMode: start returning Trajectory!");
		runAction(new SwerveTrajectoryAction(trajectorySet.testTrajectoryBackForth_Return, false));
*/		System.out.println("Finished auto!");
	}
	// spotless:on
}
