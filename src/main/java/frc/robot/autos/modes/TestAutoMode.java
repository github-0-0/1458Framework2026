package frc.robot.autos.modes;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.autos.actions.WaitAction;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;

public class TestAutoMode extends AutoModeBase {
    private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
    private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
    public String autoString;

    public TestAutoMode() {
        autoString = "Path CS 1 R 1 P 1 Stop Wait 2 Path P 1 CS 2 R 2 Stop";
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("TestAutoMode: Running test auto mode!");
        String[] actions = autoString.split(" ");
		String action;
        Boolean isFirstTrajectory = true;
        for (int i = 0; i < actions.length; i++) {
			action = actions[i];
            switch (action) {
				case "Path":
					while (!actions[i+3].equals("Stop")) {
                        String start = actions[++i]+actions[++i];
                        String end = actions[++i]+actions[++i];
                        System.out.println(start+"-"+end+" "+isFirstTrajectory);
                        runAction(new SwerveTrajectoryAction(trajectorySet.set.get(start+"-"+end),!isFirstTrajectory?ResetWheelTracker.NO:ResetWheelTracker.SET_TO_STARTING_POS));
                        isFirstTrajectory = false;
                        i--;
                        i--;
					}
                    i += 3;
                    break;
                case "Wait":
                    if (i + 1 < actions.length) {
                        String waitTime = actions[++i];
                        runAction(new WaitAction(Double.parseDouble(waitTime)));
                    }
                    break;
                default:
                    System.out.println("Unknown action: " + action);
                    break;
            }
			
        }
        System.out.println("Finished auto!");
    }
}
