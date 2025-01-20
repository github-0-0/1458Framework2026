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
        autoString = "Path CS 1 R 1 P 1 Stop Wait 2 Path CS 2 R 2 Stop";
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("TestPathMode: Running test auto mode!");
        String[] actions = autoString.split(" ");
		String action = null;
        for (int i = 0; i < actions.length; i++) {
			action = actions[i];
            switch (action) {
				case "Path":
					action = actions[i++];
					while (action != "Stop") {
						switch (action) {
							case "CS":
							//cook here
								//runAction(new SwerveTrajectoryAction(trajectorySet.set.get("CS1 R1")));
						}
						action = actions[++i];
					}
                case "Wait":
                    // Assuming the next part in the string is a time duration for the wait action
                    if (i + 1 < actions.length) {
                        String waitTime = actions[++i]; // Increment i to get the wait time
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
