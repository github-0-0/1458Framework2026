package frc.robot.autos.modes;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.autos.actions.WaitAction;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;

public class TestAutoMode2 extends AutoModeBase {
    private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
    private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
    public String autoString;

    public TestAutoMode2() {
        autoString = "[ ( Wait 1 Elevator L4 Shoot ) ( CS 1 R 1 P 1 ) ] Wait 2 P 1 CS 2 R 2 ";
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("TestAutoMode: Running test auto mode!");
        String[] actions = autoString.split(" ");
		String action;
        Boolean isFirstTrajectory = true;
        String lastPoint = null;
        for (int i = 0; i < actions.length; i++) {
			action = actions[i];
            switch (action) {
				case "CS":
                    if(lastPoint==null){
                        action = lastPoint;
                    } else {
                        runAction(new SwerveTrajectoryAction(trajectorySet.set.get(lastPoint + "-CS"),isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO));     
                        action = lastPoint;
                        isFirstTrajectory = false;
                    }
                    //TODO:add the rest of the points as cases
                    //TODO: square bracket[] is parallel, () is series.
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
