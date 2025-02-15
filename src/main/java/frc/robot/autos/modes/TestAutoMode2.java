package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;
import frc.robot.autos.actions.*;
//not worke
public class TestAutoMode2 extends AutoModeBase {
    private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
    private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
    public String autoString;
    String lastPoint = null;
    Boolean isFirstTrajectory = true;


    public TestAutoMode2() {
        autoString = "[ ( Wait 1 Elevator L4 Shoot ) ( CS 1 R 1 P 1 ) ] Wait 2 P 1 CS 2 R 2 ";
    }
    public Action parsePath(String action) {
        Action returnAction;
        switch (action) {
            case "CS":
                if (lastPoint == null) {
                    action = lastPoint;
                    return parsePath(action);
                } else {
                    action = lastPoint;
                    returnAction = new SwerveTrajectoryAction(trajectorySet.set.get(lastPoint + "-CS"),isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO);     
                    isFirstTrajectory = false;
                }
                break;
            case "P":
                if (lastPoint == null) {
                    action = lastPoint;
                    return parsePath(action);
                } else {
                    returnAction = new SwerveTrajectoryAction(trajectorySet.set.get(lastPoint+"-P"),isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO);
                    isFirstTrajectory = false;
                }
                break;
            case "S":
                if (lastPoint == null) {
                    action = lastPoint;
                    return parsePath(action);
                } else {
                    returnAction = new SwerveTrajectoryAction(trajectorySet.set.get(lastPoint+"-S"),isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO);
                    isFirstTrajectory = false;
                }
                break;
            case "R":
                if (lastPoint == null) {
                    action = lastPoint;
                    return parsePath(action);
                } else {
                    returnAction = new SwerveTrajectoryAction(trajectorySet.set.get(lastPoint+"-R"),isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO);
                    isFirstTrajectory = false;
                }
                break;
            default:
                return null;
            }
        return returnAction;
    } 
    
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("TestAutoMode: Running test auto mode!");
        String[] actions = autoString.split(" ");
        lastPoint = null;
        isFirstTrajectory = true;
		String action;
        for (int i = 0; i < actions.length; i++) {
			action = actions[i];
            switch (action) {
                case("P"):
                    runAction(parsePath(action));
                case("CS"):
                    runAction(parsePath(action));
                case("S"):
                    runAction(parsePath(action));
                case ("R"):
                    runAction(parsePath(action));
                case "[":
                    List<Action> listOfAutos = new ArrayList<Action>();
                    while (!(actions[i].equals("]"))) {
                        List<Action> listOfActions = new ArrayList<Action>();
                        while (!(actions[i].equals(")"))) {
                            if (actions[i].equals("(")) {
                                i++;
                            }
                            listOfActions.add(parsePath(actions[i]));
                        }
                        i++;
                        Action auto = new SeriesAction(listOfActions);
                        listOfAutos.add(auto);
                    }
                    ParallelAction finalAction = new ParallelAction(listOfAutos);
                    runAction(finalAction);
                    i++;
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
