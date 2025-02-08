package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
import frc.robot.autos.actions.*;

public class AutoStringAuto extends AutoModeBase {
    public String autoString;
    private String lastPoint = null;
    private Boolean isFirstTrajectory = true;


    /**
     * Syntax:
     * Space between each command (including brackets, parentheses, etc.)
     * Space between each command and its argument
     * S, P, R, CS, and the following number denotes a point (REMEMBER TO ADD A SPACE BETWEEN THE LETTER AND THE NUMBER). See {@link Robot2025\src\main\deploy\pathplanner\Naming}
     * After stating a point the robot will try to move to that point
     * Always check if the trajectory exists in the set
     * [ actions ] denote a parallel action, where actions are run in parallel
     * ( actions ) denote a series action, where actions are run one by one
     * { actions } denote a repeat action, where actions are repeated a number of times denoted by the number after the closing bracket
     * Wait and the following number denotes a wait action for that number of seconds
     * CIntake denotes a coral intake action
     * CShoot denotes a coral shoot action
     * AIntake denotes an algae intake action
     * AShoot denotes an algae shoot action
     * Elevator and the following number denotes an elevator action to that height index
     * Snap and the follwing number denotes a snap to nearest apriltag, where the number is 1 if we are aiming for the right reef and 0 if we are aiming for the left reef.
     */
    
    public AutoStringAuto(String string) {
        autoString = string;
    }
        
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("TestAutoMode: Running test auto mode!");
        lastPoint = null; //last point on field
        isFirstTrajectory = true;
        runAction(new SeriesAction(parseAuto(autoString)));
        System.out.println("Finished auto!");
    }

    private List<Action> parseAuto(String string) {
        List<Action> listOfActions = new ArrayList<Action>();
        String[] actionStrings = string.split(" "); //splits by spaces
		String curString;
        String subString = "";
        String point = null;
        for (int i = 0; i < actionStrings.length; i++) {
			curString = actionStrings[i];
            switch (curString) {
                case("P"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {}
                    else {
                        listOfActions.add(new SwerveTrajectoryAction(lastPoint+"-"+point,isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO));
                        lastPoint = point;
                        isFirstTrajectory = false;
                    }
                    break;
                case("CS"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {lastPoint = point;}
                    else {
                        listOfActions.add(new SwerveTrajectoryAction(lastPoint+"-"+point,isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO));
                        lastPoint = point;
                        isFirstTrajectory = false;
                    }
                    break;
                case("S"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {lastPoint = point;}
                    else {
                        listOfActions.add(new SwerveTrajectoryAction(lastPoint+"-"+point,isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO));
                        lastPoint = point;
                        isFirstTrajectory = false;
                    }
                    break;
                case ("R"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {lastPoint = point;}
                    else {
                        listOfActions.add(new SwerveTrajectoryAction(lastPoint+"-"+point,isFirstTrajectory?ResetWheelTracker.SET_TO_STARTING_POS:ResetWheelTracker.NO));
                        lastPoint = point;
                        isFirstTrajectory = false;
                    }
                    break;
                case "[":
                    subString = "";
                    while(!actionStrings[i].equals("]")) {
                        subString += actionStrings[++i] + " ";
                    }
                    subString = subString.substring(0,subString.length()-2);
                    listOfActions.add(new ParallelAction(parseAuto(subString)));
                    System.out.println("Parallel Action: "+subString);
                    break;
                case "(":
                    subString = "";
                    while(!actionStrings[i].equals(")")) {
                        subString += actionStrings[++i] + " ";
                    }
                    subString = subString.substring(0,subString.length()-2);
                    System.out.println("Series Action: "+subString);
                    listOfActions.add(new SeriesAction(parseAuto(subString)));
                    break;
                case "{":
                    subString = "";
                    while(!actionStrings[i].equals("}")) {
                        subString += actionStrings[++i] + " ";
                    }
                    int repeats = Integer.parseInt(actionStrings[++i]);
                    subString = subString.substring(0,subString.length()-3);
                    System.out.println("Repeat Action: " + subString);
                    ArrayList<Action> repeatActions = new ArrayList<Action>();
                    for (int j = 0; j < repeats; j++) {
                        repeatActions.add(new SeriesAction(parseAuto(subString)));
                    }
                    listOfActions.add(new SeriesAction(repeatActions));
                    break;
                case "Wait":
                    listOfActions.add(new WaitAction(Double.parseDouble(actionStrings[++i])));
                    break; 
                case "CIntake":
                    listOfActions.add(new CoralIntakeAction());
                    break;
                case "CShoot":
                    listOfActions.add(new CoralShooterAction());
                    break;
                case "AShoot":
                    listOfActions.add(new AlgaeShooterAction());
                    break;
                case "AIntake":
                    listOfActions.add(new AlgaeIntakeAction());
                    break;
                case "Elevator":
                    listOfActions.add(new ElevatorAction(Integer.parseInt(actionStrings[++i])));
                    break;
                case "Snap":
                    listOfActions.add(new SnapToTag(Integer.parseInt(actionStrings[++i])));
                    break;
                default:
                    System.out.println("Unknown action: " + curString);
                    break;
            }
        }
        return listOfActions;
    }
}
