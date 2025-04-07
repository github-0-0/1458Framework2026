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
     * <ul>
     * <li> Space between each command (including brackets, parentheses, etc.) </li>
     * <li> Space between each command and its argument </li>
     * <li> S and the following number denotes a point (REMEMBER TO ADD A SPACE BETWEEN THE LETTER AND THE NUMBER). See {@link Naming} </li>
     * <li> After stating a point the robot will try to move to that point </li>
     * <li> Always check if the trajectory exists in the set </li>
     * <li> [ actions ] denote a parallel action, where actions are run in parallel </li>
     * <li> ( actions ) denote a series action, where actions are run one by one </li>
     * <li> { actions } denote a repeat action, where actions are repeated a number of times denoted by the number after the closing bracket </li>
     * <li> Wait and the following number denotes a wait action for that number of seconds </li>
     * <li> Snap and following offset ({@link SnapToTag#SnapToTag(String)}) </li>
     * </ul>
     */
    
    public AutoStringAuto(String string) {
        autoString = string;
    }
        
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("AutoStringAuto: run auto =" + autoString);
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
                case("S"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {
                        lastPoint = point;
                    } else {
                        listOfActions.add(new SwerveTrajectoryAction(
                                lastPoint + "-" + point,
                                isFirstTrajectory ? 
                                    ResetWheelTracker.SET_TO_STARTING_POS 
                                    : ResetWheelTracker.NO
                        ));
                        lastPoint = point;
                        isFirstTrajectory = false;
                    }
                    break;                
                case("E"):
                    point = actionStrings[i] + actionStrings[++i];
                    System.out.println("Trajectory Action: "+lastPoint+"-"+point);
                    if(lastPoint == null) {
                        lastPoint = point;
                    } else {
                        listOfActions.add(new SwerveTrajectoryAction(
                                lastPoint + "-" + point,
                                isFirstTrajectory ? 
                                    ResetWheelTracker.SET_TO_STARTING_POS 
                                    : ResetWheelTracker.NO
                        ));
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
                case "Snap":
                    listOfActions.add(new SnapToTag(actionStrings[++i]));
                    break;
                default:
                    System.out.println("Unknown action: " + curString);
                    break;
            }
        }
        return listOfActions;
    }
}
