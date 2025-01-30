package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeEndedException;
import frc.robot.autos.actions.SwerveTrajectoryAction.ResetWheelTracker;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.lib.trajectory.TrajectoryGenerator.TrajectorySet;
import frc.robot.autos.actions.*;

public class TestAutoMode3 extends AutoModeBase {
    private TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();
    private TrajectorySet trajectorySet = trajectoryGenerator.getTrajectorySet();
    public String autoString;
    private String lastPoint = null;
    private Boolean isFirstTrajectory = true;

    public TestAutoMode3() {
        autoString = "S 1 { ( Wait 1 R 2 ) ( Wait 1 CS 1 ) } 4";
        //autoString = "S 1 { [ ( Wait 1 Elevator 4 Shoot ) ( R 1 ) ] [ ( Wait 1 Elevator 0 Intake ) ( CS 1 ) ] } 4 Wait 2 R 1 P 1 CS 2 R 2 ";
    }
    //TODO: Fix Errors here
        
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
                default:
                    System.out.println("Unknown action: " + curString);
                    break;
            }
        }
        return listOfActions;
    }
}
