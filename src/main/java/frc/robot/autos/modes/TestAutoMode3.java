package frc.robot.autos.modes;

public class TestAutoMode3 extends AutoStringAuto {
    public TestAutoMode3()   {        
            //super("S 2 R 6 CS 1 R 6 CS 2 R 3 CS 2 R 1 ");
            //super("S 2 R 6 Elevator L2 CS 1");
            super("S 2 R 1 CS 1 ");//CS 1 R 2 CS 1");
        }
/*  
         
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
                case "CShoot":
                    listOfActions.add(new CoralShootAction());
                    break;
                case "AShoot":
                    listOfActions.add(new AlgaeShooterAction());
                    break;
                case "AIntake":
                    listOfActions.add(new AlgaeIntakeAction());
                    break;
                case "Elevator":
                    listOfActions.add(new ElevatorAction(actionStrings[++i]));
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
*/        
}
