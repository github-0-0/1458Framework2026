package frc.robot.autos.modes;

public class AutoModeLeftSide extends AutoStringAuto {
    public AutoModeLeftSide() {
        super("S 4 " + 
                "[ ( R 5r ) ( Wait 1.2 Elevator L4 ) ] Wait 1 CShoot Wait 1 Elevator Ground " + 
                "CS 2 Wait 1 " + 
                "[ ( R 4l ) ( Wait 1 Elevator L4 ) ] Wait 1 CShoot Wait 1 Elevator Ground " +
                "CS 2 Wait 1 " + 
                "[ ( R 4r ) ( Wait 1 Elevator L4 ) ] Wait 1 CShoot Wait 1 Elevator Ground ");
    }
}
