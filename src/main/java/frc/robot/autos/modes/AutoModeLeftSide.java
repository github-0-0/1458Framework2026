package frc.robot.autos.modes;

public class AutoModeLeftSide extends AutoStringAuto {
    public AutoModeLeftSide() {
        super("S 4 " + 
                "R 5r Elevator L4 CShoot Wait 1 Elevator Ground " + 
                "CS 2 Wait 1 " + 
                "R 4l Elevator L4 CShoot Wait 1 Elevator Ground " +
                "CS 2 Wait 1 " + 
                "R 4r Elevator L4 CShoot Wait 1 Elevator Ground ");
    }
}
