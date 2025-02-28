package frc.robot.autos.modes;

public class AutoModeLeftSide extends AutoStringAuto {
    public AutoModeLeftSide() {
        super("S 4o S 4 " + 
                "Elevator DEF " +
                "[ ( R 5r ) ( Wait 2.5 Elevator L4 ) ] CShoot Wait 1 Elevator DEF " + 
                "CS 2 Wait 1 " + 
                "[ ( R 4l ) ( Wait 2.5 Elevator L4 ) ] CShoot Wait 1 Elevator DEF ");
    }
    // Paths used: S4-R5l, R5l-CS2, CS2-R5r
}
