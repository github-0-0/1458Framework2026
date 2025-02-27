package frc.robot.autos.modes;

public class AutoModeLeftSide extends AutoStringAuto {
    public AutoModeLeftSide() {
        super("Elevator DEF S 3 R 5 l " +
                "Elevator L4 CShoot Wait 1 " +
                "Elevator DEF " + 
                "CS 2 Wait 1 " + 
                "Elevator DEF R 5 r " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
    // Paths used: S4-R5l, R5l-CS2, CS2-R5r
}
