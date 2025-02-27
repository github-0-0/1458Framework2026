package frc.robot.autos.modes;

public class AutoModeLeftSide extends AutoStringAuto {
    public AutoModeLeftSide() {
        super("Elevator DEF S 4 R 5 " +
                "Elevator L4 CShoot Wait 1 " +
                "Elevator DEF " + 
                "CS 2 Wait 1 " + 
                "Elevator DEF R 5 " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
    // Paths used: S4-R5, R5-CS2, CS2-R5
}
