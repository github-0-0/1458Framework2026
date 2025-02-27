package frc.robot.autos.modes;

public class AutoModeRightSide extends AutoStringAuto {
    public AutoModeRightSide() {
        super("Elevator DEF S 1 R 1 l "+
                "Elevator L4 CShoot Wait 1 Elevator DEF " + 
                "CS 1 Wait 1 " + 
                "Elevator DEF R 1 r " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
}
// Paths used: S2-R1l, R1l-CS1, CS1-R1r
