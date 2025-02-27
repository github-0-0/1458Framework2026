package frc.robot.autos.modes;

public class AutoModeRightSide extends AutoStringAuto {
    public AutoModeRightSide() {
        super("Elevator DEF S 1 " +
                "[ ( R 1 l ) ] [ ( Wait 4 Elevator L4 ) ] CShoot Wait 1 Elevator DEF " + 
                "CS 1 Wait 1 Elevator DEF " + 
                "[ ( R 1 r ) ] [ ( Wait 5 Elevator L4 ) ] CShoot Wait 1 Elevator DEF ");
    }
}
// Paths used: S1-R1l, R1l-CS1, CS1-R1r
