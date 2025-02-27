package frc.robot.autos.modes;

public class RightAroundTheReefL4 extends AutoStringAuto {
    public RightAroundTheReefL4() {
        super("S 1 R 1 Elevator 4 CShoot Elevator 0 "
                   + "( CS 1 CIntake R 2 Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 3 Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 2 CIntake R 4 Elevator 4 CShoot Elevator 0 )");
    }
}
// Paths used: S1-R1, R1-CS1, CS1-R2, R2-CS1, CS1-R3, R3-CS2, CS2-R4