package frc.robot.autos.modes;

public class RightAroundTheReefL4 extends AutoStringAuto {
    public RightAroundTheReefL4() {
        super("S 1 R 1 l Elevator 4 CShoot Elevator 0 "
                   + "( CS 1 CIntake R 2 l Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 3 l Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 2 CIntake R 4 l Elevator 4 CShoot Elevator 0 )");
    }
}
// Paths used: S1-R1l, R1l-CS1, CS1-R2l, R2l-CS1, CS1-R3l, R3l-CS2, CS2-R4l