package frc.robot.autos.modes;

public class Start3RightSideCoral extends AutoStringAuto {
    public Start3RightSideCoral() {
        super("S 3 R 6 l Elevator 4 CShoot Elevator 0 "
                   + "( CS 1 CIntake R 1 l Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 2 l Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 2 r Elevator 3 CShoot Elevator 0 )");
    }
}
// Paths used: S3-R6l, R6l-CS1, CS1-R1l, R1l-CS1, CS1-R2l, R2l-CS1, CS1-R2r