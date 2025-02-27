package frc.robot.autos.modes;

public class Start3RightSideCoral extends AutoStringAuto {
    public Start3RightSideCoral() {
        super("S 3 R 6 Elevator 4 CShoot Elevator 0 "
                   + "( CS 1 CIntake R 1 Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 2 Elevator 4 CShoot Elevator 0 ) "
                   + "( CS 1 CIntake R 2 Elevator 3 CShoot Elevator 0 )");
    }
}
// Paths used: S3-R6, R6-CS1, CS1-R1, R1-CS1, CS1-R2, R2-CS1, CS1-R2