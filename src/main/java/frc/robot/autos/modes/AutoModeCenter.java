package frc.robot.autos.modes;

public class AutoModeCenter extends AutoStringAuto {
    public AutoModeCenter() {
        super("Elevator DEF S 3 R 6 l "+
                "Elevator L2 CShoot Wait 1 "+
                "R 6 r" +
                "Elevator A1 AIntake" + 
                "CS1 Wait 1 " + 
                "Elevator DEF R 1 l " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
    // Paths used: S3-R6l, R6l-R6r, R6r-CS1, CS1-R1l
}
