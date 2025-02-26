package frc.robot.autos.modes;

public class AutoModeCenter extends AutoStringAuto {
    public AutoModeCenter() {
        super("Elevator DEF S 3 R 6 "+
                "Elevator L2 CShoot Wait 1 "+
                "R 66"
                "Elevator A1 AIntake" + 
                "CS1 Wait 1 " + 
                "Elevator DEF R 1 " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
}
