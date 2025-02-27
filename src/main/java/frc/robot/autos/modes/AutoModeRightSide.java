package frc.robot.autos.modes;

public class AutoModeRightSide extends AutoStringAuto {
    public AutoModeRightSide() {
        super("Elevator DEF S 2 R 1 "+
                "Elevator L4 CShoot Wait 1 Elevator DEF " + 
                "CS 1 Wait 1 " + 
                "Elevator DEF R 1 " + 
                "Elevator L4 CShoot Wait 1 Elevator DEF ");
    }
}
