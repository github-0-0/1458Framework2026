package frc.robot.autos.modes;

public class AutoModeRightSide extends AutoStringAuto {
    public AutoModeRightSide() { 
        super("S 1 " + 
                "R 1l Elevator L4 CShoot Wait 1 Elevator Ground " + 
                "CS 1 Wait 1 " + 
                "R 2r Elevator L4 CShoot Wait 1 Elevator Ground " +
                "CS 1 Wait 1 " + 
                "R 2r Elevator L3 CShoot Wait 1 Elevator Ground ");
    }
}

