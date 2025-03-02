package frc.robot.autos.modes;

public class AutoModeCenter extends AutoStringAuto {
    public AutoModeCenter() { 
        super("S 2 "+
                "[ ( R 6l ) ( Wait 0.5 Elevator L4 ) ] Wait 1 CShoot Wait 1 "+
                "Elevator Ground");
    }
    // Paths used: S2-R6l, R6l-R6c, R6c-CS1, CS1-R1l
}
