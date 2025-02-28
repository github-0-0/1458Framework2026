package frc.robot.autos.modes;

public class AutoModeCenter extends AutoStringAuto {
    public AutoModeCenter() { 
        super("Elevator DEF S 2 [ ( R 6l ) ] "+
                "[ ( Wait 2 Elevator L3 ) ] CShoot Wait 1 "+
                "[ ( R 6c ) ] [ ( Wait 2 Elevator A1 ) ] " + 
                "AIntake P 1 AShoot ");
    }
    // Paths used: S2-R6l, R6l-R6c, R6c-CS1, CS1-R1l
}
