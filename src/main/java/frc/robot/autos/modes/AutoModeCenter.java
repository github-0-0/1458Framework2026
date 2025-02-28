package frc.robot.autos.modes;

public class AutoModeCenter extends AutoStringAuto {
    public AutoModeCenter() { 
        super("Elevator DEF S 2 [ ( R 6l ) ] "+
                "[ ( Wait 2 Elevator L2 ) ] CShoot Wait 1 "+
                "[ ( R 6c ) ] [ ( Wait 2 Elevator A1 ) ] " + 
                "AIntake CS 1 AShoot Wait 1 Elevator DEF Wait 1 " + 
                "[ ( R 1l ) ] [ ( Wait 5 Elevator L4 ) ] " +
                "CShoot Wait 1 Elevator DEF ");
    }
    // Paths used: S2-R6l, R6l-R6c, R6c-CS1, CS1-R1l
}
