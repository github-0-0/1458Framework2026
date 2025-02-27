package frc.robot.autos.modes;

public class Coral42Point extends AutoStringAuto {
    public Coral42Point(){
        super("S 1 "+
        "[ ( R 1 l ) ( Elevator 4 ) ] CShoot CS 1 CIntake Elevator 0 "+
        "[ ( R 2 l ) ( Elevator 4 ) ] CShoot CS 1 CIntake Elevator 0 "+
        "[ ( R 3 l ) ( Elevator 4 ) ] CShoot CS 1 CIntake Elevator 0 ");
    }
}
// Paths used: S1-R1l, R1l-CS1, CS1-R2l, R2l-CS1, CS1-R3l, R3l-CS1