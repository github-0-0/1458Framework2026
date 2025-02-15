package frc.robot.autos.modes;

public class LMiddleCoralScoreAutoMode extends AutoStringAuto {
    public LMiddleCoralScoreAutoMode() {
        /* This autoString simply makes the robot start at Start pos 1,
         * goes to Reef 1, snaps to right reef apriltag, then gets to elevator level 2,
         *   and shoots coral, then it goes to Coral station 1, runs coral intake, then it goes to reef 2,
         *  snaps to left reef apriltag, then gets to elevator level 1, and shoots coral, and repeats the process
         *  but with different elevator levels
         */ 
        // starting pos 2, goes to reef 1, snaps apriltag left,  elevator 2 activated, coral shot, coral station 1, coral intake, reef 2, apriltag right, elevator 1, coral shot, repeat 3 times, change elevator level on each iteration
        super("S 2 R 1 Elevator 2 CShoot " // Initial sequence: 1.5 seconds
                   + "( CS 1 CIntake R 2 Elevator 1 CShoot ) " // First iteration: 4 seconds
                   + "( CS 1 CIntake R 2 Elevator 2 CShoot ) " // Second iteration: 4 seconds
                   + "( CS 1 CIntake R 2 Elevator 3 CShoot )"); // Third iteration: 4 seconds
    }
}
