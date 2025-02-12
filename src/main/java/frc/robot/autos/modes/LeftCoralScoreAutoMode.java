package frc.robot.autos.modes;

public class LeftCoralScoreAutoMode extends AutoStringAuto {
    public LeftCoralScoreAutoMode() {
        /* This autoString simply makes the robot start at Start pos 1,
         * goes to Reef 1, snaps to right reef apriltag, then gets to elevator level 2,
         *   and shoots coral, then it goes to Coral station 1, runs coral intake, then it goes to reef 2,
         *  snaps to left reef apriltag, then gets to elevator level 1, and shoots coral, and repeats the process
         *  but with different elevator levels
         */ 
        // Start pos 1, reef 1, snap right apriltag, elevator 2 activated, Coral shot, Coral station 1, Coral intake, Reef 2, snap right apriltag, elevator 1, Coral shot, repeat 3 times, change elvator level on each iteration
        super("S 1 R 1 Elevator 2 CShoot " // Initial sequence: 3 seconds
                   + "( CS 1 CIntake R 1 Elevator 1 CShoot ) " // First iteration: 4 seconds
                   + "( CS 1 CIntake R 2 Elevator 2 CShoot ) " // Second iteration: 4 seconds
                   + "( CS 1 CIntake R 2 Elevator 3 CShoot Elevator 0 )"); // Third iteration: 4 seconds
    }
}