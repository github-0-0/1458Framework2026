package frc.robot.lib.trajectory;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.DummySubsystem;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;

//dc.10.21.2024, this class is going to load all paths pre-defined via PathWeaver tool

public class TrajectoryGenerator {
    
    //the trajectories used in auto mode
    private TrajectorySet mTrajectorySet = null;

    public class TrajectorySet {
		//the parent folder NEEDs to be "./src/main/deploy/"
        public Trajectory testTrajectoryZigzag = loadTrajectory("paths/output/s.0.0.zigzag.wpilib.json"); 
        public Trajectory testTrajectoryBackForth = loadTrajectory("paths/output/s.0.0.back.n.forth.n.return.wpilib.json"); 
        public Trajectory testTrajectoryStraightForward = loadTrajectory("paths/output/s.0.0.StraightForward.n.return.wpilib.json"); 
        public Trajectory testTrajectorySlowCurve = loadTrajectory("paths/output/s.0.0.SlowCurve.wpilib.json"); 
        public Trajectory testTrajectoryOneCircle = loadTrajectory("paths/output/s.0.0.oneCircle.cw.wpilib.json"); 
        public Trajectory testTrajectoryTwoCircle = loadTrajectory("paths/output/s.0.0.twoCircle.wpilib.json"); 
        public Trajectory testTrajectoryBeeDancing = loadTrajectory("paths/output/s.0.0.BeeDancing.wpilib.json");         
        public Trajectory testTrajectorySmallLoop = loadTrajectory("paths/output/s.0.0.smallloop.wpilib.json");         
        public Trajectory testTrajectoryOhNo = loadTrajectory("paths/output/s.0.0.oh.no.wpilib.json");
        public Trajectory testTrajectoryForwardBack = loadTrajectory("paths/output/s.0.0.shrimple.wpilib.json");
        /* dc.10.21.2024, additional trajectory can be added similar to the TestTrajectory */

        private Trajectory loadTrajectory (String sJsonFile){
            try{
                // Get the path to the deployed JSON file
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(sJsonFile);                
                // Load the trajectory
                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                System.out.println("Trajectory loaded successfully! =" + trajectoryPath.toString());
                return trajectory;
            } catch(IOException err){
                System.out.println("Trajectory loaded failed! =" + sJsonFile + ";err=" + err);
                return null;
            }
        }
    }

    // instanciation code 
    private static TrajectoryGenerator mInstance;
	public static TrajectoryGenerator getInstance() {
		if (mInstance == null) {
			mInstance = new TrajectoryGenerator();
		}
		return mInstance;
	}

    //constructor code
    public TrajectoryGenerator (){
        //dc.to be implemented
        //TODO: finish trajectory generator constructor
    }

    //actually create the trajectory object
    public void generateTrajectories (){
        if (mTrajectorySet == null) {
			System.out.println("Generating trajectories...");
			mTrajectorySet = new TrajectorySet();
			System.out.println("Finished trajectory generation");
		}
    }    

    //access to the trajectory set 
	public TrajectorySet getTrajectorySet() {
		return mTrajectorySet;
	}
}
