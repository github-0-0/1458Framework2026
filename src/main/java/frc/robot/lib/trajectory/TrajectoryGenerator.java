package frc.robot.lib.trajectory;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import java.io.File;
import java.io.IOException;

public class TrajectoryGenerator {

    private static TrajectoryGenerator mInstance;
    
	public static TrajectoryGenerator getInstance() {
		if (mInstance == null) {
			mInstance = new TrajectoryGenerator();
		}
		return mInstance;
	}

    public TrajectoryGenerator (){
        if (mTrajectorySet == null) {
			System.out.println("Generating trajectories...");
			mTrajectorySet = new TrajectorySet();
			System.out.println("Finished trajectory generation");
		}
    }

    //the trajectories used in auto mode
    private TrajectorySet mTrajectorySet = null;
    /*
     * S: start
     */
    public class TrajectorySet {
		//the parent folder NEEDs to be "./src/main/deploy/"
        public HashMap<String, PathPlannerTrajectory> set = new HashMap<>();
        public TrajectorySet() {}

        public PathPlannerTrajectory loadPathPlannerTrajectory (String sJsonFile) {
            if (set.keySet().contains(sJsonFile)) {
                return set.get(sJsonFile);
            } else {
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(sJsonFile);
                        System.out.println("Trajectory loaded successfully: " + sJsonFile);
                        return path.getIdealTrajectory(Constants.PathPlannerRobotConfig.config).get();
                    
                } catch (Exception e) {
                    System.out.println("Trajectory loaded failed! =" + sJsonFile + ";err=" + e);
                    return null;
                }
            }
        }

    }

    //access to the trajectory set 
	public TrajectorySet getTrajectorySet() {
		return mTrajectorySet;
	}
}
