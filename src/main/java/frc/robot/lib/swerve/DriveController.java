package frc.robot.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.trajectory.TrajectoryIterator;

public interface DriveController {
    public void setTrajectory(TrajectoryIterator trajectory);
    public void reset();
    public ChassisSpeeds calculate();
    public boolean isDone();
}
