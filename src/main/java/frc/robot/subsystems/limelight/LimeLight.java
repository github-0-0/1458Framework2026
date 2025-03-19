package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Translation2d;


import java.util.ArrayList;
import java.util.List;

public class LimeLight extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final GoalTracker goalTracker;

    public LimeLight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        GoalTracker.Configuration config = new GoalTracker.Configuration();
        goalTracker = new GoalTracker(config);
    }

    @Override
    public void periodic() {
        updateGoalTracker();
    }

    private void updateGoalTracker() {
        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        List<Translation2d> goals = getGoalsFromLimelight();
        goalTracker.update(timestamp, goals);
    }

    private List<Translation2d> getGoalsFromLimelight() {
        List<Translation2d> goals = new ArrayList<>();

        // Example of reading data from the Limelight NetworkTable
        double validTargets = limelightTable.getEntry("tv").getDouble(0);
        if (validTargets > 0) {
            double tx = limelightTable.getEntry("tx").getDouble(0);
            double ty = limelightTable.getEntry("ty").getDouble(0);
            // Assuming some method to convert tx, ty to Translation2d
            Translation2d goalPosition = convertToTranslation2d(tx, ty);
            goals.add(goalPosition);
        }

        return goals;
    }

    private Translation2d convertToTranslation2d(double tx, double ty) {
        // Implement your own conversion logic here
        return new Translation2d(tx, ty);
    }

    public boolean hasTracks() {
        return goalTracker.hasTracks();
    }

    public List<GoalTracker.TrackReport> getTracks() {
        return goalTracker.getTracks();
    }

    public double getX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }
}
