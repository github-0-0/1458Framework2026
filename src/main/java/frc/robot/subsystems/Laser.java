package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class Laser {
    public LaserCan lc;

    public Laser(int ID) throws ConfigurationFailedException {
        lc = new LaserCan(ID);
        lc.setRangingMode(LaserCan.RangingMode.SHORT);
        lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    }

    public double getMeasurement() {
        return lc.getMeasurement().distance_mm;
    }
}
