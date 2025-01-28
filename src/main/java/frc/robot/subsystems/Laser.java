package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.Constants;

public class Laser {
    public static LaserCan intakeLaser = new LaserCan(Constants.Shooter.kIntakeLimitSwitchId);
    public static LaserCan shooterLaser = new LaserCan(Constants.Shooter.kShooterLimitSwitchId);

    public Laser() {}

    public static double getMeasurementIntake() {
        return intakeLaser.getMeasurement().distance_mm;
    }

    public static double getMeasurementShooter() {
        return shooterLaser.getMeasurement().distance_mm;
    }

    public static boolean inRangeIntake() {
        LaserCan.Measurement measurement = intakeLaser.getMeasurement();
        return (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }

    public static boolean inRangeShooter() {
        LaserCan.Measurement measurement = shooterLaser.getMeasurement();
        return (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }

}
