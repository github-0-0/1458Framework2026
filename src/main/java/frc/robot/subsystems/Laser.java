package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.Constants;
import frc.robot.Ports;

public class Laser {    
    public static LaserCan intakeLaser = new LaserCan(Ports.LaserCanIDCoralBack.getDeviceNumber());
    public static LaserCan shooterLaser = new LaserCan(Ports.LaserCanIDCoralFront.getDeviceNumber());
    public static LaserCan algaeShooterLaser = new LaserCan(Ports.LaserCanIDAlgae.getDeviceNumber());

    public Laser() {}

    public static double getMeasurementIntake() {
        return intakeLaser.getMeasurement().distance_mm;
    }

    public static double getMeasurementShooter() {
        return shooterLaser.getMeasurement().distance_mm;
    }

    public static double getMeasurementAlgaeShooter() {
        return algaeShooterLaser.getMeasurement().distance_mm;
    }

    public static boolean inRangeIntake() {
        LaserCan.Measurement measurement = intakeLaser.getMeasurement();
        return (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }

    public static boolean inRangeShooter() {
        LaserCan.Measurement measurement = shooterLaser.getMeasurement();
        return (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }

    public static boolean inRangeAlgaeShooter() {
        LaserCan.Measurement measurement = algaeShooterLaser.getMeasurement();
        return (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }
}
