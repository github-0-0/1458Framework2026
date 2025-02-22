package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class Laser {    
    public static LaserCan intakeLaser = new LaserCan(Ports.LaserCanIDCoralBack.getDeviceNumber());
    public static LaserCan shooterLaser = new LaserCan(Ports.LaserCanIDCoralFront.getDeviceNumber());
    public static LaserCan algaeShooterLaser = new LaserCan(Ports.LaserCanIDAlgae.getDeviceNumber());


    public Laser() {}


    public static double getMeasurementIntake() {
        if (Robot.isSimulation()) {
            return 200;

        }
        else {
            return intakeLaser.getMeasurement().distance_mm;
        }
    }

    public static double getMeasurementShooter() {
        if (Robot.isSimulation()) {
            return 200;
        }
        else {
        return shooterLaser.getMeasurement().distance_mm;
        }
    }

    public static double getMeasurementAlgaeShooter() {
        if (Robot.isSimulation()) {
            return 200;
        }
        else {
        return algaeShooterLaser.getMeasurement().distance_mm;
        }
    }

    public static boolean inRangeIntake() {
        return getMeasurementIntake() < 100;
    }
    public static boolean inRangeShooter() {
        return getMeasurementShooter() < 100;
    }

    public static boolean inRangeAlgaeShooter() {
        return getMeasurementAlgaeShooter() < 5;
    }

    public static void testLaser() {
        SmartDashboard.putNumber("Laser/Intake sensor",inRangeIntake()?1:0);
        SmartDashboard.putNumber("Laser/Shooter sensor",inRangeShooter()?1:0);
        SmartDashboard.putNumber("Laser/Algae sensor",inRangeAlgaeShooter()?1:0);
    }
}
