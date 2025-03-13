package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.LegacySwerveControlRequestParameters;

import au.grapplerobotics.LaserCan;

public class DigitalSensor {
    
    public static DigitalInput ground = new DigitalInput(0);
    public static DigitalInput level2 = new DigitalInput(1);
    public static DigitalInput level3 = new DigitalInput(2);
    public static DigitalInput level4 = new DigitalInput(3);
    public static DigitalInput algae = new DigitalInput(4);
    
    
    

    public DigitalSensor() {}

    public static boolean getSensor(int channel) {
        switch (channel) {
            case 0:
                return !ground.get();
            case 1:
                return !level2.get();
            case 2:
                return !level3.get();
            case 3:
                return !level4.get();
            case 4:
                return !algae.get();
            default:
                return false;
        }
    }
}