package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import au.grapplerobotics.LaserCan;

public class DigitalSensor {
    
    public static DigitalInput level0 = new DigitalInput(0);
    public static DigitalInput level1 = new DigitalInput(1);
    public static DigitalInput level2 = new DigitalInput(2);
    public static DigitalInput level3 = new DigitalInput(3);
    public static DigitalInput level4 = new DigitalInput(4);
    
    
    

    public DigitalSensor() {
    }

    public static boolean getSensor(int channel) {
        switch (channel) {
            case 0:
                return level0.get();
            case 1:
                return level1.get();
            case 2:
                return level2.get();
            case 3:
                return level3.get();
            case 4:
                return level4.get();
            default:
                return false;
        }
    }
}