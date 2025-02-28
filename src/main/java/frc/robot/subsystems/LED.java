
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LED extends Subsystem {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int count;
    private long timer;

    private static LED m_Instance;


    public static LED getInstance() {
        if (m_Instance == null) {
            m_Instance = new LED();
            m_Instance.setThreeAlternatingColor(255,0,0,255,255,255,0,0,255);
        }
        return m_Instance;
    }

    public LED() {
        led = new AddressableLED(3);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDS.ledLength);
        count = 100; //arbritrary positive number
        timer = System.currentTimeMillis();
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void rainbowPulse() {
        for(int i = Constants.LEDS.ledStart; i < 53; i++) {
            ledBuffer.setHSV(i, (53 + count - i) % 180 , 255, 255);
          }
          for(int i = 53; i < 106; i++) {
            ledBuffer.setHSV(i, (count + i - 53) % 180 , 255, 255);
          }
          count++;
          led.setData(ledBuffer);
    }

    public void ThreeWayGradient(int r1, int g1, int b1, int r2, int g2, int b2, int r3, int g3, int b3) {

    }

    public void TwoWayGradient(int r1, int g1, int b1, int r2, int g2, int b2) {
        int length = ledBuffer.getLength();
        for (int i = 0; i < length; i++) {
            double ratio = (double) i / (length);
            int r = (int) ((1 - ratio) * r1 + ratio * r2);
            int g = (int) ((1 - ratio) * g1 + ratio * g2);
            int b = (int) ((1 - ratio) * b1 + ratio * b2);
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void ChristmasStream() {
        int red = 0, green = 0, blue = 0;
        for(int i = Constants.LEDS.ledStart; i < 106; i++) {
            if(i % 53 < 26) {
                red = 255;
                green = 0;
                blue = 0;
            }
            else {
                red = 0;
                green = 255;
                blue = 0;
            }

            if(timer < System.currentTimeMillis()) {
                timer = System.currentTimeMillis() + 100;
                count++;
            }
            ledBuffer.setRGB((i + count) % 108, red, green ,blue);
        }
          led.setData(ledBuffer);
    }


    public void random() {
        setSolidColor((int)(Math.random() * 50) + 100, (int)(Math.random() * 50) + 100, (int)(Math.random() * 50) + 100, 0, 117);
    }
    
    public void yellow() {
        setSolidColor(255, 255, 0, 0, 117);
    }

    // Red = Error
    public void red() {
        setSolidColor(255, 0, 0, 0, 117);
    }

    // Pink = Elevator L4x
    public void pink() {
        setSolidColor(200, 50, 50, 0, 117);
    }

    // White = Intake Coral
    public void white() {
        setSolidColor(255, 255, 255, 0, 117);
    }


    public void orange() {
        setSolidColor(255, 50, 0, 0, 117);
    }

    // Green = Intake Algae
    public void green() {
        setSolidColor(0, 255, 0, 0, 117);
    }

    
    public void lightBlue() {
        setSolidColor(0, 255, 255, 0, 117);
    }

    // Blue = Enabled (Ground)
    public void blue() {
        for (int i = 0; i<=100; i+=1) {
            System.out.println("blue LED should work");
        }
        setSolidColor(0, 0, 255, 0, 116);
    }


    public void purple() {
        setSolidColor(255,0, 255, 0, 117);
    }

    public void setSolidColor(int r, int g, int b, int startLEDnum, int endLEDnum) {
        for(int i = startLEDnum; i < endLEDnum && i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void setAlternatingColorSolid(int r1, int g1, int b1, int r2, int g2, int b2) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 2 == 0) {
                ledBuffer.setRGB(i, r1, g1, b1);
            } else {
                ledBuffer.setRGB(i, r2, g2, b2);
            }
        }
        led.setData(ledBuffer);
    }

    public void setThreeAlternatingColor(int r1, int g1, int b1, int r2, int g2, int b2, int r3, int g3, int b3) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 3 == 0) {
                ledBuffer.setRGB(i, r3, g3, b3);
            } 
            else if (i % 2 == 0){
                ledBuffer.setRGB(i, r2, g2, b2);
            }
            else {
                ledBuffer.setRGB(i, r1, g1, b1);
            }
        }
        led.setData(ledBuffer);
    }
}