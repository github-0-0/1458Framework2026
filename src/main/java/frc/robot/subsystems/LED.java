package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LEDS;

public class LED extends Subsystem {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int count;
    private long timer;
    private double time;
    private boolean isOn;

    private String currColor;
    private String setColor;

    private static LED m_Instance;
    private Timer timer2;

    public static LED getInstance() {
        if (m_Instance == null) {
            m_Instance = new LED();
            //m_Instance.TwoWayGradient(255, 0, 0, 69, 69, 69);
           // m_Instance.setThreeAlternatingColor(255,0,0,255,255,255,0,0,255);
        }
        return m_Instance;
    }
    @Override
    public void readPeriodicInputs() {
        time = Timer.getFPGATimestamp();
        
    }


    @Override
	public void writePeriodicOutputs() {
		
	}

    public LED() {
        led = new AddressableLED(3);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDS.ledLength);
        count = 100; //arbritrary positive number
        timer = System.currentTimeMillis();
        led.setLength(ledBuffer.getLength());
        led.start();
        timer2 = new Timer();
        isOn = true;
        currColor = "DNE";
    }

    public void rainbowPulse() {
        currColor = "RPulse";
        for(int i = Constants.LEDS.ledStart; i < 53; i++) {
            ledBuffer.setHSV(i, (53 + count - i) % 180 , 255, 255);
          }
          for(int i = 53; i < 106; i++) {
            ledBuffer.setHSV(i, (count + i - 53) % 180 , 255, 255);
          }
          count++;
          led.setData(ledBuffer);
    }

    //public void ThreeWayGradient(int r1, int g1, int b1, int r2, int g2, int b2, int r3, int g3, int b3) {
    //}

    public void TwoWayGradient(int r1, int g1, int b1, int r2, int g2, int b2) {
        currColor = "TWG";
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
        currColor = "Christmas";
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
        setColor = "Yellow";
        setSolidColor(255, 255, 0, 0, 117);
        
    }

    // Red = Error
    public void red() {
        setColor = "Red";
        setSolidColor(255, 0, 0, 0, 117);
        
    }

    // Pink = Elevator L4x
    public void pink() {
        setColor = "Pink";
        setSolidColor(200, 50, 50, 0, 117);
    }

    // White = Intake Coral
    public void white() {
        setColor = "White";
        setSolidColor(255, 255, 255, 0, 117);
      
    }


    public void orange() {
        setColor = "Orange";
        setSolidColor(255, 50, 0, 0, 117);
        
    }

    // Green = Intake Algae
    public void green() {
        setColor = "Green";
        setSolidColor(0, 255, 0, 0, 117);
    }

    
    public void lightBlue() {
        setColor = "Light Blue";
        setSolidColor(0, 255, 255, 0, 117);
    }

    // Blue = Enabled (Ground)
    public void blue() {
        setColor = "Blue";
        setSolidColor(0, 0, 255, 0, 116);
    }


    public void purple() {
        setColor = "Purple";
        setSolidColor(255,0, 255, 0, 117);
    }

    public void setSolidColor(int r, int g, int b, int startLEDnum, int endLEDnum) {
        if(setColor.equals(currColor)) {
            return;
        }
        setColor = currColor;
        for(int i = startLEDnum; i < endLEDnum && i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void blinkerLights(int r, int g, int b, double freq, int startLEDnum, int endLEDnum) {
        currColor = "Blink";
        if(timer2.hasElapsed(freq)) {
        if (isOn) {
            for(int i = startLEDnum; i < endLEDnum && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0,0,0);
            }
        }
        else {
            for(int i = startLEDnum; i < endLEDnum && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, r,g,b);
            }
        }
        isOn = !isOn;
        led.setData(ledBuffer);
        timer2.reset();
        }
        else {
            timer2.start();
        }
    }


    public void blinkerLightsFull(int r, int g, int b, double freq) {
        blinkerLights(r, g, b, freq, 0, 117);
    }

    public void blinkerLightsSnap(int mNum){
        if (mNum == 0) {
            blinkerLightsRight(255, 0, 0, 1);
        }
        else {
            blinkerLightsLeft(255, 0, 0, 1);
        }
     }

    public void blinkerLightsRight(int r, int g, int b, double freq) {
        if(timer2.hasElapsed(freq)) {
        if (isOn) {
            for(int i = 45; i < 117 && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0,0,0);
            }
        }
        else {
            for(int i = 45; i < 117 && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, r,g,b);
            }
        }
        isOn = !isOn;
        led.setData(ledBuffer);
        timer2.reset();
        }
        else {
            timer2.start();
        }
    }

    public void blinkerLightsLeft(int r, int g, int b, double freq) {
        if(timer2.hasElapsed(freq)) {
        if (isOn) {
            for(int i = 0; i < 47 && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0,0,0);
            }
        }
        else {
            for(int i = 0; i < 47 && i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, r,g,b);
            }
        }
        isOn = !isOn;
        led.setData(ledBuffer);
        timer2.reset();
        }
        else {
            timer2.start();
        }
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