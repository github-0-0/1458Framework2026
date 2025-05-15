package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.lib.loops.ILooper;
import frc.robot.lib.loops.Loop;

public class LED extends Subsystem {
    private static LED m_Instance;

    public static LED getInstance() {
        if (m_Instance == null) {
            m_Instance = new LED();
        }
        return m_Instance;
    }

    public enum State {
        OFF,
        SOLID_COLOR
    }

    public State mState;
    
    private AddressableLED mLed;
    private AddressableLEDBuffer mLedBuffer;
    private Color mSolidColor;

    public LED() {
        mLed = new AddressableLED(Constants.LED.LED_PORT_ID);
        mLedBuffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);
        mState = State.OFF;
    }


    @Override
    public void readPeriodicInputs() {
        setLedBuffer();
    }

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {}
		});
    }

    @Override
	public void writePeriodicOutputs() {
        mLed.setData(mLedBuffer);
	}

    public void setLedBuffer() {
        switch (mState) {
            case OFF:
                for (int i = Constants.LED.LED_START; i < Constants.LED.LED_LENGTH; i++) {
                    mLedBuffer.setHSV(i, 0, 0, 0);
                }
                break;
            case SOLID_COLOR:
                for (int i = Constants.LED.LED_START; i < Constants.LED.LED_LENGTH; i++) {
                    mLedBuffer.setLED(i, mSolidColor);
                }
                break;
            default:
                setOff();
                System.out.println("Invalid LED state");
                break;
        }
    }

    public void setOff() {
        mState = State.OFF;
    }

    public void setSolidColor(Color color) {
        mSolidColor = color;
        mState = State.SOLID_COLOR;
    }
}