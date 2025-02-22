package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.lib.util.TunableNumber;
import frc.robot.lib.util.MovingAverage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class VisionDeviceManager extends Subsystem {
	private static VisionDeviceManager mInstance;

	public static VisionDeviceManager getInstance() {
		if (mInstance == null) {
			mInstance = new VisionDeviceManager();
		}
		return mInstance;
	}

	private VisionDevice mLeftCamera;
	private VisionDevice mRightCamera;
	private VisionDevice mFrontCamera;
	private VisionDevice mBackCamera;

	private List<VisionDevice> mAllCameras;

	private static TunableNumber timestampOffset = new TunableNumber("VisionTimestampOffset", (0.1), false);

	private MovingAverage mHeadingAvg = new MovingAverage(100);
	private double mMovingAvgRead = 0.0;

	private static boolean disable_vision = false;

	private VisionDeviceManager() {
		mLeftCamera = new VisionDevice(Constants.kLeftVisionDevice);
		mRightCamera = new VisionDevice(Constants.kRightVisionDevice);
		mFrontCamera = new VisionDevice(Constants.kFrontVisionDevice);
		mBackCamera = new VisionDevice(Constants.kBackVisionDevice);
		mAllCameras = List.of(mLeftCamera, mRightCamera, mFrontCamera, mBackCamera);
	}

	@Override
	public void readPeriodicInputs() {
		//System.out.println("VisionDevice.readPeriodic()");
		mAllCameras.forEach(VisionDevice::readPeriodicInputs);
		mMovingAvgRead = mHeadingAvg.getAverage();
	}

	@Override
	public void writePeriodicOutputs() {
		mAllCameras.forEach(VisionDevice::writePeriodicOutputs);
	}

	@Override
	public void outputTelemetry() {
		mAllCameras.forEach(VisionDevice::outputTelemetry);
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAverageRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
	}

	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public synchronized MovingAverage getMovingAverage() {
		return mHeadingAvg;
	}

	public synchronized boolean fullyConnected() {
		return mLeftCamera.isConnected()
			&& mRightCamera.isConnected()
			&& mFrontCamera.isConnected()
			&& mBackCamera.isConnected();
	}

	public synchronized VisionDevice getLeftVision() {
		return mLeftCamera;
	}

	public synchronized VisionDevice getRightVision() {
		return mRightCamera;
	}

	public synchronized VisionDevice getFrontVision() {
		return mFrontCamera;
	}

	public synchronized VisionDevice getBackVision() {
		return mBackCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset.get();
	}

	public static boolean visionDisabled() {
		return disable_vision;
	}

	public static void setDisableVision(boolean disable) {
		disable_vision = disable;
	}
}
