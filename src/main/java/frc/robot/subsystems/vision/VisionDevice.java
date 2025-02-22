package frc.robot.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.FieldLayout;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionUpdate;
import frc.robot.lib.drivers.Pigeon;
import frc.robot.subsystems.Subsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Vector;

public class VisionDevice extends Subsystem {
	private final VisionDeviceConstants mConstants;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private Pigeon mPigeon;

	private NetworkTable mConfigTable;
	private NetworkTable mOutputTable;
	private NetworkTable mCalibTable;

	private IntegerSubscriber mVisible;
	private DoubleArraySubscriber mObservations;
	private DoubleArraySubscriber mStdDevs;
	private IntegerSubscriber mFPS;

	public Field2d robotField;

	public VisionDevice(VisionDeviceConstants constants) {
		robotField = new Field2d();
		SmartDashboard.putData("VisionDevice/" + constants.kTableName, robotField);

		mPigeon = Pigeon.getInstance();

		mConstants = constants;
		mConfigTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName + "/configs");
		mCalibTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName + "/calibration");
		mOutputTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName);

		mVisible = mOutputTable
				.getIntegerTopic("tv")
				.subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mObservations = mOutputTable
				.getDoubleArrayTopic("botpose_orb_wpiblue")
				.subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mStdDevs = mOutputTable
				.getDoubleArrayTopic("stddevs")
				.subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mFPS = mOutputTable
				.getIntegerTopic("fps")
				.subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mConfigTable.getDoubleTopic("fiducial_size_m").publish().set(FieldLayout.kApriltagWidth);
		try {
			mConfigTable
					.getStringTopic("tag_layout")
					.publish()
					.set(new ObjectMapper().writeValueAsString(FieldLayout.kTagMap));
		} catch (JsonProcessingException e) {
			throw new RuntimeException("Failed to serialize apriltag layout");
		}

		mConfigTable.getEntry("camera_exposure").setDouble(mPeriodicIO.camera_exposure);
		mConfigTable.getEntry("camera_auto_exposure").setDouble(mPeriodicIO.camera_auto_exposure ? 0.0 : 1.0);
		mConfigTable.getEntry("camera_gain").setDouble(mPeriodicIO.camera_gain);
	}

	private static Twist2d log(final Pose2d transform) {
		final double dtheta = transform.getRotation().getRadians();
		final double half_dtheta = 0.5 * dtheta;
		final double cos_minus_one = transform.getRotation().getCos() - 1.0;
		double halftheta_by_tan_of_halfdtheta;
		if (Math.abs(cos_minus_one) < 1E-9) {
			halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
		} else {
			halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
		}
		final Translation2d translation_part = transform.getTranslation()
				.rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
		return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
	}

	private Pose2d inverse(Translation2d translation, Rotation2d rotation) {
		Rotation2d rotation_inverted = Rotation2d.fromRadians(-rotation.getRadians());
		return new Pose2d(new Translation2d(-translation.getX(), -translation.getY()).rotateBy(rotation_inverted),
				rotation_inverted);
	}

	private void processFrames() {
//		System.out.println("VisionDevice.processFrame");
		if (mVisible.get() == 0) {
			return;
		}
//		System.out.println("VisionDevice.processFrame, mVisible is true");

		double[] mt2Pose = mObservations.get();
		double[] stdDevs = mStdDevs.get();
		double timestamp = Timer.getFPGATimestamp() * Math.pow(10, 6) - VisionDeviceManager.getTimestampOffset();
		
		if (mt2Pose.length == 0) {
//			System.out.println("VisionDevice.processFrame, mt2Pose is zero length, mt2 from helper=" );
			return;
		}

//		System.out.println("VisionDevice.processFrame, mt2Pose.length=" + mt2Pose.length);


		LimelightHelpers.SetRobotOrientation(mConstants.kTableName, mPigeon.getYaw().getDegrees(), 0, 0, 0, 0, 0);	

		Pose2d botPose = new Pose2d(mt2Pose[0], mt2Pose[1], new Rotation2d(mt2Pose[5] * Math.PI / 180));
		Vector<N2> stdDevsVec = VecBuilder.fill(stdDevs[6], stdDevs[7]);

		robotField.setRobotPose(botPose);

		RobotState.getInstance()
				.addVisionUpdate(
						new VisionUpdate(
								timestamp,
								botPose.getTranslation(),
								new Translation2d(0, 0),
								stdDevsVec),
						botPose.getRotation());
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.fps = mFPS.get();

		mPeriodicIO.is_connected = !(Timer.getFPGATimestamp() - mPeriodicIO.latest_timestamp > 1.0);

		processFrames();
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber(
				"Vision " + mConstants.kTableName + "/Last Update Timestamp Timestamp", mPeriodicIO.latest_timestamp);
		SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/N Queued Updates", mPeriodicIO.frames.size());
		SmartDashboard.putBoolean("Vision " + mConstants.kTableName + "/is Connnected", mPeriodicIO.is_connected);
	}

	@Override
	public void writePeriodicOutputs() {
		// No-op
	}

	public void captureCalibrationFrame() {
		mCalibTable.getEntry("wants_frames").setBoolean(true);
	}

	public boolean isConnected() {
		return mPeriodicIO.is_connected;
	}

	public static class PeriodicIO {
		// inputs
		double camera_exposure = 20;
		boolean camera_auto_exposure = false;
		double camera_gain = 10;

		// Outputs
		long fps = -1;
		double latest_timestamp = 0.0;
		List<VisionFrame> frames = new ArrayList<VisionFrame>();
		boolean is_connected;
	}
}
