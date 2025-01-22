package frc.robot.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.FieldLayout;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionUpdate;
import frc.robot.subsystems.Subsystem;
import frc.robot.lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class VisionDevice extends Subsystem {
	private final VisionDeviceConstants mConstants;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private NetworkTable mConfigTable;
	private NetworkTable mOutputTable;
	private NetworkTable mCalibTable;

	private DoubleArraySubscriber mObservations;
	private IntegerSubscriber mFPS;

	public VisionDevice(VisionDeviceConstants constants) {
		mConstants = constants;
		mConfigTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName + "/configs");
		mCalibTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName + "/calibration");
		mOutputTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName + "/output");

		mObservations = mOutputTable
				.getDoubleArrayTopic("observations")
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
		if (mPeriodicIO.frames.size() == 0) {
			return;
		}
		for (int frame_idx = 0; frame_idx < mPeriodicIO.frames.size(); frame_idx++) {
			VisionFrame frame = mPeriodicIO.frames.get(frame_idx);
			double[] data = frame.frame_data;
			if (data.length == 0 || data[0] == 0)
				continue;
			double timestamp = frame.timestamp - VisionDeviceManager.getTimestampOffset();
			Pose2d camera_pose = null;

			switch ((int) data[0]) {
				// Multiple tags are seen and a merged pose is recieved
				case 1:
					camera_pose = new Pose2d(
							data[2],
							data[3],
							Rotation2d.fromRadians(
									new Rotation3d(new Quaternion(data[5], data[6], data[7], data[8])).getAngle()));
					break;
				// A single tag is used and there are two uncertain positions
				case 2:
					// Reprojection errors for the possible positions (lower is better)
					double e_0 = data[1];
					double e_1 = data[9];
					// If the reprojection error for one pose is much lower, use it
					if (e_0 < e_1 * 0.15) {
						camera_pose = new Pose2d(
								data[2],
								data[3],
								Rotation2d.fromRadians(
										new Rotation3d(new Quaternion(data[5], data[6], data[7], data[8])).getAngle()));
					} else if (e_1 < e_0 * 0.15) {
						camera_pose = new Pose2d(
								data[10],
								data[11],
								Rotation2d.fromRadians(
										new Rotation3d(new Quaternion(data[13], data[14], data[15], data[16]))
												.getAngle()));
					}
				default:
					break;
			}
			if (camera_pose == null)
				continue;

			double std_dev_multiplier = 1.0;

			List<Pose3d> tagPoses = new ArrayList<>();
			// Iterate through all remaining data to find tags involved in calculation
			for (int tag_idx = (data[0] == 1 ? 9 : 17); tag_idx < data.length; tag_idx++) {
				int tagId = (int) data[tag_idx];
				Optional<Pose3d> tagPose = FieldLayout.kTagMap.getTagPose((int) tagId);
				tagPose.ifPresent(tagPoses::add);
			}
			if (tagPoses.size() == 0) {
				continue;
			}

			double total_tag_dist = 0.0;
			double lowest_dist = Double.POSITIVE_INFINITY;
			for (Pose3d pose3d : tagPoses) {
				Pose2d pose2 = pose3d.toPose2d();
				final Pose2d inverse = inverse(pose2.getTranslation(), pose2.getRotation());
				final Twist2d logTwist = log(new Pose2d(
						inverse.getTranslation().plus(camera_pose.getTranslation().rotateBy(inverse.getRotation())),
						inverse.getRotation().rotateBy(camera_pose.getRotation())));
				double dist = logTwist.dy == 0
						? Math.abs(logTwist.dx)
						: Math.hypot(logTwist.dx, logTwist.dy);
				total_tag_dist += dist;
				lowest_dist = Math.min(dist, lowest_dist);
			}
			double avg_dist = total_tag_dist / tagPoses.size();

			// Estimate standard deviation of vision measurement
			double xyStdDev = std_dev_multiplier
					* (0.1)
					* ((0.01 * Math.pow(lowest_dist, 2.0)) + (0.005 * Math.pow(avg_dist, 2.0)))
					/ tagPoses.size();
			xyStdDev = Math.max(0.02, xyStdDev);

			// LogUtil.recordPose3d("Vision " + mConstants.kTableName + "/Tag Poses",
			tagPoses.toArray(new Pose3d[0]);
			SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/N Tags Seen", tagPoses.size());
			SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/Calculated STDev", xyStdDev);
			// LogUtil.recordPose2d("Vision " + mConstants.kTableName + "/Camera Pose",
			// camera_pose);
			// LogUtil.recordPose2d(
			// "Vision " + mConstants.kTableName + "/Robot Pose",
			// camera_pose.transformBy(mConstants.kRobotToCamera));
			// LogUtil.recordPose2d(
			// "Vision " + mConstants.kTableName + "/Relevant Odometry Pose",
			// RobotState.getInstance().getFieldToVehicle(timestamp));

			if (VisionDeviceManager.visionDisabled()) {
				continue;
			}

			RobotState.getInstance()
				.addVisionUpdate(
					new VisionUpdate(
						timestamp,
						camera_pose.getTranslation(),
						mConstants.kRobotToCamera.getTranslation(),
						xyStdDev
					)
				);

			double rotation_degrees = camera_pose
					.transformBy(mConstants.kRobotToCamera)
					.getRotation()
					.getDegrees()
					+ 180.0;

			// Avoid angle wrapping issues
			if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() != Alliance.Red) {
				rotation_degrees = Util.boundAngleNeg180to180Degrees(rotation_degrees);
			} else {
				rotation_degrees = Util.boundAngle0to360Degrees(rotation_degrees);
			}

			SmartDashboard.putNumber("Vision Heading/" + mConstants.kTableName, rotation_degrees);
			VisionDeviceManager.getInstance().getMovingAverage().addNumber(rotation_degrees);
		}
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.fps = mFPS.get();

		TimestampedDoubleArray[] queue = mObservations.readQueue();
		VisionFrame[] updates = new VisionFrame[queue.length];
		for (int i = 0; i < queue.length; i++) {
			updates[i] = new VisionFrame();
			updates[i].timestamp = queue[i].timestamp / 1000000.0;
			updates[i].frame_data = queue[i].value;
		}
		Arrays.sort(updates, Comparator.comparingDouble(VisionFrame::getTimestamp));
		mPeriodicIO.frames = Arrays.asList(updates);
		try {
			if (mPeriodicIO.frames.size() >= 6) {
				mPeriodicIO.frames = mPeriodicIO.frames.subList(mPeriodicIO.frames.size() - 6,
						mPeriodicIO.frames.size() - 1);
			}
		} catch (Exception e) {
			System.out.println(e.getStackTrace());
		}

		mPeriodicIO.is_connected = !(Timer.getFPGATimestamp() - mPeriodicIO.latest_timestamp > 1.0);

		if (updates.length > 0) {
			mPeriodicIO.latest_timestamp = updates[updates.length - 1].timestamp;
		}
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
