package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.VisionPoseAcceptor;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import frc.robot.lib.util.MovingAverageTwist2d;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.interpolation.*;


public class RobotState {
	private static final int OBSERVATION_BUFFER_SIZE = 50;
	private static final Matrix<N2, N1> STATE_STD_DEVS = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
	private static final Matrix<N2, N1> LOCAL_MEASUREMENT_STD_DEVS = VecBuilder.fill(
			Math.pow(0.02, 1), // vision
			Math.pow(0.02, 1));
	public static Optional<VisionUpdate> mLatestVisionUpdate;

	private static Optional<InterpolatingTranslation2d> mInitialFieldToOdom = Optional.empty();
	private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingPose2d> mOdometryToVehicle;
	private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingTranslation2d> mFieldToOdometry;
	private static ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
	private static VisionPoseAcceptor mPoseAcceptor;

	private static Twist2d mVehicleVelocityMeasured;
	private static Twist2d mVehichleVelocityPredicted;
	private static MovingAverageTwist2d mVehicleVelocityMeasuredFiltered;

	private static boolean mHasRecievedVisionUpdate = false;
	private static boolean mIsInAuto = false;
	private static Optional<Alliance> mAlliance = null;

	public static double mLastTimestamp = 0;

	/**
	 * Adds new odometry pose update.
	 *
	 * @param now                Timestamp of observation.
	 * @param odometryPose      Reported pose from odometry.
	 * @param measuredVelocity  Measured field-relative velocity.
	 * @param predictedVelocity Predicted field-relative velocity (usually swerve
	 *                           setpoint).
	 */

	public static synchronized void reset(double now, InterpolatingPose2d initialOdomToVehicle) {
		mOdometryToVehicle = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingPose2d>(
				OBSERVATION_BUFFER_SIZE);
		mOdometryToVehicle.put(new InterpolatingDouble(now), initialOdomToVehicle);
		mFieldToOdometry = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingTranslation2d>(
				OBSERVATION_BUFFER_SIZE);
		mFieldToOdometry.put(new InterpolatingDouble(now), getmInitialFieldToOdom());
		mVehicleVelocityMeasured = new Twist2d();
		mVehichleVelocityPredicted = new Twist2d();
		mVehicleVelocityMeasuredFiltered = new MovingAverageTwist2d(25);

		mLastTimestamp = now;
		mLatestVisionUpdate = Optional.empty();
		mPoseAcceptor = new VisionPoseAcceptor();
	}

	/**
	 * Reconstructs Kalman Filter.
	 */
	public static synchronized void resetKalman() {
		mKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2> (
				Nat.N2(), // Dimensions of output (x, y)
				Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
				Nat.N2(), // Dimensions of vision (x, y)
				(x, u) -> u, // The derivative of the output is predicted shift (always 0)
				(x, u) -> x, // The output is position (x, y)
				STATE_STD_DEVS, // Standard deviation of position (uncertainty propagation with no vision)
				LOCAL_MEASUREMENT_STD_DEVS, // Standard deviation of vision measurements
				Constants.LOOPER_DT);
	}

	public static synchronized void addOdometryUpdate(
			double now, InterpolatingPose2d odometryPose, Twist2d measuredVelocity, Twist2d predictedVelocity) {
		mOdometryToVehicle.put(new InterpolatingDouble(now), new InterpolatingPose2d(odometryPose));
		mKalmanFilter.predict(
				VecBuilder.fill(0.0, 0.0), Constants.LOOPER_DT); // Propagate error of  current vision prediction
		mVehicleVelocityMeasured = measuredVelocity;
		mVehicleVelocityMeasuredFiltered.add(measuredVelocity);
		mVehichleVelocityPredicted = predictedVelocity;

		mLastTimestamp = now;
	}

	/**
	 * Adds new vision pose update.
	 *
	 * @param update Info about vision update.
	 */
	public static synchronized void addVisionUpdate(VisionUpdate update) {
		// If it's the first update don't do filtering
		if (mLatestVisionUpdate.isEmpty() || mInitialFieldToOdom.isEmpty()) {
			double visionTimestamp = update.mTimestamp;
			mLastTimestamp = update.mTimestamp;
			Pose2d proximateDtPose = mOdometryToVehicle.getInterpolated(new InterpolatingDouble(visionTimestamp));
			Translation2d fieldToVision = update.mFieldToCamera
					.plus(update.getmRobotToCamera()
							.rotateBy(getLatestOdomToVehicle().getValue().getRotation()).unaryMinus());
			Translation2d odomToVehicleTranslation = proximateDtPose.getTranslation();
			Translation2d fieldToOdom = fieldToVision
					.plus(odomToVehicleTranslation.unaryMinus());
			mFieldToOdometry.put(new InterpolatingDouble(visionTimestamp), new InterpolatingTranslation2d(fieldToOdom));
			mInitialFieldToOdom = Optional.of(mFieldToOdometry.lastEntry().getValue());
			mKalmanFilter.setXhat(0, fieldToOdom.getX());
			mKalmanFilter.setXhat(1, fieldToOdom.getY());
			mLatestVisionUpdate = Optional.ofNullable(update);

			Drive.getInstance().resetOdometry(new Pose2d(fieldToOdom, getLatestOdomToVehicle().getValue().getRotation()));
		} else {
			double visionTimestamp = mLatestVisionUpdate.get().mTimestamp;
			mLastTimestamp = mLatestVisionUpdate.get().mTimestamp;
			Pose2d proximateDtPose = mOdometryToVehicle.getInterpolated(new InterpolatingDouble(visionTimestamp));
			mLatestVisionUpdate = Optional.ofNullable(update);
			Translation2d fieldToVision = mLatestVisionUpdate
					.get().mFieldToCamera
					.plus(mLatestVisionUpdate
							.get()
							.getmRobotToCamera()
							.rotateBy(proximateDtPose.getRotation()).unaryMinus());

			if (mPoseAcceptor.shouldAcceptVision(
					visionTimestamp,
					new Pose2d(fieldToVision, new Rotation2d()),
					getLatestFieldToVehicle(),
					mVehicleVelocityMeasured,
					mIsInAuto)) {
				Translation2d fieldToOdom = fieldToVision.plus(
						proximateDtPose.getTranslation().unaryMinus());
				try {
					Vector<N2> stdevs = update.mXyStdev;
					mKalmanFilter.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									fieldToOdom.getX(),
									fieldToOdom.getY()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					mFieldToOdometry.put(
							new InterpolatingDouble(visionTimestamp),
							new InterpolatingTranslation2d(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))));
					if (!getHasRecievedVisionUpdate()) {
						mHasRecievedVisionUpdate = true;
					}
				} catch (Exception e) {
					DriverStation.reportError(update.mXyStdev + "//QR Decomposition failed: ", e.getStackTrace());
				}
			}
		}
	}

	/**
	 * Gets initial odometry error. Odometry initializes to the origin, eile the
	 * robot starts at an unknown position on the field.
	 *
	 * @return Initial odometry error translation.
	 */
	public static synchronized InterpolatingTranslation2d getmInitialFieldToOdom() {
		if (mInitialFieldToOdom.isEmpty())
			return new InterpolatingTranslation2d();
		return mInitialFieldToOdom.get();
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public static synchronized Pose2d getLatestFieldToVehicle() {
		Pose2d odomToVehicle = getLatestOdomToVehicle().getValue();

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(Util.translateBy(fieldToOdom, odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public static synchronized Pose2d getFieldToVehicle(double timestamp) {
		Pose2d odomToVehicle = getOdomToVehicle(timestamp);

		Translation2d fieldToOdom = getFieldToOdom(timestamp);
		return new Pose2d(Util.translateBy(fieldToOdom, odomToVehicle.getTranslation()), odomToVehicle.getRotation());

	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookaheadTime Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public static synchronized Pose2d getPredictedFieldToVehicle(double lookaheadTime) {
		Pose2d odomToVehicle = getPredictedOdomToVehicle(lookaheadTime);

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(Util.translateBy(fieldToOdom, odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * @return Latest odometry pose.
	 */
	public static synchronized Map.Entry<InterpolatingDouble, InterpolatingPose2d> getLatestOdomToVehicle() {
		return mOdometryToVehicle.lastEntry();
	}

	/**
	 * Gets odometry pose from history. Linearly interpolates between gaps.
	 *
	 * @param timestamp Timestamp to loop up.
	 * @return Odometry relative robot pose at timestamp.
	 */
	public static synchronized Pose2d getOdomToVehicle(double timestamp) {
		return mOdometryToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * Gets interpolated odometry pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookaheadTime Scalar for predicted velocity.
	 * @return Predcited odometry pose at lookahead time.
	 */
	public static synchronized Pose2d getPredictedOdomToVehicle(double lookaheadTime) {
		return getLatestOdomToVehicle()
				.getValue()
				.transformBy(new Transform2d(new Pose2d(),
						Util.expMap(Util.scaledTwist2d(mVehichleVelocityPredicted, lookaheadTime))));
	}

	/**
	 * @return Latest odometry error translation.
	 */
	public static synchronized Translation2d getLatestFieldToOdom() {
		return getFieldToOdom(mFieldToOdometry.lastKey().value);
	}

	/**
	 * Gets odometry error translation at timestamp. Linearly interpolates between
	 * gaps.
	 * 
	 * @param timestamp Timestamp to look up.
	 * @return Odometry error at timestamp.
	 */
	public static synchronized Translation2d getFieldToOdom(double timestamp) {
		if (mFieldToOdometry.isEmpty())
			return new Translation2d();
		return mFieldToOdometry.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * @return Predicted robot velocity from last odometry update.
	 */
	public static synchronized Twist2d getPredictedVelocity() {
		return mVehichleVelocityPredicted;
	}

	/**
	 * @return Measured robot velocity from last odometry update.
	 */
	public static synchronized Twist2d getMeasuredVelocity() {
		return mVehicleVelocityMeasured;
	}

	/**
	 * @return Measured robot velocity smoothed using a moving average filter.
	 */
	public static synchronized Twist2d getSmoothedVelocity() {
		return mVehicleVelocityMeasuredFiltered.getAverage();
	}

	/**
	 * @return Gets if estimator has recieved a vision update.
	 */
	public static synchronized boolean getHasRecievedVisionUpdate() {
		return mHasRecievedVisionUpdate;
	}

	/**
	 * Updates tracker to use stricter auto vision filtering.
	 * @param inAuto If auto filters should be used.
	 */
	public static synchronized void setIsInAuto(boolean inAuto) {
		mIsInAuto = inAuto;
	}

	/**
	 * Class to hold information about a vision update.
	 */
	public static class VisionUpdate {
		private double mTimestamp;
		private Translation2d mFieldToCamera;
		private Translation2d mRobotToCamera;
		private Vector<N2> mXyStdev;

		public VisionUpdate(
				double timestamp, Translation2d fieldToCamera, Translation2d robotToCamera, Vector<N2> xyStdev) {
			mTimestamp = timestamp;
			mFieldToCamera = fieldToCamera;
			mRobotToCamera = robotToCamera;
			mXyStdev = xyStdev;
		}

		public double getmTimestamp() {
			return mTimestamp;
		}

		public Translation2d getFieldToVehicle() {
			return mFieldToCamera;
		}

		public Translation2d getmRobotToCamera() {
			return mRobotToCamera;
		}

		public Vector<N2> getXYStdev() {
			return mXyStdev;
		}
	}

	public static void setAlliance(Optional<Alliance> alliance) {
		mAlliance = alliance;
		System.out.println(alliance.toString());
	}

	public static Optional<Alliance> getAlliance() {
		return mAlliance;
	}
}
