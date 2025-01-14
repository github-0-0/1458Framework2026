package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.Matrix;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.InterpolatingPose2d;
import frc.robot.lib.util.InterpolatingTranslation2d;
import frc.robot.lib.util.Interpolable;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.MovingAverageTwist2d;
import frc.robot.lib.util.Util;
//dc.10.25.2024 TODO: a dummy RobotState class to pass compilation, placeholder for actual implementation
//priority level 2, for presentation purpose
public class RobotState {
	private static RobotState mInstance;

	public static RobotState getInstance() {
		if (mInstance == null) {
			mInstance = new RobotState();
		}
		return mInstance;
	}

	private static final int kObservationBufferSize = 50;
	private static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
	private static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(
			Math.pow(0.02, 1), // vision
			Math.pow(0.02, 1));
	//private Optional<VisionUpdate> mLatestVisionUpdate;

	private Optional<InterpolatingTranslation2d> initial_field_to_odom = Optional.empty();
	private InterpolatingTreeMap<InterpolatingDouble, InterpolatingPose2d> odometry_to_vehicle;
	private InterpolatingTreeMap<InterpolatingDouble, InterpolatingTranslation2d> field_to_odometry;
	private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
	//private VisionPoseAcceptor mPoseAcceptor;

	private Twist2d vehicle_velocity_measured;
	private Twist2d vehicle_velocity_predicted;
	private MovingAverageTwist2d vehicle_velocity_measured_filtered;

	private boolean mHasRecievedVisionUpdate = false;
	private boolean mIsInAuto = false;

	public RobotState() {
		reset(0.0, new InterpolatingPose2d());
	}

	/**
	 * Adds new odometry pose update.
	 *
	 * @param now                Timestamp of observation.
	 * @param odometry_pose      Reported pose from odometry.
	 * @param measured_velocity  Measured field-relative velocity.
	 * @param predicted_velocity Predicted field-relative velocity (usually swerve
	 *                           setpoint).
	 */
	
	 public synchronized void reset(double now, InterpolatingPose2d initial_odom_to_vehicle) {
		odometry_to_vehicle = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingPose2d>(kObservationBufferSize);
		odometry_to_vehicle.put(new InterpolatingDouble(now), initial_odom_to_vehicle);
		field_to_odometry = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingTranslation2d>(kObservationBufferSize);
		field_to_odometry.put(new InterpolatingDouble(now), getInitialFieldToOdom());
		vehicle_velocity_measured = new Twist2d();
		vehicle_velocity_predicted = new Twist2d();
		vehicle_velocity_measured_filtered = new MovingAverageTwist2d(25);
		//mLatestVisionUpdate = new Optional();
		//mPoseAcceptor = new VisionPoseAcceptor();
	}
	

	public synchronized void addOdometryUpdate(
        double now, InterpolatingPose2d odometry_pose, Twist2d measured_velocity, Twist2d predicted_velocity) {
		odometry_to_vehicle.put(new InterpolatingDouble(now), odometry_pose);
		//mKalmanFilter.predict(
		//		VecBuilder.fill(0.0, 0.0), Constants.kLooperDt); // Propagate error of current vision prediction
		vehicle_velocity_measured = measured_velocity;
		vehicle_velocity_measured_filtered.add(measured_velocity);
		vehicle_velocity_predicted = predicted_velocity;
    }

	/**
	 * Gets initial odometry error. Odometry initializes to the origin, while the
	 * robot starts at an unknown position on the field.
	 *
	 * @return Initial odometry error translation.
	 */
	public synchronized InterpolatingTranslation2d getInitialFieldToOdom() {
        if (initial_field_to_odom.isEmpty()) return new InterpolatingTranslation2d();
		return initial_field_to_odom.get();
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public synchronized Pose2d getLatestFieldToVehicle() {
        Pose2d odomToVehicle = getLatestOdomToVehicle().getValue();

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(Util.translateBy(fieldToOdom,odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public synchronized Pose2d getFieldToVehicle(double timestamp) {
        Pose2d odomToVehicle = getOdomToVehicle(timestamp);

		Translation2d fieldToOdom = getFieldToOdom(timestamp);
		return new Pose2d(Util.translateBy(fieldToOdom,odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	
	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        Pose2d odomToVehicle = getPredictedOdomToVehicle(lookahead_time);

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(Util.translateBy(fieldToOdom,odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * @return Latest odometry pose.
	 * TODO: find replacement of InterpolatingDouble, or port team254 code*/
	public synchronized Map.Entry<InterpolatingDouble, InterpolatingPose2d> getLatestOdomToVehicle() {
        return odometry_to_vehicle.lastEntry();
	}

	/**
	 * Gets odometry pose from history. Linearly interpolates between gaps.
	 *
	 * @param timestamp Timestamp to loop up.
	 * @return Odometry relative robot pose at timestamp.
	 */
	public synchronized Pose2d getOdomToVehicle(double timestamp) {
        return odometry_to_vehicle.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * Gets interpolated odometry pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited odometry pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
        return getLatestOdomToVehicle()
				.getValue()
				.transformBy(new Transform2d(new Pose2d(),Util.expMap(Util.scaledTwist2d(vehicle_velocity_predicted,lookahead_time))));
	}

	/**
	 * @return Latest odometry error translation.
	 */
	public synchronized Translation2d getLatestFieldToOdom() {
        return getFieldToOdom(field_to_odometry.lastKey().value);
	}

	/**
	 * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
	 * @param timestamp Timestamp to look up.
	 * @return Odometry error at timestamp.
	 */
	public synchronized Translation2d getFieldToOdom(double timestamp) {
        if (field_to_odometry.isEmpty()) return new Translation2d();
		return field_to_odometry.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * @return Predicted robot velocity from last odometry update.
	 */
	public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted;
	}

	/**
	 * @return Measured robot velocity from last odometry update.
	 */
	public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured;
	}

	/**
	 * @return Measured robot velocity smoothed using a moving average filter.
	 */
	public synchronized Twist2d getSmoothedVelocity() {
		return vehicle_velocity_measured_filtered.getAverage();
	}

	/**
	 * @return Gets if estimator has recieved a vision update.
	 */
	public synchronized boolean getHasRecievedVisionUpdate() {
        return mHasRecievedVisionUpdate;
	}    
}
