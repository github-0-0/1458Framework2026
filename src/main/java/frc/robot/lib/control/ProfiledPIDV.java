package frc.robot.lib.control;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile. Users should
 * call reset() when they first start running the controller to avoid unwanted
 * behavior.
 */
public class ProfiledPIDV implements Sendable {
    private static int instances;

    private PIDV mController;
    private double mMinInput;
    private double mMaxInput;

    private TrapezoidProfile.Constraints mConstraints;
    private TrapezoidProfile mProfile;
    private TrapezoidProfile.State mGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State();

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public ProfiledPIDV(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        this(Kp, Ki, Kd, constraints, 0.02);
    }

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param period      The period between controller updates in seconds. The
     *                    default is 0.02 seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public ProfiledPIDV(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
        mController = new PIDV(Kp, Ki, Kd, period);
        mConstraints = constraints;
        mProfile = new TrapezoidProfile(mConstraints);
        instances++;

        SendableRegistry.add(this, "ProfiledPIDController", instances);
        MathSharedStore.reportUsage(MathUsageId.kController_ProfiledPIDController, instances);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>
     * Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     * @param Ki The integral coefficient. Must be &gt;= 0.
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setPID(double Kp, double Ki, double Kd) {
        mController.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double Kp) {
        mController.setP(Kp);
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double Ki) {
        mController.setI(Ki);
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double Kd) {
        mController.setD(Kd);
    }

    /**
     * Sets the IZone range. When the absolute value of the position error is
     * greater than IZone, the
     * total accumulated error will reset to zero, disabling integral gain until the
     * absolute value of
     * the position error is less than IZone. This is used to prevent integral
     * windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain.
     * Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables IZone functionality.
     *
     * @param iZone Maximum magnitude of error to allow integral control.
     * @throws IllegalArgumentException if iZone &lt;= 0
     */
    public void setIZone(double iZone) {
        mController.setIZone(iZone);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return mController.getP();
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return mController.getI();
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return mController.getD();
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return mController.getIZone();
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return mController.getPeriod();
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return mController.getPositionTolerance();
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return mController.getVelocityTolerance();
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        mGoal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        mGoal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     *
     * @return The goal.
     */
    public TrapezoidProfile.State getGoal() {
        return mGoal;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atGoal() {
        return atSetpoint() && mGoal.equals(mSetpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        mConstraints = constraints;
        mProfile = new TrapezoidProfile(mConstraints);
    }

    /**
     * Get the velocity and acceleration constraints for this controller.
     *
     * @return Velocity and acceleration constraints.
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return mConstraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfile.State getSetpoint() {
        return mSetpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atSetpoint() {
        return mController.atSetpoint();
    }

    /**
     * Enables continuous input.
     *
     * <p>
     * Rather then using the max and min input range as constraints, it considers
     * them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        mController.enableContinuousInput(minimumInput, maximumInput);
        mMinInput = minimumInput;
        mMaxInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        mController.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>
     * When the cap is reached, the integrator value is added to the controller
     * output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        mController.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        mController.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return mController.getPositionError();
    }

    /**
     * Returns the change in error per second.
     *
     * @return The change in error per second.
     */
    public double getVelocityError() {
        return mController.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    public double calculate(double measurement, double measurementVelocity) {
        if (mController.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (mMaxInput - mMinInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(mGoal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance = MathUtil.inputModulus(mSetpoint.position - measurement, -errorBound,
                    errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest
            // path. The goal
            // may be outside the input range after this operation, but that's OK because
            // the controller
            // will still go there and report an error of zero. In other words, the setpoint
            // only needs to
            // be offset from the measurement by the input range modulus; they don't need to
            // be equal.
            mGoal.position = goalMinDistance + measurement;
            mSetpoint.position = setpointMinDistance + measurement;
        }

        mSetpoint = mProfile.calculate(getPeriod(), mSetpoint, mGoal);
        return mController.calculate(measurement, measurementVelocity, mSetpoint.position, mSetpoint.velocity);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, double measurementVelocity, TrapezoidProfile.State goal) {
        setGoal(goal);
        return calculate(measurement, measurementVelocity);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     * @return The controller's next output.
     */
    public double calculate(
            double measurement, double measuredVelocity, TrapezoidProfile.State goal,
            TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return calculate(measurement, measuredVelocity, goal);
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfile.State measurement) {
        mController.reset();
        mSetpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The
     *                         velocity is assumed to
     *                         be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ProfiledPIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
                "izone",
                this::getIZone,
                (double toSet) -> {
                    try {
                        setIZone(toSet);
                    } catch (IllegalArgumentException e) {
                        MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
                    }
                });
        builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
    }
}