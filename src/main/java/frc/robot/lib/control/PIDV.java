package frc.robot.lib.control;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Implements a PID control loop. */
public class PIDV implements Sendable, AutoCloseable {
    private static int instances;

    // Factor for "proportional" control
    private double kP;

    // Factor for "integral" control
    private double kI;

    // Factor for "derivative" control
    private double kD;

    // The error range where "integral" control applies
    private double mIZone = Double.POSITIVE_INFINITY;

    // The period (in seconds) of the loop that calls the controller
    private final double m_period;

    private double mMaxIntegral = 1.0;
    private double mMinIntegral = -1.0;

    private double mMaxInput;
    private double mMinInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    private boolean mContinuous;

    // The error at the time of the most recent call to calculate()
    private double mPosError;
    private double mVelError;

    // The sum of the errors for use in the integral calc
    private double mTotalError;

    // The error that is considered at setpoint.
    private double mPosTolerance = 0.05;
    private double mVelTolerance = Double.POSITIVE_INFINITY;

    private double mSetpoint;
    private double mSetpointVel;
    private double mMeasurement;
    private double mMeasurementVel;

    private boolean mHaveMeasurement;
    private boolean mHaveSetpoint;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a
     * default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public PIDV(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.02);
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp     The proportional coefficient.
     * @param ki     The integral coefficient.
     * @param kd     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public PIDV(double kp, double ki, double kd, double period) {
        kP = kp;
        kI = ki;
        kD = kd;

        if (kp < 0.0) {
            throw new IllegalArgumentException("Kp must be a non-negative number!");
        }
        if (ki < 0.0) {
            throw new IllegalArgumentException("Ki must be a non-negative number!");
        }
        if (kd < 0.0) {
            throw new IllegalArgumentException("Kd must be a non-negative number!");
        }
        if (period <= 0.0) {
            throw new IllegalArgumentException("Controller period must be a positive number!");
        }
        m_period = period;

        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);

        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double kp) {
        kP = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double ki) {
        kI = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double kd) {
        kD = kd;
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
     * @throws IllegalArgumentException if iZone &lt; 0
     */
    public void setIZone(double iZone) {
        if (iZone < 0) {
            throw new IllegalArgumentException("IZone must be a non-negative number!");
        }
        mIZone = iZone;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return kP;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return kI;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return kD;
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return mIZone;
    }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    public double getPeriod() {
        return m_period;
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return mPosTolerance;
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return mVelTolerance;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint, double setpointVelocity) {
        mSetpoint = setpoint;
        mSetpointVel = setpointVelocity;
        mHaveSetpoint = true;

        if (mContinuous) {
            double errorBound = (mMaxInput - mMinInput) / 2.0;
            mPosError = MathUtil.inputModulus(mSetpoint - mMeasurement, -errorBound, errorBound);
        } else {
            mPosError = mSetpoint - mMeasurement;
        }

        mVelError = mSetpointVel - mMeasurementVel;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return mSetpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return mHaveMeasurement
                && mHaveSetpoint
                && Math.abs(mPosError) < mPosTolerance
                && Math.abs(mVelError) < mVelTolerance;
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
        mContinuous = true;
        mMinInput = minimumInput;
        mMaxInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        mContinuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return mContinuous;
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
        mMinIntegral = minimumIntegral;
        mMaxIntegral = maximumIntegral;
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
        mPosTolerance = positionTolerance;
        mVelTolerance = velocityTolerance;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return mPosError;
    }

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     */
    public double getVelocityError() {
        return mVelError;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double measurementVelocity, double setpoint, double setpointVelocity) {
        mSetpoint = setpoint;
        mSetpointVel = setpointVelocity;
        mHaveSetpoint = true;
        return calculate(measurement, measurementVelocity);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement, double measurementVelocity) {
        mMeasurement = measurement;
        mMeasurementVel = measurementVelocity;
        mHaveMeasurement = true;

        if (mContinuous) {
            double errorBound = (mMaxInput - mMinInput) / 2.0;
            mPosError = MathUtil.inputModulus(mSetpoint - mMeasurement, -errorBound, errorBound);
        } else {
            mPosError = mSetpoint - mMeasurement;
        }

        mVelError = mSetpointVel - mMeasurementVel;

        // If the absolute value of the position error is greater than IZone, reset the
        // total error
        if (Math.abs(mPosError) > mIZone) {
            mTotalError = 0;
        } else if (kI != 0) {
            mTotalError = MathUtil.clamp(
                    mTotalError + mPosError * m_period,
                    mMinIntegral / kI,
                    mMaxIntegral / kI);
        }

        return kP * mPosError + kI * mTotalError + kD * mVelError;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        mPosError = 0;
        mTotalError = 0;
        mVelError = 0;
        mHaveMeasurement = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
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
        // builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}