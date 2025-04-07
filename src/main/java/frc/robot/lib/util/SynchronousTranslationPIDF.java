package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class implements a PID Control Loop.
 * <p>
 * Does all computation synchronously (i.e. the calculate() function must be
 * called by the user from their own thread)
 */
public class SynchronousTranslationPIDF {
    private double m_P; // factor for "proportional" control
    private double m_I; // factor for "integral" control
    private double m_D; // factor for "derivative" control
    private double m_F; // factor for feed forward gain
    private Translation2d m_prevError = new Translation2d(); // the prior sensor input (used to compute velocity)
    private Translation2d m_totalError = new Translation2d(); // the sum of the errors for use in the integral calc
    private Translation2d m_setpoint = new Translation2d();
    private Translation2d m_error = new Translation2d();
    private Translation2d m_result = new Translation2d();
    private Translation2d m_last_input = null;
    private double m_deadband = 0.0; // If the absolute error is less than deadband then treat error for the proportional term as 0
    private double m_last_timestamp = Timer.getFPGATimestamp();

    public SynchronousTranslationPIDF() {}

    /**
     * Allocate a PID object with the given constants for P, I, D
     *
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public SynchronousTranslationPIDF(double Kp, double Ki, double Kd) {
        setPIDF(Kp, Ki, Kd, 0);
    }

    /**
     * Allocate a PID object with the given constants for P, I, D
     *
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     * @param Kf the feed forward gain coefficient
     */
    public SynchronousTranslationPIDF(double Kp, double Ki, double Kd, double Kf) {
        setPIDF(Kp, Ki, Kd, Kf);
    }

    public Translation2d calculate(Translation2d input) {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - m_last_timestamp;
        m_last_timestamp = timestamp;

        return calculate(input, dt);
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should be called at a constant rate by the user (ex. in a timed thread)
     *
     * @param input the input
     * @param dt    time passed since previous call to calculate
     */
    public Translation2d calculate(Translation2d input, double dt) {
        if (dt < 1E-6) {
            dt = 1E-6;
        }

        m_last_input = input;
        m_error = m_setpoint.minus(input);

        // Don't blow away m_error so as to not break derivative
        Translation2d proportionalError = m_error.getNorm() < m_deadband ? new Translation2d() : m_error;

        m_totalError = m_totalError.plus(proportionalError);
        
        m_result = (proportionalError.times(m_P)).plus(m_totalError.times(m_I)).plus(m_error.minus(m_prevError).times(m_D/dt))
                .plus(m_setpoint.times(m_F));
        m_prevError = m_error;
        

        System.out.println(m_result.getNorm());

        return m_result;
    }

    /**
     * Set the PID controller gain parameters. Set the proportional, integral, and
     * differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d) {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Set the PID controller gain parameters. Set the proportional, integral, and
     * differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     * @param f Feed forward coefficient
     */
    public void setPIDF(double p, double i, double d, double f) {
        setPID(p, i, d);
        m_F = f;
    }

    /**
     * Get the Proportional coefficient
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_D;
    }

    /**
     * Get the Feed forward coefficient
     *
     * @return feed forward coefficient
     */
    public double getF() {
        return m_F;
    }

    /**
     * Return the current PID result This is always centered on zero and constrained
     * the the max and min outs
     *
     * @return the latest calculated output
     */
    public Translation2d get() {
        return m_result;
    }

    public void setDeadband(double deadband) {
        m_deadband = deadband;
    }

    /**
     * Set the setpoint for the PID controller
     *
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(Translation2d setpoint) {
        m_setpoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PID controller
     *
     * @return the current setpoint
     */
    public Translation2d getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns the current difference of the input from the setpoint
     *
     * @return the current error
     */
    public Translation2d getError() {
        return m_error;
    }

    /**
     * Return true if the error is within the tolerance
     *
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget(double tolerance) {
        return m_last_input.getDistance(m_setpoint) < tolerance;
    }

    /**
     * Reset all internal terms.
     */
    public void reset() {
        m_last_input = null;
        m_prevError = new Translation2d();
        m_totalError = new Translation2d();
        m_result = new Translation2d();
        m_setpoint = new Translation2d();
    }

    public void resetIntegrator() {
        m_totalError = new Translation2d();
    }

    @Override
    public String toString() {
        String lState = "";

        lState += "Kp: " + m_P + "\n";
        lState += "Ki: " + m_I + "\n";
        lState += "Kd: " + m_D + "\n";

        return lState;
    }
}