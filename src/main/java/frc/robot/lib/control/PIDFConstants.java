package frc.robot.lib.control;

/** PIDF constants used to create PIDF controllers */
public class PIDFConstants {
    /** P */
    public final double kP;
    /** I */
    public final double kI;
    /** D */
    public final double kD;
    /** F */
    public final double kF;

    /**
     * Create a new PIDFConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     * @param kF F
     */
    public PIDFConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Create a new PIDFConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PIDFConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 1.0);
    }

    /**
     * Create a new PIDFConstants object
     *
     * @param kP P
     * @param kD D
     */
    public PIDFConstants(double kP, double kD) {
        this(kP, 0, kD);
    }

    /**
     * Create a new PIDFConstants object
     *
     * @param kP P
     */
    public PIDFConstants(double kP) {
        this(kP, 0, 0);
    }
}
