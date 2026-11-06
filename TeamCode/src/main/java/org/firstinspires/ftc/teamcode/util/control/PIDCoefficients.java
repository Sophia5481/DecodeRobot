package org.firstinspires.ftc.teamcode.util.control;

/**
 * Proportional, integral, and derivative (PID) gains used by PIDFController.
 */
public class PIDCoefficients {

    /** Proportional gain. */
    public double kP;

    /** Integral gain. */
    public double kI;

    /** Derivative gain. */
    public double kD;

    /** Default constructor with all gains set to zero. */
    public PIDCoefficients() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Constructor for PIDCoefficients.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /** Creates a copy of this object. */
    public PIDCoefficients copy() {
        return new PIDCoefficients(kP, kI, kD);
    }

    @Override
    public String toString() {
        return String.format("PIDCoefficients(kP=%.6f, kI=%.6f, kD=%.6f)", kP, kI, kD);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof PIDCoefficients)) return false;
        PIDCoefficients other = (PIDCoefficients) obj;
        return Double.compare(kP, other.kP) == 0 &&
                Double.compare(kI, other.kI) == 0 &&
                Double.compare(kD, other.kD) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(kP, kI, kD);
    }
}