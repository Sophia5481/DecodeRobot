package org.firstinspires.ftc.teamcode.util.control;

import static java.lang.Math.*;

/**
 * PID controller with various feedforward components.
 */
public class PIDFController {

    private static final double EPSILON = 1e-6;

    private final PIDCoefficients pid;
    private final double kV;
    private final double kA;
    private final double kStatic;
    private final FeedforwardFunction kF;

    private double errorSum = 0.0;
    private double lastUpdateTimestamp = Double.NaN;

    private boolean inputBounded = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;

    /** Target position (that is, the controller setpoint). */
    public double targetPosition = 0.0;

    /** Target velocity. */
    public double targetVelocity = 0.0;

    /** Target acceleration. */
    public double targetAcceleration = 0.0;

    /** Error computed in the last call to update(). */
    private double lastError = 0.0;

    public double getLastError() {
        return lastError;
    }

    /**
     * Functional interface for custom feedforward functions.
     * Takes (targetPosition, measuredPosition, measuredVelocity) and returns a feedforward term.
     */
    @FunctionalInterface
    public interface FeedforwardFunction {
        double apply(double targetPosition, double measuredPosition, Double measuredVelocity);
    }

    /**
     * Constructor for PIDFController.
     *
     * @param pid traditional PID coefficients
     * @param kV feedforward velocity gain
     * @param kA feedforward acceleration gain
     * @param kStatic additive feedforward constant
     * @param kF custom feedforward that depends on position and/or velocity (e.g., a gravity term for arms)
     */
    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic, FeedforwardFunction kF) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF != null ? kF : (p, m, v) -> 0.0;
    }

    /** Convenience constructor with no custom feedforward function. */
    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        this(pid, kV, kA, kStatic, null);
    }

    /** Convenience constructor with only PID terms (no feedforward). */
    public PIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0, null);
    }

    /** Sets bounds on the input (e.g., for angular wrap-around). */
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    /** Sets bounds on the output. */
    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    private double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (abs(error) > inputRange / 2.0) {
                error -= signum(error) * inputRange;
            }
        }
        return error;
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity (can be null)
     * @return control output
     */
    public double update(double measuredPosition, Double measuredVelocity) {
        double currentTimestamp = System.nanoTime() / 1e9;
        double error = getPositionError(measuredPosition);

        if (Double.isNaN(lastUpdateTimestamp)) {
            lastError = error;
            lastUpdateTimestamp = currentTimestamp;
            return 0.0;
        }

        double dt = currentTimestamp - lastUpdateTimestamp;
        errorSum += 0.5 * (error + lastError) * dt;
        double errorDeriv = (error - lastError) / dt;

        lastError = error;
        lastUpdateTimestamp = currentTimestamp;

        double baseOutput = pid.kP * error
                + pid.kI * errorSum
                + pid.kD * (measuredVelocity != null ? (targetVelocity - measuredVelocity) : errorDeriv)
                + kV * targetVelocity
                + kA * targetAcceleration
                + kF.apply(targetPosition, measuredPosition, measuredVelocity);

        double output = epsilonEquals(baseOutput, 0.0) ? 0.0 : baseOutput + signum(baseOutput) * kStatic;

        if (outputBounded) {
            return max(minOutput, min(output, maxOutput));
        } else {
            return output;
        }
    }

    /** Overload for when velocity measurement is unavailable. */
    public double update(double measuredPosition) {
        return update(measuredPosition, null);
    }

    /** Reset the controller's integral sum and state. */
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }

    /** Floating-point comparison helper. */
    private static boolean epsilonEquals(double a, double b) {
        return abs(a - b) < EPSILON;
    }

    public void setTargetPosition(int n) {
        targetPosition = n;
    }
}
