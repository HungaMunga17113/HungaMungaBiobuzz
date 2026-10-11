package org.firstinspires.ftc.teamcode.util.control;

/**
 * pid w/ cool stuff
 * izone, conditional anti-windup, filtered + on-measurement kD, output clamp,
 * also continuous input for turrets + setpt tolerance
 */
public class PIDControllerEx {
    protected double kP, kI, kD;

    // 0 = off, else only when |error| < iZone
    private double iZone = 0;
    
    // cap on i term's contribution to output
    private double maxIntegralTerm = Double.POSITIVE_INFINITY;

    private double minOutput = -1.0, maxOutput = 1.0;

    // 1.0 = unfiltered, smaller = heavier lowpass on d
    private double dAlpha = 1.0;
    private boolean dOnMeasurement = true;

    private double tolerance = 0;
    private double velTolerance = Double.POSITIVE_INFINITY;

    private boolean continuous = false;
    private double minInput, maxInput;

    private double integral = 0;
    private double error = 0, prevError = 0;
    private double prevMeasurement = 0;
    private double derivative = 0;
    private double setpoint = 0;
    private boolean firstRun = true;

    private long prevNanos = 0;

    public PIDControllerEx(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // ---- config ----

    public PIDControllerEx setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    /** integrate only near the setpt, 0 to disable */
    public PIDControllerEx setIZone(double iZone) {
        this.iZone = iZone;
        if (iZone > 0) integral = 0;
        return this;
    }

    /** hard cap on kI*integral in output units */
    public PIDControllerEx setMaxIntegralTerm(double max) {
        this.maxIntegralTerm = Math.abs(max);
        return this;
    }

    public PIDControllerEx setOutputRange(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
        return this;
    }

    /** alpha in (0,1], 1 = no filter. use ~0.1-0.3 on noisy encoder vel */
    public PIDControllerEx setDerivativeFilter(double alpha) {
        this.dAlpha = Math.min(Math.max(alpha, 1e-6), 1.0);
        return this;
    }

    /** on-measurement kills kD kick when setpoint jumps */
    public PIDControllerEx setDerivativeOnMeasurement(boolean onMeasurement) {
        this.dOnMeasurement = onMeasurement;
        return this;
    }

    public PIDControllerEx setTolerance(double tolerance) {
        return setTolerance(tolerance, Double.POSITIVE_INFINITY);
    }

    public PIDControllerEx setTolerance(double tolerance, double velTolerance) {
        this.tolerance = Math.abs(tolerance);
        this.velTolerance = Math.abs(velTolerance);
        return this;
    }

    /** wraps error into [min,max), e.g. (-180,180) for a turret in deg */
    public PIDControllerEx setContinuousInput(double min, double max) {
        this.continuous = true;
        this.minInput = Math.min(min, max);
        this.maxInput = Math.max(min, max);
        return this;
    }

    public PIDControllerEx setContinuousInputDisabled() {
        this.continuous = false;
        return this;
    }

    // ---- run ----

    /** dt from internal clock, first call is treated as dt=0 */
    public double calculate(double measurement, double setpoint) {
        long now = System.nanoTime();
        double dt = prevNanos == 0 ? 0 : (now - prevNanos) / 1e9;
        prevNanos = now;
        return calculate(measurement, setpoint, dt);
    }

    public double calculate(double measurement, double setpoint, double dt) {
        this.setpoint = setpoint;
        error = wrapError(setpoint - measurement);

        if (firstRun) {
            prevError = error;
            prevMeasurement = measurement;
            firstRun = false;
        }

        if (dt > 0) {
            double raw = dOnMeasurement
                    ? -wrapError(measurement - prevMeasurement) / dt
                    : (error - prevError) / dt;
            derivative += dAlpha * (raw - derivative);
        }

        boolean inIZone = iZone <= 0 || Math.abs(error) < iZone;
        if (dt > 0 && inIZone) {
            integral += error * dt;
        } else if (!inIZone) {
            integral = 0;
        }

        double iTerm = clamp(kI * integral, -maxIntegralTerm, maxIntegralTerm);
        double raw = kP * error + iTerm + kD * derivative + feedforward(setpoint, measurement, dt);
        double out = clamp(raw, minOutput, maxOutput);

        // conditional integration: undo deepened saturation
        if (out != raw && dt > 0 && Math.signum(error) == Math.signum(raw - out)) {
            integral -= error * dt;
        }

        prevError = error;
        prevMeasurement = measurement;
        return out;
    }

    /** overridden by PIDFControllerEx, 0 here */
    protected double feedforward(double setpoint, double measurement, double dt) {
        return 0;
    }

    public void reset() {
        integral = 0;
        error = 0;
        prevError = 0;
        prevMeasurement = 0;
        derivative = 0;
        firstRun = true;
        prevNanos = 0;
    }

    /** drop integral only, keeps kD state */
    public void resetIntegral() {
        integral = 0;
    }

    public boolean atSetpoint() {
        return Math.abs(error) <= tolerance && Math.abs(derivative) <= velTolerance;
    }

    public double getError() { return error; }
    public double getDerivative() { return derivative; }
    public double getIntegral() { return integral; }
    public double getSetpoint() { return setpoint; }

    protected double wrapError(double e) {
        if (!continuous) return e;
        double range = maxInput - minInput;
        if (range <= 0) return e;
        e %= range;
        if (e > range / 2) e -= range;
        else if (e < -range / 2) e += range;
        return e;
    }

    protected static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}
