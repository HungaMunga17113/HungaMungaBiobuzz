package org.firstinspires.ftc.teamcode.util.control;

/**
 * pid + feedforward. two types (additive):
 * kF    sdk-style, kF*setpoint
 * f-vas kS static friction, kV velocity, kA accel (from d/dt setpoint), kG gravity
 *
 * ff units are output units (-1..1 power, unless the output range says otherwise)
 */
public class PIDFControllerEx extends PIDControllerEx {
    public enum Gravity {
        /** no kG term */
        NONE,
        /** kG as-is, elevators / linear slides */
        CONSTANT,
        /** kG*cos(measurement), arms, measurement in rad w/ 0 = horizontal */
        COSINE
    }

    private double kF = 0;
    private double kS = 0, kV = 0, kA = 0, kG = 0;
    private Gravity gravity = Gravity.NONE;

    // deadband so kS doesn't buzz the mech at rest
    private double sDeadband = 1e-6;

    private double prevSetpoint = 0;
    private boolean firstFf = true;
    private double setpointAccel = 0;
    private double accelAlpha = 1.0;

    public PIDFControllerEx(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }

    /** sdk-equivalent pidf */
    public PIDFControllerEx(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }

    public PIDFControllerEx setF(double kF) {
        this.kF = kF;
        return this;
    }

    /** velocity mechs: flywheels, shooters. setpoint is a velocity */
    public PIDFControllerEx setSVA(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public PIDFControllerEx setGravity(double kG, Gravity mode) {
        this.kG = kG;
        this.gravity = mode;
        return this;
    }

    /** setpoint must exceed this before kS kicks in */
    public PIDFControllerEx setStaticDeadband(double deadband) {
        this.sDeadband = Math.abs(deadband);
        return this;
    }

    /** lowpass on d/dt setpoint used by kA, 1 = off */
    public PIDFControllerEx setAccelFilter(double alpha) {
        this.accelAlpha = Math.min(Math.max(alpha, 1e-6), 1.0);
        return this;
    }

    @Override
    protected double feedforward(double setpoint, double measurement, double dt) {
        if (firstFf) {
            prevSetpoint = setpoint;
            firstFf = false;
        }

        if (dt > 0) {
            double raw = (setpoint - prevSetpoint) / dt;
            setpointAccel += accelAlpha * (raw - setpointAccel);
        }
        prevSetpoint = setpoint;

        double ff = kF * setpoint + kV * setpoint + kA * setpointAccel;

        if (Math.abs(setpoint) > sDeadband) {
            ff += kS * Math.signum(setpoint);
        }

        if (gravity == Gravity.CONSTANT) {
            ff += kG;
        } else if (gravity == Gravity.COSINE) {
            ff += kG * Math.cos(measurement);
        }

        return ff;
    }

    @Override
    public void reset() {
        super.reset();
        prevSetpoint = 0;
        setpointAccel = 0;
        firstFf = true;
    }

    public double getSetpointAccel() { return setpointAccel; }
}
