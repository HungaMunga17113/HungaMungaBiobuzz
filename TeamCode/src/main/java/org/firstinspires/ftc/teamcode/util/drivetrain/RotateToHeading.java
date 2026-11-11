package org.firstinspires.ftc.teamcode.util.drivetrain;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.control.PIDControllerEx;

/**
 * pd snap-to-heading, heading src agnostic (imu / pinpoint), deg ccw+
 * out is drivetrain rot, +rot = clockwise
 */
@Configurable
public class RotateToHeading {
    public static double kP = 0.02, kD = 0.0015;
    public static double kStatic = 0.05; // min power to break friction
    public static double maxPower = 0.8;
    public static double tolerance = 1.5; // deg
    public static double velTolerance = 20; // deg/s
    public static double settleMs = 100;
    public static double overrideDb = 0.05; // stick mag that cancels

    private final PIDControllerEx pid = new PIDControllerEx(kP, 0, kD);

    private double target;
    private boolean hasTarget, active;
    private double settledMs;

    public RotateToHeading() {
        pid.setContinuousInput(-180, 180).setDerivativeFilter(0.3);
    }

    public void setTarget(double deg) {
        target = wrap(deg);
        hasTarget = true;
    }

    public void captureTarget(double currentDeg) { setTarget(currentDeg); }

    /** no-op w/o target */
    public void start() {
        if (!hasTarget) return;
        active = true;
        settledMs = 0;
        pid.reset();
    }

    public void cancel() { active = false; }

    public boolean isActive() { return active; }
    public boolean hasTarget() { return hasTarget; }
    public double getTarget() { return target; }
    public double getError() { return pid.getError(); }

    /** rot power while active, else stickRot; stick input past overrideDb cancels */
    public double calculate(double headingDeg, double stickRot, double dt) {
        if (active && Math.abs(stickRot) > overrideDb) active = false;
        if (!active) return stickRot;

        pid.setPID(kP, 0, kD)
                .setOutputRange(-maxPower, maxPower)
                .setTolerance(tolerance, velTolerance);
        double ccw = pid.calculate(headingDeg, target, dt);

        if (pid.atSetpoint()) {
            settledMs += dt * 1000;
            if (settledMs >= settleMs) active = false;
            return 0;
        }
        settledMs = 0;

        if (Math.abs(ccw) < kStatic) ccw = Math.signum(pid.getError()) * kStatic;
        return -ccw;
    }

    static double wrap(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        else if (deg < -180) deg += 360;
        return deg;
    }
}
