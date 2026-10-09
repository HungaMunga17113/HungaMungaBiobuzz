package org.firstinspires.ftc.teamcode.util;

public class SlewRateLimiter {
    private double velocity = 0.0;
    private double maxAccelUp, maxAccelDown;

    public SlewRateLimiter(double rateFactor) { this(rateFactor, rateFactor); }
    public SlewRateLimiter(double accelUp, double accelDown) {
        this.maxAccelUp = accelUp;
        this.maxAccelDown = accelDown;
    }

    public void setRates(double accelUp, double accelDown) {
        this.maxAccelUp = accelUp;
        this.maxAccelDown = accelDown;
    }

    public double calculate(double target, double dt) {
        if (dt <= 0) {
            velocity = target;
            return target;
        }

        double clampedDt = Math.min(Math.max(dt, 0.015), 0.1);

        // accel or decel?
        boolean isAccelerating = Math.abs(target) > Math.abs(velocity);

        // if signs opposite, decel to 0 first
        if (target != 0 && velocity != 0 && Math.signum(target) != Math.signum(velocity)) {
            isAccelerating = false;
        }

        // calc max allowable change on correct rate limit
        double maxChange = (isAccelerating ? maxAccelUp : maxAccelDown) * clampedDt;
        double error = target - velocity;

        velocity += Math.max(-maxChange, Math.min(error, maxChange));

        // snap residual drift to 0
        if (Math.abs(velocity) < 0.02) velocity = 0.0;

        return velocity;
    }

    public void reset() {
        velocity = 0.0;
    }
}