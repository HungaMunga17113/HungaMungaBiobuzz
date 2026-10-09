package org.firstinspires.ftc.teamcode.util;

/**
 * slew limits entire 2d vector
 * keeps direction true + same accel cap in every direction (no sqrt2 diagonals)
 */
public class SlewRateLimiter2D {
    private double x = 0.0, y = 0.0;
    private double maxAccelUp, maxAccelDown;

    public SlewRateLimiter2D(double rateFactor) { this(rateFactor, rateFactor); }
    public SlewRateLimiter2D(double accelUp, double accelDown) {
        this.maxAccelUp = accelUp;
        this.maxAccelDown = accelDown;
    }

    public void setRates(double accelUp, double accelDown) {
        this.maxAccelUp = accelUp;
        this.maxAccelDown = accelDown;
    }

    /** moves output toward (tx, ty) */
    public void calculate(double tx, double ty, double dt) {
        if (dt <= 0) {
            x = tx;
            y = ty;
            return;
        }

        double clampedDt = Math.min(Math.max(dt, 0.015), 0.1);

        // accel if target is longer and not pointing backwards (reversal = decel)
        double curMag = Math.hypot(x, y);
        double dot = tx * x + ty * y;
        boolean isAccelerating = Math.hypot(tx, ty) > curMag && dot >= 0;

        double maxChange = (isAccelerating ? maxAccelUp : maxAccelDown) * clampedDt;
        double ex = tx - x;
        double ey = ty - y;
        double errMag = Math.hypot(ex, ey);

        if (errMag <= maxChange) {
            x = tx;
            y = ty;
        } else {
            double s = maxChange / errMag;
            x += ex * s;
            y += ey * s;
        }

        // reset residual drift
        if (Math.hypot(x, y) < 0.02) {
            x = 0.0;
            y = 0.0;
        }
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public void reset() {
        x = 0.0;
        y = 0.0;
    }
}
