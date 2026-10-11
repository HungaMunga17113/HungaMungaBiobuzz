package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter2D;
import org.junit.Test;

public class SlewRateLimiter2DTest {
    private static final double EPS = 1e-9;

    @Test
    public void dtZeroSnaps() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(1.0);
        s.calculate(0.6, -0.8, 0);
        assertEquals(0.6, s.getX(), EPS);
        assertEquals(-0.8, s.getY(), EPS);
    }

    @Test
    public void noSqrt2Diagonals() {
        SlewRateLimiter2D diag = new SlewRateLimiter2D(1.0);
        SlewRateLimiter2D axis = new SlewRateLimiter2D(1.0);

        diag.calculate(1, 1, 0.1);
        axis.calculate(1, 0, 0.1);

        assertEquals(Math.hypot(axis.getX(), axis.getY()), Math.hypot(diag.getX(), diag.getY()), EPS);
        assertEquals(0.1, Math.hypot(diag.getX(), diag.getY()), EPS);
    }

    @Test
    public void keepsDirection() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(1.0);
        s.calculate(0.3, 0.9, 0.1);
        assertEquals(Math.atan2(0.9, 0.3), Math.atan2(s.getY(), s.getX()), 1e-9);
    }

    @Test
    public void noOvershoot() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(100.0);
        s.calculate(0.5, 0.5, 0.1);
        assertEquals(0.5, s.getX(), EPS);
        assertEquals(0.5, s.getY(), EPS);
    }

    @Test
    public void reversalDecels() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(10.0, 1.0);
        s.calculate(1, 0, 0);
        // dot < 0 -> decel even though the target is not shorter
        s.calculate(-1, 0, 0.1);
        assertEquals(0.9, s.getX(), EPS);
    }

    @Test
    public void decelRate() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(10.0, 1.0);
        s.calculate(1, 0, 0);
        s.calculate(0, 0, 0.1);
        assertEquals(0.9, s.getX(), EPS);
    }

    @Test
    public void accelRate() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(1.0, 10.0);
        s.calculate(0.5, 0, 0);
        s.calculate(1, 0, 0.1);
        assertEquals(0.6, s.getX(), EPS);
    }

    @Test
    public void dtCeiling() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(1.0);
        s.calculate(1, 0, 10.0);
        assertEquals(0.1, s.getX(), EPS);
    }

    @Test
    public void snapsDrift() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(0.1);
        s.calculate(1, 1, 0.1);
        assertEquals(0.0, s.getX(), EPS);
        assertEquals(0.0, s.getY(), EPS);
    }

    @Test
    public void reset() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(1.0);
        s.calculate(1, 1, 0);
        s.reset();
        assertEquals(0.0, s.getX(), EPS);
        assertEquals(0.0, s.getY(), EPS);
    }

    @Test
    public void magCappedAtOne() {
        SlewRateLimiter2D s = new SlewRateLimiter2D(3.0);
        for (int i = 0; i < 100; i++) {
            s.calculate(1 / Math.sqrt(2), 1 / Math.sqrt(2), 0.02);
            assertTrue(Math.hypot(s.getX(), s.getY()) <= 1.0 + EPS);
        }
        assertEquals(1.0, Math.hypot(s.getX(), s.getY()), 1e-9);
    }
}
