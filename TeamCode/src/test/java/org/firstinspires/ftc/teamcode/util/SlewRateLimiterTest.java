package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SlewRateLimiterTest {
    private static final double EPS = 1e-9;

    @Test
    public void dtZeroSnaps() {
        SlewRateLimiter s = new SlewRateLimiter(1.0);
        assertEquals(0.8, s.calculate(0.8, 0), EPS);
        assertEquals(-0.3, s.calculate(-0.3, -1), EPS);
    }

    @Test
    public void rampUp() {
        SlewRateLimiter s = new SlewRateLimiter(2.0);
        assertEquals(0.2, s.calculate(1.0, 0.1), EPS);
        assertEquals(0.4, s.calculate(1.0, 0.1), EPS);
    }

    @Test
    public void noOvershoot() {
        SlewRateLimiter s = new SlewRateLimiter(100.0);
        assertEquals(0.5, s.calculate(0.5, 0.1), EPS);
        assertEquals(0.5, s.calculate(0.5, 0.1), EPS);
    }

    @Test
    public void decelRate() {
        SlewRateLimiter s = new SlewRateLimiter(10.0, 1.0);
        s.calculate(1.0, 0);
        assertEquals(0.9, s.calculate(0.5, 0.1), EPS);
    }

    @Test
    public void reversalDecels() {
        SlewRateLimiter s = new SlewRateLimiter(10.0, 1.0);
        s.calculate(1.0, 0);
        // |target| > |velocity| would say accel; opp signs must override
        assertEquals(0.9, s.calculate(-2.0, 0.1), EPS);
    }

    @Test
    public void dtCeiling() {
        SlewRateLimiter s = new SlewRateLimiter(1.0);
        // dt clamped to 0.1 -> 0.1 of change, not 10.0
        assertEquals(0.1, s.calculate(1.0, 10.0), EPS);
    }

    @Test
    public void dtFloor() {
        SlewRateLimiter s = new SlewRateLimiter(4.0);
        // dt clamped up to 0.015, progress is never ~0
        assertEquals(0.06, s.calculate(1.0, 1e-6), EPS);
    }

    @Test
    public void snapsDrift() {
        SlewRateLimiter s = new SlewRateLimiter(0.1);
        assertEquals(0.0, s.calculate(1.0, 0.1), EPS);
    }

    @Test
    public void reset() {
        SlewRateLimiter s = new SlewRateLimiter(1.0);
        s.calculate(1.0, 0);
        s.reset();
        assertEquals(0.1, s.calculate(1.0, 0.1), EPS);
    }

    @Test
    public void setRates() {
        SlewRateLimiter s = new SlewRateLimiter(1.0);
        s.setRates(5.0, 5.0);
        assertEquals(0.5, s.calculate(1.0, 0.1), EPS);
    }

    @Test
    public void neverExceedsTarget() {
        SlewRateLimiter s = new SlewRateLimiter(3.0);
        double v = 0;
        for (int i = 0; i < 50; i++) {
            v = s.calculate(0.7, 0.02);
            assertTrue(v <= 0.7 + EPS);
        }
        assertEquals(0.7, v, EPS);
    }
}
