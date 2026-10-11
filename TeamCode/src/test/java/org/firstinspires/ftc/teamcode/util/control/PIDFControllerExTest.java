package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** ff terms, and that ff stacks on top of pid */
public class PIDFControllerExTest {
    private static final double EPS = 1e-9;

    private static PIDFControllerEx open(double p, double i, double d) {
        PIDFControllerEx c = new PIDFControllerEx(p, i, d);
        c.setOutputRange(-1000, 1000);
        return c;
    }

    @Test
    public void kF() {
        PIDFControllerEx c = open(0, 0, 0).setF(0.5);
        assertEquals(5.0, c.calculate(0, 10, 0.02), EPS);
    }

    @Test
    public void kFIgnoresMeasurement() {
        PIDFControllerEx c = open(0, 0, 0).setF(0.5);
        assertEquals(5.0, c.calculate(10, 10, 0.02), EPS);
    }

    @Test
    public void kV() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0, 0.5, 0);
        assertEquals(5.0, c.calculate(0, 10, 0.02), EPS);
    }

    @Test
    public void kS() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0.1, 0, 0);
        assertEquals(0.1, c.calculate(0, 10, 0.02), EPS);
        assertEquals(-0.1, c.calculate(0, -10, 0.02), EPS);
    }

    @Test
    public void kSDeadband() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0.1, 0, 0).setStaticDeadband(1.0);
        assertEquals(0.0, c.calculate(0, 0.5, 0.02), EPS);
        assertEquals(0.1, c.calculate(0, 2.0, 0.02), EPS);
    }

    @Test
    public void kA() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0, 0, 1);
        // first call has no prior setpoint, so no phantom accel
        assertEquals(0.0, c.calculate(0, 10, 0.1), EPS);
        assertEquals(100.0, c.calculate(0, 20, 0.1), EPS);
    }

    @Test
    public void kAZeroOnSteadySetpoint() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0, 0, 1);
        c.calculate(0, 10, 0.1);
        c.calculate(0, 10, 0.1);
        assertEquals(0.0, c.getSetpointAccel(), EPS);
    }

    @Test
    public void accelFilter() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0, 0, 1).setAccelFilter(0.5);
        c.calculate(0, 10, 0.1);
        assertEquals(50.0, c.calculate(0, 20, 0.1), EPS);
    }

    @Test
    public void gravityConstant() {
        PIDFControllerEx c = open(0, 0, 0).setGravity(0.2, PIDFControllerEx.Gravity.CONSTANT);
        assertEquals(0.2, c.calculate(0, 0, 0.02), EPS);
        assertEquals(0.2, c.calculate(50, 0, 0.02), EPS);
    }

    @Test
    public void gravityCosine() {
        PIDFControllerEx c = open(0, 0, 0).setGravity(1.0, PIDFControllerEx.Gravity.COSINE);
        assertEquals(1.0, c.calculate(0, 0, 0.02), EPS);
        assertEquals(0.0, c.calculate(Math.PI / 2, 0, 0.02), 1e-12);
        assertEquals(-1.0, c.calculate(Math.PI, 0, 0.02), 1e-12);
    }

    @Test
    public void gravityNoneByDefault() {
        assertEquals(0.0, open(0, 0, 0).calculate(0, 0, 0.02), EPS);
    }

    @Test
    public void ffStacksOnPid() {
        PIDFControllerEx c = open(1, 0, 0).setF(0.5);
        assertEquals(3.0, c.calculate(0, 2, 0.02), EPS);
    }

    @Test
    public void ffClampedWithPid() {
        PIDFControllerEx c = new PIDFControllerEx(0, 0, 0, 0.5);
        assertEquals(1.0, c.calculate(0, 10, 0.02), EPS);
    }

    @Test
    public void ffCountsTowardWindup() {
        // ff alone saturates, so the i term must not bank on top of it
        PIDFControllerEx c = new PIDFControllerEx(0, 1, 0, 1.0);
        for (int k = 0; k < 50; k++) {
            c.calculate(0, 2, 0.02);
        }
        assertEquals(0.0, c.getIntegral(), EPS);
    }

    @Test
    public void reset() {
        PIDFControllerEx c = open(0, 0, 0).setSVA(0, 0, 1);
        c.calculate(0, 10, 0.1);
        c.calculate(0, 20, 0.1);
        c.reset();
        assertEquals(0.0, c.getSetpointAccel(), EPS);
        assertEquals(0.0, c.calculate(0, 30, 0.1), EPS);
    }

    @Test
    public void flywheelHoldsWithFf() {
        // kV sized for the plant, so p barely works and there is no steady-state droop
        PIDFControllerEx c = open(0.001, 0, 0).setSVA(0, 1.0 / 2000, 0);
        double v = 0;
        for (int k = 0; k < 400; k++) {
            double u = c.calculate(v, 1500, 0.02);
            v += (u * 2000 - v) * 0.02 * 5;
        }
        assertEquals(1500.0, v, 10.0);
        assertTrue(c.getError() < 10.0);
    }
}
