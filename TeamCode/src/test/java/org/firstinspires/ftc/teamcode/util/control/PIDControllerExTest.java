package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** izone, windup, d handling, wrapping */
public class PIDControllerExTest {
    private static final double EPS = 1e-9;

    private static PIDControllerEx open(double p, double i, double d) {
        return new PIDControllerEx(p, i, d).setOutputRange(-1000, 1000);
    }

    @Test
    public void pTerm() {
        assertEquals(5.0, open(0.5, 0, 0).calculate(0, 10, 0.02), EPS);
    }

    @Test
    public void iAccumulates() {
        PIDControllerEx c = open(0, 1, 0);
        assertEquals(1.0, c.calculate(0, 10, 0.1), EPS);
        assertEquals(2.0, c.calculate(0, 10, 0.1), EPS);
    }

    @Test
    public void iZoneGates() {
        PIDControllerEx c = open(0, 1, 0).setIZone(5);
        assertEquals(0.0, c.calculate(0, 10, 0.1), EPS);
        assertEquals(0.2, c.calculate(0, 2, 0.1), EPS);
    }

    @Test
    public void iZoneClearsOnExit() {
        PIDControllerEx c = open(0, 1, 0).setIZone(5);
        c.calculate(0, 2, 0.1);
        c.calculate(0, 50, 0.1);
        assertEquals(0.0, c.getIntegral(), EPS);
    }

    @Test
    public void iTermCapped() {
        PIDControllerEx c = open(0, 1, 0).setMaxIntegralTerm(0.3);
        for (int k = 0; k < 20; k++) {
            c.calculate(0, 10, 0.1);
        }
        assertEquals(0.3, c.calculate(0, 10, 0.1), EPS);
    }

    @Test
    public void antiWindup() {
        PIDControllerEx c = new PIDControllerEx(1, 1, 0);
        for (int k = 0; k < 50; k++) {
            c.calculate(0, 10, 0.02);
        }
        // nothing should have banked
        assertEquals(0.0, c.getIntegral(), EPS);
        // unsat immediately
        assertEquals(-1.0, c.calculate(10, 0, 0.02), EPS);
    }

    @Test
    public void unwindsWhenUnsaturated() {
        PIDControllerEx c = new PIDControllerEx(0, 1, 0);
        for (int k = 0; k < 10; k++) {
            c.calculate(0, 10, 0.02);
        }
        assertTrue(c.getIntegral() > 0);
    }

    @Test
    public void outputClamped() {
        PIDControllerEx c = new PIDControllerEx(10, 0, 0).setOutputRange(-0.4, 0.6);
        assertEquals(0.6, c.calculate(0, 10, 0.02), EPS);
        assertEquals(-0.4, c.calculate(10, 0, 0.02), EPS);
    }

    @Test
    public void noKickOnSetpointJump() {
        PIDControllerEx c = open(0, 0, 1);
        c.calculate(0, 0, 0.1);
        assertEquals(0.0, c.calculate(0, 100, 0.1), EPS);
    }

    @Test
    public void kickOnError() {
        PIDControllerEx c = open(0, 0, 1).setDerivativeOnMeasurement(false);
        c.calculate(0, 0, 0.1);
        assertEquals(1000.0, c.calculate(0, 100, 0.1), EPS);
    }

    @Test
    public void dOpposesMotion() {
        PIDControllerEx c = open(0, 0, 1);
        c.calculate(0, 0, 0.1);
        assertEquals(-10.0, c.calculate(1, 0, 0.1), EPS);
    }

    @Test
    public void dFilter() {
        PIDControllerEx c = open(0, 0, 1).setDerivativeFilter(0.5);
        c.calculate(0, 0, 0.1);
        assertEquals(-5.0, c.calculate(1, 0, 0.1), EPS);
    }

    @Test
    public void dtZeroSkipsID() {
        PIDControllerEx c = open(0, 1, 1);
        assertEquals(0.0, c.calculate(0, 10, 0), EPS);
        assertEquals(0.0, c.getIntegral(), EPS);
    }

    @Test
    public void wrapsShortWay() {
        PIDControllerEx c = open(1, 0, 0).setContinuousInput(-180, 180);
        assertEquals(20.0, c.calculate(170, -170, 0.02), EPS);
        assertEquals(-20.0, c.calculate(-170, 170, 0.02), EPS);
    }

    @Test
    public void wrapDisabled() {
        PIDControllerEx c = open(1, 0, 0);
        assertEquals(-340.0, c.calculate(170, -170, 0.02), EPS);
    }

    @Test
    public void wrapsZeroToRange() {
        PIDControllerEx c = open(1, 0, 0).setContinuousInput(0, 360);
        assertEquals(20.0, c.calculate(350, 10, 0.02), EPS);
    }

    @Test
    public void atSetpoint() {
        PIDControllerEx c = open(1, 0, 0).setTolerance(1.0);
        c.calculate(0, 10, 0.02);
        assertFalse(c.atSetpoint());
        c.calculate(9.5, 10, 0.02);
        assertTrue(c.atSetpoint());
    }

    @Test
    public void velToleranceBlocks() {
        PIDControllerEx c = open(1, 0, 0).setTolerance(1.0, 1.0);
        c.calculate(0, 10, 0.1);
        // in tolerance but still moving fast
        c.calculate(10, 10, 0.1);
        assertFalse(c.atSetpoint());
    }

    @Test
    public void reset() {
        PIDControllerEx c = open(0, 1, 1);
        c.calculate(0, 10, 0.1);
        c.reset();
        assertEquals(0.0, c.getIntegral(), EPS);
        assertEquals(0.0, c.getDerivative(), EPS);
        assertEquals(0.0, c.calculate(5, 5, 0.1), EPS);
    }

    @Test
    public void resetIntegralKeepsD() {
        PIDControllerEx c = open(0, 1, 1);
        c.calculate(0, 0, 0.1);
        c.calculate(1, 0, 0.1);
        c.resetIntegral();
        assertEquals(0.0, c.getIntegral(), EPS);
        assertEquals(-10.0, c.getDerivative(), EPS);
    }

    @Test
    public void converges() {
        // 1st order plant, x += u*dt
        PIDControllerEx c = new PIDControllerEx(2.0, 0.5, 0.05).setTolerance(0.02);
        double x = 0;
        for (int k = 0; k < 500; k++) {
            x += c.calculate(x, 1.0, 0.02) * 0.02 * 10;
        }
        assertEquals(1.0, x, 0.02);
        assertTrue(c.atSetpoint());
    }
}
