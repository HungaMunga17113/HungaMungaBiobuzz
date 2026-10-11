package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;
import org.junit.Test;

/** shared curve invariants + the branchy ones */
public class CurveTest {
    private static final double EPS = 1e-9;

    private static CurveParams params() {
        return new CurveParams();
    }

    @Test
    public void zeroIn() {
        for (Curve c : Curve.values()) {
            assertEquals(c.name(), 0.0, c.apply(0.0, params()), EPS);
        }
    }

    @Test
    public void fullStick() {
        for (Curve c : Curve.values()) {
            assertEquals(c.name(), 1.0, c.apply(1.0, params()), 1e-9);
            assertEquals(c.name(), -1.0, c.apply(-1.0, params()), 1e-9);
        }
    }

    @Test
    public void odd() {
        for (Curve c : Curve.values()) {
            for (double x = 0.05; x <= 1.0; x += 0.05) {
                assertEquals(c.name() + " @ " + x, -c.apply(x, params()), c.apply(-x, params()), 1e-12);
            }
        }
    }

    @Test
    public void monotonicInRange() {
        for (Curve c : Curve.values()) {
            double prev = -1e9;
            for (double x = 0.0; x <= 1.0; x += 0.01) {
                double y = c.apply(x, params());
                assertTrue(c.name() + " range @ " + x, y >= -EPS && y <= 1 + EPS);
                assertTrue(c.name() + " monotonic @ " + x, y >= prev - 1e-12);
                prev = y;
            }
        }
    }

    @Test
    public void linearIdentity() {
        assertEquals(0.37, Curve.LINEAR.apply(0.37, params()), EPS);
    }

    @Test
    public void expZeroSharpness() {
        CurveParams p = params();
        p.sharpness = 0;
        assertEquals(0.42, Curve.EXPONENTIAL.apply(0.42, p), EPS);
    }

    @Test
    public void expSoftLowEnd() {
        CurveParams p = params();
        p.sharpness = 2.5;
        assertTrue(Curve.EXPONENTIAL.apply(0.5, p) < 0.5);
    }

    @Test
    public void sCurveMidpoints() {
        assertTrue(Curve.SMOOTHSTEP.apply(0.1, params()) < 0.1);
        assertTrue(Curve.QUINTIC.apply(0.1, params()) < Curve.SMOOTHSTEP.apply(0.1, params()));
        assertEquals(0.5, Curve.SMOOTHSTEP.apply(0.5, params()), EPS);
        assertEquals(0.5, Curve.QUINTIC.apply(0.5, params()), EPS);
    }

    @Test
    public void bezierControlPoints() {
        CurveParams soft = params();
        soft.bezierP1 = 0.0;
        soft.bezierP2 = 0.0;
        CurveParams aggressive = params();
        aggressive.bezierP1 = 1.0;
        aggressive.bezierP2 = 1.0;

        double s = Curve.CUBIC_BEZIER.apply(0.5, soft);
        double a = Curve.CUBIC_BEZIER.apply(0.5, aggressive);

        assertTrue(s < a);
        assertEquals(0.125, s, EPS);
        assertEquals(1.0, Curve.CUBIC_BEZIER.apply(1.0, soft), EPS);
        assertEquals(1.0, Curve.CUBIC_BEZIER.apply(1.0, aggressive), EPS);
    }
}
