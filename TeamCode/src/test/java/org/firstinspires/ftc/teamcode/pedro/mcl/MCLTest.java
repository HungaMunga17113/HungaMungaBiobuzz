package org.firstinspires.ftc.teamcode.pedro.mcl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** raycast geometry, sensor scaling, filter behavior */
public class MCLTest {
    /** 144x144 square, pedro coords */
    private static final double[][] FIELD = {{0, 0}, {144, 0}, {144, 144}, {0, 144}, {0, 0}};
    private static final double MAX_READING = 200.0;
    private static final double EPS = 1e-6;

    /** fake sensor: voltage scaled so readInches() == inches */
    private static AnalogInput sensorAt(double inches) {
        AnalogInput in = mock(AnalogInput.class);
        when(in.getMaxVoltage()).thenReturn(3.3);
        when(in.getVoltage()).thenReturn(inches / 100.0 * 3.3);
        return in;
    }

    /** maxRangeMm such that full scale == 100 in */
    private static MCL.RangeSensor rangeSensor(double inches, double fw, double lat, double turn) {
        return new MCL.RangeSensor(sensorAt(inches), fw, lat, turn, 100.0 * MCL.MM_PER_INCH);
    }

    // ---- raycast ----

    @Test
    public void raycastWalls() {
        assertEquals(72.0, MCL.raycast(FIELD, 72, 72, 0, MAX_READING), 1e-9);
        assertEquals(72.0, MCL.raycast(FIELD, 72, 72, Math.PI / 2, MAX_READING), 1e-9);
        assertEquals(72.0, MCL.raycast(FIELD, 72, 72, Math.PI, MAX_READING), 1e-9);
        assertEquals(72.0, MCL.raycast(FIELD, 72, 72, -Math.PI / 2, MAX_READING), 1e-9);
    }

    @Test
    public void raycastNearest() {
        // 10 in from +x wall, 134 from -x wall
        assertEquals(10.0, MCL.raycast(FIELD, 134, 72, 0, MAX_READING), 1e-9);
    }

    @Test
    public void raycastDiagonal() {
        assertEquals(Math.hypot(72, 72), MCL.raycast(FIELD, 72, 72, Math.PI / 4, MAX_READING), 1e-6);
    }

    @Test
    public void raycastCap() {
        assertEquals(20.0, MCL.raycast(FIELD, 72, 72, 0, 20.0), 1e-9);
    }

    @Test
    public void raycastBehind() {
        // outside the square facing away, nothing in front
        assertEquals(MAX_READING, MCL.raycast(FIELD, 200, 72, 0, MAX_READING), 1e-9);
    }

    @Test
    public void raycastParallel() {
        // ray along the y=0 wall, denom ~ 0 for it; must not NaN or return a bogus hit
        double d = MCL.raycast(FIELD, 72, 0, 0, MAX_READING);
        assertTrue(Double.isFinite(d));
        assertEquals(72.0, d, 1e-9);
    }

    // ---- expectedReading ----

    @Test
    public void forwardOffset() {
        MCL.RangeSensor s = rangeSensor(0, 6, 0, 0);
        // center (72,72) facing +x, sensor 6 in forward -> 66 to the wall
        assertEquals(66.0, MCL.expectedReading(s, 72, 72, 0, FIELD, MAX_READING), 1e-9);
    }

    @Test
    public void lateralOffset() {
        MCL.RangeSensor s = rangeSensor(0, 0, 10, 0);
        // lateral +10 moves +y when facing +x, so x distance is unchanged
        assertEquals(72.0, MCL.expectedReading(s, 72, 72, 0, FIELD, MAX_READING), 1e-9);
        // facing +y, lateral +10 moves -x
        assertEquals(72.0, MCL.expectedReading(s, 72, 72, Math.PI / 2, FIELD, MAX_READING), 1e-9);
        MCL.RangeSensor fw = rangeSensor(0, 10, 0, 0);
        assertEquals(62.0, MCL.expectedReading(fw, 72, 72, Math.PI / 2, FIELD, MAX_READING), 1e-9);
    }

    @Test
    public void sensorTurn() {
        // body faces +x, sensor turned +90 deg looks at the +y wall
        MCL.RangeSensor s = rangeSensor(0, 0, 0, Math.PI / 2);
        assertEquals(72.0, MCL.expectedReading(s, 72, 72, 0, FIELD, MAX_READING), 1e-9);
        assertEquals(40.0, MCL.expectedReading(s, 72, 104, 0, FIELD, MAX_READING), 1e-9);
    }

    @Test
    public void rotatesWithHeading() {
        MCL.RangeSensor s = rangeSensor(0, 0, 0, 0);
        assertEquals(44.0, MCL.expectedReading(s, 100, 72, 0, FIELD, MAX_READING), 1e-9);
        assertEquals(100.0, MCL.expectedReading(s, 100, 72, Math.PI, FIELD, MAX_READING), 1e-9);
    }

    // ---- RangeSensor ----

    @Test
    public void voltageToInches() {
        AnalogInput in = mock(AnalogInput.class);
        when(in.getMaxVoltage()).thenReturn(3.3);
        when(in.getVoltage()).thenReturn(1.65);
        MCL.RangeSensor s = new MCL.RangeSensor(in, 0, 0, 0, 4000);

        assertEquals(0.5, s.readRatio(), EPS);
        assertEquals(2000 / MCL.MM_PER_INCH, s.readInches(), EPS);
    }

    // ---- mclStep ----

    @Test
    public void sensorlessIsOdomOnly() {
        MCL mcl = new MCL(Collections.emptyList(), FIELD, 50, 1.0, MAX_READING, 54);
        mcl.reset(20, 30, 0);

        mcl.mclStep(5, -2, 0);
        mcl.mclStep(5, -2, 0);

        assertEquals(30.0, mcl.getEstimateX(), EPS);
        assertEquals(26.0, mcl.getEstimateY(), EPS);
    }

    @Test
    public void convergesFromBadSeed() {
        // two sensors: +x wall and +y wall. truth (100, 60) -> 44 and 84
        List<MCL.RangeSensor> sensors = Arrays.asList(
                rangeSensor(44, 0, 0, 0),
                rangeSensor(84, 0, 0, Math.PI / 2));
        MCL mcl = new MCL(sensors, FIELD, 600, 1.0, MAX_READING, 120);
        mcl.reset(90, 70, 2.0);

        double before = Math.hypot(mcl.getEstimateX() - 100, mcl.getEstimateY() - 60);
        for (int i = 0; i < 120; i++) {
            mcl.mclStep(0, 0, 0);
        }
        double after = Math.hypot(mcl.getEstimateX() - 100, mcl.getEstimateY() - 60);

        assertTrue("error grew: " + before + " -> " + after, after < before / 2);
        assertTrue("did not converge: " + after, after < 4.0);
    }

    @Test
    public void skipsBeyondMaxUsable() {
        // reading says 44 in, but expected 72 > maxUsable 20 -> reading discarded, odom only
        List<MCL.RangeSensor> sensors = Collections.singletonList(rangeSensor(44, 0, 0, 0));
        MCL mcl = new MCL(sensors, FIELD, 400, 1.0, MAX_READING, 20);
        mcl.reset(72, 72, 0);

        for (int i = 0; i < 40; i++) {
            mcl.mclStep(0, 0, 0);
        }

        // no correction, so it only random-walks; it must not be dragged to x=100
        assertTrue(Math.abs(mcl.getEstimateX() - 72) < 10);
    }

    @Test
    public void weightCollapseStaysFinite() {
        // impossible reading -> all weights ~ 0
        List<MCL.RangeSensor> sensors = Collections.singletonList(rangeSensor(0, 0, 0, 0));
        MCL mcl = new MCL(sensors, FIELD, 200, 0.5, MAX_READING, 120);
        mcl.reset(72, 72, 2.0);

        for (int i = 0; i < 20; i++) {
            mcl.mclStep(0, 0, 0);
        }

        assertTrue(Double.isFinite(mcl.getEstimateX()));
        assertTrue(Double.isFinite(mcl.getEstimateY()));
    }

    @Test
    public void reset() {
        MCL mcl = new MCL(new ArrayList<>(), FIELD, 100, 1.0, MAX_READING, 54);
        mcl.reset(12, 34, 3.0);

        assertEquals(12.0, mcl.getEstimateX(), EPS);
        assertEquals(34.0, mcl.getEstimateY(), EPS);

        // reseeding moves the estimate, no leftover state
        mcl.reset(-5, 0, 0);
        assertEquals(-5.0, mcl.getEstimateX(), EPS);
        assertEquals(0.0, mcl.getEstimateY(), EPS);
    }
}
