package org.firstinspires.ftc.teamcode.pedro.mcl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * adapter only
 * sensorless MCL
 */
public class MCLLocalizerTest {
    private static final double[][] FIELD = {{0, 0}, {144, 0}, {144, 144}, {0, 144}, {0, 0}};
    private static final double EPS = 1e-6;

    private Localizer odometry;
    private MCL mcl;

    @Before
    public void setUp() {
        odometry = mock(Localizer.class);
        when(odometry.pose()).thenReturn(new Pose(0, 0, 0));
        when(odometry.state()).thenReturn(MotionState.zero());
        when(odometry.debug()).thenReturn(new HashMap<>());
        mcl = new MCL(Collections.emptyList(), FIELD, 50, 1.0, 100, 54);
    }

    @Test
    public void seedsFromOdom() {
        when(odometry.pose()).thenReturn(new Pose(24, 48, 0));

        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);

        assertEquals(24.0, mcl.getEstimateX(), EPS);
        assertEquals(48.0, mcl.getEstimateY(), EPS);
        assertEquals(24.0, loc.pose().x(), EPS);
    }

    @Test
    public void usesOdomDelta() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        when(odometry.pose()).thenReturn(new Pose(10, 0, 0));
        loc.update();
        when(odometry.pose()).thenReturn(new Pose(10, 5, 0));
        loc.update();

        assertEquals(10.0, loc.pose().x(), EPS);
        assertEquals(5.0, loc.pose().y(), EPS);
    }

    @Test
    public void headingPassthrough() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        when(odometry.pose()).thenReturn(new Pose(0, 0, 1.25));

        loc.update();

        assertEquals(1.25, loc.pose().heading(), EPS);
    }

    @Test
    public void updatesBeforeReading() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        loc.update();

        InOrder order = inOrder(odometry);
        order.verify(odometry).update();
        order.verify(odometry).pose();
    }

    @Test
    public void skipsNaN() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        when(odometry.pose()).thenReturn(new Pose(20, 0, 0));
        loc.update();

        when(odometry.pose()).thenReturn(new Pose(Double.NaN, 0, 0));
        loc.update();
        assertEquals(20.0, loc.pose().x(), EPS);

        when(odometry.pose()).thenReturn(new Pose(25, 0, 0));
        loc.update();
        assertEquals(25.0, loc.pose().x(), EPS);
        assertTrue(Double.isFinite(loc.pose().x()));
    }

    @Test
    public void setPoseReseeds() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        Pose target = new Pose(100, 60, 0.5);

        loc.setPose(target);

        verify(odometry).setPose(target);
        assertEquals(100.0, mcl.getEstimateX(), EPS);
        assertEquals(60.0, mcl.getEstimateY(), EPS);
        assertEquals(100.0, loc.pose().x(), EPS);
    }

    @Test
    public void setPoseClearsDelta() {
        // odom uses setPose like fr hw
        doAnswer(inv -> {
            when(odometry.pose()).thenReturn(inv.getArgument(0));
            return null;
        }).when(odometry).setPose(any(Pose.class));

        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        when(odometry.pose()).thenReturn(new Pose(40, 0, 0));
        loc.update();

        loc.setPose(new Pose(0, 0, 0));
        loc.update();

        assertEquals(0.0, loc.pose().x(), EPS);
    }

    @Test
    public void reset() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        when(odometry.pose()).thenReturn(new Pose(80, 80, 0));
        loc.update();
        when(odometry.pose()).thenReturn(new Pose(0, 0, 0));

        loc.reset();

        verify(odometry).reset();
        assertEquals(0.0, loc.pose().x(), EPS);
        assertEquals(0.0, mcl.getEstimateX(), EPS);
    }

    @Test
    public void stateSwapsPose() {
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);
        MotionState odomState = mock(MotionState.class);
        MotionState swapped = mock(MotionState.class);
        when(odometry.state()).thenReturn(odomState);
        when(odomState.withPose(any(Pose.class))).thenReturn(swapped);
        when(odometry.pose()).thenReturn(new Pose(12, 34, 0));
        loc.update();

        assertEquals(swapped, loc.state());
        verify(odomState).withPose(any(Pose.class));
    }

    @Test
    public void debugKeys() {
        Map<String, Object> odomDebug = new HashMap<>();
        odomDebug.put("odo/foo", 1);
        when(odometry.debug()).thenReturn(odomDebug);
        MCLLocalizer loc = new MCLLocalizer(odometry, mcl);

        Map<String, Object> d = loc.debug();

        assertEquals(1, d.get("odo/foo"));
        assertTrue(d.containsKey("mcl/x"));
        assertTrue(d.containsKey("mcl/y"));
        assertTrue(d.containsKey("mcl/odomX"));
        assertTrue(d.containsKey("mcl/odomY"));

        assertTrue(!odomDebug.containsKey("mcl/x"));
    }
}
