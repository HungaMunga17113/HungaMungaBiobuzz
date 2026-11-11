package org.firstinspires.ftc.teamcode.TeleOp.OpModes;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShootOnMoveOpModeTest {
    private static final double EPS = 1e-9;

    private double goalX, goalY, latency, offset;
    private int iters;
    private double[] dist, rpm, tof;

    @Before
    public void setUp() {
        goalX = ShootOnMoveOpMode.goalX;
        goalY = ShootOnMoveOpMode.goalY;
        latency = ShootOnMoveOpMode.latency;
        offset = ShootOnMoveOpMode.shooterOffset;
        iters = ShootOnMoveOpMode.solveIters;
        dist = ShootOnMoveOpMode.LUT_DIST;
        rpm = ShootOnMoveOpMode.LUT_RPM;
        tof = ShootOnMoveOpMode.LUT_TOF;

        // goal 100 in down +x, tof = d/100, rpm = 10*d
        ShootOnMoveOpMode.goalX = 100;
        ShootOnMoveOpMode.goalY = 0;
        ShootOnMoveOpMode.latency = 0;
        ShootOnMoveOpMode.shooterOffset = 0;
        ShootOnMoveOpMode.solveIters = 3;
        ShootOnMoveOpMode.LUT_DIST = new double[]{0, 100};
        ShootOnMoveOpMode.LUT_RPM = new double[]{0, 1000};
        ShootOnMoveOpMode.LUT_TOF = new double[]{0, 1};
    }

    @After
    public void tearDown() {
        ShootOnMoveOpMode.goalX = goalX;
        ShootOnMoveOpMode.goalY = goalY;
        ShootOnMoveOpMode.latency = latency;
        ShootOnMoveOpMode.shooterOffset = offset;
        ShootOnMoveOpMode.solveIters = iters;
        ShootOnMoveOpMode.LUT_DIST = dist;
        ShootOnMoveOpMode.LUT_RPM = rpm;
        ShootOnMoveOpMode.LUT_TOF = tof;
    }

    // ---- lerp ----

    @Test
    public void lerpMid() {
        assertEquals(500, ShootOnMoveOpMode.lerp(ShootOnMoveOpMode.LUT_RPM, 50), EPS);
    }

    @Test
    public void lerpClamps() {
        assertEquals(0, ShootOnMoveOpMode.lerp(ShootOnMoveOpMode.LUT_RPM, -10), EPS);
        assertEquals(1000, ShootOnMoveOpMode.lerp(ShootOnMoveOpMode.LUT_RPM, 200), EPS);
    }

    @Test
    public void lerpMultiKnot() {
        ShootOnMoveOpMode.LUT_DIST = new double[]{0, 50, 100};
        double[] ys = {0, 800, 1000};
        assertEquals(800, ShootOnMoveOpMode.lerp(ys, 50), EPS);
        assertEquals(900, ShootOnMoveOpMode.lerp(ys, 75), EPS);
        assertEquals(400, ShootOnMoveOpMode.lerp(ys, 25), EPS);
    }

    // ---- wrap ----

    @Test
    public void wrapRange() {
        assertEquals(0, ShootOnMoveOpMode.wrap(0), EPS);
        assertEquals(-170, ShootOnMoveOpMode.wrap(190), EPS);
        assertEquals(170, ShootOnMoveOpMode.wrap(-190), EPS);
        assertEquals(180, ShootOnMoveOpMode.wrap(540), EPS);
        assertEquals(-180, ShootOnMoveOpMode.wrap(-540), EPS);
    }

    // ---- aim ----

    @Test
    public void stillAim() {
        double[] a = ShootOnMoveOpMode.aim(0, 0, 0, 0);
        assertEquals(100, a[0], EPS);
        assertEquals(1000, a[1], EPS);
        assertEquals(0, a[2], EPS);
        assertEquals(0, a[3], EPS);
    }

    @Test
    public void stillHalfway() {
        double[] a = ShootOnMoveOpMode.aim(50, 0, 0, 0);
        assertEquals(50, a[0], EPS);
        assertEquals(500, a[1], EPS);
    }

    @Test
    public void headingQuadrant() {
        ShootOnMoveOpMode.goalX = 0;
        ShootOnMoveOpMode.goalY = 100;
        assertEquals(90, ShootOnMoveOpMode.aim(0, 0, 0, 0)[2], EPS);
    }

    @Test
    public void shooterOffset() {
        ShootOnMoveOpMode.shooterOffset = 180;
        assertEquals(180, ShootOnMoveOpMode.aim(0, 0, 0, 0)[2], EPS);
    }

    @Test
    public void latencyLeads() {
        ShootOnMoveOpMode.solveIters = 0;
        ShootOnMoveOpMode.latency = 0.5;
        // predicted x = 10, goal not shifted w/ 0 iters
        assertEquals(90, ShootOnMoveOpMode.aim(0, 0, 20, 0)[0], EPS);
    }

    @Test
    public void strafeOneIter() {
        ShootOnMoveOpMode.solveIters = 1;
        // tof(100) = 1 -> virtual goal (100, -10)
        double[] a = ShootOnMoveOpMode.aim(0, 0, 0, 10);
        assertEquals(Math.hypot(100, 10), a[0], EPS);
        assertEquals(1000, a[1], EPS);
        assertEquals(Math.toDegrees(Math.atan2(-10, 100)), a[2], EPS);
        assertEquals(Math.toDegrees(-1000.0 / 10100.0), a[3], EPS);
    }

    @Test
    public void strafeConverges() {
        ShootOnMoveOpMode.solveIters = 20;
        ShootOnMoveOpMode.LUT_DIST = new double[]{0, 200};
        ShootOnMoveOpMode.LUT_TOF = new double[]{0, 2};
        ShootOnMoveOpMode.LUT_RPM = new double[]{0, 2000};
        double[] a = ShootOnMoveOpMode.aim(0, 0, 0, 10);
        // fixed pt: gy = -vy * tof(dist)
        double gy = 100 * Math.tan(Math.toRadians(a[2]));
        assertEquals(-10 * a[0] / 100, gy, 1e-6);
    }

    @Test
    public void omegaSign() {
        // moving +y past a +x goal, bearing turns cw
        assertEquals(-1, Math.signum(ShootOnMoveOpMode.aim(0, 0, 0, 10)[3]), EPS);
        assertEquals(1, Math.signum(ShootOnMoveOpMode.aim(0, 0, 0, -10)[3]), EPS);
    }

    @Test
    public void approachNoOmega() {
        assertEquals(0, ShootOnMoveOpMode.aim(0, 0, 20, 0)[3], EPS);
    }

    @Test
    public void onGoal() {
        double[] a = ShootOnMoveOpMode.aim(100, 0, 0, 0);
        assertEquals(0, a[0], EPS);
        assertEquals(0, a[3], EPS);
    }
}
