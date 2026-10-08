package org.firstinspires.ftc.teamcode.pedro.mcl;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import java.util.HashMap;
import java.util.Map;

/**
 * adapter between MCL class and Pedro Localizer interface
 * extracted mainly for scalability, all fancy math stuff still goes in MCL class
 * updates odom, calcs x/y deltas since last loop, calls mclStep, creates a pose from allat
 * reseeds particles when pedro calls setPose / reset
 * usage: see Constants.create()
 */
public class MCLLocalizer implements Localizer {
    public static final double START_SPREAD = 2.0;

    private final Localizer odometry;
    private final MCL mcl;
    private Pose lastOdom;
    private Pose pose;

    public MCLLocalizer(Localizer odometry, MCL mcl) {
        this.odometry = odometry;
        this.mcl = mcl;
        sync(odometry.pose());
    }

    @Override
    public void setPose(Pose setPose) {
        odometry.setPose(setPose);
        sync(setPose);
    }

    private void sync(Pose p) {
        mcl.reset(p.x(), p.y(), START_SPREAD);
        lastOdom = p;
        pose = p;
    }

    @Override
    public void update() {
        odometry.update();
        Pose odom = odometry.pose();
        if (isNaN(odom)) {
            return;
        }
        double deltaX = odom.x() - lastOdom.x();
        double deltaY = odom.y() - lastOdom.y();
        lastOdom = odom;

        mcl.mclStep(deltaX, deltaY, odom.heading());
        pose = new Pose(mcl.getEstimateX(), mcl.getEstimateY(), odom.heading());
    }

    /** odometry velocity, MCL-corrected pose */
    @Override
    public MotionState state() {
        return odometry.state().withPose(pose);
    }

    @Override
    public void reset() {
        odometry.reset();
        odometry.update();
        sync(odometry.pose());
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> d = new HashMap<>(odometry.debug());
        d.put("mcl/odomX", lastOdom.x());
        d.put("mcl/odomY", lastOdom.y());
        d.put("mcl/x", pose.x());
        d.put("mcl/y", pose.y());
        return d;
    }

    private static boolean isNaN(Pose p) {
        return p == null || Double.isNaN(p.x()) || Double.isNaN(p.y()) || Double.isNaN(p.heading());
    }
}
