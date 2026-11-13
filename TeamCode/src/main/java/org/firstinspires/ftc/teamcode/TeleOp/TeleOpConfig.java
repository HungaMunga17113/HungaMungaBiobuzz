package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Conversions;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;

@Configurable
public class TeleOpConfig {
    /// drivetrain
    public static double AIM_TURN_SCALE = 0.2;
    public static double STICK_DB = 0.03;
    public static Curve DRIVE_CURVE = Curve.EXPONENTIAL; // linear, cubic_bezier, smoothstep, exponential, quintic, (ALL CAPS)
    public static CurveParams CURVE_PARAMS = new CurveParams();
    public static boolean useFieldCentricDrive = true;

    /// field localization + snap-to-goal
    // off GLOBAL (pedro) positions
    // heading deg ccw+, 0 = facing right (+x) como el unit circle
    // each alliance localizes on the close side, in one of GLOBAL top or GLOBAL bottom
    public static double FIELD_SIZE = 142; // shud be 144 but 142 realistic

    // localization poses (driver-relative rx)
    // intake faces inward, toward the goals when localizing
    public static double BLUE_LOC_X = 8,   BLUE_TOP_Y = 132, BLUE_TOP_H = -90, BLUE_BOTTOM_Y = 8, BLUE_BOTTOM_H = 90;
    public static double RED_LOC_X  = 132, RED_TOP_Y  = 132, RED_TOP_H  = -90, RED_BOTTOM_Y  = 8, RED_BOTTOM_H  = 90;

    // goals (global), alliances shoot onto far side tipper
    public static double RED_GOAL_X  = 58, RED_TOP_GOAL_Y  = 83, RED_BOTTOM_GOAL_Y  = 55;
    public static double BLUE_TOP_GOAL_X = 83, BLUE_TOP_GOAL_Y = 81, BLUE_BOTTOM_GOAL_X = 84, BLUE_BOTTOM_GOAL_Y = 55;

    // driver's forward
    public static double BLUE_FORWARD_H = 0, RED_FORWARD_H = 180;

    // goal lock heading pd (deg in, power out)
    public static double LOCK_P = 0.025, LOCK_D = 0.0015;
    public static double LOCK_MAX_POWER = 0.8;
    public static double LOCK_MIN_DIST = 6; // in, no aim inside this

    /// slew vals
    public static double ACCEL_UP = 4;
    public static double ACCEL_DOWN = 11; // higher = faster stop
    public static double TURN_ACCEL_UP = 9.5;
    public static double TURN_ACCEL_DOWN = 12;

    /// shooter stuff

    public static int rpm_1 = -6000;
    public static int rpm_2 = 0;
    public static enum TuningModes { BUILT_IN, CUSTOM, SVA};
    public static TuningModes TuningMode = TuningModes.BUILT_IN;

    // builtin pid+f
    public static double bkP = 0.42;
    public static double bkI = 1;
    public static double bkD = 0.282;
    public static double bkF = 12.3;

    // custom pid+f
    public static double ckP = 60;
    public static double ckI = 0;
    public static double ckD = 10;
    public static double ckF = 0;

    // custom pid+sva
    public static double vkP = 0;
    public static double vkI = 0;
    public static double vkD = 0;
    public static double vkS = 0;
    public static double vkV = 0;
    public static double vkA = 0;

}
