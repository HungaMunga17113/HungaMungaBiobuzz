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

    /// slew vals
    public static double ACCEL_UP = 4;
    public static double ACCEL_DOWN = 11; // higher = faster stop
    public static double TURN_ACCEL_UP = 9.5;
    public static double TURN_ACCEL_DOWN = 12;

    /// shooter stuff

    public static int rpm_1 = -5200;
    public static int rpm_2 = 0;
    public static enum TuningModes { BUILT_IN, CUSTOM, SVA};
    public static TuningModes TuningMode = TuningModes.BUILT_IN;

    // builtin pid+f
    public static double bkP = 7.3;
    public static double bkI = 4;
    public static double bkD = 20;
    public static double bkF = 10.2;

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
