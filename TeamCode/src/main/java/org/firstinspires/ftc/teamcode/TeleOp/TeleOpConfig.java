package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;

@Config
public class TeleOpConfig {
    /// drivetrain
    public static double AIM_TURN_SCALE = 0.2;
    public static double STICK_DB = 0.03;
    public static Curve DRIVE_CURVE = Curve.EXPONENTIAL; // linear, cubic_bezier, smoothstep, exponential, quintic, (ALL CAPS)
    public static CurveParams CURVE_PARAMS = new CurveParams();
    public static boolean useFieldCentricDrive = false;

    /// slew vals
    public static double ACCEL_UP = 4;
    public static double ACCEL_DOWN = 6.5; // higher = faster stop
    public static double TURN_ACCEL_UP = 6.7;
    public static double TURN_ACCEL_DOWN = 6.7;

    /// shooter stuff
    public static enum TuningModes { BUILT_IN, CUSTOM, SVA};
    public static TuningModes TuningMode = TuningModes.BUILT_IN;

    public static double bkP = 0;
    public static double bkI = 0;
    public static double bkD = 0;
    public static double bkF = 0;

    public static double ckP = 0;
    public static double ckI = 0;
    public static double ckD = 0;
    public static double ckF = 0;
    public static double vkP = 0;
    public static double vkI = 0;
    public static double vkD = 0;
    public static double vkS = 0;
    public static double vkV = 0;
    public static double vkA = 0;

}
