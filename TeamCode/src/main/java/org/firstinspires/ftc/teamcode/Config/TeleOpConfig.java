package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.helpers.Curve;
import org.firstinspires.ftc.teamcode.helpers.CurveParams;

@Config
public class TeleOpConfig {
    // drivetrain
    public static double AIM_TURN_SCALE = 0.2;
    public static double STICK_DB = 0.03;

    public static Curve DRIVE_CURVE = Curve.EXPONENTIAL; // linear, cubic_bezier, smoothstep, exponential, quintic, (ALL CAPS)
    public static CurveParams CURVE_PARAMS = new CurveParams();

    public static boolean useFieldCentricDrive = true;

    // slew vals
    public static double ACCEL_UP = 4;
    public static double ACCEL_DOWN = 6.5; // 6.5 = slower
    public static double TURN_ACCEL_UP = 6.7;
    public static double TURN_ACCEL_DOWN = 6.7;


    // shooter values
    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 60;
    public static double FAST = 3000;
    public static double SLOW = 1100;
}
