package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.helpers.Curve;
import org.firstinspires.ftc.teamcode.helpers.CurveParams;

@Config
public class TeleOpConfig {
    public static double AIM_TURN_SCALE = 0.2;
    public static double STICK_DB = 0.05;
    public static Curve DRIVE_CURVE = Curve.SIGMOID; //linear, cubic_bezier, smoothstep, exponential, quintic, sigmoid (ALL CAPS)
    //preset specific
    public static CurveParams CURVE_PARAMS = new CurveParams();

    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 60;
    public static double FAST = 3000;
    public static double SLOW = 1100;
}
