package org.firstinspires.ftc.teamcode.helpers;

public class Conversions {

    ///goBILDA 5202/5203/5204 Yellow Jacket ticks/rotation at output shaft
    public static final double TPR_6000_RPM = 28.0;   // 1:1 Ratio
    public static  final double TPR_1620_RPM = 103.8;  // 3.7:1 Ratio
    public static final double TPR_1150_RPM = 145.1;  // 5.2:1 Ratio
    public static final double TPR_435_RPM  = 384.5;  // 13.7:1 Ratio
    public static final double TPR_312_RPM  = 537.6;  // 19.2:1 Ratio
    public static final double TPR_223_RPM  = 751.8;  // 26.9:1 Ratio
    public static final double TPR_117_RPM  = 1425.1; // 50.9:1 Ratio
    public static final double TPR_84_RPM   = 1993.6; // 71.2:1 Ratio
    public static final double TPR_60_RPM   = 2786.2; // 99.5:1 Ratio
    public static final double TPR_43_RPM   = 3895.9; // 139:1 Ratio
    public static final double TPR_30_RPM   = 5281.1; // 188:1 Ratio

    /**
     * converts target RPM to TPS
     * usage: motor.setVelocity(Conversions.rpmToTps(100, Conversions.TPR_312_RPM));
     */
    public static double rpmToTps(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    /**
     * converts TPS to RPM.
     * usage: double currentRPM = Conversions.tpsToRpm(motor.getVelocity(), Conversions.TPR_312_RPM);
     */
    public static double tpsToRpm(double tps, double ticksPerRev) {
        return (tps * 60.0) / ticksPerRev;
    }

    /**
     * converts revolutions to ticks
     */
    public static double revToTicks(double rev, double ticksPerRev) {
        return rev * ticksPerRev;
    }

    /**
     * converts ticks to revolutions
     */
    public static double ticksToRev(double ticks, double ticksPerRev) {
        return ticks / ticksPerRev;
    }

    /**
     * clamp a value between a min and max
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * return max tps of a motor
     */
    public static double maxTps(double rpm, double ticksPerRev) {
        return rpmToTps(rpm, ticksPerRev);
    }

    /// =======================

    // converts inches to ticks given inches to move, wheel diameter and tpr
    public static double inchesToTicks(double inches, double wheelDiameterInches, double ticksPerRev) {
        double circumference = Math.PI * wheelDiameterInches;
        return (inches / circumference) * ticksPerRev;
    }

    // converts ticks to inches given ticks to move, wheel diameter and tpr
    public static double ticksToInches(double ticks, double wheelDiameterInches, double ticksPerRev) {
        double circumference = Math.PI * wheelDiameterInches;
        return (ticks / ticksPerRev) * circumference;
    }

    public static double degreesToTicks(double degrees, double ticksPerRev) {
        return (degrees / 360.0) * ticksPerRev;
    }

    public static double ticksToDegrees(double ticks, double ticksPerRev) {
        return (ticks / ticksPerRev) * 360.0;
    }
}