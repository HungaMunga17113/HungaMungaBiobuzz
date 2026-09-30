package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.helpers.Conversions;

/** Shooter wrapper with simple RPM target using built-in velocity control. */
public class Shooter {
    private final DcMotorEx shooter;
    private final double ticksPerRev;
    private double targetRpm;

    public Shooter(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction, double ticksPerRev) {
        shooter = hardwareMap.get(DcMotorEx.class, motorName);
        shooter.setDirection(direction);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //values before cosmo -- kP:580 kI:8.8 kD:4 kF:47.85
        shooter.setVelocityPIDFCoefficients(580,0,0,0);
        this.ticksPerRev = ticksPerRev;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        shooter.setVelocity(Conversions.rpmToTps(rpm, ticksPerRev));
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return Conversions.tpsToRpm(shooter.getVelocity(), ticksPerRev);
    }
    public boolean waitForRpm(double targetRPM) {
        double currentVel = shooter.getVelocity();

        return Math.abs(targetRPM - currentVel) > 100;

    }

    public void stop() {
        shooter.setPower(0.0);
    }
}
