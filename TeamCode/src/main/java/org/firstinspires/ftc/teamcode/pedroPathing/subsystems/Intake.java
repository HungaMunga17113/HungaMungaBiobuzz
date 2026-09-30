package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intake;

    public Intake(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        intake = hardwareMap.get(DcMotorEx.class, motorName);
        intake.setDirection(direction);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
    public void stop() {
        setPower(0.0);
    }
}
