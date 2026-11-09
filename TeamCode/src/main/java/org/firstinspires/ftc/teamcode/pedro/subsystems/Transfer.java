package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private final Servo transfer;

    public Transfer(HardwareMap hardwareMap, String servoName) {
        transfer = hardwareMap.get(Servo.class, servoName);
    }

    public void shoot() {
        transfer.setPosition(0.65);
    }
    public void hold() { transfer.setPosition(1.0); }
}
