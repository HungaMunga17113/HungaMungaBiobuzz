package org.firstinspires.ftc.teamcode.TeleOp.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="shooter TESTER", group="misc")
public class ShooterTestingOpMode extends OpMode {
    private DcMotorEx shooter;


    @Override
    public void init() {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("status: ", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        shooter.setPower(gamepad1.right_trigger);
    }
}
