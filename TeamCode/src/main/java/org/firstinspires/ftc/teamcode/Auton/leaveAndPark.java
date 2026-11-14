package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Leave and park (use this one)", group = "Autonomous")
public class leaveAndPark extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        DcMotorEx[] motors = {leftBack, rightBack, leftFront, rightFront};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // positive power means spin wheel forwards
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        resetRuntime();
        double timer = getRuntime();

        leftBack.setPower(0.5);
        leftFront.setPower(0.5);
        rightBack.setPower(0.5);
        rightFront.setPower(0.5);
        while (timer < .3) {
            timer = getRuntime();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}
