package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.TeleOpConfig;

@TeleOp(name = "drivetrain only", group = "Main")
public class Dt extends OpMode {
    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        DcMotorEx[] motors = {leftBack, rightBack, leftFront, rightFront};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }

    @Override
    public void loop() {
        drive();
    }

    private void drive() {
        // get heading
        pinpoint.update();
        double currHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        telemetry.addData("heading", currHeading);

        double leftX = deadband(gamepad1.left_stick_x);
        double leftY = -deadband(gamepad1.left_stick_y);
        double rightX = deadband(gamepad1.right_stick_x);
        double rightY = -deadband(-gamepad1.right_stick_y);

        boolean aim = gamepad1.left_stick_button;
        double xCurved = aim ? TeleOpConfig.AIM_TURN_SCALE * leftX : curve(leftX);
        double yCurved = aim ? TeleOpConfig.AIM_TURN_SCALE * leftY : curve(leftY);

        double xOut;
        double yOut;
        double rotOut;
        if (TeleOpConfig.useFieldCentricDrive) {
//            telemetry.addData("right x", rightX);
//            telemetry.addData("right y", rightY);
//            double targetHeading = Math.toDegrees(-Math.atan2(rightY, rightX));
//            telemetry.addData("target heading", targetHeading);
//            double rotDelta = targetHeading - currHeading;
//            telemetry.addData("rot delta", rotDelta);
//            //TODO: add deadband
//            //TODO: cap rotDelta between (-180, 180]
//
//
//            //TODO: rotate xCurved/yCurved by rotDelta
//            xOut = 0;
//            yOut = 0;
//
//            //TODO: convert degrees to motor power
//            // positive rotDelta means turn the robot ccw, negative means turn cw
//            rotOut = 0;
            rotOut = gamepad1.right_stick_button
                    ? TeleOpConfig.AIM_TURN_SCALE * rightX
                    : curve(rightX);
            xOut = xCurved * Math.cos(Math.toRadians(currHeading)) + yCurved * Math.sin(Math.toRadians((currHeading)));
            yOut = yCurved * Math.cos(Math.toRadians(currHeading)) - xCurved * Math.sin(Math.toRadians((currHeading)));


        } else {
            xOut = xCurved;
            yOut = yCurved;
            rotOut = gamepad1.right_stick_button
                    ? TeleOpConfig.AIM_TURN_SCALE * rightX
                    : curve(rightX);
        }

        // update motors
        double denominator = Math.max(Math.abs(yOut) + Math.abs(xOut) + Math.abs(rotOut), 1.0);
        //TODO: debloat this code (currently, rotOut controls strafing and xOut controls rotation
        leftFront.setPower((yOut + xOut + rotOut) / denominator);
        leftBack.setPower((yOut + xOut - rotOut) / denominator);
        rightFront.setPower((yOut - xOut - rotOut) / denominator);
        rightBack.setPower((yOut - xOut + rotOut) / denominator);

        telemetry.update();
    }

    private double curve(double input) {
        return TeleOpConfig.DRIVE_CURVE.apply(input, TeleOpConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }
}