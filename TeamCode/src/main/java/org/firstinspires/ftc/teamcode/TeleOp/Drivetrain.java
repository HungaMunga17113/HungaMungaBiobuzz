package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter2D;

import java.util.List;

@TeleOp(name = "drivetrain only", group = "Main")
public class Drivetrain extends OpMode {
    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopDt;
    private final SlewRateLimiter2D driveLimiter = new SlewRateLimiter2D(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
    private final SlewRateLimiter rxLimiter = new SlewRateLimiter(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);

    @Override
    public void init() {
        // lynx bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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

        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        telemetry.addData("status: ", "initializing");
    }

    @Override
    public void start() { loopTimer.reset(); }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        // get loop time for slew rate limiters
        double rawDt = loopTimer.seconds();
        loopTimer.reset();
        loopDt = Math.min(Math.max(rawDt, 0.0), 0.12);

        drive(loopDt);
        telemetry.update();
    }

    private void drive(double dt) {
        // get heading
        pinpoint.update();
        double currHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        telemetry.addData("heading", currHeading);

        // apply deadbands
        double leftX = deadband(gamepad1.left_stick_x);
        double leftY = -deadband(gamepad1.left_stick_y);
        double rightX = deadband(gamepad1.right_stick_x);

        // apply curve <=> !slow mode
        boolean aim = gamepad1.left_stick_button;
        double xCurved = aim ? TeleOpConfig.AIM_TURN_SCALE * leftX : curve(leftX);
        double yCurved = aim ? TeleOpConfig.AIM_TURN_SCALE * leftY : curve(leftY);

        driveLimiter.setRates(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
        rxLimiter.setRates(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
        driveLimiter.calculate(xCurved, yCurved, dt);

        double xLim = driveLimiter.getX();
        double yLim = driveLimiter.getY();
        double rotOut = rxLimiter.calculate(gamepad1.right_stick_button
                ? TeleOpConfig.AIM_TURN_SCALE * rightX
                : curve(rightX), dt);

        double xOut;
        double yOut;

        // fcd calculations
        if (TeleOpConfig.useFieldCentricDrive) {
            double h = Math.toRadians(currHeading);
            xOut = xLim * Math.cos(h) + yLim * Math.sin(h);
            yOut = yLim * Math.cos(h) - xLim * Math.sin(h);
        } else {
            xOut = xLim;
            yOut = yLim;
        }

        // update motors
        double denominator = Math.max(Math.abs(yOut) + Math.abs(xOut) + Math.abs(rotOut), 1.0);
        leftFront.setPower((yOut + xOut + rotOut) / denominator);
        leftBack.setPower((yOut - xOut + rotOut) / denominator);
        rightFront.setPower((yOut - xOut - rotOut) / denominator);
        rightBack.setPower((yOut + xOut - rotOut) / denominator);

        // telemetry on rc
        telemetry.addData("FL: ", (yOut + xOut + rotOut) / denominator);
        telemetry.addData("BL:", (yOut - xOut + rotOut) / denominator);
        telemetry.addData("FR:", (yOut - xOut - rotOut) / denominator);
        telemetry.addData("BR:", (yOut + xOut - rotOut) / denominator);
    }

    private double curve(double input) {
        return TeleOpConfig.DRIVE_CURVE.apply(input, TeleOpConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }
}