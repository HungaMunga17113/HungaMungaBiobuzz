package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter2D;

import java.util.List;

@TeleOp(name = "MAIN TELE (LM1)", group = "Main")
public class Main extends OpMode {
    private DcMotorEx leftBack, rightBack, leftFront, rightFront, intake, shooter;
    private Servo transfer;

    private GoBildaPinpointDriver pinpoint;

    ///TODO: SERVO STUFF
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopDt;
    private final SlewRateLimiter2D driveLimiter = new SlewRateLimiter2D(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);

    private int tVel;

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

        pinpoint.resetPosAndIMU();

        transfer = hardwareMap.get(Servo.class, "transfer");

        intake = hardwareMap.get(DcMotorEx.class,
                "intake");

        shooter = hardwareMap.get(DcMotorEx.class,
                "flywheel");


        DcMotorEx[] motors = {leftBack, rightBack, leftFront, rightFront, intake};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        transfer.setDirection(Servo.Direction.FORWARD);

        // positive power means spin wheel forwards
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

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
        intake();
        shooter();
        transfer();


        telemetry.update();
    }

    private void drive(double dt) {
        // stick conventions: leftY + = forward, leftX + = strafe right, rightX + = turn right
        double leftX = deadband(gamepad1.left_stick_x);
        double leftY = -deadband(gamepad1.left_stick_y);
        double rightX = deadband(gamepad1.right_stick_x) * 0.9;

        // apply curve <=> !slow mode
        boolean aim = gamepad1.left_stick_button;
        double x = aim ? TeleOpConfig.AIM_TURN_SCALE * leftX : curve(leftX);
        double y = aim ? TeleOpConfig.AIM_TURN_SCALE * leftY : curve(leftY);

        driveLimiter.setRates(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
        driveLimiter.calculate(x, y, dt);
        x = driveLimiter.getX();
        y = driveLimiter.getY();

        rotLimiter.setRates(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
        double rot = 0.8 * rotLimiter.calculate(gamepad1.right_stick_button
                ? TeleOpConfig.AIM_TURN_SCALE * rightX
                : curve(rightX), dt);

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        telemetry.addData("heading", heading);
        if (gamepad1.dpad_left) {
            pinpoint.resetPosAndIMU();
        }

        if (TeleOpConfig.useFieldCentricDrive) {
            double cos = Math.cos(Math.toRadians(heading));
            double sin = Math.sin(Math.toRadians(heading));
            double fieldX = x * cos + y * sin;
            double fieldY = y * cos - x * sin;
            x = fieldX;
            y = fieldY;
        }

        // +y forward, +x right, +rot clockwise
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1.0);
        double fl = (y + x + rot) / denominator;
        double bl = (y - x + rot) / denominator;
        double fr = (y - x - rot) / denominator;
        double br = (y + x - rot) / denominator;

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);

        telemetry.addData("FL: ", fl);
        telemetry.addData("BL:", bl);
        telemetry.addData("FR:", fr);
        telemetry.addData("BR:", br);
    }

    private double curve(double input) {
        return TeleOpConfig.DRIVE_CURVE.apply(input, TeleOpConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }

    private void intake() { intake.setPower( (gamepad1.left_trigger>0.2) ? 1 :  (gamepad1.dpad_down) ? -1 : 0); }
    private void shooter() {
        shooter.setVelocityPIDFCoefficients(TeleOpConfig.bkP, TeleOpConfig.bkI, TeleOpConfig.bkD, TeleOpConfig.bkF);
        if (gamepad1.right_trigger > 0.15) { tVel = (int) Conversions.rpmToTps(TeleOpConfig.rpm_1, Conversions.TPR_6000_RPM); }
        else if (gamepad1.right_bumper) { tVel = (int) Conversions.rpmToTps(TeleOpConfig.rpm_2, Conversions.TPR_6000_RPM); }
        shooter.setVelocity(tVel);

        telemetry.addData("speed", Conversions.tpsToRpm(shooter.getVelocity(), Conversions.TPR_6000_RPM));
        telemetry.addData("target speed", TeleOpConfig.rpm_1);
       // telemetry.addData("input speed", tVel);
        telemetry.update();
        //shooter.setPower( (gamepad1.right_trigger>0.2) ? -1 :  0);

    }
    private void transfer() {
//        if (gamepad1.a) {
//            transfer.setPosition(0.65);
//        }
//        else if (gamepad1.b) { transfer.setPosition(1.0); }
        if(gamepad1.right_trigger > 0.15){
            transfer.setPosition(0.65);
        }else{
            transfer.setPosition(1.0);
        }
    }
}