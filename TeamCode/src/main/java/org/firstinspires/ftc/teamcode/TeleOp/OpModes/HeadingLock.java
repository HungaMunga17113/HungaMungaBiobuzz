package org.firstinspires.ftc.teamcode.TeleOp.OpModes;

import static org.firstinspires.ftc.teamcode.util.Panels.panels;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.control.PIDControllerEx;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter2D;

import java.util.List;

/**
 * target-centric drive, orbits a locked point

 * controls:
 * a = toggle lock
 * locked: l_y = toward/away, l_x = orbit (holds radius), r_x = manual turn override
 * unlocked: field centric
 * l_stick_btn = slow
 * dpad left = reset pose
 */
@Configurable
@TeleOp(name = "loclock test", group = "Main")
public class HeadingLock extends OpMode {
    /// field (pedro coords, in / deg)
    public static double startX = 56, startY = 8, startH = 90;
    public static double targetX = 132, targetY = 136;
    public static double faceOffset = 0; // deg, eg 180 to face away

    /// lock
    public static double minDist = 12; // in, no aim/approach inside this
    public static double radiusP = 0.04; // power per in of radius err
    public static double radiusMaxPower = 0.4;

    /// heading pid (deg in, power out)
    public static double hP = 0.025, hI = 0, hD = 0.0015;
    public static double hFF = 0.004; // power per deg/s
    public static double hMaxPower = 0.8;

    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final SlewRateLimiter2D driveLimiter = new SlewRateLimiter2D(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
    private final PIDControllerEx headingPid = new PIDControllerEx(hP, hI, hD);

    private double x, y, h, vx, vy;
    private double dist, bearing, headingErr, lockRadius;
    private boolean isLocked, prevA;

    @Override
    public void init() {
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
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        pinpoint.resetPosAndIMU();
        headingPid.setContinuousInput(-180, 180).setDerivativeFilter(0.3);

        panels.addData("status: ", "initialized");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        resetPose();
        loopTimer.reset();
        headingPid.reset();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        double dt = Math.min(Math.max(loopTimer.seconds(), 0.0), 0.12);
        loopTimer.reset();

        if (gamepad1.dpad_left) resetPose();
        updatePose();

        if (gamepad1.a && !prevA) toggleLock();
        prevA = gamepad1.a;

        drive(dt);

        panels.addData("locked", isLocked);
        panels.addData("pose", String.format("%.1f, %.1f, %.1f", x, y, h));
        panels.addData("target", String.format("%.1f, %.1f", targetX, targetY));
        panels.addData("dist", dist);
        panels.addData("lock radius", lockRadius);
        panels.addData("bearing", bearing);
        panels.addData("heading err", headingErr);
        panels.addData("loop ms", dt * 1000);
        panels.update(telemetry);
    }

    private void resetPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startH));
    }

    private void updatePose() {
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        h = pinpoint.getHeading(AngleUnit.DEGREES);
        vx = pinpoint.getVelX(DistanceUnit.INCH);
        vy = pinpoint.getVelY(DistanceUnit.INCH);

        double dx = targetX - x, dy = targetY - y;
        dist = Math.hypot(dx, dy);
        bearing = wrap(Math.toDegrees(Math.atan2(dy, dx)) + faceOffset);
        headingErr = wrap(bearing - h);
    }

    private void toggleLock() {
        isLocked = !isLocked;
        lockRadius = dist;
        headingPid.reset();
    }

    private void drive(double dt) {
        // stick conventions: leftY + = forward, leftX + = strafe right, rightX + = turn right
        double leftX = deadband(gamepad1.left_stick_x);
        double leftY = -deadband(gamepad1.left_stick_y);
        double rightX = deadband(gamepad1.right_stick_x) * 0.9;

        boolean slow = gamepad1.left_stick_button;
        double sx = slow ? TeleOpConfig.AIM_TURN_SCALE * leftX : curve(leftX);
        double sy = slow ? TeleOpConfig.AIM_TURN_SCALE * leftY : curve(leftY);

        driveLimiter.setRates(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
        driveLimiter.calculate(sx, sy, dt);
        sx = driveLimiter.getX();
        sy = driveLimiter.getY();

        rotLimiter.setRates(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
        double manualRot = 0.8 * rotLimiter.calculate(gamepad1.right_stick_button
                ? TeleOpConfig.AIM_TURN_SCALE * rightX
                : curve(rightX), dt);

        double fwd, right, rot;
        if (isLocked && dist > minDist) {
            // radial (toward target) + tangential (robot's right when facing target), field frame
            double ux = (targetX - x) / dist, uy = (targetY - y) / dist;
            double tx = uy, ty = -ux;

            double radial = sy;
            if (Math.abs(sy) > 1e-3) lockRadius = dist;
            else radial = Conversions.clamp(radiusP * (dist - lockRadius), -radiusMaxPower, radiusMaxPower);

            double fx = radial * ux + sx * tx;
            double fy = radial * uy + sx * ty;

            // field -> robot frame
            double hr = Math.toRadians(h);
            fwd = fx * Math.cos(hr) + fy * Math.sin(hr);
            right = fx * Math.sin(hr) - fy * Math.cos(hr);

            if (Math.abs(rightX) > 0) {
                headingPid.reset();
                rot = manualRot;
            } else {
                // bearing rate ff, +rot = clockwise, heading is ccw
                double omega = Math.toDegrees(((targetY - y) * vx - (targetX - x) * vy) / (dist * dist));
                headingPid.setPID(hP, hI, hD).setOutputRange(-hMaxPower, hMaxPower);

                double ccw = headingPid.calculate(h, bearing, dt) + hFF * omega;
                rot = -Conversions.clamp(ccw, -hMaxPower, hMaxPower);
            }
        } else {
            fwd = sy;
            right = sx;
            rot = manualRot;
            if (TeleOpConfig.useFieldCentricDrive) {
                double rel = Math.toRadians(h - startH);
                double cos = Math.cos(rel), sin = Math.sin(rel);
                right = sx * cos + sy * sin;
                fwd = sy * cos - sx * sin;
            }
        }

        // +fwd forward, +right right, +rot clockwise
        double denominator = Math.max(Math.abs(fwd) + Math.abs(right) + Math.abs(rot), 1.0);
        double fl = (fwd + right + rot) / denominator;
        double bl = (fwd - right + rot) / denominator;
        double fr = (fwd - right - rot) / denominator;
        double br = (fwd + right - rot) / denominator;

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }

    static double wrap(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        else if (deg < -180) deg += 360;
        return deg;
    }

    private double curve(double input) {
        return TeleOpConfig.DRIVE_CURVE.apply(input, TeleOpConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }
}
