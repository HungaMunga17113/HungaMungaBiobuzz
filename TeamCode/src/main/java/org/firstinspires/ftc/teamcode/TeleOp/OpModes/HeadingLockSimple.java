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
 * atan2 goal lock, shooter (back) faces goal, strafing orbits

 * gp1: left bumper = toggle lock (r_x disabled while locked)
 * gp2: X = blue, B = red, dpad left/right = driver-relative localize
 */
@Configurable
@TeleOp(name = "loclock simple", group = "Main")
public class HeadingLockSimple extends OpMode {
    /// heading pd (deg in, power out)
    public static double hP = 0.025, hD = 0.0015;
    public static double hMaxPower = 0.8;
    public static double minDist = 6; // in, no aim inside this

    private enum Alliance { NONE, BLUE, RED }

    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final SlewRateLimiter2D driveLimiter = new SlewRateLimiter2D(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
    private final PIDControllerEx headingPid = new PIDControllerEx(hP, 0, hD);

    private Alliance alliance = Alliance.NONE;
    private boolean localized, locked;
    private boolean prevLb, prevDpadLeft, prevDpadRight;
    private String locCorner = "none";
    private double fieldForwardH = 0;

    private double x, y, h;
    private double goalX, goalY, dist, targetH;

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
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        localize();
        telem();
    }

    @Override
    public void start() {
        loopTimer.reset();
        headingPid.reset();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        double dt = Math.min(Math.max(loopTimer.seconds(), 0.0), 0.12);
        loopTimer.reset();

        pinpoint.update();
        localize();
        updatePose();

        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLb && localized) {
            locked = !locked;
            headingPid.reset();
        }
        prevLb = lb;

        drive(dt);
        telem();
    }

    private void localize() {
        if (gamepad2.x) selectAlliance(Alliance.BLUE);
        else if (gamepad2.b) selectAlliance(Alliance.RED);

        boolean left = gamepad2.dpad_left, right = gamepad2.dpad_right;
        if (alliance != Alliance.NONE) {
            if (left && !prevDpadLeft) localizeRelative(true);
            else if (right && !prevDpadRight) localizeRelative(false);
        }
        prevDpadLeft = left;
        prevDpadRight = right;
    }

    private void selectAlliance(Alliance a) {
        if (alliance == a) return;
        alliance = a;
        fieldForwardH = a == Alliance.BLUE ? TeleOpConfig.BLUE_FORWARD_H : TeleOpConfig.RED_FORWARD_H;
        localized = false;
        locked = false;
        locCorner = "none";
    }

    /** blue: rel left = global top, red: rel left = global bottom */
    private void localizeRelative(boolean relLeft) {
        boolean blue = alliance == Alliance.BLUE;
        boolean globalTop = blue == relLeft;
        double px = blue ? TeleOpConfig.BLUE_LOC_X : TeleOpConfig.RED_LOC_X;
        double py, ph;
        if (blue) {
            py = globalTop ? TeleOpConfig.BLUE_TOP_Y : TeleOpConfig.BLUE_BOTTOM_Y;
            ph = globalTop ? TeleOpConfig.BLUE_TOP_H : TeleOpConfig.BLUE_BOTTOM_H;
        } else {
            py = globalTop ? TeleOpConfig.RED_TOP_Y : TeleOpConfig.RED_BOTTOM_Y;
            ph = globalTop ? TeleOpConfig.RED_TOP_H : TeleOpConfig.RED_BOTTOM_H;
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, px, py, AngleUnit.DEGREES, ph));
        pinpoint.update();
        localized = true;
        headingPid.reset();
        locCorner = "rel " + (relLeft ? "LEFT" : "RIGHT") + " (global " + (globalTop ? "TOP" : "BOTTOM") + ")";
    }

    private void updatePose() {
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        h = pinpoint.getHeading(AngleUnit.DEGREES);

        boolean top = y > TeleOpConfig.FIELD_SIZE / 2;
        if (alliance == Alliance.BLUE) {
            goalX = top ? TeleOpConfig.BLUE_TOP_GOAL_X : TeleOpConfig.BLUE_BOTTOM_GOAL_X;
            goalY = top ? TeleOpConfig.BLUE_TOP_GOAL_Y : TeleOpConfig.BLUE_BOTTOM_GOAL_Y;
        } else {
            goalX = TeleOpConfig.RED_GOAL_X;
            goalY = top ? TeleOpConfig.RED_TOP_GOAL_Y : TeleOpConfig.RED_BOTTOM_GOAL_Y;
        }

        double dx = goalX - x, dy = goalY - y;
        dist = Math.hypot(dx, dy);
        targetH = wrap(Math.toDegrees(Math.atan2(dy, dx)) + 180); // back faces goal
    }

    private void drive(double dt) {
        // leftY + = forward, leftX + = strafe right, rightX + = turn right
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
        double rot = 0.8 * rotLimiter.calculate(locked ? 0 : gamepad1.right_stick_button
                ? TeleOpConfig.AIM_TURN_SCALE * rightX
                : curve(rightX), dt);

        if (locked) {
            if (dist > minDist) {
                headingPid.setPID(hP, 0, hD).setOutputRange(-hMaxPower, hMaxPower);
                rot = -Conversions.clamp(headingPid.calculate(h, targetH, dt), -hMaxPower, hMaxPower); // pid ccw, +rot cw
            } else rot = 0;
        }

        double fwd = sy, right = sx;
        if (TeleOpConfig.useFieldCentricDrive) {
            double rel = Math.toRadians(h - fieldForwardH);
            double cos = Math.cos(rel), sin = Math.sin(rel);
            right = sx * cos + sy * sin;
            fwd = sy * cos - sx * sin;
        }

        // +fwd forward, +right right, +rot clockwise
        double denominator = Math.max(Math.abs(fwd) + Math.abs(right) + Math.abs(rot), 1.0);
        leftFront.setPower((fwd + right + rot) / denominator);
        leftBack.setPower((fwd - right + rot) / denominator);
        rightFront.setPower((fwd - right - rot) / denominator);
        rightBack.setPower((fwd + right - rot) / denominator);
    }

    private void telem() {
        panels.addData("alliance", alliance == Alliance.NONE ? "NONE - gp2 X = blue, B = red" : alliance);
        panels.addData("localized", localized ? locCorner
                : alliance == Alliance.NONE ? "NO - pick alliance first" : "NO - gp2 dpad left/right");
        panels.addData("locked", locked);
        panels.addData("pose", String.format("%.1f, %.1f, %.1f", x, y, h));
        panels.addData("goal", String.format("%.1f, %.1f", goalX, goalY));
        panels.addData("dist", dist);
        panels.addData("target h", targetH);
        panels.addData("heading err", wrap(targetH - h));
        panels.update(telemetry);
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
