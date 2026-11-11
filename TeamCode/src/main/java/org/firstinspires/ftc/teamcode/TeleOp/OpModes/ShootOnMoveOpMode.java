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
import org.firstinspires.ftc.teamcode.pedro.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.control.PIDControllerEx;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.drivetrain.SlewRateLimiter2D;

import java.util.List;

/**
 * shoot while moving

 * controls:
 * l_btn = aim + spin up
 * r_trigger = fire (gated)
 * l_trigger = intake
 * dpad down = outtake
 * dpad left = reset pose
 */
@Configurable
@TeleOp(name = "shoot on move", group = "misc")
public class ShootOnMoveOpMode extends OpMode {
    /// field (pedro coords, in / deg)
    public static double startX = 56, startY = 8, startH = 90;
    public static double goalX = 132, goalY = 136;

    // TODO: fill out lookup table
    public static double[] LUT_DIST = {24,    48,    72,    96,    120}; // in
    public static double[] LUT_RPM =  {3800,  4400,  5000,  5500,  5900};
    public static double[] LUT_TOF =  {0.35,  0.45,  0.55,  0.65,  0.75}; // time-of-flight sec
    public static double rpmSign = -1;

    /// sotm
    public static double latency = 0.08; // s, loop + feed delay compensation
    public static int solveIters = 3;
    public static double velAlpha = 0.3; // low-pass
    public static double shooterOffset = 0; // deg, eg 180 if outtake opp side from intake

    /// aim pid (deg in, power out)
    public static double hP = 0.025, hI = 0, hD = 0.0015;
    public static double hFF = 0.004; // power per deg/s
    public static double hMaxPower = 0.8;

    /// fire gates
    public static double rpmTol = 120;
    public static double headingTol = 2.5; // deg
    public static double maxShootSpeed = 60; // in/s

    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private GoBildaPinpointDriver pinpoint;
    private Robot robot;
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final SlewRateLimiter2D driveLimiter = new SlewRateLimiter2D(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
    private final PIDControllerEx headingPid = new PIDControllerEx(hP, hI, hD);

    private double x, y, h;
    private double vx, vy;
    private double prevX, prevY;
    private boolean hasPrev = false;

    private double aimDist, aimRpm, aimHeading, aimOmega, headingErr;
    private boolean ready;

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
        robot = new Robot(hardwareMap);

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
        resetPose();

        headingPid.setContinuousInput(-180, 180).setDerivativeFilter(0.3);

        panels.addData("status: ", "initialized");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        loopTimer.reset();
        headingPid.reset();
        robot.transfer.hold();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) { module.clearBulkCache(); }

        double dt = Math.min(Math.max(loopTimer.seconds(), 0.0), 0.12);
        loopTimer.reset();

        if (gamepad1.dpad_left) resetPose();
        updatePose(dt);

        boolean aiming = gamepad1.left_bumper;
        if (aiming) solve(); else ready = false;

        drive(dt, aiming);
        shooter(aiming);
        intakeAndFire();

        panels.addData("pose", String.format("%.1f, %.1f, %.1f", x, y, h));
        panels.addData("vel", String.format("%.1f, %.1f", vx, vy));
        panels.addData("aim dist", aimDist);
        panels.addData("aim rpm", aimRpm);
        panels.addData("aim heading", aimHeading);
        panels.addData("heading err", headingErr);
        panels.addData("rpm", robot.shooter.getCurrentRpm());
        panels.addData("ready", ready);
        panels.addData("loop ms", dt * 1000);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.shooter.stop();
        robot.intake.stop();
    }

    private void resetPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startH));
        hasPrev = false;
        vx = 0;
        vy = 0;
    }

    private void updatePose(double dt) {
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        h = pinpoint.getHeading(AngleUnit.DEGREES);

        if (hasPrev && dt > 0) {
            vx += velAlpha * ((x - prevX) / dt - vx);
            vy += velAlpha * ((y - prevY) / dt - vy);
        }
        prevX = x;
        prevY = y;
        hasPrev = true;
    }

    private void solve() {
        double[] a = aim(x, y, vx, vy);
        aimDist = a[0];
        aimRpm = a[1];
        aimHeading = a[2];
        aimOmega = a[3];

        headingErr = wrap(aimHeading - h);
        ready = Math.abs(Math.abs(robot.shooter.getCurrentRpm()) - aimRpm) < rpmTol
                && Math.abs(headingErr) < headingTol
                && Math.hypot(vx, vy) < maxShootSpeed;
    }

    private void drive(double dt, boolean aiming) {
        double leftX = deadband(gamepad1.left_stick_x);
        double leftY = -deadband(gamepad1.left_stick_y);
        double rightX = deadband(gamepad1.right_stick_x) * 0.9;

        double dx = curve(leftX);
        double dy = curve(leftY);

        driveLimiter.setRates(TeleOpConfig.ACCEL_UP, TeleOpConfig.ACCEL_DOWN);
        driveLimiter.calculate(dx, dy, dt);
        dx = driveLimiter.getX();
        dy = driveLimiter.getY();

        // +rot = clockwise, heading is ccw
        double rot;
        if (aiming) {
            headingPid.setPID(hP, hI, hD).setOutputRange(-hMaxPower, hMaxPower);
            double ccw = headingPid.calculate(h, aimHeading, dt) + hFF * aimOmega;
            rot = -Conversions.clamp(ccw, -hMaxPower, hMaxPower);
            rotLimiter.calculate(rot, dt);
        } else {
            headingPid.reset();
            rotLimiter.setRates(TeleOpConfig.TURN_ACCEL_UP, TeleOpConfig.TURN_ACCEL_DOWN);
            rot = 0.8 * rotLimiter.calculate(curve(rightX), dt);
        }

        if (TeleOpConfig.useFieldCentricDrive) {
            double rel = Math.toRadians(h - startH);
            double cos = Math.cos(rel);
            double sin = Math.sin(rel);
            double fieldX = dx * cos + dy * sin;
            double fieldY = dy * cos - dx * sin;
            dx = fieldX;
            dy = fieldY;
        }

        double denominator = Math.max(Math.abs(dy) + Math.abs(dx) + Math.abs(rot), 1.0);
        leftFront.setPower((dy + dx + rot) / denominator);
        leftBack.setPower((dy - dx + rot) / denominator);
        rightFront.setPower((dy - dx - rot) / denominator);
        rightBack.setPower((dy + dx - rot) / denominator);
    }

    private void shooter(boolean aiming) {
        if (aiming) robot.shooter.setTargetRpm(rpmSign * aimRpm);
        else if (gamepad1.right_bumper) robot.shooter.setTargetRpm(TeleOpConfig.rpm_1);
        else robot.shooter.stop();
    }

    private void intakeAndFire() {
        boolean fire = gamepad1.right_trigger > 0.15 && ready;
        if (fire) {
            robot.transfer.shoot();
            robot.intake.in();
        } else {
            robot.transfer.hold();
            if (gamepad1.left_trigger > 0.2) robot.intake.in();
            else if (gamepad1.dpad_down) robot.intake.out();
            else robot.intake.stop();
        }
    }

    /**
     * virtual goal; shift goal by -v*tof, iterate since tof depends on dist
     * returns {dist, rpm, heading deg, d(bearing)/dt deg/s}
     */
    static double[] aim(double x, double y, double vx, double vy) {
        double px = x + vx * latency;
        double py = y + vy * latency;

        double gx = goalX, gy = goalY;
        for (int i = 0; i < solveIters; i++) {
            double tof = lerp(LUT_TOF, Math.hypot(gx - px, gy - py));
            gx = goalX - vx * tof;
            gy = goalY - vy * tof;
        }

        double dx = gx - px, dy = gy - py;
        double dist = Math.hypot(dx, dy);
        double heading = wrap(Math.toDegrees(Math.atan2(dy, dx)) + shooterOffset);
        double omega = dist < 1e-3 ? 0 : Math.toDegrees((dy * vx - dx * vy) / (dist * dist));
        return new double[]{dist, lerp(LUT_RPM, dist), heading, omega};
    }

    /** piecewise linear over LUT_DIST (clamped) */
    static double lerp(double[] ys, double d) {
        double[] xs = LUT_DIST;
        int n = Math.min(xs.length, ys.length);
        if (d <= xs[0]) return ys[0];
        if (d >= xs[n - 1]) return ys[n - 1];
        for (int i = 1; i < n; i++) {
            if (d <= xs[i]) {
                double t = (d - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys[n - 1];
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
