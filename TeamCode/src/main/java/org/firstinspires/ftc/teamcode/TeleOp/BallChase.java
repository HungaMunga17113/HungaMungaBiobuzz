
package org.firstinspires.ftc.teamcode.opModes.teleop;

import android.util.Size;

import org.firstinspires.ftc.teamcode.CV.BallDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

@TeleOp(name = "Ball Chase (Toggle Mode)", group = "Hive Vision")
public class BallChase extends LinearOpMode {
    private static final String WEBCAM_NAME = "Webcam";
    private static final int CLASS_YELLOW = 0;
    private static final int CLASS_RED = 1;
    private static final int CLASS_BLUE = 2;

    private Set<Integer> allowedClasses =
            new HashSet<>(Arrays.asList(CLASS_RED, CLASS_BLUE));

    public static long LOCK_LOST_MS = 300;

    // If your robot drives backward, change this to a negative value (e.g., -0.40)
    public static double CHASE_FWD_POWER = 0.40;
    public static double TURN_KP = 0.025;
    public static double MIN_TURN = 0.08;
    public static double MAX_TURN = 0.60;
    public static double SEARCH_TURN = 0.30;
    public static double PICKUP_TA = 0.0;
    public static double COAST_MAX_DIST = 26.0;
    public static double COAST_POWER = 0.25;
    public static long COAST_MS = 550;
    public static long TARGET_LOSS_GRACE_MS = 400; // Prevents flickering when target is lost briefly
    public static long PICKUP_DWELL_MS = 600;
    public static double INTAKE_POWER = 1.0;

    private static final String MOTOR_LF = "frontLeft";
    private static final String MOTOR_RF = "frontRight";
    private static final String MOTOR_LB = "backLeft";
    private static final String MOTOR_RB = "backRight";

    private enum State {
        SEARCHING,
        CHASING,
        COASTING,
        PICKUP
    }

    private DcMotor lf, rf, lb, rb;

    private State state = State.SEARCHING;

    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime pickupTimer = new ElapsedTime();
    private final ElapsedTime lockAge = new ElapsedTime();

    private double lastSeenDist = Double.MAX_VALUE;
    private Double lockTx = null;
    private Double lockTy = null;

    private boolean rightBumperPrev = false;
    private boolean autoChaseActive = false;

    BallDetectorPipeline baller = new BallDetectorPipeline();


    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, MOTOR_LF);
        rf = hardwareMap.get(DcMotor.class, MOTOR_RF);
        lb = hardwareMap.get(DcMotor.class, MOTOR_LB);
        rb = hardwareMap.get(DcMotor.class, MOTOR_RB);
        //intake = hardwareMap.tryGet(DcMotor.class, INTAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{lf, rf, lb, rb}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        WebcamName webcamName = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);

        if(webcamName == null){
            telemetry.addLine("Can not connect to webcam.");
            telemetry.update();
            requestOpModeStop();
        }

        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(webcamName)
            .setCameraResolution(new Size(1280, 720))
            .addProcessor(baller)
            .enableLiveView(true)
            .build();

        //selectAllowedColors();

        telemetry.addLine("Click RIGHT BUMPER once to toggle Auto-Chase ON/OFF.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean rightBumperCurrent = gamepad1.right_bumper;
            if (rightBumperCurrent && !rightBumperPrev) {
                autoChaseActive = !autoChaseActive;
            }
            rightBumperPrev = rightBumperCurrent;

            if (autoChaseActive) {
                autoChase();
                telemetry.addData("Mode", "AUTO CHASE ACTIVE");
            } else {
                state = State.SEARCHING;
                clearLock();
                manualDrive();
                telemetry.addData("Mode", "MANUAL DRIVE");
            }

            telemetry.addData("state", state);
            telemetry.update();
        }

        drive(0, 0, 0);
        setIntake(0);
    }

    private void selectAllowedColors() {
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) allowedClasses = setOf(CLASS_RED);
            if (gamepad1.dpad_down) allowedClasses = setOf(CLASS_BLUE);
            if (gamepad1.dpad_left) allowedClasses = setOf(CLASS_RED, CLASS_BLUE);
            if (gamepad1.dpad_right) allowedClasses = setOf(CLASS_RED, CLASS_BLUE, CLASS_YELLOW);

            telemetry.addLine("D-pad: UP red | DOWN blue | LEFT red+blue | RIGHT all");
            telemetry.addData("targets", describeAllowed());
            telemetry.update();
        }
    }

    private String describeAllowed() {
        if (allowedClasses.size() == 1 && allowedClasses.contains(CLASS_RED)) return "RED only";
        if (allowedClasses.size() == 1 && allowedClasses.contains(CLASS_BLUE)) return "BLUE only";
        if (allowedClasses.size() == 2) return "RED + BLUE";
        return "RED + BLUE + YELLOW";
    }

    private static Set<Integer> setOf(Integer... ids) {
        return new HashSet<>(Arrays.asList(ids));
    }

    private void autoChase() {
        if (state == State.PICKUP) {
            drive(0, 0, 0);
            setIntake(INTAKE_POWER);

            if (pickupTimer.milliseconds() >= PICKUP_DWELL_MS) {
                state = State.SEARCHING;
                lastSeenDist = Double.MAX_VALUE;
                clearLock();
            }
            return;
        }

        BallDetectorPipeline.BallBlob yellow = baller.bestOf(BallDetectorPipeline.BallColor.YELLOW);
        if (yellow != null) {
            telemetry.addData("Yellow", "cx=%.2f cy=%.2f", yellow.cxNorm, yellow.cyNorm);

            lastSeenTimer.reset();
            lastSeenDist = ((0.15748*3)/yellow.rect.width);

            double turn = Range.clip(yellow.cxNorm * TURN_KP, -MAX_TURN, MAX_TURN);
            if (Math.abs(turn) < MIN_TURN) {
                turn = Math.copySign(MIN_TURN, yellow.cxNorm);
            }
            double fwd = CHASE_FWD_POWER;

            state = State.CHASING;
            setIntake(INTAKE_POWER);
            drive(fwd, 0, turn);

            telemetry.addData("ball", "tx=%+.1f dist≈%.1f", yellow.cxNorm, lastSeenDist);
            return;
        }

        // Grace period check: if we just lost the ball briefly, keep going straight/intaking instead of flickering
        if (lastSeenTimer.milliseconds() < TARGET_LOSS_GRACE_MS && state == State.CHASING) {
            //setIntake(INTAKE_POWER);
            drive(CHASE_FWD_POWER, 0, 0);
            telemetry.addData("drive", "grace period maintaining chase");
            return;
        }

        boolean wasClose = (state == State.CHASING || state == State.COASTING)
                && lastSeenDist <= COAST_MAX_DIST;

        if (wasClose) {
            if (lastSeenTimer.milliseconds() < COAST_MS) {
                state = State.COASTING;
                setIntake(INTAKE_POWER);
                drive(COAST_POWER, 0, 0);
            } else {
                beginPickup();
            }
            return;
        }

        state = State.SEARCHING;
        setIntake(0);
        drive(0, 0, SEARCH_TURN);
    }

    private void beginPickup() {
        state = State.PICKUP;
        pickupTimer.reset();
        drive(0, 0, 0);
        setIntake(INTAKE_POWER);
    }


    private void dropStaleLock() {
        if (lockTx != null && lockAge.milliseconds() > LOCK_LOST_MS) {
            clearLock();
        }
    }

    private void clearLock() {
        lockTx = null;
        lockTy = null;
    }

    private void manualDrive() {
        double fwd = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        drive(fwd, strafe, turn);
        setIntake(gamepad1.left_bumper ? INTAKE_POWER : 0);
    }

    private void drive(double fwd, double strafe, double turn) {
        double pLf = fwd + strafe + turn;
        double pRf = fwd - strafe - (turn);
        double pLb = fwd - strafe + turn;
        double pRb = fwd + strafe - (turn);

        double largestPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(pLf), Math.abs(pRf)),
                        Math.max(Math.abs(pLb), Math.abs(pRb))
                )
        );

        lf.setPower(pLf / largestPower);
        rf.setPower(pRf / largestPower);
        lb.setPower(pLb / largestPower);
        rb.setPower(pRb / largestPower);
    }

    private void setIntake(double power) {
//        if (intake != null) {
//            intake.setPower(power);
//        }
//        We did not use Intake
    }
}