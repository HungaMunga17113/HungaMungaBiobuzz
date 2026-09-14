package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kD;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kF;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kI;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kP;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.TeleOpConfig;

import java.util.List;

@TeleOp(name = "main tele", group = "Main")
public class TeleOpMain extends OpMode {
    private DcMotorEx leftBack, rightBack, leftFront, rightFront;
    private DcMotorEx intake, transfer;
    private DcMotorEx shooter;

    private int intakeState = 0;
    private boolean prevIn = false;
    private boolean prevOut = false;

    private double lastP, lastI, lastD, lastF;

    private List<LynxModule> allHubs;


    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DcMotorEx[] motors = {leftBack, rightBack, leftFront, rightFront, intake, transfer};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        drive();
        intake();
        shooter();
        transfer();
    }

    private void intake() {
        boolean in = gamepad1.left_bumper;
        boolean out = gamepad1.left_trigger > 0.3;

        if (in && !prevIn)
            intakeState = intakeState == 1 ? 0 : 1;

        if (out && !prevOut)
            intakeState = intakeState == -1 ? 0 : -1;

        prevIn = in;
        prevOut = out;

        intake.setPower(intakeState);
    }

    private void shooter() {
        if (kP != lastP || kI != lastI || kD != lastD || kF != lastF) {
            shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            lastP = kP; lastI = kI; lastD = kD; lastF = kF;
        }

        double targetVelocity = (gamepad1.right_trigger > 0.3) ? TeleOpConfig.FAST : (gamepad1.a ? TeleOpConfig.SLOW : 0);
        shooter.setVelocity(targetVelocity);
    }

    private void transfer() { transfer.setPower(gamepad1.right_bumper ? 1 : -1); }

    private void drive() {
        double y = -deadband(gamepad1.left_stick_y);
        double x = -deadband(-gamepad1.left_stick_x) * 1.1;
        double rx = -deadband(-gamepad1.right_stick_x);

        boolean aim = gamepad1.left_stick_button;

        y = aim ? TeleOpConfig.AIM_TURN_SCALE * y : curve(y);
        x = aim ? TeleOpConfig.AIM_TURN_SCALE * x : curve(x);
        rx = gamepad1.right_stick_button
                ? TeleOpConfig.AIM_TURN_SCALE * rx
                : curve(rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);
    }

    private double curve(double input) {
        return TeleOpConfig.DRIVE_CURVE.apply(input, TeleOpConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        if (Math.abs(input) < TeleOpConfig.STICK_DB) return 0;
        return (input - Math.signum(input) * TeleOpConfig.STICK_DB) / (1 - TeleOpConfig.STICK_DB);
    }
}