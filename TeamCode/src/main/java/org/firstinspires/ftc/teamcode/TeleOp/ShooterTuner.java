package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kD;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kF;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kI;
import static org.firstinspires.ftc.teamcode.Config.TeleOpConfig.kP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.TeleOpConfig;

import java.util.List;


@TeleOp(name = "flywheel tuner", group = "Main")
public class ShooterTuner extends OpMode {
    private DcMotorEx shooter;
    private Telemetry dash;
    private double lastP, lastI, lastD, lastF, targetVelocity;

    private List<LynxModule> allHubs;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        dash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        shooter();
        report();
    }

    private void shooter() {
        if (kP != lastP || kI != lastI || kD != lastD || kF != lastF) {
            shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            lastP = kP; lastI = kI; lastD = kD; lastF = kF;
        }

        targetVelocity = (gamepad1.right_trigger > 0.3) ? TeleOpConfig.FAST : (gamepad1.a ? TeleOpConfig.SLOW : 0);
        shooter.setVelocity(targetVelocity);
    }

    private void report() {
        double current = shooter.getVelocity();
        dash.addData("targ ", targetVelocity);
        dash.addData("curr ", current);
        dash.addData("err ", targetVelocity - current);
        dash.addData("", "");
        dash.addLine("kP: " + kP + " kI: " + kI + " kD: " + kD + " kF: " + kF);
        dash.update();
    }
}
