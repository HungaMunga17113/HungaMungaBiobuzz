package org.firstinspires.ftc.teamcode.TeleOp.opModes;

import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.bkD;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.bkF;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.bkI;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.bkP;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.ckD;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.ckF;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.ckI;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.ckP;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkA;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkD;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkI;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkP;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkS;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig.vkV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOpConfig;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.control.PIDFControllerEx;


@Config
@TeleOp(name="shooter TUNER", group="misc")
public class ShooterTuningOpMode extends OpMode {
    public static int targetRPM = 0; // max k

    private DcMotorEx shooter;
    private final double tpr = Conversions.TPR_6000_RPM;

    private double targetVel = 0;
    private double currentVel = 0;
    private double currentRPM = 0;
    private double error = 0;
    private double output = 0;

    private double p,i,d,f;
    private double s,v,a;
    private final PIDFControllerEx pidf = new PIDFControllerEx(0,0,0,0);
    private TeleOpConfig.TuningModes mode = TeleOpConfig.TuningModes.BUILT_IN;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mode = TeleOpConfig.TuningMode;

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        update_vals();

        if (mode == TeleOpConfig.TuningModes.BUILT_IN) {
            shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooter.setVelocityPIDFCoefficients(p, i, d, f);
        } else {
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            push_custom_gains();
            pidf.setDerivativeFilter(0.2);
            pidf.setOutputRange(0, 1);
        }

        targetVel = Conversions.rpmToTps(targetRPM, tpr);

        telemetry.addData("status: ", "initialized");
        telemetry.addData("mode: ", mode);
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clear();
        pidf.reset();
    }

    @Override
    public void loop() {
        update_vals();
        targetVel = Conversions.rpmToTps(targetRPM, tpr);
        currentVel = Math.abs(shooter.getVelocity());
        currentRPM = Conversions.tpsToRpm(currentVel, tpr);
        error = targetRPM - currentRPM;

        if (mode == TeleOpConfig.TuningModes.BUILT_IN) {
            shooter_builtin();
        } else {
            shooter_custom();
        }

        telemetry.addData("mode", mode);
        telemetry.addData("targetVel ticks", targetVel);
        telemetry.addData("currentVel ticks", currentVel);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("currentRPM", currentRPM);
        telemetry.addData("error", error);
        telemetry.addData("output", output);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.setPower(0);
    }

    private void shooter_builtin() {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
        shooter.setVelocity(targetVel);
        output = shooter.getPower();
    }

    private void shooter_custom() {
        push_custom_gains();
        output = targetRPM == 0 ? 0 : pidf.calculate(currentRPM, targetRPM);
        if (targetRPM == 0) pidf.reset();
        shooter.setPower(output);
    }

    private void push_custom_gains() {
        if (mode == TeleOpConfig.TuningModes.SVA) {
            pidf.setPID(p, i, d).setOutputRange(0, 1);
            pidf.setF(0).setSVA(s, v, a);
        } else {
            pidf.setPID(p, i, d).setOutputRange(0, 1);
            pidf.setF(f).setSVA(0, 0, 0);
        }
    }

    private void update_vals() {
        switch (mode) {
            case BUILT_IN:
                p = bkP; i = bkI; d = bkD; f = bkF;
                break;
            case CUSTOM:
                p = ckP; i = ckI; d = ckD; f = ckF;
                break;
            case SVA:
                p = vkP; i = vkI; d = vkD;
                s = vkS; v = vkV; a = vkA;
                break;
        }
    }
}
