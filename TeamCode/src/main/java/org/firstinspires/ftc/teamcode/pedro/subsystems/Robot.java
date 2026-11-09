package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Conversions;

/** orchestrator over shooter/intake/transfer */
public class Robot {
    public static double shootRpm = 5000;
    public static double feedMs = 800;

    public final Shooter shooter;
    public final Intake intake;
    public final Transfer transfer;

    public Robot(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap, "flywheel", DcMotorSimple.Direction.FORWARD, Conversions.TPR_6000_RPM);
        intake = new Intake(hardwareMap, "intake", DcMotorSimple.Direction.FORWARD);
        transfer = new Transfer(hardwareMap, "transfer");
    }

    public Command spinUp() { return spinUp(shootRpm); }
    public Command spinUp(double rpm) { return instant(() -> shooter.setTargetRpm(rpm)); }

    public Command waitForRpm() { return waitForRpm(shootRpm); }
    public Command waitForRpm(double rpm) { return waitUntil(() -> shooter.waitForRpm(rpm)); }

    public Command hold() { return instant(transfer::hold); }

    public Command stopAll() {
        return instant(() -> {
            intake.stop();
            shooter.stop();
            transfer.hold();
        });
    }

    /** intake w/ transfer closed */
    public Command pickAndHold() {
        return instant(() -> {
            transfer.hold();
            intake.in();
        });
    }

    /** intake w/ transfer closed for ms, then stop intake */
    public Command pickAndHold(double ms) {
        return sequential(pickAndHold(), waitMs(ms), instant(intake::stop));
    }

    /** wait for rpm, feed, close */
    public Command shoot() { return shoot(shootRpm); }
    public Command shoot(double rpm) {
        return sequential(
                waitForRpm(rpm),
                instant(() -> {
                    intake.in();
                    transfer.shoot();
                }),
                waitMs(feedMs),
                instant(() -> {
                    transfer.hold();
                    intake.stop();
                })
        );
    }

    /** spin up, pick for ms, shoot */
    public Command pickAndShoot(double ms) {
        return sequential(spinUp(), pickAndHold(ms), shoot());
    }
}
