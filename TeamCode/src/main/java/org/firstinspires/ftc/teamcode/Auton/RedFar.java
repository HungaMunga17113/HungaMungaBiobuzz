package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.hardware.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.pedro.FollowCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Conversions;

@Autonomous(name = "RedFar", group = "Autonomous")
public class RedFar extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(10.7415, 94.8098, 0);
    private final Pose path1Control1 = poseFactory.of(55.1401, 21.1495, 0);
    private final Pose path1Control2 = poseFactory.of(18.4362, 29.0608, 0);
    private final Pose path1Control3 = poseFactory.of(36.26, 100.477, 0);

    private Shooter shooter = new Shooter(hardwareMap, "flywheel", DcMotorSimple.Direction.FORWARD, Conversions.TPR_6000_RPM);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())//,
                //shooter.setTargetRpm(5000)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1Control3, path1).linear(start, path1);
    }
}
