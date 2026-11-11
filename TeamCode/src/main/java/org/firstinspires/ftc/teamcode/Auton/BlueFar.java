package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.pedro.FollowCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.FollowCommands.turnTo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Robot;

@Autonomous(name = "Blue Far - 1 Tip", group = "Autonomous")
public class BlueFar extends LinearOpMode {

    private Follower follower;
    private Robot robot;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75);

    private final Pose start = poseFactory.of(56, 6.5, 90);
    private final Pose path1 = poseFactory.of(16.4, 10, 180);
    private final Pose path1Control1 = poseFactory.of(27.25, 34.35, 0);
    private final Pose point2 = poseFactory.of(16.523, 6.3, 180);
    private final Pose point3 = poseFactory.of(6.2, 6.3, -180);
    private final Pose point4 = poseFactory.of(58.75, 22.75, -90);
    private final Pose point4Control1 = poseFactory.of(58.75, 10.75, 0);
    private final Pose point5 = poseFactory.of(58.75, 22.75, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                robot.spinUp(),
                robot.shoot(),
                waitMs(2000),
                robot.hold(),
                follow(follower, path1()),
                follow(follower, path2()),
                parallel(
                        robot.pickAndHold(),
                        sequential(
                                follow(follower, path3()),
                                follow(follower, path4()),
                                parallel(
                                        robot.spinUp(),
                                        turnTo(follower, point5)
                                )
                        )
                ),
                waitMs(400),
                robot.shoot(),
                waitMs(2000)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap);
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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }

}
