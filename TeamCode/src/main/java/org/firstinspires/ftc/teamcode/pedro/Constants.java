package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_back");

                c.frontLeftDirection.set(DcMotorEx.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorEx.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorEx.Direction.FORWARD);
                c.backRightDirection.set(DcMotorEx.Direction.FORWARD);
            }
    );
}