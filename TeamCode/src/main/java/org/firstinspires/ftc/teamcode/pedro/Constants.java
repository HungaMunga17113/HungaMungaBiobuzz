package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.mcl.MCL;
import org.firstinspires.ftc.teamcode.pedro.mcl.MCLLocalizer;

import java.util.ArrayList;
import java.util.List;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.0296842620128723);
        c.yPodOffset.set(-2.064802214855284);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21595649175270434);
                Controller secondaryTranslationalForward = Controller.proportional(0.07979015323888848);
                Controller primaryTranslationalLateral = Controller.proportional(0.2658635907806463);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0982294927874668);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009806446619888203));
                c.brake.set(Controller.proportionalFeedforward(0.008335479626904972));

                c.headingFeedback.set(Controller.proportional(15.717166819781378));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08115208989104518, 0.04808220951938918));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04015523356879374, 0.03350668926431001));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002620884670530773, 0.002401336114094812));

                c.maxAchievableForwardVelocity.set(88.77073706696689);
                c.maxAchievableStrafeVelocity.set(74.29377994066674);
                c.naturalForwardDeceleration.set(28.211335040568333);
                c.naturalStrafeDeceleration.set(54.920711867267);
            }
    );

    // MCL tuning
    // TODO: real hw names/offsets (in, rad) + field obstacles
    public static int mclParticles = 200;
    public static double mclSigma = 1.0;
    public static double mclMaxReading = 78.75;
    public static double mclMaxUsableReading = 54;
    public static double mclMaxRange = 4000; // mm
    /** field perimeter polyline in pedro coords (0..144 in) */
    public static double[][] fieldMap = {{0, 0}, {144, 0}, {144, 144}, {0, 144}, {0, 0}};

    public static MCL createMCL(HardwareMap h) {
        List<MCL.RangeSensor> sensors = new ArrayList<>();

        // addSensor(name, fw offset, lat offset (left +), turn relative to robot heading)
        addSensor(h, sensors, "distFront", 7, 0, 0);
        addSensor(h, sensors, "distLeft", 0, 7, Math.PI / 2);
        addSensor(h, sensors, "distRight", 0, -7, -Math.PI / 2);
        return new MCL(sensors, fieldMap, mclParticles, mclSigma, mclMaxReading, mclMaxUsableReading);
    }

    private static void addSensor(HardwareMap h, List<MCL.RangeSensor> sensors, String name,
                                  double forward, double lateral, double turn) {
        AnalogInput input = h.tryGet(AnalogInput.class, name);
        if (input != null) {
            sensors.add(new MCL.RangeSensor(input, forward, lateral, turn, mclMaxRange));
        }
    }

    public static Follower create(HardwareMap h) {
        return new Follower(
                new MCLLocalizer(new PinpointLocalizer(h, localizerConfig), createMCL(h)),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}