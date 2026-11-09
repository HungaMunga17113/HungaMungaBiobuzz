package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Field {
    double d = 141.5;
    private static PoseFactory pose = PoseFactory.degrees();
    public interface red {
        Pose startFar = pose.of(56, 8, 90);
        Pose startClose = pose.of(11, 131, 0);

    }
    public interface blue {
        Pose startFar = pose.of(83, 134, -90);
        Pose startClose = pose.of(130,12,180);
    }
}
