package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    @Tuner
    public static MecanumTuner mecanumTuner() {
        return new MecanumTuner();
    }
}