package org.firstinspires.ftc.teamcode.opmode.test.autotune;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.util.PedroConstants;

public class PedroTuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, PedroConstants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig));
    }
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, PedroConstants.localizerConfig)), () -> new Foresight(PedroConstants.foresightConfig));
    }
}