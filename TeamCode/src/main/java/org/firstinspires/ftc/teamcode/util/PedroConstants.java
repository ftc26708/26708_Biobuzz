package org.firstinspires.ftc.teamcode.util;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LF");
        c.frontRightName.set("RF");
        c.backLeftName.set("LB");
        c.backRightName.set("RB");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("PC");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.6889005946362112);
        c.yPodOffset.set(-1.5760753661628784);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1793040828556015);
                Controller secondaryTranslationalForward = Controller.proportional(0.06624806752181203);
                Controller primaryTranslationalLateral = Controller.proportional(0.2657878691430487);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09820151566569392);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013187208277953935));
                c.brake.set(Controller.proportionalFeedforward(0.011209127036260844));

                c.headingFeedback.set(Controller.proportional(3.2984915313445));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03827948126802799, 0.004854572302892118));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06014073408656407, 0.06330623822505733));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012391507852316326, 0.0011288871703024638));

                c.maxAchievableForwardVelocity.set(82.795173497452);
                c.maxAchievableStrafeVelocity.set(68.77494697130595);
                c.naturalForwardDeceleration.set(84.67012838385739);
                c.naturalStrafeDeceleration.set(94.13564861864386);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}