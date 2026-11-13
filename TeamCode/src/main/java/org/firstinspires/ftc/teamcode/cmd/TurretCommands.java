package org.firstinspires.ftc.teamcode.cmd;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsys.Turret;

import java.util.function.DoubleSupplier;


/**
 * Command factory class creating reusable PedroPathing Ivy commands for {@link Turret}.
 */
public final class TurretCommands {

    private static final double TRACKING_KP = 0.02;

    private TurretCommands() {
    }

    /**
     * Rotates the turret to a target angle in degrees using proportional control.
     *
     * @param turret         turret subsystem instance
     * @param targetAngleDeg target heading in degrees
     * @return Ivy command
     */
    public static Command turnToStaticAngleClipped(Turret turret, double targetAngleDeg) {
        return infinite(() -> {
            double clippedAngle = Range.clip(targetAngleDeg, -180, 180);
            double error = clippedAngle - turret.getHeading();
            double power = Range.clip(TRACKING_KP * error, -1, 1);
            turret.setPower(power);
        })
                .setEnd(interrupted -> turret.setPower(0))
                .requiring(turret);
    }

    /**
     *
     * @param turret              turret subsystem instance
     * @param targetAngleSupplier continuously changing target heading supplier
     * @return Ivy command
     */
    public static Command turnToDynamicAngleClipped(Turret turret, DoubleSupplier targetAngleSupplier) {
        return infinite(() -> {
            double clippedAngle = Range.clip(targetAngleSupplier.getAsDouble(), -180, 180);
            double error = clippedAngle - turret.getHeading();
            double power = Range.clip(TRACKING_KP * error, -1, 1);
            turret.setPower(power);
        })
                .setEnd(interrupted -> turret.setPower(0))
                .requiring(turret);
    }

    /**
     * Continuously aims the turret at a fixed field coordinate (targetX, targetY).
     *
     * @param turret  turret subsystem instance
     * @param follower Pedro Pathing Follower instance for live robot pose
     * @param targetX target X coordinate on the field (inches)
     * @param targetY target Y coordinate on the field (inches)
     * @return Ivy command
     */
    public static Command turnToPoint(Turret turret, Follower follower, double targetX, double targetY) {
        return turnToDynamicAngleClipped(turret, () -> {
            Pose pose = follower.pose();
            double dx = targetX - pose.x();
            double dy = targetY - pose.y();
            double fieldAngleRad = Math.atan2(dy, dx);

            double relativeRad = fieldAngleRad - pose.heading();
            return Math.toDegrees(Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad)));
        });
    }

    /**
     * Sets the turret power once
     * @param turret turret subsystem instance
     * @param power  power value
     * @return Ivy command
     */
    public static Command setPower(Turret turret, double power) {
        return instant(() -> turret.setPower(power))
                .requiring(turret);
    }

    /**
     * Continuously auto-aims the turret using Limelight vision data.
     *
     * @param turret turret subsystem instance
     * @param vision vision subsystem instance
     * @return Ivy command
     */
    /*public static Command trackTarget(Turret turret, VisionSubsystem vision) {
        return Command.build()
                .setExecute(() -> {
                    if (vision.hasTarget()) {
                        double bearingError = vision.getTargetBearing();
                        double power = Range.clip(TRACKING_KP * bearingError, -0.8, 0.8);
                        turret.setPower(power);
                    } else {
                        turret.stop();
                    }
                })
                .setDone(() -> false)
                .setEnd(interrupted -> turret.stop())
                .requiring(turret);
    }

*/
}
