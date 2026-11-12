package org.firstinspires.ftc.teamcode.cmd;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsys.Turret;


/**
 * Command factory class creating reusable PedroPathing Ivy commands for {@link Turret}.
 */
public final class TurretCommands {

    private static final double DEFAULT_SPEED = 0.5;
    private static final double ANGLE_TOLERANCE_DEG = 1.5;
    private static final double TRACKING_KP = 0.02;

    private TurretCommands() {}

    /**
     * Creates an instant command that rotates the turret left at default speed.
     *
     * @param turret turret subsystem instance
     * @return Ivy command
     */
    public static Command turnLeft(Turret turret) {
        return turnLeft(turret, DEFAULT_SPEED);
    }

    /**
     * Creates an instant command that rotates the turret left at specified speed.
     *
     * @param turret turret subsystem instance
     * @param speed speed [0.0, 1.0]
     * @return Ivy command
     */
    public static Command turnLeft(Turret turret, double speed) {
        return instant(() -> turret.turnLeft(speed)).requiring(turret);
    }

    /**
     * Creates an instant command that rotates the turret right at default speed.
     *
     * @param turret turret subsystem instance
     * @return Ivy command
     */
    public static Command turnRight(Turret turret) {
        return turnRight(turret, DEFAULT_SPEED);
    }

    /**
     * Creates an instant command that rotates the turret right at specified speed.
     *
     * @param turret turret subsystem instance
     * @param speed speed [0.0, 1.0]
     * @return Ivy command
     */
    public static Command turnRight(Turret turret, double speed) {
        return instant(() -> turret.turnRight(speed)).requiring(turret);
    }

    /**
     * Creates an instant command setting direct power to the turret motor.
     *
     * @param turret turret subsystem instance
     * @param power motor power [-1.0, 1.0]
     * @return Ivy command
     */
    public static Command setPower(Turret turret, double power) {
        return instant(() -> turret.setPower(power)).requiring(turret);
    }

    /**
     * Creates an instant command stopping the turret motor.
     *
     * @param turret turret subsystem instance
     * @return Ivy command
     */
    public static Command stop(Turret turret) {
        return instant(turret::stop).requiring(turret);
    }

    /**
     * Creates an instant command resetting the turret encoder.
     *
     * @param turret turret subsystem instance
     * @return Ivy command
     */
    public static Command resetEncoder(Turret turret) {
        return instant(turret::resetEncoder);
    }

    /**
     * Rotates the turret left for a specific duration in milliseconds, then stops.
     *
     * @param turret turret subsystem instance
     * @param speed speed [0.0, 1.0]
     * @param durationMs duration in milliseconds
     * @return Ivy command
     */
    public static Command turnLeftForMs(Turret turret, double speed, long durationMs) {
        return turnLeft(turret, speed)
                .then(waitMs(durationMs))
                .then(stop(turret));
    }

    /**
     * Rotates the turret right for a specific duration in milliseconds, then stops.
     *
     * @param turret turret subsystem instance
     * @param speed speed [0.0, 1.0]
     * @param durationMs duration in milliseconds
     * @return Ivy command
     */
    public static Command turnRightForMs(Turret turret, double speed, long durationMs) {
        return turnRight(turret, speed)
                .then(waitMs(durationMs))
                .then(stop(turret));
    }

    /**
     * Rotates the turret to a target angle in degrees using proportional control.
     *
     * @param turret turret subsystem instance
     * @param targetAngleDeg target heading in degrees
     * @return Ivy command
     */
    public static Command turnToAngle(Turret turret, double targetAngleDeg) {
        return Command.build()
                .setExecute(() -> {
                    double error = targetAngleDeg - turret.getHeading();
                    double power = Range.clip(TRACKING_KP * error, -0.8, 0.8);
                    turret.setPower(power);
                })
                .setDone(() -> Math.abs(targetAngleDeg - turret.getHeading()) <= ANGLE_TOLERANCE_DEG)
                .setEnd(interrupted -> turret.stop())
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
