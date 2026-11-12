package org.firstinspires.ftc.teamcode.cmd;

import static com.pedropathing.ivy.commands.Commands.instant;

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

    private TurretCommands() {
    }

    /**
     * Rotates the turret to a target angle in degrees using proportional control.
     *
     * @param turret         turret subsystem instance
     * @param targetAngleDeg target heading in degrees
     * @return Ivy command
     */
    public static Command turnToAngle(Turret turret, double targetAngleDeg) {
        return Command.build()
                .setExecute(() -> {
                    double error = Range.clip(targetAngleDeg, -180, 180) - turret.getHeading();
                    double power = Range.clip(TRACKING_KP * error, -1, 1);
                    turret.setPower(power);
                })
                .setDone(() -> Math.abs(Range.clip(targetAngleDeg, -180, 180) - turret.getHeading()) <= ANGLE_TOLERANCE_DEG)
                .setEnd(interrupted -> turret.setPower(0))
                .requiring(turret);
    }

    /**
     * Sets the turret power once
     * @param turret turret subsystem instance
     * @param power power value
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
