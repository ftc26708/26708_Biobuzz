package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "BiobuzzTeleOp", group = "TeleOp")
public class BiobuzzTeleop extends BaseOpMode {

    @Override
    protected void onLoop() {
        // Field-centric driver control
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.drivetrain.follower.pose().heading()
        );
        ManualDrive.driveOrHold(robot.drivetrain.follower, powers);

        // Cache vision data to avoid duplicate queries
        boolean hasTarget = robot.vision.hasTarget();
        double bearing = 0.0;
        double distance = 0.0;

        // Auto-aiming turret and shooter control
        if (gamepad1.right_bumper && hasTarget) {
            bearing = robot.vision.getTargetBearing();
            distance = robot.vision.getTargetDistance();

            robot.turret.trackTarget(bearing);
            robot.shooter.setDistanceTarget(distance);

            if (robot.turret.isAligned() && robot.shooter.isAtSpeed() && gamepad1.right_trigger > 0.5) {
                robot.shooter.feed();
            } else {
                robot.shooter.resetFeeder();
            }
        } else {
            robot.turret.stop();
            robot.shooter.stop();
        }

        // Telemetry
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Turret Aligned", robot.turret.isAligned());
        telemetry.addData("Shooter At Speed", robot.shooter.isAtSpeed());
        telemetry.addData("Bearing (deg)", bearing);
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Turret Heading (deg)", robot.turret.getHeading());
        telemetry.update();
    }

    @Override
    protected void onStop() {
        robot.vision.close();
    }
}
