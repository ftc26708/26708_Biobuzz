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

        // Simple manual turret control via DPAD
        if (gamepad1.dpad_left) {
            robot.turret.turnLeft();
        } else if (gamepad1.dpad_right) {
            robot.turret.turnRight();
        } else {
            robot.turret.stop();
        }

        // Cache vision distance for automatic flywheel RPM & hood adjustment
        boolean hasTarget = robot.vision.hasTarget();
        double distance = hasTarget ? robot.vision.getTargetDistance() : 0.0;

        if (hasTarget) {
            robot.shooter.setDistanceTarget(distance);

            if (robot.shooter.isAtSpeed() && gamepad1.right_trigger > 0.3) {
                robot.shooter.feed();
            } else {
                robot.shooter.resetFeeder();
            }
        } else {
            robot.shooter.stop();
        }

        // Telemetry
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Target Distance (in)", distance);
        telemetry.addData("Turret Heading (deg)", robot.turret.getHeading());
        telemetry.addData("Shooter At Speed", robot.shooter.isAtSpeed());
        telemetry.addData("Hood Position", robot.shooter.getHoodPosition());
        telemetry.update();
    }

    @Override
    protected void onStop() {
        robot.vision.close();
    }
}
