package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "Drivetrain Driving Test", group = "Tests")
public class DtDriveTest extends BaseOpMode {
    @Override
    protected void onLoop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robot.drivetrain.follower.pose().heading()
        );
        robot.drivetrain.follower.manual(powers);

        if (gamepad1.left_bumper) {
            robot.drivetrain.follower.setPose(new Pose(70.75, 70.75, 0));
        }

        telemetry.addData("Pose", robot.drivetrain.follower.pose());
        telemetry.update();
    }
}