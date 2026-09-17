package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "Drivetrain Localization Test", group = "Tests")
public class DtLocalizationTest extends BaseOpMode {
    @Override
    protected void onLoop() {
        telemetry.addData("Pose", robot.drivetrain.follower.pose());
        telemetry.update();
    }
}