package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Flower Calibration")
public class FlowerCalibration extends BaseOpMode {
    private Robot robot;

    // Match initial boolean states to onInit() hardware positions
    private boolean intakesDeployed = true;
    private boolean flowerUp = false;

    private boolean prevIntakeToggle = false;
    private boolean prevFlowerToggle = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    private double upPos = 0.4;
    private double downPos = 0.6;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        robot.intake.setDeploymentFront(intakesDeployed);
        robot.intake.setDeploymentBack(intakesDeployed);
        robot.flower.setPositions(upPos, downPos);
        robot.flower.setDeployment(flowerUp);
    }

    @Override
    public void onLoop() {
        // Drivetrain
        robot.drivetrain.follower.manual(
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        robot.drivetrain.follower.pose().heading()
                )
        );

        // Toggle both intake deployers
        if (gamepad1.right_stick_button && !prevIntakeToggle) {
            intakesDeployed = !intakesDeployed;
            robot.intake.setDeploymentFront(intakesDeployed);
            robot.intake.setDeploymentBack(intakesDeployed);
        }
        prevIntakeToggle = gamepad1.right_stick_button;

        // Intake motor: right bumper reverses direction
        double intakePower = gamepad1.right_trigger;
        if (gamepad1.right_bumper) {
            intakePower *= -1;
        }
        robot.intake.intake(intakePower);

        // Toggle flower up/down
        if (gamepad1.left_stick_button && !prevFlowerToggle) {
            flowerUp = !flowerUp;
            robot.flower.setDeployment(flowerUp);
        }
        prevFlowerToggle = gamepad1.left_stick_button;

        // Adjust currently selected flower position
        double adjustment = 0;
        if (gamepad1.dpad_up && !prevUp) {
            adjustment += 0.005;
        }
        if (gamepad1.dpad_down && !prevDown) {
            adjustment -= 0.005;
        }

        if (adjustment != 0) {
            if (flowerUp) {
                upPos = clip(upPos + adjustment);
            } else {
                downPos = clip(downPos + adjustment);
            }
            robot.flower.setPositions(upPos, downPos);
            robot.flower.setDeployment(flowerUp);
        }
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // Telemetry
        telemetry.addLine("DRIVE: LS = Translation | RS X = Rotation");
        telemetry.addLine("INTAKE: RS Button = Toggle Deployers");
        telemetry.addLine("INTAKE: RT = Forward | RB + RT = Reverse");
        telemetry.addLine("FLOWER: LS Button = Toggle Up/Down");
        telemetry.addLine("FLOWER: D-Pad Up/Down = Adjust Selected Position");
        telemetry.addData("Intakes", intakesDeployed ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Flower State", flowerUp ? "UP" : "DOWN");
        telemetry.addData("Flower Up Pos", "%.3f", upPos);
        telemetry.addData("Flower Down Pos", "%.3f", downPos);
        telemetry.update();
    }

    private double clip(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}