package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Transfer Calibration")
public class TransferCalibration extends BaseOpMode {

    private Robot robot;

    // Match initial boolean states to onInit() hardware positions
    private boolean intakesDeployed = true;
    private boolean blockerOpen = false;

    private boolean prevIntakeToggle = false;
    private boolean prevBlockerToggle = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    private double openPos = 0.5;
    private double closedPos = 0.0;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);

        robot.intake.setDeploymentFront(intakesDeployed);
        robot.intake.setDeploymentBack(intakesDeployed);

        robot.transfer.setPositions(openPos, closedPos);
        robot.transfer.setBlocker(blockerOpen);
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

        // Transfer motor: left bumper reverses direction
        double transferPower = gamepad1.left_trigger;
        if (gamepad1.left_bumper) {
            transferPower *= -1;
        }
        robot.transfer.transfer(transferPower);

        // Toggle blocker open/closed
        if (gamepad1.left_stick_button && !prevBlockerToggle) {
            blockerOpen = !blockerOpen;
            robot.transfer.setBlocker(blockerOpen);
        }
        prevBlockerToggle = gamepad1.left_stick_button;

        // Adjust currently selected blocker position
        double adjustment = 0;
        if (gamepad1.dpad_up && !prevUp) {
            adjustment += 0.005;
        }
        if (gamepad1.dpad_down && !prevDown) {
            adjustment -= 0.005;
        }

        if (adjustment != 0) {
            if (blockerOpen) {
                openPos = clip(openPos + adjustment);
            } else {
                closedPos = clip(closedPos + adjustment);
            }
            robot.transfer.setPositions(openPos, closedPos);
            robot.transfer.setBlocker(blockerOpen);
        }
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // Telemetry
        telemetry.addLine("DRIVE: LS = Translation | RS X = Rotation");
        telemetry.addLine("INTAKE: RS Button = Toggle Deployers | RT = Fwd | RB + RT = Rev");
        telemetry.addLine("TRANSFER: LT = Fwd | LB + LT = Rev");
        telemetry.addLine("BLOCKER: LS Button = Toggle Open/Closed | D-Pad Up/Down = Adjust");

        telemetry.addData("Intakes", intakesDeployed ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Blocker State", blockerOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Blocker Open Pos", "%.3f", openPos);
        telemetry.addData("Blocker Closed Pos", "%.3f", closedPos);

        telemetry.update();
    }

    private double clip(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}