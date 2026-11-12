package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Intake Calibration")
public class IntakeCalibration extends BaseOpMode {

    private boolean frontDeployed = false;
    private boolean backDeployed = false;

    private boolean selectedFront = true;
    private boolean selectedDeployed = true;

    private boolean prevFront, prevBack;
    private boolean prevX, prevY;
    private boolean prevUp, prevDown;
    private boolean prevA, prevB;
    private boolean prevReset;

    @Override
    public void onInit() {
        robot.intake.setDeploymentFront(false);
        robot.intake.setDeploymentBack(false);
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

        // Reset heading
        if (gamepad1.right_stick_button && !prevReset) {
            Pose pose = robot.drivetrain.follower.pose();

            robot.drivetrain.follower.setPose(
                    new Pose(pose.x(), pose.y(), 0)
            );
        }
        prevReset = gamepad1.right_stick_button;

        // Intake motor (forward only)
        robot.intake.intake(gamepad1.right_trigger);

        // Front deployer toggle
        if (gamepad1.left_bumper && !prevFront) {
            frontDeployed = !frontDeployed;
            robot.intake.setDeploymentFront(frontDeployed);
        }

        // Back deployer toggle
        if (gamepad1.right_bumper && !prevBack) {
            backDeployed = !backDeployed;
            robot.intake.setDeploymentBack(backDeployed);
        }

        prevFront = gamepad1.left_bumper;
        prevBack = gamepad1.right_bumper;

        // Select front/back
        if (gamepad2.x && !prevX) {
            selectedFront = !selectedFront;
        }

        // Select deployed/retracted
        if (gamepad2.y && !prevY) {
            selectedDeployed = !selectedDeployed;
        }

        prevX = gamepad2.x;
        prevY = gamepad2.y;

        // Position adjustment
        double adjustment = 0;

        if (gamepad2.dpad_up && !prevUp) adjustment += 0.005;
        if (gamepad2.dpad_down && !prevDown) adjustment -= 0.005;
        if (gamepad2.a && !prevA) adjustment += 0.025;
        if (gamepad2.b && !prevB) adjustment -= 0.025;

        if (adjustment != 0) {
            adjustPosition(adjustment);
        }

        prevUp = gamepad2.dpad_up;
        prevDown = gamepad2.dpad_down;
        prevA = gamepad2.a;
        prevB = gamepad2.b;

        // Telemetry
        telemetry.addLine("DRIVER: LS = Drive | RS X = Turn | RS Button = Reset Heading");
        telemetry.addLine("DRIVER: RT = Intake | LB = Front Toggle | RB = Back Toggle");
        telemetry.addLine("CALIBRATION: X = Select Side | Y = Select Position");
        telemetry.addLine("CALIBRATION: D-Pad Up/Down = +/- 0.005 | A/B = +/- 0.025");

        telemetry.addData("Selected Side",
                selectedFront ? "FRONT" : "BACK");

        telemetry.addData("Selected Position",
                selectedDeployed ? "DEPLOYED" : "RETRACTED");

        telemetry.addData("Front Deployed",
                robot.intake.frontDeployedPos);
        telemetry.addData("Front Retracted",
                robot.intake.frontRetractedPos);

        telemetry.addData("Back Deployed",
                robot.intake.backDeployedPos);
        telemetry.addData("Back Retracted",
                robot.intake.backRetractedPos);

        telemetry.addData("Front Status",
                frontDeployed ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Back Status",
                backDeployed ? "DEPLOYED" : "RETRACTED");

        telemetry.update();
    }

    private void adjustPosition(double adjustment) {
        if (selectedFront) {
            double deployed = robot.intake.frontDeployedPos;
            double retracted = robot.intake.frontRetractedPos;

            if (selectedDeployed) {
                deployed = clip(deployed + adjustment);
            } else {
                retracted = clip(retracted + adjustment);
            }

            robot.intake.setFrontPositions(deployed, retracted);

        } else {
            double deployed = robot.intake.backDeployedPos;
            double retracted = robot.intake.backRetractedPos;

            if (selectedDeployed) {
                deployed = clip(deployed + adjustment);
            } else {
                retracted = clip(retracted + adjustment);
            }

            robot.intake.setBackPositions(deployed, retracted);
        }
    }

    private double clip(double position) {
        return Math.max(0, Math.min(1, position));
    }
}