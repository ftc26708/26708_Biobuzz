package org.firstinspires.ftc.teamcode.opmode.test;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "Turret Calibration")
public class TurretCalibration extends BaseOpMode {
    private enum CalibrationStep {
        STEP_1_DIRECTION("1/3: DIRECTION & ZERO CHECK"),
        STEP_2_PID_TUNING("2/3: PID & WRAPAROUND TUNING"),
        STEP_3_AIMING_AND_DRIVE("3/3: CLOSEST GOAL & DRIVE TEST");

        final String label;
        CalibrationStep(String label) { this.label = label; }
    }

    private CalibrationStep currentStep = CalibrationStep.STEP_1_DIRECTION;
    private double targetAngleDeg = 0.0;
    private double tunableKp = 0.02;
    private Command pidTuneCommand;
    private Command closestGoalCommand;

    private boolean prevX, prevY, prevLb, prevRb;
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevA, prevB, prevReset;

    @Override
    public void onInit() {
        robot.turret.resetEncoder();
        robot.turret.setPower(0);

        pidTuneCommand = infinite(() -> {
            double clippedAngle = Range.clip(targetAngleDeg, -180, 180);
            double error = clippedAngle - robot.turret.getHeading();
            double power = Range.clip(tunableKp * error, -1, 1);
            robot.turret.setPower(power);
        })
                .setEnd(interrupted -> robot.turret.setPower(0))
                .requiring(robot.turret)
                .until(() -> currentStep != CalibrationStep.STEP_2_PID_TUNING);

        closestGoalCommand = TurretCommands.turnToClosestGoal(robot.turret, robot.drivetrain.follower)
                .until(() -> currentStep != CalibrationStep.STEP_3_AIMING_AND_DRIVE);
    }

    @Override
    public void onLoop() {
        // Global: Reset Robot Pose to (70.75, 70.75, 0)
        if (gamepad1.right_stick_button && !prevReset) {
            robot.drivetrain.follower.setPose(new Pose(70.75, 70.75, 0));
        }

        // Global: Zero Turret Encoder
        if (gamepad2.y && !prevY) {
            robot.turret.resetEncoder();
            targetAngleDeg = 0.0;
        }

        // Global: Advance State Machine (G2 X: Step 1 -> Step 2 -> Step 3 -> Step 1)
        if (gamepad2.x && !prevX) {
            switch (currentStep) {
                case STEP_1_DIRECTION:
                    currentStep = CalibrationStep.STEP_2_PID_TUNING;
                    targetAngleDeg = Range.clip(robot.turret.getHeading(), -180.0, 180.0);
                    Scheduler.schedule(pidTuneCommand);
                    break;
                case STEP_2_PID_TUNING:
                    currentStep = CalibrationStep.STEP_3_AIMING_AND_DRIVE;
                    Scheduler.schedule(closestGoalCommand);
                    break;
                case STEP_3_AIMING_AND_DRIVE:
                    currentStep = CalibrationStep.STEP_1_DIRECTION;
                    robot.turret.setPower(0);
                    break;
            }
        }

        switch (currentStep) {
            case STEP_1_DIRECTION:
                if (gamepad2.right_bumper && !prevRb) {
                    DcMotorSimple.Direction nextDir =
                            (robot.turret.turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD)
                                    ? DcMotorSimple.Direction.REVERSE
                                    : DcMotorSimple.Direction.FORWARD;
                    robot.turret.setDirection(nextDir);
                }
                // Push G2 left stick LEFT (-X) -> positive power -> must turn CCW
                robot.turret.setPower(-gamepad2.left_stick_x * 0.5);
                break;

            case STEP_2_PID_TUNING:
                if (gamepad2.left_bumper && !prevLb) {
                    targetAngleDeg = (targetAngleDeg >= 0) ? -180.0 : 180.0;
                }

                double angleAdj = 0;
                if (gamepad2.dpad_up && !prevUp) angleAdj += 5.0;
                if (gamepad2.dpad_down && !prevDown) angleAdj -= 5.0;
                if (gamepad2.a && !prevA) angleAdj += 45.0;
                if (gamepad2.b && !prevB) angleAdj -= 45.0;
                if (angleAdj != 0) {
                    targetAngleDeg = Range.clip(targetAngleDeg + angleAdj, -180.0, 180.0);
                }

                double kpAdj = 0;
                if (gamepad2.dpad_right && !prevRight) kpAdj += 0.002;
                if (gamepad2.dpad_left && !prevLeft) kpAdj -= 0.002;
                if (kpAdj != 0) {
                    tunableKp = Math.max(0.0, tunableKp + kpAdj);
                }
                break;

            case STEP_3_AIMING_AND_DRIVE:
                robot.drivetrain.follower.manual(
                        ManualDrive.fieldCentric(
                                -gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x,
                                robot.drivetrain.follower.pose().heading()
                        )
                );
                break;
        }

        prevX = gamepad2.x;
        prevY = gamepad2.y;
        prevLb = gamepad2.left_bumper;
        prevRb = gamepad2.right_bumper;
        prevUp = gamepad2.dpad_up;
        prevDown = gamepad2.dpad_down;
        prevLeft = gamepad2.dpad_left;
        prevRight = gamepad2.dpad_right;
        prevA = gamepad2.a;
        prevB = gamepad2.b;
        prevReset = gamepad1.right_stick_button;

        updateTelemetry();
    }

    private void updateTelemetry() {
        Pose pose = robot.drivetrain.follower.pose();
        double currentHeading = robot.turret.getHeading();

        telemetry.addData("=== ACTIVE STEP ===", currentStep.label);

        switch (currentStep) {
            case STEP_1_DIRECTION:
                telemetry.addLine("GOAL: Push G2 LS Left -> Turret must turn CCW & Heading +");
                telemetry.addLine("  [G2 LS X] Manual Turn | [G2 RB] Flip Motor Direction");
                telemetry.addLine("  [G2 Y] Zero Turret at Straight-Forward 0 deg");
                break;

            case STEP_2_PID_TUNING:
                telemetry.addLine("GOAL: Tune kP & test +180 <-> -180 full unwind");
                telemetry.addLine("  [G2 LB] Wraparound (+180 <-> -180)");
                telemetry.addLine("  [G2 D-Pad U/D] +/- 5 deg | [G2 A/B] +/- 45 deg");
                telemetry.addLine("  [G2 D-Pad L/R] Tune kP (+/- 0.002)");
                break;

            case STEP_3_AIMING_AND_DRIVE:
                telemetry.addLine("GOAL: Drive & spin to verify auto-aim on closer goal");
                telemetry.addLine("  [G1 LS] Drive | [G1 RS X] Turn");
                break;
        }

        telemetry.addLine("=== LIVE VALUES ===");
        telemetry.addData("  Turret Direction", robot.turret.turretMotor.getDirection());
        telemetry.addData("  Tunable kP", String.format("%.4f", tunableKp));
        telemetry.addData("  Turret Heading", String.format("%.1f deg (Target: %.1f | Err: %.1f)",
                currentHeading, targetAngleDeg, targetAngleDeg - currentHeading));
        telemetry.addData("  Robot Pose", String.format("X: %.2f | Y: %.2f | H: %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));

        telemetry.addLine("=== GLOBAL CONTROLS ===");
        telemetry.addLine("  [G2 X] Next Step | [G2 Y] Zero Turret | [G1 RS Btn] Reset Pose (70.75, 70.75, 0)");
        telemetry.update();
    }

    @Override
    public void onStop() {
        currentStep = CalibrationStep.STEP_1_DIRECTION;
        robot.turret.setPower(0);
    }
}