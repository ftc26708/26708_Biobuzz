package org.firstinspires.ftc.teamcode.opmode.test;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

    // Match goal coordinates from TurretCommands (inches)
    private static final Pose GOAL_1 = new Pose(0, 36);
    private static final Pose GOAL_2 = new Pose(0, 108);

    private CalibrationStep currentStep = CalibrationStep.STEP_1_DIRECTION;
    private int directionSign = 1; // +1 = FORWARD, -1 = REVERSED
    private double targetAngleDeg = 0.0;
    private double tunableKp = 0.02;
    private Command activeTurretCommand;

    private boolean prevX, prevY, prevLb, prevRb;
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevA, prevB;
    private boolean prevReset;

    @Override
    public void onInit() {
        robot.turret.resetEncoder();
        robot.turret.setPower(0);
        enterStep(CalibrationStep.STEP_1_DIRECTION);
    }

    @Override
    public void onLoop() {
        // Global Controls (Available in All Steps)
        if (gamepad1.right_stick_button && !prevReset) {
            robot.drivetrain.follower.setPose(new Pose(70.75, 70.75, 0));
        }

        if (gamepad2.y && !prevY) {
            robot.turret.resetEncoder();
            targetAngleDeg = 0.0;
        }

        if (gamepad2.x && !prevX) {
            switch (currentStep) {
                case STEP_1_DIRECTION:
                    enterStep(CalibrationStep.STEP_2_PID_TUNING);
                    break;
                case STEP_2_PID_TUNING:
                    enterStep(CalibrationStep.STEP_3_AIMING_AND_DRIVE);
                    break;
                case STEP_3_AIMING_AND_DRIVE:
                    enterStep(CalibrationStep.STEP_1_DIRECTION);
                    break;
            }
        }

        // Run Current Step Logic
        switch (currentStep) {
            case STEP_1_DIRECTION:
                runStep1Direction();
                break;
            case STEP_2_PID_TUNING:
                runStep2PidTuning();
                break;
            case STEP_3_AIMING_AND_DRIVE:
                runStep3AimingAndDrive();
                break;
        }

        // Update ALL edge-detection flags every loop so step transitions never misfire
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

    private void enterStep(CalibrationStep nextStep) {
        cancelActiveCommand();
        robot.turret.setPower(0);
        currentStep = nextStep;

        switch (nextStep) {
            case STEP_1_DIRECTION:
                break;

            case STEP_2_PID_TUNING:
                targetAngleDeg = getCalibratedHeading();
                activeTurretCommand = infinite(() -> runCalibratedPid(targetAngleDeg))
                        .setEnd(interrupted -> robot.turret.setPower(0))
                        .requiring(robot.turret);
                Scheduler.schedule(activeTurretCommand);
                break;

            case STEP_3_AIMING_AND_DRIVE:
                activeTurretCommand = infinite(() -> {
                    Pose pose = robot.drivetrain.follower.pose();

                    double dx1 = GOAL_1.x() - pose.x();
                    double dy1 = GOAL_1.y() - pose.y();
                    double distSq1 = dx1 * dx1 + dy1 * dy1;

                    double dx2 = GOAL_2.x() - pose.x();
                    double dy2 = GOAL_2.y() - pose.y();
                    double distSq2 = dx2 * dx2 + dy2 * dy2;

                    double dx = (distSq1 <= distSq2) ? dx1 : dx2;
                    double dy = (distSq1 <= distSq2) ? dy1 : dy2;

                    double fieldAngleRad = Math.atan2(dy, dx);
                    double relativeRad = fieldAngleRad - pose.heading();
                    targetAngleDeg = Math.toDegrees(Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad)));

                    runCalibratedPid(targetAngleDeg);
                })
                        .setEnd(interrupted -> {
                            robot.turret.setPower(0);
                            robot.drivetrain.follower.manual(ManualDrive.fieldCentric(
                                    0, 0, 0, robot.drivetrain.follower.pose().heading()
                            ));
                        })
                        .requiring(robot.turret);
                Scheduler.schedule(activeTurretCommand);
                break;
        }
    }

    private void runStep1Direction() {
        if (gamepad2.right_bumper && !prevRb) {
            directionSign *= -1;
        }
        double manualPower = -gamepad2.left_stick_x * 0.5;
        robot.turret.setPower(manualPower * directionSign);
    }

    private void runStep2PidTuning() {
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

        adjustKp();
    }

    private void runStep3AimingAndDrive() {
        robot.drivetrain.follower.manual(
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        robot.drivetrain.follower.pose().heading()
                )
        );
        adjustKp();
    }

    private void adjustKp() {
        double kpAdj = 0;
        if (gamepad2.dpad_right && !prevRight) kpAdj += 0.002;
        if (gamepad2.dpad_left && !prevLeft) kpAdj -= 0.002;
        if (kpAdj != 0) {
            tunableKp = Math.max(0.0, tunableKp + kpAdj);
        }
    }

    private void updateTelemetry() {
        Pose pose = robot.drivetrain.follower.pose();
        double currentHeading = getCalibratedHeading();

        telemetry.addData("=== ACTIVE STEP ===", currentStep.label);

        switch (currentStep) {
            case STEP_1_DIRECTION:
                telemetry.addLine("GOAL: Push G2 LS Left -> Turret must turn CCW & Heading +");
                telemetry.addLine("  [G2 LS X] Manual Turn | [G2 RB] Flip Direction");
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
                telemetry.addLine("  [G2 D-Pad L/R] Fine-Tune kP (+/- 0.002)");
                break;
        }

        telemetry.addLine("=== LIVE VALUES ===");
        telemetry.addData("  Direction Sign", directionSign == 1 ? "+1 (FORWARD)" : "-1 (SET REVERSE IN TURRET.JAVA)");
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
        cancelActiveCommand();
        robot.turret.setPower(0);
    }

    private double getCalibratedHeading() {
        return robot.turret.getHeading() * directionSign;
    }

    private void runCalibratedPid(double rawTargetDeg) {
        double clippedAngle = Range.clip(rawTargetDeg, -180, 180);
        double error = clippedAngle - getCalibratedHeading();
        double power = Range.clip(tunableKp * error, -1, 1);
        robot.turret.setPower(power * directionSign);
    }

    private void cancelActiveCommand() {
        if (activeTurretCommand != null) {
            Scheduler.cancel(activeTurretCommand);
            activeTurretCommand = null;
        }
    }
}