package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "Shooter Calibration")
public class ShooterCalibration extends BaseOpMode {
    private boolean tuningShot = false; // false = Step 1 (Hood Base), true = Step 2 (Shot & Turret)
    private Command turretTrackCommand;

    private double hoodBasePos = 0.20;
    private double shotHoodPos = 0.20;
    private double targetRPM = 3500.0;
    private boolean flywheelOn = false;

    private boolean prevX, prevY, prevLb, prevRb;
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevA, prevB, prevReset;

    @Override
    public void onInit() {
        robot.shooter.setTargetRPM(0.0);
        robot.shooter.setHoodPosition(hoodBasePos);
        robot.turret.resetEncoder();
        robot.turret.setPower(0.0);

        turretTrackCommand = TurretCommands.turnToClosestGoal(robot.turret, robot.drivetrain.follower)
                .until(() -> !tuningShot);

        // =========================================================================
        // UNCOMMENT WITH CTRL + / ONCE INTAKE & TRANSFER ARE MERGED INTO THIS BRANCH
        // =========================================================================
        // robot.intake.setDeploymentFront(true);
        // robot.intake.setDeploymentBack(true);
        // robot.transfer.setBlocker(true);
        // =========================================================================
    }

    @Override
    public void onLoop() {
        // Drivetrain & Pose Reset (70.75, 70.75, 0)
        robot.drivetrain.follower.manual(
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        robot.drivetrain.follower.pose().heading()
                )
        );

        if (gamepad1.right_stick_button && !prevReset) {
            robot.drivetrain.follower.setPose(new Pose(70.75, 70.75, 0));
        }

        // Toggle Step 1 (Hood Base) <-> Step 2 (Shot & Turret Tracking)
        if (gamepad2.x && !prevX) {
            tuningShot = !tuningShot;
            if (tuningShot) {
                shotHoodPos = hoodBasePos;
                Scheduler.schedule(turretTrackCommand);
            } else {
                flywheelOn = false;
                robot.shooter.setTargetRPM(0.0);
                robot.shooter.setHoodPosition(hoodBasePos);
                // robot.intake.intake(0.0);
                // robot.transfer.transfer(0.0);
            }
        }

        if (!tuningShot) {
            // STEP 1: Hood Base Position
            double adj = getDpadAndFaceAdjustment(0.005, 0.025);
            if (adj != 0) {
                hoodBasePos = Range.clip(hoodBasePos + adj, 0.0, 1.0);
            }
            robot.shooter.setHoodPosition(hoodBasePos);
        } else {
            // STEP 2: Shot Speed, Angle & Turret Tracking
            if (gamepad2.y && !prevY) {
                flywheelOn = !flywheelOn;
            }

            double hoodAdj = getDpadAndFaceAdjustment(0.005, 0.025);
            if (hoodAdj != 0) {
                shotHoodPos = Range.clip(shotHoodPos + hoodAdj, 0.0, 1.0);
            }

            double rpmAdj = 0;
            if (gamepad2.dpad_right && !prevRight) rpmAdj += 50.0;
            if (gamepad2.dpad_left && !prevLeft) rpmAdj -= 50.0;
            if (gamepad2.right_bumper && !prevRb) rpmAdj += 250.0;
            if (gamepad2.left_bumper && !prevLb) rpmAdj -= 250.0;
            if (rpmAdj != 0) {
                targetRPM = Range.clip(targetRPM + rpmAdj, 0.0, 6000.0);
            }

            robot.shooter.setHoodPosition(shotHoodPos);
            robot.shooter.setTargetRPM(flywheelOn ? targetRPM : 0.0);

            // =========================================================================
            // TODO: UNCOMMENT WITH CTRL + / ONCE INTAKE & TRANSFER ARE MERGED INTO THIS BRANCH
            // =========================================================================
            // robot.intake.setDeploymentFront(true);
            // robot.intake.setDeploymentBack(true);
            // robot.transfer.setBlocker(true);
            // double feedPower = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            // robot.intake.intake(feedPower);
            // robot.transfer.transfer(feedPower);
            // =========================================================================
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

    private double getDpadAndFaceAdjustment(double fine, double coarse) {
        double adj = 0;
        if (gamepad2.dpad_up && !prevUp) adj += fine;
        if (gamepad2.dpad_down && !prevDown) adj -= fine;
        if (gamepad2.a && !prevA) adj += coarse;
        if (gamepad2.b && !prevB) adj -= coarse;
        return adj;
    }

    private void updateTelemetry() {
        Pose pose = robot.drivetrain.follower.pose();

        telemetry.addData("=== ACTIVE STEP ===", tuningShot
                ? "2/2: SHOT SPEED, ANGLE & TURRET TRACKING"
                : "1/2: HOOD BASE SERVO POSITION");

        if (!tuningShot) {
            telemetry.addLine("GOAL: Set hood servo to mechanical base/rest point (65 deg)");
            telemetry.addLine("  [G2 D-Pad U/D] +/- 0.005 | [G2 A/B] +/- 0.025");
        } else {
            telemetry.addLine("GOAL: Turret auto-aims at closer goal; tune Hood Pos & RPM");
            telemetry.addLine("  [G2 Y] Toggle Flywheel ON/OFF");
            telemetry.addLine("  [G2 D-Pad U/D] Hood +/- 0.005 | [G2 A/B] Hood +/- 0.025");
            telemetry.addLine("  [G2 D-Pad L/R] RPM +/- 50 | [G2 LB/RB] RPM +/- 250");
            // telemetry.addLine("  [G1/G2 RT] Hold to Run Intake & Transfer");
        }

        telemetry.addLine("=== LIVE VALUES (RECORD THESE) ===");
        telemetry.addData("  HOOD_BASE_POSITION", String.format("%.3f", hoodBasePos));
        telemetry.addData("  Shot Hood Position", String.format("%.3f (Angle: %.1f deg)",
                shotHoodPos, robot.shooter.getHoodAngle()));
        telemetry.addData("  Flywheel State", flywheelOn ? "ON" : "OFF");
        telemetry.addData("  Target RPM", String.format("%.0f RPM", targetRPM));
        telemetry.addData("  Current RPM", String.format("%.0f RPM (At Speed: %s)",
                robot.shooter.getCurrentRPM(), robot.shooter.isAtSpeed() ? "YES" : "NO"));
        telemetry.addData("  Turret Tracking", tuningShot
                ? String.format("ACTIVE (Heading: %.1f deg)", robot.turret.getHeading()) : "OFF");
        telemetry.addData("  Robot Pose", String.format("X: %.2f | Y: %.2f | H: %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));

        telemetry.addLine("=== GLOBAL CONTROLS ===");
        telemetry.addLine("  [G1 LS] Drive | [G1 RS X] Turn | [G1 RS Btn] Reset Pose (70.75, 70.75, 0)");
        telemetry.addLine("  [G2 X] Switch Step (1 <-> 2)");
        telemetry.update();
    }

    @Override
    public void onStop() {
        tuningShot = false;
        robot.shooter.setTargetRPM(0.0);
        robot.turret.setPower(0.0);
        // robot.intake.intake(0.0);
        // robot.transfer.transfer(0.0);
        // robot.transfer.setBlocker(false);
    }
}