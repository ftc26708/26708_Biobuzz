
package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop extends BaseOpMode {

    private static final double STARTING_RPM = 3500.0;
    private static final double RPM_ADJUSTMENT = 100.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_RPM = 0.0;

    private static final Pose RELOC_POSE =
            com.pedropathing.api.PoseFactory.degrees().of(7.2, 70.75, 90);

    private double targetRPM = STARTING_RPM;

    private boolean flowerUp = false;
    private boolean holdTurretRight = false;

    private boolean previousFlowerButton = false;
    private boolean previousTurretButton = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousRelocalize = false;

    private com.pedropathing.ivy.Command closestGoalCommand;
    private com.pedropathing.ivy.Command holdRightCommand;

    @Override
    protected void onInit() {
        robot.flower.setDeployment(false);

        closestGoalCommand =
                TurretCommands.turnToClosestGoal(
                        robot.turret,
                        robot.drivetrain.follower
                ).until(() -> holdTurretRight);

        holdRightCommand =
                TurretCommands.turnToStaticAngleClipped(
                        robot.turret,
                        0.0
                ).until(() -> !holdTurretRight);

        telemetry.addLine("TeleOp initialized");
        telemetry.addLine("Turret zero = straight right");
        telemetry.update();
    }

    @Override
    protected void onStart() {
        targetRPM = STARTING_RPM;
        robot.shooter.setTargetRPM(targetRPM);

        Scheduler.schedule(closestGoalCommand);
    }

    @Override
    protected void onLoop() {
        // ---------------- DRIVE ----------------

        DrivePowers drivePowers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robot.drivetrain.follower.pose().heading()
        );

        robot.drivetrain.follower.manual(drivePowers);

        // Relocalize to the known reference pose.
        boolean relocalize = gamepad1.y;
        if (relocalize && !previousRelocalize) {
            robot.drivetrain.follower.setPose(RELOC_POSE);
        }
        previousRelocalize = relocalize;

        // ---------------- INTAKE / SHOOT ----------------

        boolean intakeButton =
                gamepad1.left_trigger > 0.1 ||
                        gamepad2.left_trigger > 0.1;

        boolean shootButton =
                gamepad1.right_trigger > 0.1 ||
                        gamepad2.right_trigger > 0.1;

        if (shootButton) {
            robot.intake.setDeploymentFront(false);
            robot.intake.setDeploymentBack(false);
            robot.intake.intake(1.0);

            robot.transfer.transfer(1.0);
            robot.transfer.setBlocker(true);
        } else if (intakeButton) {
            robot.intake.setDeploymentFront(true);
            robot.intake.setDeploymentBack(true);
            robot.intake.intake(1.0);

            robot.transfer.transfer(1.0);
            robot.transfer.setBlocker(false);
        } else {
            robot.intake.intake(0.0);
            robot.transfer.transfer(0.0);
        }

        // ---------------- FLOWER ----------------

        boolean flowerButton = gamepad1.left_bumper || gamepad2.left_bumper;

        if (flowerButton && !previousFlowerButton) {
            flowerUp = !flowerUp;
            robot.flower.setDeployment(flowerUp);
        }
        previousFlowerButton = flowerButton;

        // ---------------- TURRET ----------------

        boolean turretButton = gamepad1.right_bumper || gamepad2.right_bumper;

        if (turretButton && !previousTurretButton) {
            holdTurretRight = !holdTurretRight;

            Scheduler.schedule(
                    holdTurretRight ? holdRightCommand : closestGoalCommand
            );
        }
        previousTurretButton = turretButton;

        // ---------------- SHOOTER RPM ----------------

        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;

        if (dpadUp && !previousDpadUp) {
            targetRPM = Math.min(MAX_RPM, targetRPM + RPM_ADJUSTMENT);
        }

        if (dpadDown && !previousDpadDown) {
            targetRPM = Math.max(MIN_RPM, targetRPM - RPM_ADJUSTMENT);
        }

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;

        robot.shooter.setTargetRPM(targetRPM);

        // ---------------- TELEMETRY ----------------

        Pose currentPose = robot.drivetrain.follower.pose();

        telemetry.addData("X", "%.2f", currentPose.x());
        telemetry.addData("Y", "%.2f", currentPose.y());
        telemetry.addData("Heading", "%.1f°",
                Math.toDegrees(currentPose.heading()));

        telemetry.addData("Shooter Target", "%.0f RPM", targetRPM);
        telemetry.addData("Turret Mode",
                holdTurretRight ? "Fixed right (0°)" : "Closest goal");
        telemetry.addData("Turret Heading", "%.1f°",
                robot.turret.getHeading());
        telemetry.addData("Flower", flowerUp ? "Up" : "Down");

        telemetry.update();
    }
}