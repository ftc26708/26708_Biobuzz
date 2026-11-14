package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.LM1Calibrations;

/**
 * Barebones LM1 TeleOp:
 * - Inherits back-intake height, turret direction/kP, ending Auto turret angle, and hood pos from {@link LM1Calibrations}.
 * - Drivetrain: Field-centric sticks (G1 RS Button resets heading to 0; G1 Y relocalizes to 7.2, 70.75, 90 deg).
 * - Shooter: Runs continuously all match; D-Pad Up/Down adjusts target RPM +/- 100 per click.
 * - Turret: Continuously tracks closest goal all match; Right Bumper toggles hold-forward (+90 deg) fallback.
 * - Left Trigger (held): Deploys both intakes + runs intake & transfer + keeps blocker closed.
 * - Right Trigger (held): Retracts intakes + runs intake & transfer + opens blocker to shoot.
 * - Left Bumper (toggle): Toggles flower deployment (up/down).
 */
@TeleOp(name = "Competition TeleOp", group = "TeleOp")
public class Teleop extends BaseOpMode {
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    private static final Pose RELOC_POSE = poseFactory.of(7.2, 70.75, 90);

    private double shotHoodPos;
    private double targetRPM = 3500.0;

    private boolean flowerUp = false;
    private boolean holdTurretForward = false;

    private Command closestGoalCommand;
    private Command holdForwardCommand;

    private boolean prevResetHeading, prevReloc;
    private boolean prevLb, prevRb;
    private boolean prevUp, prevDown;

    @Override
    protected void onInit() {
        // 1. Apply saved back-intake calibration from Auto
        if (LM1Calibrations.intakeCalibrated) {
            robot.intake.setBackPositions(
                    LM1Calibrations.backDeployedPos,
                    LM1Calibrations.backRetractedPos
            );
        }

        // 2. Apply saved turret direction & kP sign from Auto
        if (LM1Calibrations.turretCalibrated && LM1Calibrations.turretDirection != null) {
            robot.turret.setDirection(LM1Calibrations.turretDirection);
            TurretCommands.TRACKING_KP = LM1Calibrations.turretKp;
        }

        // 3. Apply saved hood position from Auto
        shotHoodPos = LM1Calibrations.shotHoodPos;
        robot.shooter.setHoodPosition(shotHoodPos);

        // 4. Default safe states (intakes retracted, flower down, blocker in the way, shooter off until Start)
        robot.intake.setDeploymentFront(false);
        robot.intake.setDeploymentBack(false);
        robot.intake.intake(0.0);
        robot.flower.setDeployment(false);
        robot.transfer.setBlocker(false);
        robot.transfer.transfer(0.0);
        robot.turret.setPower(0.0);
        robot.shooter.setTargetRPM(0.0);

        // Build turret tracking & fallback hold-forward (+90 deg from right-facing zero) commands
        closestGoalCommand = TurretCommands.turnToClosestGoal(robot.turret, robot.drivetrain.follower)
                .until(() -> holdTurretForward);
        holdForwardCommand = TurretCommands.turnToStaticAngleClipped(robot.turret, 90.0)
                .until(() -> !holdTurretForward);
    }

    @Override
    protected void onStart() {
        // Start continuous flywheel immediately on Play
        robot.shooter.setTargetRPM(targetRPM);

        // Unwind turret back to its right-facing zero (0 deg) if Auto ended rotated, then start active turret mode
        double returnTargetDeg = -LM1Calibrations.turretEndAutoDeg;
        Command unwindThenTrack = sequential(
                TurretCommands.turnToStaticAngleClipped(robot.turret, returnTargetDeg)
                        .until(() -> Math.abs(returnTargetDeg - robot.turret.getHeading()) <= 1.5),
                instant(() -> {
                    robot.turret.setPower(0.0);
                    robot.turret.resetEncoder();
                    LM1Calibrations.turretEndAutoDeg = 0.0;
                })
        ).unless(() -> Math.abs(LM1Calibrations.turretEndAutoDeg) <= 1.5)
                .then(instant(() -> Scheduler.schedule(holdTurretForward ? holdForwardCommand : closestGoalCommand)));

        Scheduler.schedule(unwindThenTrack);
    }

    @Override
    protected void onLoop() {
        // 1. Field-Centric Drivetrain (Gamepad 1 Sticks)
        robot.drivetrain.follower.manual(
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        robot.drivetrain.follower.pose().heading()
                )
        );

        // Reset field-centric heading only (G1 Right Stick Button)
        if (gamepad1.right_stick_button && !prevResetHeading) {
            Pose pose = robot.drivetrain.follower.pose();
            robot.drivetrain.follower.setPose(new Pose(pose.x(), pose.y(), 0));
        }

        // Relocalize full robot pose to (7.2, 70.75, 90 deg) when flush against wall/corner (G1 Y)
        if (gamepad1.y && !prevReloc) {
            robot.drivetrain.follower.setPose(RELOC_POSE);
        }

        // 2. Intake, Transfer & Blocker Trigger Logic (accepts either Gamepad1 or Gamepad2 triggers)
        boolean intaking = Math.max(gamepad1.left_trigger, gamepad2.left_trigger) > 0.1;
        boolean shooting = Math.max(gamepad1.right_trigger, gamepad2.right_trigger) > 0.1;

        if (shooting) {
            // Right Trigger held: intakes retracted, blocker out of the way, run intake + transfer to shoot
            robot.intake.setDeploymentFront(false);
            robot.intake.setDeploymentBack(false);
            robot.transfer.setBlocker(true);
            robot.intake.intake(1.0);
            robot.transfer.transfer(1.0);
        } else if (intaking) {
            // Left Trigger held: both intakes deployed, blocker blocking, run intake + transfer
            robot.intake.setDeploymentFront(true);
            robot.intake.setDeploymentBack(true);
            robot.transfer.setBlocker(false);
            robot.intake.intake(1.0);
            robot.transfer.transfer(1.0);
        } else {
            // Default idle: intakes retracted, blocker blocking, intake & transfer stopped
            robot.intake.setDeploymentFront(false);
            robot.intake.setDeploymentBack(false);
            robot.transfer.setBlocker(false);
            robot.intake.intake(0.0);
            robot.transfer.transfer(0.0);
        }

        // 3. Flower Deployment Toggle (Left Bumper on either controller)
        boolean lbPressed = gamepad1.left_bumper || gamepad2.left_bumper;
        if (lbPressed && !prevLb) {
            flowerUp = !flowerUp;
            robot.flower.setDeployment(flowerUp);
        }

        // 4. Turret Mode Toggle: Closest-Goal Tracking <-> Hold Straight Forward (+90 deg) (Right Bumper)
        boolean rbPressed = gamepad1.right_bumper || gamepad2.right_bumper;
        if (rbPressed && !prevRb) {
            holdTurretForward = !holdTurretForward;
            Scheduler.schedule(holdTurretForward ? holdForwardCommand : closestGoalCommand);
        }

        // 5. Continuous Shooter + TeleOp-Only D-Pad Velocity Adjustment (+/- 100 RPM per click)
        boolean upPressed = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean downPressed = gamepad1.dpad_down || gamepad2.dpad_down;
        if (upPressed && !prevUp) {
            targetRPM = Math.min(6000.0, targetRPM + 100.0);
        }
        if (downPressed && !prevDown) {
            targetRPM = Math.max(0.0, targetRPM - 100.0);
        }

        robot.shooter.setHoodPosition(shotHoodPos);
        robot.shooter.setTargetRPM(targetRPM);

        // Update all edge-detection booleans
        prevResetHeading = gamepad1.right_stick_button;
        prevReloc = gamepad1.y;
        prevLb = lbPressed;
        prevRb = rbPressed;
        prevUp = upPressed;
        prevDown = downPressed;

        // Telemetry
        Pose currentPose = robot.drivetrain.follower.pose();
        telemetry.addLine("=== LM1 TELEOP ===");
        telemetry.addLine("G1: Sticks = Drive | RS Btn = Reset Heading | Y = Reloc (7.2, 70.75, 90)");
        telemetry.addLine("LT (Hold) = Deploy Intakes + Intake/Transfer (Blocker CLOSED)");
        telemetry.addLine("RT (Hold) = Retract Intakes + Shoot (Blocker OPEN)");
        telemetry.addLine("LB = Toggle Flower | RB = Toggle Turret (Auto-Aim <-> Hold Forward)");
        telemetry.addLine("D-Pad Up/Down = Target Shot RPM +/- 100");
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Pose", String.format("X: %.2f | Y: %.2f | H: %.1f deg",
                currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading())));
        telemetry.addData("Mode", shooting ? "SHOOTING (BLOCKER OPEN)" : (intaking ? "INTAKING (DEPLOYED)" : "IDLE (BLOCKED)"));
        telemetry.addData("Shooter RPM", String.format("Target: %.0f | Current: %.0f (At Speed: %s)",
                targetRPM, robot.shooter.getCurrentRPM(), robot.shooter.isAtSpeed() ? "YES" : "NO"));
        telemetry.addData("Turret", String.format("%s | Heading: %.1f deg | kP: %.3f",
                holdTurretForward ? "HOLDING FORWARD (+90)" : "AUTO-AIMING",
                robot.turret.getHeading(), TurretCommands.TRACKING_KP));
        telemetry.addData("Flower", flowerUp ? "UP" : "DOWN");
        telemetry.update();
    }

    @Override
    protected void onStop() {
        robot.intake.intake(0.0);
        robot.transfer.transfer(0.0);
        robot.transfer.setBlocker(false);
        robot.turret.setPower(0.0);
        robot.shooter.setTargetRPM(0.0);
    }
}