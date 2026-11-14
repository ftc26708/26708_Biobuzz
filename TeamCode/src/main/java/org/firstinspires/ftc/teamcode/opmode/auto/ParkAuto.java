package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.LM1Calibrations;

/**
 * Barebones LM1 Autonomous OpMode:
 * - Step 1 (init_loop): Automatic turret wire-polarity check + visual CCW confirmation & zeroing.
 * - Step 2 (init_loop): Deploys back intake so team can nudge its height + adjust hood position.
 * - Step 3 (init_loop): Auto-retracts intakes so team can place robot in starting tile.
 * - onStart(): Re-zeros turret, resets follower pose to START, and runs park path only.
 */
@Autonomous(name = "ParkAuto", group = "Auto")
public class ParkAuto extends BaseOpMode {
    private enum InitStep {
        STEP_1_TURRET,
        STEP_2_INTAKE_AND_HOOD,
        STEP_3_READY_TO_PLACE
    }

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // Update START and PARK_POSE to your exact starting tile and park spot (inches, degrees)
    private static final Pose START = poseFactory.of(7.2, 70.75, 90);
    private static final Pose PARK_POSE = poseFactory.of(12, 94, 90);

    private Path parkPath;
    private InitStep initStep = InitStep.STEP_1_TURRET;

    // Back-intake & hood calibration state (persisted to LM1Calibrations for TeleOp)
    private double backDeployedPos;
    private double backRetractedPos;
    private double shotHoodPos;

    // Turret polarity & CCW check state
    private long turretPulseStartMs = 0;
    private boolean turretPulseDone = false;
    private boolean wiresReversedDetected = false;

    // Edge-detection booleans for Gamepad2 init controls
    private boolean prevX, prevRb, prevA, prevY;
    private boolean prevUp, prevDown, prevLeft, prevRight;

    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap, START);
    }

    @Override
    protected void onInit() {
        // 1. Restore previously saved back-intake positions or seed from Robot.java defaults
        if (LM1Calibrations.intakeCalibrated) {
            backDeployedPos = LM1Calibrations.backDeployedPos;
            backRetractedPos = LM1Calibrations.backRetractedPos;
            robot.intake.setBackPositions(backDeployedPos, backRetractedPos);
        } else {
            backDeployedPos = robot.intake.backDeployedPos;
            backRetractedPos = robot.intake.backRetractedPos;
        }

        // 2. Restore saved hood position or seed from LM1Calibrations default
        shotHoodPos = LM1Calibrations.shotHoodPos;
        robot.shooter.setHoodPosition(shotHoodPos);

        // 3. Restore saved turret settings or start automatic wire-polarity check in Step 1
        robot.turret.resetEncoder();
        robot.turret.setPower(0.0);

        if (LM1Calibrations.turretCalibrated && LM1Calibrations.turretDirection != null) {
            robot.turret.setDirection(LM1Calibrations.turretDirection);
            TurretCommands.TRACKING_KP = LM1Calibrations.turretKp;
            turretPulseDone = true;
            wiresReversedDetected = (LM1Calibrations.turretKp < 0);
        } else {
            LM1Calibrations.turretDirection = robot.turret.turretMotor.getDirection();
            LM1Calibrations.turretKp = Math.abs(TurretCommands.TRACKING_KP);
            TurretCommands.TRACKING_KP = LM1Calibrations.turretKp;
            turretPulseStartMs = System.currentTimeMillis();
            turretPulseDone = false;
            wiresReversedDetected = false;
        }

        // 4. Keep intakes retracted and all non-drivetrain motors off initially
        robot.flower.setDeployment(false);
        robot.intake.setDeploymentFront(false);
        robot.intake.setDeploymentBack(false);
        robot.intake.intake(0.0);
        robot.transfer.setBlocker(false);
        robot.transfer.transfer(0.0);
        robot.shooter.setTargetRPM(0.0);

        initStep = InitStep.STEP_1_TURRET;
        parkPath = Paths.line(START, PARK_POSE);
    }

    @Override
    protected void onInitLoop() {
        switch (initStep) {
            case STEP_1_TURRET:
                runInitStep1Turret();
                break;
            case STEP_2_INTAKE_AND_HOOD:
                runInitStep2IntakeAndHood();
                break;
            case STEP_3_READY_TO_PLACE:
                runInitStep3Ready();
                break;
        }

        prevX = gamepad2.x;
        prevRb = gamepad2.right_bumper;
        prevA = gamepad2.a;
        prevY = gamepad2.y;
        prevUp = gamepad2.dpad_up;
        prevDown = gamepad2.dpad_down;
        prevLeft = gamepad2.dpad_left;
        prevRight = gamepad2.dpad_right;
    }

    // --- INIT STEP 1: Turret Wire Polarity & CCW Direction Check ---
    private void runInitStep1Turret() {
        if (!turretPulseDone) {
            if (System.currentTimeMillis() - turretPulseStartMs < 150) {
                robot.turret.setPower(0.15);
            } else {
                robot.turret.setPower(0.0);
                turretPulseDone = true;

                if (robot.turret.getTicks() < -5) {
                    wiresReversedDetected = true;
                    LM1Calibrations.turretKp = -Math.abs(TurretCommands.TRACKING_KP);
                } else {
                    wiresReversedDetected = false;
                    LM1Calibrations.turretKp = Math.abs(TurretCommands.TRACKING_KP);
                }
                TurretCommands.TRACKING_KP = LM1Calibrations.turretKp;
            }
        }

        if (gamepad2.right_bumper && !prevRb) {
            DcMotorSimple.Direction flipped =
                    (robot.turret.turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD)
                            ? DcMotorSimple.Direction.REVERSE
                            : DcMotorSimple.Direction.FORWARD;
            robot.turret.setDirection(flipped);
            LM1Calibrations.turretDirection = flipped;

            robot.turret.resetEncoder();
            turretPulseStartMs = System.currentTimeMillis();
            turretPulseDone = false;
        }

        if (gamepad2.a && !prevA) {
            robot.turret.resetEncoder();
        }

        if (gamepad2.x && !prevX) {
            robot.turret.setPower(0.0);
            LM1Calibrations.turretCalibrated = true;
            robot.intake.setDeploymentBack(true);
            initStep = InitStep.STEP_2_INTAKE_AND_HOOD;
        }

        telemetry.addLine("========================================");
        telemetry.addLine("  INIT STEP 1/2: TURRET DIRECTION CHECK");
        telemetry.addLine("========================================");
        telemetry.addLine("1. Watch initial nudge: Did turret turn COUNTERCLOCKWISE (LEFT)?");
        telemetry.addLine("   -> If it turned CLOCKWISE (RIGHT), press [G2 RB] to flip!");
        telemetry.addLine("2. Align turret straight forward (0 deg), then press [G2 A] to zero.");
        telemetry.addLine("3. Press [G2 X] to CONTINUE to Step 2 (Back Intake & Hood).");
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Wire Polarity", wiresReversedDetected
                ? "SWAPPED WIRES AUTO-FIXED (kP negative)"
                : "NORMAL (kP positive)");
        telemetry.addData("Direction / kP", robot.turret.turretMotor.getDirection()
                + " | kP = " + TurretCommands.TRACKING_KP);
        telemetry.addData("Turret Heading (deg)", String.format("%.1f", robot.turret.getHeading()));
        telemetry.update();
    }

    // --- INIT STEP 2: Deployed Back Intake Height & Hood Servo Calibration ---
    private void runInitStep2IntakeAndHood() {
        double intakeDelta = 0;
        if (gamepad2.dpad_up && !prevUp) intakeDelta += 0.01;
        if (gamepad2.dpad_down && !prevDown) intakeDelta -= 0.01;

        if (intakeDelta != 0) {
            backDeployedPos = Math.max(0.0, Math.min(1.0, backDeployedPos + intakeDelta));
            backRetractedPos = Math.max(0.0, Math.min(1.0, backRetractedPos + intakeDelta));

            robot.intake.setBackPositions(backDeployedPos, backRetractedPos);
            robot.intake.setDeploymentBack(true);

            LM1Calibrations.backDeployedPos = backDeployedPos;
            LM1Calibrations.backRetractedPos = backRetractedPos;
            LM1Calibrations.intakeCalibrated = true;
        }

        double hoodDelta = 0;
        if (gamepad2.dpad_right && !prevRight) hoodDelta += 0.01;
        if (gamepad2.dpad_left && !prevLeft) hoodDelta -= 0.01;

        if (hoodDelta != 0) {
            shotHoodPos = Math.max(0.0, Math.min(1.0, shotHoodPos + hoodDelta));
            robot.shooter.setHoodPosition(shotHoodPos);

            LM1Calibrations.shotHoodPos = shotHoodPos;
            LM1Calibrations.hoodCalibrated = true;
        }

        if (gamepad2.x && !prevX) {
            robot.intake.setDeploymentFront(false);
            robot.intake.setDeploymentBack(false);
            initStep = InitStep.STEP_3_READY_TO_PLACE;
        }

        if (gamepad2.y && !prevY) {
            robot.intake.setDeploymentBack(false);
            initStep = InitStep.STEP_1_TURRET;
        }

        telemetry.addLine("========================================");
        telemetry.addLine("  INIT STEP 2/2: BACK INTAKE & HOOD");
        telemetry.addLine("========================================");
        telemetry.addLine("Back intake is DEPLOYED so you can match its height to the front.");
        telemetry.addLine("1. [G2 D-Pad Up/Down] Nudge Back Intake Height (+/- 0.01)");
        telemetry.addLine("2. [G2 D-Pad Left/Right] Nudge Shooter Hood Pos (+/- 0.01)");
        telemetry.addLine("3. Press [G2 X] to FINISH & AUTO-RETRACT INTAKES.");
        telemetry.addLine("   (Need to re-check turret? Press [G2 Y] to go back)");
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Back Deployed / Retracted", String.format("%.3f / %.3f", backDeployedPos, backRetractedPos));
        telemetry.addData("Shooter Hood Pos", String.format("%.3f (Angle: %.1f deg)",
                shotHoodPos, robot.shooter.getHoodAngle()));
        telemetry.update();
    }
    // --- INIT STEP 3: Intakes Auto-Retracted -> Team Places Robot on Starting Tile ---
    private void runInitStep3Ready() {
        robot.intake.setDeploymentFront(false);
        robot.intake.setDeploymentBack(false);
        if (gamepad2.x && !prevX) {
            robot.intake.setDeploymentBack(true);
            initStep = InitStep.STEP_2_INTAKE_AND_HOOD;
        }
        if (gamepad2.y && !prevY) {
            initStep = InitStep.STEP_1_TURRET;
        }
        telemetry.addLine("========================================");
        telemetry.addLine("  ALL CHECKS COMPLETE - READY FOR MATCH!");
        telemetry.addLine("========================================");
        telemetry.addLine("1. Both intakes are AUTO-RETRACTED inside 18 inches.");
        telemetry.addLine("2. Place the robot into its exact STARTING POSITION now.");
        telemetry.addLine("3. Press PLAY when ready (Start Pose & Turret zero lock on Play).");
        telemetry.addLine("   (Re-adjust intake/hood? Press [G2 X] | Re-check turret? Press [G2 Y])");
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Saved Turret", robot.turret.turretMotor.getDirection()
                + " | kP = " + TurretCommands.TRACKING_KP);
        telemetry.addData("Saved Back Intake", String.format("Dep: %.3f | Ret: %.3f",
                backDeployedPos, backRetractedPos));
        telemetry.addData("Saved Hood Pos", String.format("%.3f", shotHoodPos));
        telemetry.update();
    }
    @Override
    protected void onStart() {
        robot.drivetrain.follower.setPose(START);
        robot.turret.resetEncoder();
        robot.intake.setDeploymentFront(false);
        robot.intake.setDeploymentBack(false);
        robot.intake.intake(0.0);
        robot.transfer.setBlocker(false);
        robot.transfer.transfer(0.0);
        robot.turret.setPower(0.0);
        robot.shooter.setTargetRPM(0.0);
        schedule(follow(robot.drivetrain.follower, parkPath));
    }
    @Override
    protected void onLoop() {
        robot.intake.intake(0.0);
        robot.transfer.transfer(0.0);
        robot.turret.setPower(0.0);
        robot.shooter.setTargetRPM(0.0);
        Pose currentPose = robot.drivetrain.follower.pose();
        telemetry.addLine("=== PARK AUTO RUNNING (DT ONLY) ===");
        telemetry.addData("Robot Pose", String.format("X: %.2f | Y: %.2f | H: %.1f deg",
                currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading())));
        telemetry.addData("Follower Mode", robot.drivetrain.follower.mode());
        telemetry.addData("Intakes", "RETRACTED | Non-DT Motors: OFF");
        telemetry.update();
    }
    @Override
    protected void onStop() {
        LM1Calibrations.turretEndAutoDeg = robot.turret.getHeading();
        robot.intake.intake(0.0);
        robot.transfer.transfer(0.0);
        robot.turret.setPower(0.0);
        robot.shooter.setTargetRPM(0.0);
    }
}