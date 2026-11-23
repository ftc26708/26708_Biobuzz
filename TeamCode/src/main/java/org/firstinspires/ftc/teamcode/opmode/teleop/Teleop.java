package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cmd.ControlCommands;
import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "TeleOp")
public class Teleop extends BaseOpMode {

    private static final double STARTING_RPM = 2150.0;
    private static final double RPM_ADJUSTMENT = 25.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_RPM = 0.0;
    private static final double INITIAL_INTAKE_SPEED_THRESHOLD = 2.0;

    private static final Pose RELOC_POSE = PoseFactory.degrees().of(7.2, 70.75, 90);

    private enum ControlState {
        PAUSED, FRONT, BACK, FLOWER, TRANSFER
    }

    private double targetRPM = STARTING_RPM;

    private boolean flowerDown = false;
    private boolean zeroTurret = false;

    private boolean previousFlowerButton = false;
    private boolean previousTurretButton = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousIntakeClutch = false;
    private boolean previousRelocalize = false;

    private ControlState controlState = null;

    private Command closestGoalCommand;
    private Command zeroTurretCommand;

    @Override
    protected void onInit() {
        closestGoalCommand = TurretCommands
                .turnToClosestGoal(robot.turret, robot.drivetrain.follower)
                .until(() -> zeroTurret);

        zeroTurretCommand = TurretCommands
                .turnToStaticAngleClipped(robot.turret, 0.0)
                .until(() -> !zeroTurret);
    }

    @Override
    protected void onStart() {
        robot.shooter.setTargetRPM(targetRPM * 0.5);
        Scheduler.schedule(closestGoalCommand);
    }

    @Override
    protected void onLoop() {
        // --- GAMEPAD 1: Chassis Drive & Controls ---
        DrivePowers drivePowers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.drivetrain.follower.pose().heading()
        );

        robot.drivetrain.follower.manual(drivePowers);

        // Relocalize (Gamepad 1: Y button)
        boolean relocalize = gamepad1.y;
        if (relocalize && !previousRelocalize) {
            robot.drivetrain.follower.setPose(RELOC_POSE);
        }
        previousRelocalize = relocalize;

        // Intake direction selection (Gamepad 1: Left Trigger & Left Bumper)
        boolean intake = gamepad1.left_trigger > 0.1;
        boolean intakeClutch = gamepad1.left_bumper;
        double forwardVelocity = robot.drivetrain.follower.twist().vx;

        // Flower toggle (Gamepad 1: Right Bumper)
        boolean flowerButton = gamepad1.right_bumper;
        if (flowerButton && !previousFlowerButton) {
            flowerDown = !flowerDown;
        }
        previousFlowerButton = flowerButton;

        // Shooting / Transfer (Gamepad 1: Right Trigger)
        boolean shooting = gamepad1.right_trigger > 0.1;

        // Control State Determination
        ControlState requestedState;

        if (shooting) {
            requestedState = ControlState.TRANSFER;
            flowerDown = false;
        } else if (flowerDown) {
            requestedState = ControlState.FLOWER;
        } else if (intake) {
            if (controlState == ControlState.FRONT) {
                requestedState = (intakeClutch && !previousIntakeClutch)
                        ? ControlState.BACK : ControlState.FRONT;
            } else if (controlState == ControlState.BACK) {
                requestedState = (intakeClutch && !previousIntakeClutch)
                        ? ControlState.FRONT : ControlState.BACK;
            } else if (forwardVelocity >= INITIAL_INTAKE_SPEED_THRESHOLD) {
                requestedState = ControlState.FRONT;
            } else if (forwardVelocity <= -INITIAL_INTAKE_SPEED_THRESHOLD) {
                requestedState = ControlState.BACK;
            } else {
                requestedState = ControlState.PAUSED;
            }
        } else {
            requestedState = ControlState.PAUSED;
        }

        if (controlState != null || intake || flowerDown || shooting) {
            setControlState(requestedState);
        }

        // Turret Toggle (Gamepad 1: X button)
        boolean turretButton = gamepad1.x;
        if (turretButton && !previousTurretButton) {
            zeroTurret = !zeroTurret;
            Scheduler.schedule(
                    zeroTurret ? zeroTurretCommand : closestGoalCommand
            );
        }
        previousTurretButton = turretButton;

        // Shooter RPM Adjustment (Gamepad 1: D-Pad Up / Down)
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !previousDpadUp) {
            targetRPM = Math.min(targetRPM + RPM_ADJUSTMENT, MAX_RPM);
        }

        if (dpadDown && !previousDpadDown) {
            targetRPM = Math.max(targetRPM - RPM_ADJUSTMENT, MIN_RPM);
        }

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
        previousIntakeClutch = intakeClutch;

        // Shooter Target RPM: Full speed when shooting, half speed when idle
        robot.shooter.setTargetRPM(shooting ? targetRPM : targetRPM * 0.5);

        // Telemetry
        telemetry.addData("Forward Velocity", "%.2f in/s", forwardVelocity);
        telemetry.addData("Control State", controlState);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter Idle (Half)", !shooting);
        telemetry.addData("Turret Mode", zeroTurret ? "Fixed Zero" : "Tracking Goal");
        telemetry.addData("Turret Heading", robot.turret.getHeading());
        telemetry.addData("Flower", flowerDown);
        telemetry.update();
    }

    private void setControlState(ControlState state) {
        if (state == controlState) {
            return;
        }

        controlState = state;

        switch (state) {
            case PAUSED:
                Scheduler.schedule(ControlCommands.intakePause(
                        robot.intake, robot.transfer, robot.flower));
                break;

            case FRONT:
                Scheduler.schedule(ControlCommands.intakeFront(
                        robot.intake, robot.transfer, robot.flower));
                break;

            case BACK:
                Scheduler.schedule(ControlCommands.intakeBack(
                        robot.intake, robot.transfer, robot.flower));
                break;

            case FLOWER:
                Scheduler.schedule(ControlCommands.intakeFlower(
                        robot.intake, robot.transfer, robot.flower));
                break;

            case TRANSFER:
                Scheduler.schedule(ControlCommands.transfer(
                        robot.intake, robot.transfer, robot.flower, robot.shooter));
                break;
        }
    }
}
