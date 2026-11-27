package org.firstinspires.ftc.teamcode.cmd;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class ControlCommands {
    private ControlCommands() {
    }

    public static Command intakeFront(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(true);
            intake.setDeploymentBack(false);
            intake.intake(1);
            transfer.transfer(-0.75);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }

    public static Command intakeBack(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(false);
            intake.setDeploymentBack(true);
            intake.intake(1);
            transfer.transfer(1);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }

    public static Command intakeFlower(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(false);
            intake.setDeploymentBack(true);
            intake.intake(1);
            transfer.transfer(1);
            transfer.setBlockerOpen(false);
            flower.setDeployment(true);
        }).requiring(intake, transfer, flower);
    }

    public static Command transfer(Intake intake, Transfer transfer, Flower flower) {
        return transfer(intake, transfer, flower, null);
    }

    public static Command transfer(Intake intake, Transfer transfer, Flower flower, Shooter shooter) {
        Command waitCommand = (shooter != null) ? waitUntil(shooter::isAtSpeed) : instant(() -> {});
        return sequential(
                waitCommand,
                instant(() -> {
                    transfer.setBlockerOpen(true);
                    transfer.transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    transfer.transfer.setTargetPosition(0);
                    transfer.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.transfer(1);
                    transfer.transfer.setTargetPosition(-20);
                }),
                waitMs(300),
                instant(() -> {
                    transfer.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setDeploymentFront(false);
                    intake.setDeploymentBack(false);
                    intake.intake(1);
                    transfer.transfer(1);
                    transfer.setBlockerOpen(true);
                    flower.setDeployment(false);
                })).requiring(intake, transfer, flower)
                   .setEnd((endCondition) ->
                           transfer.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                   );
    }

    public static Command passThrough(Intake intake, Transfer transfer, Flower flower) {
        return sequential(
                instant(() -> {
                    transfer.setBlockerOpen(true);
                    transfer.transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    transfer.transfer.setTargetPosition(0);
                    transfer.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    transfer.transfer(1);
                    transfer.transfer.setTargetPosition(-20);
                }),
                waitMs(300),
                instant(() -> {
                    transfer.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setDeploymentFront(true);
                    intake.setDeploymentBack(true);
                    intake.intake(1);
                    transfer.transfer(1);
                    transfer.setBlockerOpen(true);
                    flower.setDeployment(false);
                })).requiring(intake, transfer, flower)
                .setEnd((endCondition) ->
                    transfer.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                );
    }

    public static Command intakePause(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(false);
            intake.setDeploymentBack(false);
            intake.intake(0);
            transfer.transfer(0);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }

    public static Command intakeDecide(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(true);
            intake.setDeploymentBack(true);
            intake.intake(1);
            transfer.transfer(0);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }
    public static Command intakeStop(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(false);
            intake.setDeploymentBack(false);
            intake.frontDeployer.setPosition(0.56);
            intake.backDeployer.setPosition(0.71);
            intake.intake(0);
            transfer.transfer(0);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }
}