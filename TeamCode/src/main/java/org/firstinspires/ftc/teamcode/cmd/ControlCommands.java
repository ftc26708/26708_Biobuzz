package org.firstinspires.ftc.teamcode.cmd;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class ControlCommands {
    private ControlCommands() {
    }

    public static Command intakeFront(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.setDeploymentFront(true);
            intake.setDeploymentBack(false);
            intake.intake(1);
            transfer.transfer(-0.5);
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
        return instant(() -> {
            intake.setDeploymentFront(false);
            intake.setDeploymentBack(false);
            intake.intake(1);
            transfer.transfer(1);
            transfer.setBlockerOpen(true);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
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

    public static Command intakeStop(Intake intake, Transfer transfer, Flower flower) {
        return instant(() -> {
            intake.frontDeployer.setPosition(0.56);
            intake.backDeployer.setPosition(0.71);
            intake.intake(0);
            transfer.transfer(0);
            transfer.setBlockerOpen(false);
            flower.setDeployment(false);
        }).requiring(intake, transfer, flower);
    }
}