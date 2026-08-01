package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;

public class Robot {
    public final Drivetrain drivetrain;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new Drivetrain(hardwareMap, startPose);
    }

    public void periodic() {
        drivetrain.periodic();
    }
}