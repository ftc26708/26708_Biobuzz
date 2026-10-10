package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;

/**
 * Robot container class that manages the life cycle and composition of all subsystems.
 */
public class Robot {
    public final Drivetrain drivetrain;

    // Initializes subsystems and restores the robot's last known field position
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }

    // Initializes subsystems with a specific starting location on the field
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new Drivetrain(hardwareMap, startPose);
    }

    // Runs the periodic update loop for every subsystem
    public void periodic() {
        drivetrain.periodic();
    }
}