package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;

/**
 * Composes and owns the robot's subsystems.
 *
 * <p>This class provides a single access point to the robot's hardware
 * abstractions while keeping subsystems isolated from one another.
 */
public class Robot {
    public final Drivetrain drivetrain;

    /**
     * Creates the robot using the drivetrain's default localization behavior.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     */
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }

    /**
     * Creates the robot with an explicit drivetrain starting pose.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     * @param startPose initial field pose of the robot
     */
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new Drivetrain(hardwareMap, startPose);
    }

    /**
     * Updates every subsystem once per scheduler cycle.
     */
    public void periodic() {
        drivetrain.periodic();
    }
}