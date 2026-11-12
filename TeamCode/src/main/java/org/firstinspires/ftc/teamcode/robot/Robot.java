package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

/**
 * Robot container class that manages the life cycle and composition of all subsystems.
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Flower flower;

    // Initializes subsystems and restores the robot's last known field position
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap,
                "intake",
                "frontDeployer",
                "backDeployer",
                DcMotorEx.Direction.FORWARD,
                0.35,
                0.54,
                0.41,
                0.67);
        transfer = new Transfer(hardwareMap,
                "transfer",
                "blocker",
                DcMotorEx.Direction.REVERSE,
                0.4,
                0.6);
        flower = new Flower(hardwareMap,
                "flower",
                0.4,
                0.6);
    }

    // Initializes subsystems with a specific starting location on the field
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new Drivetrain(hardwareMap, startPose);
        intake = new Intake(hardwareMap,
                "intake",
                "frontDeployer",
                "backDeployer",
                DcMotorEx.Direction.REVERSE,
                0.37,
                0.67,
                0.37,
                0.67);
        transfer = new Transfer(hardwareMap,
                "transfer",
                "blocker",
                DcMotorEx.Direction.REVERSE,
                0.4,
                0.6);
        flower = new Flower(hardwareMap,
                "flower",
                0.4,
                0.6);
    }

    // Runs the periodic update loop for every subsystem
    public void periodic() {
        drivetrain.periodic();
    }
}