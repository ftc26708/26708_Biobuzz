package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
import org.firstinspires.ftc.teamcode.subsys.Turret;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

/**
 * Composes and owns the robot's subsystems.
 *
 * <p>This class provides a single access point to the robot's hardware
 * abstractions while keeping subsystems isolated from one another.
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Turret turret;
    public final Shooter shooter;
    public final Intake intake;
    public final Transfer transfer;
    public final Flower flower;

    /**
     * Creates the robot using the drivetrain's default localization behavior.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     */
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
                0.5,
                0.0);
        flower = new Flower(hardwareMap,
                "flower",
                0.25,
                0.6);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    /**
     * Creates the robot with an explicit drivetrain starting pose.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     * @param startPose initial field pose of the robot
     */
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new Drivetrain(hardwareMap, startPose);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
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

    /**
     * Updates every subsystem once per scheduler cycle.
     */
    public void periodic() {
        drivetrain.periodic();
        shooter.periodic();
    }
}