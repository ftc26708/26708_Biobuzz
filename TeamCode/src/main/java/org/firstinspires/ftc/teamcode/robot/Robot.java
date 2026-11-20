package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
import org.firstinspires.ftc.teamcode.subsys.Transfer;
import org.firstinspires.ftc.teamcode.subsys.Turret;

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
     * Master constructor: all other overloads chain into this one.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     * @param startPose   initial field pose of the robot, or null for continued localization from previous OpMode
     */
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = (startPose != null)
                ? new Drivetrain(hardwareMap, startPose)
                : new Drivetrain(hardwareMap);

        intake = new Intake(hardwareMap, "intake", "frontDeployer", "backDeployer",
                DcMotorEx.Direction.FORWARD, 0.36, 0.51, 0.46, 0.61);
        transfer = new Transfer(hardwareMap, "transfer", "blocker",
                DcMotorEx.Direction.FORWARD, 0.55, 0.7);
        flower = new Flower(hardwareMap, "flower", 0.2, 0.6);
        turret = new Turret(hardwareMap, "turret", DcMotorEx.Direction.REVERSE);
        shooter = new Shooter(hardwareMap, "shooter", "hood", DcMotorEx.Direction.FORWARD, 0.280);
    }

    /**
     * Creates the robot using the drivetrain's continued localization from previous OpMode.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Updates every subsystem once per scheduler cycle.
     */
    public void periodic() {
        drivetrain.periodic();
        shooter.periodic();
    }
}