package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsys.Drivetrain;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
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

    /**
     * Creates the robot using the drivetrain's default localization behavior.
     *
     * @param hardwareMap FTC hardware map used to initialize robot hardware
     */
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
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
    }

    /**
     * Updates every subsystem once per scheduler cycle.
     */
    public void periodic() {
        drivetrain.periodic();
        shooter.periodic();
    }
}
