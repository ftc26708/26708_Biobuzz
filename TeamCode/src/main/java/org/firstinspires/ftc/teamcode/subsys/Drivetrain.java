package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PedroConstants;

/**
 * Subsystem wrapper for Pedro Pathing that automatically persists the robot's
 * position across OpModes unless the constructor is given a new position to use.
 */
public class Drivetrain {
    public final Follower follower;

    // This constructor restores the robot's previous pose
    public Drivetrain(HardwareMap hardwareMap) {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "PC");
        odo.update();
        Pose savedPose = new Pose (
                odo.getPosX(DistanceUnit.INCH),
                odo.getPosY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.RADIANS)
        );

        follower = PedroConstants.create(hardwareMap);
        follower.setPose(savedPose);
        follower.update();
    }

    // This constructor sets a new pose
    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = PedroConstants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    // Run every loop to maintain accurate tracking
    public void periodic() {
        follower.update();
    }
}