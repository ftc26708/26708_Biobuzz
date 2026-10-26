package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PedroConstants;

/**
 * Wraps Pedro Pathing's {@link Follower} as the robot's drivetrain subsystem.
 *
 * <p>This subsystem owns the follower instance and can preserve localization
 * across OpMode transitions by polling the odometry setup directly.
 * Unless an explicit starting pose is supplied, newly constructed drivetrain
 * instances restore the robot's previous field pose.
 */
public class Drivetrain {
    public final Follower follower;

    /**
     * Creates a drivetrain that restores the robot's previous field pose by
     * polling the odometry setup directly prior to initializing the follower.
     *
     * @param hardwareMap FTC hardware map used to retrieve robot hardware
     */
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

    /**
     * Creates a drivetrain with an explicit starting pose.
     *
     * <p>This constructor bypasses localization restoration and instead
     * initializes the follower to the supplied pose.
     *
     * @param hardwareMap FTC hardware map used to retrieve robot hardware
     * @param startPose initial field pose of the robot
     */
    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = PedroConstants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    /**
     * Updates the drivetrain and records the current localization offsets
     * for restoration by future drivetrain instances.
     */
    public void periodic() {
        follower.update();
    }
}