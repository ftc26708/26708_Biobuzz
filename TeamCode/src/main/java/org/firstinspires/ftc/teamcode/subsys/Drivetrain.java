package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PedroConstants;

/**
 * Wraps Pedro Pathing's {@link Follower} as the robot's drivetrain subsystem.
 *
 * <p>This subsystem owns the follower instance and preserves localization across
 * OpMode transitions by caching offsets from the {@link PoseTracker}.
 * Unless an explicit starting pose is supplied, newly constructed drivetrain
 * instances restore the robot's previous field pose whenever possible.
 */
public class Drivetrain {
    public final Follower follower;
    private final PoseTracker tracker;

    /* Cached between OpModes so Pedro can reconstruct its field pose from the
     * Pinpoint's absolute position during the next initialization. */
    private static Pose offsets;

    /**
     * Creates a drivetrain that restores the robot's previous field pose when
     * cached localization offsets are available. If no offsets have been cached,
     * the follower starts from the default pose at the center of the field.
     *
     * @param hardwareMap FTC hardware map used to retrieve robot hardware
     */
    public Drivetrain(HardwareMap hardwareMap) {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "PC");
        odo.update();

        Pose localizerPose = new Pose (
                odo.getPosX(DistanceUnit.INCH),
                odo.getPosY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.RADIANS)
        );

        follower = PedroConstants.createFollower(hardwareMap);
        tracker = follower.getPoseTracker();

        if (offsets != null) {
            tracker.setXOffset(offsets.getX());
            tracker.setYOffset(offsets.getY());
            tracker.setHeadingOffset(offsets.getHeading());

            Pose pedroPose = tracker.applyOffset(localizerPose);
            follower.setStartingPose(pedroPose);
        } else {
            follower.setStartingPose(new Pose(70.75, 70.75, Math.toRadians(90)));
        }

        saveOffsets();
    }

    /**
     * Creates a drivetrain with an explicit starting pose.
     *
     * <p>This constructor bypasses localization restoration and instead initializes
     * the follower to the supplied pose.
     *
     * @param hardwareMap FTC hardware map used to retrieve robot hardware
     * @param startPose initial field pose of the robot
     */
    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        tracker = follower.getPoseTracker();
        saveOffsets();
    }

    /**
     * Updates the drivetrain and records the current localization offsets for
     * restoration by future drivetrain instances.
     */
    public void periodic() {
        follower.update();
        saveOffsets();
    }

    /**
     * Saves the follower's current localization offsets so they can be restored
     * across OpMode transitions.
     */
    private void saveOffsets() {
        offsets = new Pose(
                tracker.getXOffset(),
                tracker.getYOffset(),
                tracker.getHeadingOffset()
        );
    }
}