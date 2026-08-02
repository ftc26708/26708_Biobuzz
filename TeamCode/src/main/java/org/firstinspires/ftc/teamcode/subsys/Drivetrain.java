package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PedroConstants;

public class Drivetrain {
    public final Follower follower;
    private final PoseTracker tracker;
    private static Pose offsets;

    public Drivetrain(HardwareMap hardwareMap) {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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

    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        tracker = follower.getPoseTracker();
        saveOffsets();
    }

    public void periodic() {
        follower.update();
        saveOffsets();
    }

    private void saveOffsets() {
        offsets = new Pose(
                tracker.getXOffset(),
                tracker.getYOffset(),
                tracker.getHeadingOffset()
        );
    }
}