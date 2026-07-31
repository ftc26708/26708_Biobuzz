package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Drivetrain {
    public final Follower follower;
    private static PoseTracker tracker;
    private static Pose offsets;

    public Drivetrain(HardwareMap hardwareMap) {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.update();

        Pose localizerPose = new Pose (
                odo.getPosX(DistanceUnit.INCH),
                odo.getPosY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.RADIANS)
        );

        follower = Constants.createFollower(hardwareMap);
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

        saveTracker();
    }

    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        saveTracker();
    }

    public void periodic() {
        follower.update();
        saveTracker();
    }

    public void saveTracker() {
        tracker = follower.getPoseTracker();
        offsets = new Pose(
                tracker.getXOffset(),
                tracker.getYOffset(),
                tracker.getHeadingOffset()
        );
    }
}