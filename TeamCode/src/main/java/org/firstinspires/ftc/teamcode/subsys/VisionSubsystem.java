package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Subsystem wrapping FTC VisionPortal and AprilTagProcessor.
 * Detects target AprilTags and provides targeting information (bearing, distance).
 */
public class VisionSubsystem {
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private boolean targetFound = false;
    private double targetBearing = 0.0;
    private double targetDistance = 0.0;
    private int targetId = 1;

    public VisionSubsystem(HardwareMap hardwareMap) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    public void setTargetTagId(int id) {
        this.targetId = id;
    }

    public void periodic() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        targetFound = false;

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                    if (singleDetection.metadata != null && singleDetection.id == targetId) {
                        targetFound = true;
                        targetBearing = singleDetection.ftcPose.bearing;
                        targetDistance = singleDetection.ftcPose.range;
                        break;
                    }
                }
            }
        }
    }

    public boolean hasTarget() {
        return targetFound;
    }

    public double getTargetBearing() {
        return targetBearing;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
