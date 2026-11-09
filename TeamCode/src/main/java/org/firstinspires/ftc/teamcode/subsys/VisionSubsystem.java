package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem wrapping Limelight 3A vision sensor.
 * Provides targeting information (horizontal bearing offset, distance) for turret auto-aim
 * and undershooter hood angle adjustment.
 */
public class VisionSubsystem {
    private final Limelight3A limelight;

    private boolean targetFound = false;
    private double targetBearing = 0.0;
    private double targetDistance = 0.0;

    // Limelight mounting geometry for trigonometric distance calculation
    private static final double LIMELIGHT_HEIGHT_INCHES = 15.0;  // Height of Limelight lens above floor
    private static final double TARGET_HEIGHT_INCHES = 50.0;     // Height of target center above floor
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 20.0; // Mounting pitch angle in degrees

    public VisionSubsystem(HardwareMap hardwareMap) {
        Limelight3A limelightInstance;
        try {
            limelightInstance = hardwareMap.get(Limelight3A.class, "limelight");
            limelightInstance.pipelineSwitch(0);
            limelightInstance.start();
        } catch (Exception e) {
            limelightInstance = null;
        }
        this.limelight = limelightInstance;
    }

    public void periodic() {
        if (limelight == null) {
            targetFound = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            targetFound = true;
            targetBearing = result.getTx(); // Horizontal offset angle (degrees) to target

            // Calculate distance using Limelight vertical angle (ty)
            double ty = result.getTy();
            double totalAngleRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
            double heightDiff = TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;

            if (Math.abs(Math.tan(totalAngleRad)) > 1e-4) {
                targetDistance = heightDiff / Math.tan(totalAngleRad);
            } else {
                targetDistance = 48.0; // Fallback distance in inches
            }
        } else {
            targetFound = false;
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
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {}
        }
    }
}
