package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the turret rotation motor.
 * Uses PID control to track target bearing.
 */
public class TurretSubsystem {
    private final DcMotorEx turretMotor;

    private static final double ALIGNMENT_TOLERANCE_DEG = 1.5;
    private static final double KP = 0.02;

    private double currentError = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drives the turret to center on the AprilTag based on bearing error (degrees).
     *
     * @param bearingErrorDeg horizontal bearing offset from target
     */
    public void trackTarget(double bearingErrorDeg) {
        this.currentError = bearingErrorDeg;
        double power = KP * bearingErrorDeg;
        power = Math.max(-0.8, Math.min(0.8, power));
        turretMotor.setPower(power);
    }

    public void stop() {
        currentError = 0.0;
        turretMotor.setPower(0);
    }

    public boolean isAligned() {
        return Math.abs(currentError) <= ALIGNMENT_TOLERANCE_DEG;
    }

    public void periodic() {
        // Additional periodic logic (e.g. limit switch checks, soft limits)
    }
}
