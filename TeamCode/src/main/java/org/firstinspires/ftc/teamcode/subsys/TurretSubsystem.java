package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem controlling the turret rotation motor.
 * Uses PID control to track target bearing.
 */
public class TurretSubsystem {
    private final DcMotorEx turretMotor;

    private static final double ALIGNMENT_TOLERANCE_DEG = 1.5;
    private static final double KP = 0.02;
    private static final double TICKS_PER_DEGREE = 28.0;

    private double currentError = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Returns the current heading angle of the turret in degrees.
     *
     * @return turret heading in degrees
     */
    public double getHeading() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Drives the turret to center on the AprilTag based on bearing error (degrees).
     *
     * @param bearingErrorDeg horizontal bearing offset from target
     */
    public void trackTarget(double bearingErrorDeg) {
        this.currentError = bearingErrorDeg;
        double power = Range.clip(KP * bearingErrorDeg, -0.8, 0.8);
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
