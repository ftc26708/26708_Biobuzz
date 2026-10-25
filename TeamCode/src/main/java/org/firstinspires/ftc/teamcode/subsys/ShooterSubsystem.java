package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem controlling the shooter flywheel motor and feeder servo.
 */
public class ShooterSubsystem {
    private final DcMotorEx flywheelMotor;
    private final Servo feederServo;

    private static final double RPM_TOLERANCE = 50.0;
    private static final double TICKS_PER_REV = 28.0;

    private double targetRPM = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederServo = hardwareMap.get(Servo.class, "feeder");

        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Calculates and sets target flywheel RPM based on target distance in inches.
     *
     * @param distanceInches distance to target AprilTag in inches
     */
    public void setDistanceTarget(double distanceInches) {
        double calculatedRPM = 2000.0 + (distanceInches * 15.0);
        setRPM(calculatedRPM);
    }

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV;
        flywheelMotor.setVelocity(ticksPerSec);
    }

    public boolean isAtSpeed() {
        if (targetRPM <= 0) return false;
        double currentRPM = (flywheelMotor.getVelocity() / TICKS_PER_REV) * 60.0;
        return Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;
    }

    public void feed() {
        feederServo.setPosition(1.0);
    }

    public void resetFeeder() {
        feederServo.setPosition(0.0);
    }

    public void stop() {
        setRPM(0);
        resetFeeder();
    }

    public void periodic() {
        // Additional periodic logic
    }
}
