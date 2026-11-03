package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem controlling the turret-mounted single-flywheel undershooter.
 *
 * <p>Hardware Design (Undershooter):
 * <ul>
 *   <li>Top Flywheel: High-inertia wheel driven by a planetary gearmotor.
 *   <li>Bottom Hood: Fixed curved ramp beneath the flywheel.
 *   <li>Ball trajectory: Game pieces pass UNDER the top flywheel,
 *       compressed against the hood ramp to launch with backspin.
 *   <li>Feeder: Servo indexer pushing balls into the wheel/hood pinch point.
 * </ul>
 */
public class ShooterSubsystem {
    private final DcMotorEx flywheelMotor;
    private final Servo feederServo;

    // Motor encoder resolution (ticks per revolution)
    private static final double TICKS_PER_REV = 28.0;

    // Velocity control & RPM tolerance
    private static final double RPM_TOLERANCE = 50.0;
    private static final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 14.0);

    // Feeder servo positions
    private static final double FEED_POSITION = 1.0;
    private static final double REST_POSITION = 0.0;

    // Distance-to-RPM curve tuned for undershooter compression trajectory
    private static final double BASE_RPM = 2000.0;
    private static final double RPM_PER_INCH = 15.0;

    private double targetRPM = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederServo = hardwareMap.get(Servo.class, "feeder");

        // Undershooter motor orientation
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Allow high-mass flywheel to coast to prevent mechanical strain on planetary gearbox
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure velocity control with PIDF tuning for fast recovery under compression
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);

        resetFeeder();
    }

    /**
     * Calculates and sets target flywheel RPM based on target distance in inches.
     * Adjusted for undershooter friction and trajectory.
     *
     * @param distanceInches distance to target in inches
     */
    public void setDistanceTarget(double distanceInches) {
        double calculatedRPM = BASE_RPM + (distanceInches * RPM_PER_INCH);
        setRPM(calculatedRPM);
    }

    /**
     * Sets target RPM for the undershooter flywheel.
     *
     * @param rpm target flywheel revolutions per minute
     */
    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        if (rpm <= 0) {
            flywheelMotor.setVelocity(0);
        } else {
            double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV;
            flywheelMotor.setVelocity(ticksPerSec);
        }
    }

    /**
     * Returns the current flywheel RPM calculated from encoder velocity.
     *
     * @return current RPM
     */
    public double getCurrentRPM() {
        return (flywheelMotor.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    /**
     * Returns target flywheel RPM.
     *
     * @return target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Checks if the flywheel is within acceptable RPM tolerance for firing.
     *
     * @return true if target RPM is active and current RPM is within tolerance
     */
    public boolean isAtSpeed() {
        if (targetRPM <= 0) return false;
        return Math.abs(getCurrentRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    /**
     * Actuates the feeder servo to push a ball into the undershooter flywheel.
     */
    public void feed() {
        feederServo.setPosition(FEED_POSITION);
    }

    /**
     * Resets the feeder servo back to the resting position.
     */
    public void resetFeeder() {
        feederServo.setPosition(REST_POSITION);
    }

    /**
     * Stops the shooter flywheel and resets the feeder servo.
     */
    public void stop() {
        setRPM(0);
        resetFeeder();
    }

    public void periodic() {
        // Additional periodic telemetry or safety checks
    }
}
