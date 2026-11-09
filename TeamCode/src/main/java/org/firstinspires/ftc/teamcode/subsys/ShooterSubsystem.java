package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem controlling the turret-mounted single-flywheel undershooter with adjustable hood.
 *
 * <p>Hardware Design (Undershooter with Adjustable Hood):
 * <ul>
 *   <li>Top Flywheel: High-inertia wheel driven by a planetary gearmotor.
 *   <li>Adjustable Hood: Servo-driven curved ramp beneath the flywheel that adjusts launch angle for arch shots.
 *   <li>Ball trajectory: Game pieces pass UNDER the top flywheel,
 *       compressed against the adjustable hood ramp to launch in a parabolic arch.
 *   <li>Feeder: Servo indexer pushing balls into the wheel/hood pinch point.
 * </ul>
 */
public class ShooterSubsystem {
    private final DcMotorEx flywheelMotor;
    private final Servo feederServo;
    private final Servo hoodServo;

    // Motor encoder resolution (ticks per revolution)
    private static final double TICKS_PER_REV = 28.0;

    // Velocity control & RPM tolerance
    private static final double RPM_TOLERANCE = 50.0;
    private static final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 14.0);

    // Feeder servo positions
    private static final double FEED_POSITION = 1.0;
    private static final double REST_POSITION = 0.0;

    // Adjustable Hood servo positions for undershooter arch trajectory
    private static final double HOOD_MIN_POS = 0.20; // Lower angle for far distance
    private static final double HOOD_MAX_POS = 0.80; // High arch angle for close distance
    private static final double MIN_DISTANCE_INCHES = 24.0;
    private static final double MAX_DISTANCE_INCHES = 120.0;

    // Distance-to-RPM curve tuned for undershooter compression trajectory
    private static final double BASE_RPM = 2000.0;
    private static final double RPM_PER_INCH = 15.0;

    private double targetRPM = 0.0;
    private double currentHoodPosition = HOOD_MAX_POS;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederServo = hardwareMap.get(Servo.class, "feeder");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        // Undershooter motor orientation
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Allow high-mass flywheel to coast to prevent mechanical strain on planetary gearbox
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure velocity control with PIDF tuning for fast recovery under compression
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);

        resetFeeder();
        setHoodPosition(HOOD_MAX_POS);
    }

    /**
     * Calculates and sets target flywheel RPM and hood angle based on target distance from Limelight.
     * Adjusts the hood angle to maintain an optimal arch launch trajectory for the undershooter.
     *
     * @param distanceInches distance to target in inches
     */
    public void setDistanceTarget(double distanceInches) {
        // Calculate flywheel speed for distance
        double calculatedRPM = BASE_RPM + (distanceInches * RPM_PER_INCH);
        setRPM(calculatedRPM);

        // Adjust hood angle for undershooter arch trajectory based on distance
        double normalizedDist = Range.clip(
                (distanceInches - MIN_DISTANCE_INCHES) / (MAX_DISTANCE_INCHES - MIN_DISTANCE_INCHES),
                0.0, 1.0
        );
        double hoodPos = HOOD_MAX_POS - normalizedDist * (HOOD_MAX_POS - HOOD_MIN_POS);
        setHoodPosition(hoodPos);
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
     * Sets the position of the adjustable hood servo.
     *
     * @param position hood servo position [0.0, 1.0]
     */
    public void setHoodPosition(double position) {
        this.currentHoodPosition = Range.clip(position, 0.0, 1.0);
        hoodServo.setPosition(currentHoodPosition);
    }

    /**
     * Returns current target hood position.
     *
     * @return hood position
     */
    public double getHoodPosition() {
        return currentHoodPosition;
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
