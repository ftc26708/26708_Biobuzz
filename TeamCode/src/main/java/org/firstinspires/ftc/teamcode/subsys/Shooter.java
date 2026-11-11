package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem controlling the turret-mounted single-flywheel undershooter with adjustable hood.
 *
 * <p>Hardware Design (Undershooter with Adjustable Hood):
 * <ul>
 *   <li>Top Flywheel: High-inertia wheel driven by a 6000 RPM goBILDA Yellowjacket motor.
 *   <li>Adjustable Hood: Servo-driven curved ramp beneath the flywheel that adjusts launch angle.
 *   <li>Ball trajectory: Game pieces pass UNDER the top flywheel compressed against the adjustable
 *   hood to then be launch in a parabolic arc at the angle of the end of the hood.
 * </ul>
 */
public class Shooter {
    private final DcMotorEx shooter;
    private final Servo hood;

    // Motor encoder resolution (ticks per revolution)
    private static final double TICKS_PER_REV = 28.0;

    // Velocity control & RPM tolerance
    private static final double RPM_TOLERANCE = 50.0;

    // Adjustable Hood servo positions for undershooter arch trajectory
    private static final double HOOD_BASE_POSITION = 0.20; // TODO: find actual value for hood servo base position
    private static final double HOOD_BASE_ANGLE = 65.0; // Degrees from horizontal plane
    private static final double DEGREES_PER_HUNDREDTH = (300.0/100.0)*(32.0/44.0);
    private static final double MIN_DISTANCE_INCHES = 24.0;
    private static final double MAX_DISTANCE_INCHES = 120.0;

    // State variables
    private double targetRPM = 0.0;
    private double actualRPM = 0.0;
    private double targetHoodAngle = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");

        // TODO: find actual motor direction
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Allow high-mass flywheel to coast to prevent wasted energy and excess stress
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Not using inbuilt PIDF but rather Bang-Bang controller in periodic()
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        setHoodPosition(HOOD_BASE_POSITION);
    }

    /**
     * Sets the target angle of the hood.
     * @param angle hood angle from horizontal
     */
    public void setHoodAngle(double angle) {
        setHoodPosition(angleToPosition(angle));
    }

    /**
     * Sets the position of the adjustable hood servo.
     *
     * @param position hood servo position [0.0, 1.0]
     */
    public void setHoodPosition(double position) {
        hood.setPosition(Range.clip(position, 0.0, 1.0));
    }

    /**
     * Returns current target hood angle.
     *
     * @return hood angle from horizontal
     */
    public double getHoodAngle() {
        return positionToAngle(getHoodPosition());
    }

    /**
     * Returns current hood position.
     *
     * @return hood servo position [0.0, 1.0]
     */
    public double getHoodPosition() {
        return hood.getPosition();
    }

    /**
     *
     * @param position hood servo position [0.0, 1.0]
     * @return hood angle from horizontal
     */
    public double positionToAngle(double position) {
        return HOOD_BASE_ANGLE + (position - HOOD_BASE_POSITION) * 100 * DEGREES_PER_HUNDREDTH;
    }

    public double angleToPosition(double angle) {
        return (angle - HOOD_BASE_ANGLE) / 100 / DEGREES_PER_HUNDREDTH + HOOD_BASE_POSITION;
    }

    /**
     * Returns the current flywheel RPM calculated from encoder velocity.
     *
     * @return current RPM
     */
    public double getCurrentRPM() {
        return (shooter.getVelocity() / TICKS_PER_REV) * 60.0;
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
     * Sets target RPM for the shooter motor.
     *
     * @param rpm target revolutions per minute
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;;
    }

    /**
     * Checks if the flywheel is within acceptable RPM tolerance for firing.
     *
     * @return true if target RPM is active and current RPM is within tolerance
     */
    public boolean isAtSpeed() {
        return getCurrentRPM() > targetRPM;
    }

    public void periodic() {
        // Bang-Bang controller run every loop
        if (isAtSpeed() || targetRPM <= 0.0) {
            shooter.setPower(0.0);
        } else {
            shooter.setPower(1.0);
        }
    }
}