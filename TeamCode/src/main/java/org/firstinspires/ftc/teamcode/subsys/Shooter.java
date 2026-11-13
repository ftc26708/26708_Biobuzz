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
    public final DcMotorEx shooter;
    public final Servo hood;

    // Calculated hardware constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double HOOD_BASE_ANGLE = 65.0; // Degrees from horizontal plane
    private static final double DEGREES_PER_HUNDREDTH = (300.0 / 100.0) * (32.0 / 44.0);

    // Tunable hood base servo position
    public double hoodBasePosition;

    // State variables
    private double targetRPM = 0.0;

    public Shooter(
            HardwareMap hardwareMap,
            String flywheelName,
            String hoodName,
            DcMotorSimple.Direction direction,
            double hoodBasePosition) {
        this.shooter = hardwareMap.get(DcMotorEx.class, flywheelName);
        this.hood = hardwareMap.get(Servo.class, hoodName);
        this.hoodBasePosition = hoodBasePosition;

        this.shooter.setDirection(direction);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setHoodPosition(hoodBasePosition);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        shooter.setDirection(direction);
    }

    public void setHoodBasePosition(double hoodBasePosition) {
        this.hoodBasePosition = hoodBasePosition;
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
     * Converts hood servo position to angle from horizontal.
     *
     * @param position hood servo position [0.0, 1.0]
     * @return hood angle from horizontal
     */
    public double positionToAngle(double position) {
        return HOOD_BASE_ANGLE + (position - hoodBasePosition) * 100.0 * DEGREES_PER_HUNDREDTH;
    }

    /**
     * Converts angle from horizontal to hood servo position .
     *
     * @param angle hood servo position [0.0, 1.0]
     * @return hood angle from horizontal
     */
    public double angleToPosition(double angle) {
        return (angle - HOOD_BASE_ANGLE) / 100.0 / DEGREES_PER_HUNDREDTH + hoodBasePosition;
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
        this.targetRPM = rpm;
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