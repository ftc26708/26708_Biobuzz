package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Dumb Turret Subsystem controlling turret rotation.
 * Provides simple, direct manual commands (setPower, turnLeft, turnRight, stop).
 */
public class TurretSubsystem {
    private final DcMotorEx turretMotor;

    private static final double DEFAULT_TURRET_SPEED = 0.5;
    private static final double TICKS_PER_DEGREE = 28.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Directly sets power to the turret motor.
     *
     * @param power motor power [-1.0, 1.0]
     */
    public void setPower(double power) {
        turretMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Rotates the turret to the left at default speed.
     */
    public void turnLeft() {
        turnLeft(DEFAULT_TURRET_SPEED);
    }

    /**
     * Rotates the turret to the left at specified speed.
     *
     * @param speed speed [0.0, 1.0]
     */
    public void turnLeft(double speed) {
        setPower(-Math.abs(speed));
    }

    /**
     * Rotates the turret to the right at default speed.
     */
    public void turnRight() {
        turnRight(DEFAULT_TURRET_SPEED);
    }

    /**
     * Rotates the turret to the right at specified speed.
     *
     * @param speed speed [0.0, 1.0]
     */
    public void turnRight(double speed) {
        setPower(Math.abs(speed));
    }

    /**
     * Stops the turret rotation.
     */
    public void stop() {
        turretMotor.setPower(0.0);
    }

    /**
     * Returns current position of the turret in encoder ticks.
     *
     * @return encoder ticks
     */
    public int getTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Returns current heading angle of the turret in degrees.
     *
     * @return turret heading in degrees
     */
    public double getHeading() {
        return getTicks() / TICKS_PER_DEGREE;
    }

    /**
     * Resets turret encoder to zero.
     */
    public void resetEncoder() {
        DcMotor.RunMode currentMode = turretMotor.getMode();
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(currentMode);
    }

    public void periodic() {
        // Additional periodic logic if needed
    }
}
