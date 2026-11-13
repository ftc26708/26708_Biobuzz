package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Dumb Turret Subsystem controlling turret rotation.
 * Provides simple, direct manual commands and PedroPathing Ivy command factories.
 */
public class Turret {
    public final DcMotorEx turretMotor;
    private static final double TICKS_PER_DEGREE = 133.0 / 66.0;

    public Turret(HardwareMap hardwareMap, String motorName, DcMotorEx.Direction direction) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.turretMotor.setDirection(direction);
        this.turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDirection(DcMotorEx.Direction direction) {
        turretMotor.setDirection(direction);
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
        DcMotorEx.RunMode currentMode = turretMotor.getMode();
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(currentMode);
    }
}
