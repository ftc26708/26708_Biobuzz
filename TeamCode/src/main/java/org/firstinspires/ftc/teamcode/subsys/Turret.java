package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Dumb Turret Subsystem controlling turret rotation.
 * Provides simple, direct manual commands and PedroPathing Ivy command factories.
 */
public class Turret {
    public final DcMotorEx turret;
    private static final double TICKS_PER_DEGREE = 133.0 / 66.0;

    public Turret(HardwareMap hardwareMap, String motorName, DcMotorEx.Direction direction) {
        this.turret = hardwareMap.get(DcMotorEx.class, motorName);
        this.turret.setDirection(direction);
        this.turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setDirection(DcMotorEx.Direction direction) {
        turret.setDirection(direction);
    }

    /**
     * Directly sets power to the turret motor.
     *
     * @param power motor power [-1.0, 1.0]
     */
    public void setPower(double power) {
        turret.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Returns current position of the turret in encoder ticks.
     *
     * @return encoder ticks
     */
    public int getTicks() {
        return turret.getCurrentPosition();
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
        this.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
