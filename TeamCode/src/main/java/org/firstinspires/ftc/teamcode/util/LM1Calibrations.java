package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Static runtime calibration data persisted across Auto -> TeleOp during LM1.
 */
public final class LM1Calibrations {
    // Slipped back intake positions
    public static boolean intakeCalibrated = false;
    public static double backDeployedPos = 0.0;
    public static double backRetractedPos = 0.0;

    // Turret polarity, CCW direction & ending Auto angle
    public static boolean turretCalibrated = false;
    public static DcMotorSimple.Direction turretDirection = null;
    public static double turretKp = 0.02;
    public static double turretEndAutoDeg = 0.0;

    // Shooter hood servo position (constant shot RPM is fixed in TeleOp)
    public static boolean hoodCalibrated = false;
    public static double shotHoodPos = 0.35;

    private LM1Calibrations() { }
}