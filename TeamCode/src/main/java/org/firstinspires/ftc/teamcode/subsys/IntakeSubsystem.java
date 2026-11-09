package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the robot's roller intake motor.
 */
public class IntakeSubsystem {
    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the motor power for the intake.
     *
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Runs the intake at full forward power.
     */
    public void intake() {
        setPower(1.0);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        setPower(0);
    }

    public void periodic() {
        // Additional periodic intake logic
    }
}
