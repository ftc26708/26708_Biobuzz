package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public final DcMotorEx transfer;
    public final Servo blocker;
    private double openPos;
    private double closedPos;

    /**
     * Makes open position and closed position variables
     * Makes the servo blocker and transfer
     * @param hardwareMap Maps the Servo and Dc Motor (for this code) to the code.
     * @param transferName The set name for the transfer
     * @param blockerName The set name for the blocker
     * @param direction The direction for the servo blocker, either open or closed
     * @param openPos The position were the servo blocked open
     * @param closedPos The position were the servo blocker is closed
     */
    public Transfer(HardwareMap hardwareMap,
                    String transferName,
                    String blockerName,
                    DcMotorEx.Direction direction,
                    double openPos,
                    double closedPos) {
        this.transfer = hardwareMap.get(DcMotorEx.class, transferName);
        this.blocker = hardwareMap.get(Servo.class, blockerName);
        this.transfer.setDirection(direction);
        this.openPos = openPos;
        this.closedPos = closedPos;
    }

    /**
     * Restates the open position and closed position and creates the string variable of transfer and blocker
     * Maps the DC Motor and the Servo for the transfer and the servo blocker
     * Sets the transfer direction
     * Sets variables for the open and closed positions of the servo blocker
     * @param direction
     */
    public void setDirection(DcMotorEx.Direction direction) {
        transfer.setDirection(direction);
    }

    /**
     * Sets the transfer direction
     * @param openPos
     * @param closedPos
     */
    public void setPositions(double openPos, double closedPos) {
        this.openPos = openPos;
        this.closedPos = closedPos;
    }

    /**
     * Sets the open position and closed position to their corresponding varibales
     * @param power
     */
    public void transfer(double power) {
        transfer.setPower(power);
    }

    /**
     * Sets the variable for the power for the transfer
     * @param open
     */
    public void setBlockerOpen(boolean open) {
        blocker.setPosition(open ? openPos : closedPos);
    }
}
/**
 * Sets the variable for the Servo blocker
 */