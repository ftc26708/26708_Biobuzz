package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public final DcMotorEx transfer;
    public final Servo blocker;
    private double openPos;
    private double closedPos;

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

    public void setDirection(DcMotorEx.Direction direction) {
        transfer.setDirection(direction);
    }

    public void setPositions(double openPos, double closedPos) {
        this.openPos = openPos;
        this.closedPos = closedPos;
    }

    public void transfer(double power) {
        transfer.setPower(power);
    }

    public void setBlocker(boolean open) {
        blocker.setPosition(open ? openPos : closedPos);
    }
}
