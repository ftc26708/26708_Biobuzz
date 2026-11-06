package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flower {
    public final Servo flower;
    private double upPos;
    private double downPos;

    public Flower(HardwareMap hardwareMap,
                    String flowerName,
                    DcMotorEx.Direction direction,
                    double openPos,
                    double closedPos) {
        this.flower = hardwareMap.get(Servo.class, flowerName);
        this.upPos = openPos;
        this.downPos = closedPos;
    }

    public void setPositions(double openPos, double closedPos) {
        this.upPos = openPos;
        this.downPos = closedPos;
    }

    public void setDeployment(boolean up) {
        flower.setPosition(up ? upPos : downPos);
    }
}