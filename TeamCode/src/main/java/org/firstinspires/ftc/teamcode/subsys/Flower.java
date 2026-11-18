package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flower {
    public final Servo flower;
    private double upPos;
    private double downPos;

    public Flower(HardwareMap hardwareMap,
                  String flowerName,
                  double upPos,
                  double downPos) {
        this.flower = hardwareMap.get(Servo.class, flowerName);
        this.upPos = upPos;
        this.downPos = downPos;
    }

    public void setPositions(double upPos, double downPos) {
        this.upPos = upPos;
        this.downPos = downPos;
    }

    public void setDeployment(boolean down) {
        flower.setPosition(down ? downPos : upPos);
    }
}