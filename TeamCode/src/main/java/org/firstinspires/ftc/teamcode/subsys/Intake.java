package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotorEx intake;
    public final Servo frontDeployer;
    public final Servo backDeployer;
    public double frontDeployedPos;
    public double frontRetractedPos;
    public double backDeployedPos;
    public double backRetractedPos;

    public Intake(HardwareMap hardwareMap,
                  String intakeName,
                  String frontDeployerName,
                  String backDeployerName,
                  DcMotorEx.Direction direction,
                  double frontDeployedPos,
                  double frontRetractedPos,
                  double backDeployedPos,
                  double backRetractedPos) {
        this.intake = hardwareMap.get(DcMotorEx.class, intakeName);
        this.frontDeployer = hardwareMap.get(Servo.class, frontDeployerName);
        this.backDeployer = hardwareMap.get(Servo.class, backDeployerName);
        this.intake.setDirection(direction);
        this.frontDeployedPos = frontDeployedPos;
        this.frontRetractedPos = frontRetractedPos;
        this.backDeployedPos = backDeployedPos;
        this.backRetractedPos = backRetractedPos;
    }

    public void setDirection(DcMotorEx.Direction direction) {
        intake.setDirection(direction);
    }

    public void setFrontPositions(double frontDeployedPos, double frontRetractedPos) {
        this.frontDeployedPos = frontDeployedPos;
        this.frontRetractedPos = frontRetractedPos;
    }

    public void setBackPositions(double backDeployedPos, double backRetractedPos) {
        this.backDeployedPos = backDeployedPos;
        this.backRetractedPos = backRetractedPos;
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void setDeploymentFront(boolean deployed) {
        frontDeployer.setPosition(deployed ? frontDeployedPos : frontRetractedPos);
    }

    public void setDeploymentBack(boolean deployed) {
        backDeployer.setPosition(deployed ? backDeployedPos : backRetractedPos);
    }
}