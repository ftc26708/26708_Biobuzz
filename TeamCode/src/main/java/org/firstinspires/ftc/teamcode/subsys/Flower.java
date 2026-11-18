package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Creates private variable upPos and downPos
 */
public class Flower {
    public final Servo flower;
    private double upPos;
    private double downPos;

    /**
     * Restates up position and down position
     * Sets up the servo for flower
     * @param hardwareMap This maps the device to the code
     * @param flowerName A string variable used for stating the flower
     * @param upPos The variable for up position; when the flower is not deployed
     * @param downPos The variable for down position; when the flower is fully deployed ready to intake from flower
     */
    public Flower(HardwareMap hardwareMap,
                  String flowerName,
                  double upPos,
                  double downPos) {
        this.flower = hardwareMap.get(Servo.class, flowerName);
        this.upPos = upPos;
        this.downPos = downPos;
    }

    /**
     * Maps the servo for the different positions for flower
     * @param upPos
     * @param downPos
     */
    public void setPositions(double upPos, double downPos) {
        this.upPos = upPos;
        this.downPos = downPos;
    }

    /**
     * Sets the position of the deployment as a boolean (true or false)
     * @param down
     */
    public void setDeployment(boolean down) {
        flower.setPosition(down ? downPos : upPos);
    }
}