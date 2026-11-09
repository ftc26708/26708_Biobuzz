package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower createFollower(HardwareMap hardwareMap) {
        return PedroConstants.create(hardwareMap);
    }
}
