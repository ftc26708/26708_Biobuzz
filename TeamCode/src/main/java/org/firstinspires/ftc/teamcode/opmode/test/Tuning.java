package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Legacy Tuning class placeholder for Pedro Pathing 3.
 * For tuning in Pedro Pathing 3, use FTControl Panels with the procedures registered in PedroTuning.
 */
@Disabled
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Notice", "Pedro Pathing 3 autotuning procedures are available via FTControl Panels.");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
