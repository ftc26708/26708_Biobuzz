package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

public class BaseOpMode extends OpMode {
    protected Robot robot;
    private List<LynxModule> hubs;

    @Override
    public final void init() {
        Scheduler.reset();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = createRobot();
        Scheduler.schedule(infinite(robot::periodic));

        clearBulkCache();
        onInit();
        Scheduler.execute();
    }

    @Override
    public final void init_loop() {
        clearBulkCache();
        onInitLoop();
        Scheduler.execute();
    }

    @Override
    public final void start() {
        clearBulkCache();
        onStart();
        Scheduler.execute();
    }

    @Override
    public final void loop() {
        clearBulkCache();
        onLoop();
        Scheduler.execute();
    }

    @Override
    public final void stop() {
        clearBulkCache();
        onStop();
    }

    protected Robot createRobot() {
        return new Robot(hardwareMap);
    }

    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    protected void onInit() {}
    protected void onInitLoop() {}
    protected void onStart() {}
    protected void onLoop() {}
    protected void onStop() {}
}