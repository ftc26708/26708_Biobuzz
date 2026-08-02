package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class BaseOpMode extends OpMode {
    protected Robot robot;

    @Override
    public final void init() {
        Scheduler.reset();
        robot = createRobot();
        Scheduler.schedule(infinite(robot::periodic));
        onInit();
        Scheduler.execute();
    }

    @Override
    public final void init_loop() {
        onInitLoop();
        Scheduler.execute();
    }

    @Override
    public final void start() {
        onStart();
        Scheduler.execute();
    }

    @Override
    public final void loop() {
        onLoop();
        Scheduler.execute();
    }

    @Override
    public final void stop() {
        onStop();
    }

    protected Robot createRobot() {
        return new Robot(hardwareMap);
    }

    protected void onInit() {}
    protected void onInitLoop() {}
    protected void onStart() {}
    protected void onLoop() {}
    protected void onStop() {}
}