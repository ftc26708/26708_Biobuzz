package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

/**
 * Base class for all OpModes in the project.
 *
 * <p>This class manages the FTC OpMode lifecycle, configures REV Hub bulk
 * caching, initializes the robot, and runs Ivy's scheduler. Subclasses should
 * override the provided hook methods instead of the FTC lifecycle methods
 * directly.
 */
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

    /**
     * Creates the robot used by this OpMode.
     *
     * <p>Override this method to construct the robot with a different
     * constructor, such as supplying an explicit starting pose for an
     * autonomous OpMode.
     *
     * @return the robot instance used by this OpMode
     */
    protected Robot createRobot() {
        return new Robot(hardwareMap);
    }

    /**
     * Clears the manual bulk cache on every connected REV Hub.
     *
     * <p>This ensures all hardware reads during the current control cycle
     * return fresh values.
     */
    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Called once after the framework has completed initialization.
     */
    protected void onInit() {}

    /**
     * Called repeatedly while the OpMode is initialized but not yet started.
     */
    protected void onInitLoop() {}

    /**
     * Called once when PLAY is pressed.
     */
    protected void onStart() {}

    /**
     * Called once per control loop while the OpMode is active.
     */
    protected void onLoop() {}

    /**
     * Called once after STOP is pressed.
     */
    protected void onStop() {}
}