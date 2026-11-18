package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cmd.ControlCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

/**
 * Base class for all OpModes in the project.
 *
 * <p>This class manages the OpMode lifecycle and runs the Ivy command
 * scheduler. Note that subclasses should override the provided hook
 * methods at the bottom of the class rather than using the FTC SDK's
 * native lifecycle methods.
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
        Scheduler.schedule(stopMechanisms());
        onStop();
        Scheduler.execute();
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
     * Called once on initialization.
     */
    protected void onInit() {}

    /**
     * Called repeatedly after initialization but before start.
     */
    protected void onInitLoop() {}

    /**
     * Called once on start.
     */
    protected void onStart() {}

    /**
     * Called repeatedly after start.
     */
    protected void onLoop() {}

    /**
     * Called once on stop.
     */
    protected void onStop() {}

    protected Command stopMechanisms() {
        return parallel(
                ControlCommands.intakeStop(robot.intake, robot.transfer, robot.flower),
                instant(() -> {
                    robot.turret.setPower(0.0);
                    robot.shooter.setTargetRPM(0.0);
                }).requiring(robot.intake, robot.transfer, robot.turret, robot.shooter, robot.flower)
        );
    }
}