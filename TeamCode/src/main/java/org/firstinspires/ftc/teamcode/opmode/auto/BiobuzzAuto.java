package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "BiobuzzAuto", group = "Auto")
public class BiobuzzAuto extends BaseOpMode {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose START = poseFactory.of(30.8423, 8.5053, 90);
    private static final Pose POINT_1 = poseFactory.of(11.1179, 93.1762, 0);
    private static final Pose POINT_1_SEGMENT_1_START = poseFactory.of(11.1179, 93.1762, -90);
    private static final Pose POINT_1_SEGMENT_1_END = poseFactory.of(11.1179, 93.1762, 0);

    private Path path1;

    /** Builds the robot with the auto's starting pose instead of the default. */
    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap, START);
    }

    @Override
    protected void onInit() {
        path1 = Paths.line(START, POINT_1)
                .heading(Interpolator.piecewise()
                        .until(1, Interpolator.linear(POINT_1_SEGMENT_1_START, POINT_1_SEGMENT_1_END)));

        telemetry.addLine("BiobuzzAuto ready");
        telemetry.update();
    }

    @Override
    protected void onInitLoop() {
        telemetry.addLine("Waiting for start...");
        telemetry.update();
    }

    @Override
    protected void onStart() {
        schedule(follow(robot.drivetrain.follower, path1));
    }

    @Override
    protected void onLoop() {
        telemetry.addData("Pose", robot.drivetrain.follower.pose());
        telemetry.addData("Mode", robot.drivetrain.follower.mode());
        telemetry.update();
    }
}
