
package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "ParkAuto")
public class ParkAuto extends BaseOpMode {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose START = poseFactory.of(7.2, 70.75, 90);
    private static final Pose PARK_POSE = poseFactory.of(12, 94, 90);

    private Path parkPath;

    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap, START);
    }

    @Override
    protected void onInit() {
        parkPath = Paths.line(START, PARK_POSE);
        schedule(stopMechanisms());
    }

    @Override
    protected void onStart() {
        robot.drivetrain.follower.setPose(START);

        schedule(stopMechanisms());
        schedule(follow(robot.drivetrain.follower, parkPath));
    }

    @Override
    protected void onLoop() {
        Pose currentPose = robot.drivetrain.follower.pose();

        telemetry.addLine("=== PARK AUTO ===");
        telemetry.addData("Pose", String.format(
                "X: %.2f | Y: %.2f | H: %.1f deg",
                currentPose.x(),
                currentPose.y(),
                Math.toDegrees(currentPose.heading())
        ));
        telemetry.addData("Follower Mode", robot.drivetrain.follower.mode());
        telemetry.update();
    }

    @Override
    protected void onStop() {
        schedule(stopMechanisms());
    }
}