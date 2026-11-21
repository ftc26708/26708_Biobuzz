
package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cmd.ControlCommands;
import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "ParkAuto")
public class OneTipAuto extends BaseOpMode {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose START = poseFactory.of(55.1, 8.1, 180);
    private static final Pose PARK_POSE = poseFactory.of(11.53, 88.89, 180);

    private static final double SHOOT_RPM = 3500.0;
    private static final double SPIN_UP_MS = 1000.0;
    private static final double SHOOT_TIME_MS = 2500.0;

    private Path parkPath;

    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap, START);
    }

    @Override
    protected void onInit() {
        parkPath = Paths.line(START, PARK_POSE).heading(Interpolator.constant(START.heading()));
        schedule(stopMechanisms());
    }

    @Override
    protected void onStart() {
        robot.drivetrain.follower.setPose(START);

        Command shootThenPark = sequential(
                // 1. Spin up shooter flywheel
                instant(() -> robot.shooter.setTargetRPM(SHOOT_RPM)),

                // 2. Wait for flywheel to reach speed
                waitMs(SPIN_UP_MS),

                // 3. Run transfer & intake to shoot elements
                ControlCommands.transfer(robot.intake, robot.transfer, robot.flower),

                // 4. Hold transfer/intake to shoot for specified duration
                waitMs(SHOOT_TIME_MS),

                // 5. Stop intake, transfer, and shooter mechanisms
                stopMechanisms(),

                // 6. Follow park path
                follow(robot.drivetrain.follower, parkPath)
        );

        // Run shooting & park sequence while continuously aiming turret at closest goal
        schedule(deadline(shootThenPark, TurretCommands.turnToClosestGoal(robot.turret, robot.drivetrain.follower)));
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
        telemetry.addData("Shooter RPM", String.format("%.0f / %.0f", robot.shooter.getCurrentRPM(), robot.shooter.getTargetRPM()));
        telemetry.addData("Turret Heading", "%.1f deg", robot.turret.getHeading());
        telemetry.addData("Follower Mode", robot.drivetrain.follower.mode());
        telemetry.update();
    }

    @Override
    protected void onStop() {
        schedule(stopMechanisms());
    }
}