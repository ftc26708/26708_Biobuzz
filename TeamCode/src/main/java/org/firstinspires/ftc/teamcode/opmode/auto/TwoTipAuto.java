package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cmd.ControlCommands;
import org.firstinspires.ftc.teamcode.cmd.TurretCommands;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Locale;

@Autonomous(name = "Two Tip Auto")
public class TwoTipAuto extends BaseOpMode {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose START = poseFactory.of(56.2792, 8.2684, 180);

    private final Pose movetointakeballs = poseFactory.of(7.9911, 12.5851, 90);
    private final Pose movetointakeballsControl1 = poseFactory.of(31.484, 23.7518, 0);

    private final Pose moveToShootPositStart = poseFactory.of(7.9911, 12.5851, 90);
    private final Pose moveToShootPosit = poseFactory.of(63.2976, 133.5006, 180);
    private final Pose moveToShootPositControl1 = poseFactory.of(29.8261, 104.361, 0);

    private final Pose moveToIntakeMoreBallsStart = poseFactory.of(63.2976, 133.5006, 180);
    private final Pose moveToIntakeMoreBalls = poseFactory.of(11.4035, 107.593, 0);
    private final Pose moveToIntakeMoreBallsControl1 = poseFactory.of(53.1285, 95.248, 0);

    private final Pose moveToShootAgainStart = poseFactory.of(11.4035, 107.593, 0);
    private final Pose moveToShootAgain = poseFactory.of(62.966, 133.0572, 180);
    private final Pose moveToShootAgainControl1 = poseFactory.of(55.4355, 90.0791, 0);

    private final Pose parkEndStart = poseFactory.of(62.966, 133.0572, 180);
    private final Pose parkEnd = poseFactory.of(15.1186, 90.3326, 180);
    private final Pose parkEndControl1 = poseFactory.of(60.5118, 98.1446, 0);

    private static final double SHOOT_RPM = 2160.0;
    private static final double INTAKE_TIME_MS = 2000.0;
    private static final double SPIN_UP_MS = 4000.0;
    private static final double SHOOT_TIME_MS = 2500.0;

    private Path movetointakeballsPath;
    private Path moveToShootPositPath;
    private Path moveToIntakeMoreBallsPath;
    private Path moveToShootAgainPath;
    private Path parkEndPath;

    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap, START);
    }

    @Override
    protected void onInit() {
        movetointakeballsPath = movetointakeballs();
        moveToShootPositPath = moveToShootPosit();
        moveToIntakeMoreBallsPath = moveToIntakeMoreBalls();
        moveToShootAgainPath = moveToShootAgain();
        parkEndPath = parkEnd();

        schedule(stopMechanisms());
    }

    @Override
    protected void onStart() {
        robot.drivetrain.follower.setPose(START);

        Command twoTipSequence = sequential(
                // 1. Move to intake balls & run intake for 2000 ms
                parallel(
                        follow(robot.drivetrain.follower, movetointakeballsPath),
                        sequential(
                                ControlCommands.intakeFront(robot.intake, robot.transfer, robot.flower),
                                waitMs(INTAKE_TIME_MS)
                        )
                ),

                // 2. Move to shoot position, spin up shooter flywheel, then turn on intake and transfer to shoot
                parallel(
                        follow(robot.drivetrain.follower, moveToShootPositPath),
                        instant(() -> robot.shooter.setTargetRPM(SHOOT_RPM))
                ),
                waitMs(SPIN_UP_MS),
                ControlCommands.transfer(robot.intake, robot.transfer, robot.flower, robot.shooter),
                waitMs(SHOOT_TIME_MS),
                ControlCommands.intakePause(robot.intake, robot.transfer, robot.flower),

                // 3. Move to intake more balls & run intake for 2000 ms
                parallel(
                        follow(robot.drivetrain.follower, moveToIntakeMoreBallsPath),
                        sequential(
                                ControlCommands.intakeFront(robot.intake, robot.transfer, robot.flower),
                                waitMs(INTAKE_TIME_MS)
                        )
                ),

                // 4. Move to shoot position again, spin up shooter, then turn on intake and transfer to shoot
                parallel(
                        follow(robot.drivetrain.follower, moveToShootAgainPath),
                        instant(() -> robot.shooter.setTargetRPM(SHOOT_RPM))
                ),
                waitMs(SPIN_UP_MS),
                ControlCommands.transfer(robot.intake, robot.transfer, robot.flower, robot.shooter),
                waitMs(SHOOT_TIME_MS),

                // 5. Stop mechanisms & follow park path
                stopMechanisms(),
                follow(robot.drivetrain.follower, parkEndPath)
        );

        schedule(deadline(twoTipSequence, TurretCommands.turnToClosestGoal(robot.turret, robot.drivetrain.follower)));
    }

    @Override
    protected void onLoop() {
        Pose currentPose = robot.drivetrain.follower.pose();

        telemetry.addLine("TWO TIP AUTO");
        telemetry.addData("Pose", String.format(
                Locale.US,
                "X: %.2f | Y: %.2f | H: %.1f deg",
                currentPose.x(),
                currentPose.y(),
                Math.toDegrees(currentPose.heading())
        ));
        telemetry.addData("Shooter RPM", String.format(Locale.US, "%.0f / %.0f", robot.shooter.getCurrentRPM(), robot.shooter.getTargetRPM()));
        telemetry.addData("Turret Heading", "%.1f deg", robot.turret.getHeading());
        telemetry.addData("Follower Mode", robot.drivetrain.follower.mode());
        telemetry.update();
    }

    @Override
    protected void onStop() {
        schedule(stopMechanisms());
    }

    public Path movetointakeballs() {
        return Paths.curve(START, movetointakeballsControl1, movetointakeballs).linear(START, movetointakeballs);
    }

    public Path moveToShootPosit() {
        return Paths.curve(moveToShootPositStart, moveToShootPositControl1, moveToShootPosit).linear(moveToShootPositStart, moveToShootPosit);
    }

    public Path moveToIntakeMoreBalls() {
        return Paths.curve(moveToIntakeMoreBallsStart, moveToIntakeMoreBallsControl1, moveToIntakeMoreBalls).linear(moveToIntakeMoreBallsStart, moveToIntakeMoreBalls);
    }

    public Path moveToShootAgain() {
        return Paths.curve(moveToShootAgainStart, moveToShootAgainControl1, moveToShootAgain).linear(moveToShootAgainStart, moveToShootAgain);
    }

    public Path parkEnd() {
        return Paths.curve(parkEndStart, parkEndControl1, parkEnd).linear(parkEndStart, parkEnd);
    }
}
