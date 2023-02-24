package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.FollowTrajectoryCommand;

@Autonomous
public class TestRightTrajectory extends BaseOpMode {

    private Trajectory preload, toConeStack, toHighJunction;

    @Override
    public void initialize() {
        super.initialize();

        rrDrive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));

        preload = rrDrive.trajectoryBuilder(new Pose2d(36, -62, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, -11.86, Math.toRadians(0)))
                .build();

        toConeStack = rrDrive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(62.67, -11.86, Math.toRadians(0)))
                .build();

        toHighJunction = rrDrive.trajectoryBuilder(toConeStack.end())
                .lineToLinearHeading(new Pose2d(34.05, -11.86, Math.toRadians(0)))
                .build();

        schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(rrDrive, preload),
                        new FollowTrajectoryCommand(rrDrive, toConeStack),
                        new FollowTrajectoryCommand(rrDrive, toHighJunction), // 1
                        new FollowTrajectoryCommand(rrDrive, toConeStack),
                        new FollowTrajectoryCommand(rrDrive, toHighJunction), // 2
                        new FollowTrajectoryCommand(rrDrive, toConeStack),
                        new FollowTrajectoryCommand(rrDrive, toHighJunction), // 3
                        new FollowTrajectoryCommand(rrDrive, toConeStack),
                        new FollowTrajectoryCommand(rrDrive, toHighJunction), // 4
                        new FollowTrajectoryCommand(rrDrive, toConeStack),
                        new FollowTrajectoryCommand(rrDrive, toHighJunction) // 5
                )
        );
    }
}
