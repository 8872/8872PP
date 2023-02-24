package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.*;

@Autonomous
public class TestRightTrajectory extends BaseOpMode {

    private Trajectory preload;
    private Trajectory toConeStack;
    private Trajectory toHighJunction;

    @Override
    public void initialize() {
        super.initialize();

        rrDrive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));

        schedule(
                new SequentialCommandGroup(
                        new FollowPreloadTrajectory(rrDrive),
                        new InstantCommand(() -> sleep(1000)),
                        new FollowConestackTrajectory(rrDrive),
                        new InstantCommand(() -> sleep(1000)),
                        new FollowHighJunctionTrajectory(rrDrive),
                        new InstantCommand(() -> sleep(1000)),// 1
                        new FollowConestackTrajectory(rrDrive),
                        new InstantCommand(() -> sleep(1000)),
                        new FollowHighJunctionTrajectory(rrDrive) //2
                )
        );
    }
}
