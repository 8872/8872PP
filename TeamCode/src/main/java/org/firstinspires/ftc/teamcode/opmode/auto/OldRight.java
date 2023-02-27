package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.command.group.HighSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowTrajectoryCommand;


public class OldRight extends BaseOpMode {
    private Height[] coneStack = {Height.FIRST, Height.SECOND, Height.THIRD, Height.FOURTH};

    private Trajectory preload, toConeStack, toHighJunction;

    @Override
    public void initialize() {
        super.initialize();

        // TODO: tune trajectories and start pose
        preload = rrDrive.trajectoryBuilder(new Pose2d(33.59, -63.86, Math.toRadians(90.00)))
                .lineTo(new Vector2d(33.59, -14.61))
                .build();

        toConeStack = rrDrive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(62.67, -12.01, Math.toRadians(0.00)))
                .build();

        toHighJunction = rrDrive.trajectoryBuilder(toConeStack.end())
                .lineTo(new Vector2d(34.05, -11.86))
                .build();

        schedule(
                new SequentialCommandGroup(
                        claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                        new FollowTrajectoryCommand(rrDrive, preload),
                        new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD),
                        // camera align
                        claw.release(),
                        new DownSequence(lift, turret, arm, claw)
                )
        );

        // cycle
        for (int i = 0; i < 5; i++) {
            schedule(
                    cycleConeStack(i)
            );
        }

        // park

    }

    private SequentialCommandGroup cycleConeStack(int i) {
        return new SequentialCommandGroup(
                new FollowTrajectoryCommand(rrDrive, toConeStack),
                lift.goTo(coneStack[i]).alongWith(arm.goTo(ArmSys.Pose.HORIZONTAL)),
                claw.grab(),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(rrDrive, toHighJunction),
                        new HighSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD)
                ),
                // camera align
                claw.release(),
                new DownSequence(lift, turret, arm, claw)
        );
    }
}
