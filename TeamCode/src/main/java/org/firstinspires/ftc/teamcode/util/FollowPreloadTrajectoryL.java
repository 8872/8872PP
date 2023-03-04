package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowPreloadTrajectoryL extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowPreloadTrajectoryL(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        //the velocityconstraint stuff was to limit the velocity, but I don't use it anymore
        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-36, -50, Math.toRadians(90))) // 36
                .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-36, -7.75, Math.toRadians(180)))
                .build());
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
