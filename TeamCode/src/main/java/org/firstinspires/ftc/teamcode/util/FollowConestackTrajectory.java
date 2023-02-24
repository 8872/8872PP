package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowConestackTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowConestackTrajectory(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        Log.d("was", "" + drive.isBusy());

        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(61,-11.11,0))
                .build());
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        Log.d("wasd", "" + drive.isBusy());
        return !drive.isBusy();
    }
}
