package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowPreloadTrajectory extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowPreloadTrajectory(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        //the velocityconstraint stuff was to limit the velocity, but I don't use it anymore
        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(36, -50, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(36, -10, 0),
                        SampleMecanumDrive.getVelocityConstraint(50,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(
                                DriveConstants.MAX_ACCEL))
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
