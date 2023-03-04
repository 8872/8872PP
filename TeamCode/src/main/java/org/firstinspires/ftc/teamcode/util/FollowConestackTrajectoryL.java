package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowConestackTrajectoryL extends CommandBase {
    private final SampleMecanumDrive drive;

    public FollowConestackTrajectoryL(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        //the SampleMecanumDrive velocity stuff is to cap the velocity at the given value (35 in this case)
        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60.85,-9.75,Math.toRadians(180)), // 61
                        SampleMecanumDrive.getVelocityConstraint(35,
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
