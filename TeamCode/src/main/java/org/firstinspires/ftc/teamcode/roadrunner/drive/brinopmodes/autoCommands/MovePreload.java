package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class MovePreload extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Pose2d startPose;
    private final Pose2d endPose;
    public MovePreload(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        this.drive = drive;
        this.startPose = startPose;
        this.endPose = endPose;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build());
    }

    @Override
    public boolean isFinished() {
        return drive.isBusy();
    }
}
