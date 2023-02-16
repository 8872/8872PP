package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequenceCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final TrajectorySequence sequence;

    public FollowTrajectorySequenceCommand(SampleMecanumDrive drive, TrajectorySequence sequence) {
        this.drive = drive;
        this.sequence = sequence;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(sequence);
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
