package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class FollowTrajectoryCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Trajectory trajectory;

    public FollowTrajectoryCommand(SampleMecanumDrive drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        drive.followTrajectoryAsync(trajectory);
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
