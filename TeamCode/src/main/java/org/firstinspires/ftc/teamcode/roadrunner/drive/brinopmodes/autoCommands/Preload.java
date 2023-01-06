package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.command.group.LiftUp;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

public class Preload extends ParallelCommandGroup {
    public Preload(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, LiftSubsystem lift, SlideSubsystem slide, Junction junction) {
        addCommands(
                new TrajectoryCommand(() -> {
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(endPose)
                            .build());
                }, drive::isBusy),
                new LiftUp(lift, slide, junction)
        );
        addRequirements(lift, slide);
    }
}
