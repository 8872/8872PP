package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;

public class Retrieve extends SequentialCommandGroup {
    public Retrieve(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, LiftSubsystem lift, SlideSubsystem slide, ClawSubsystem claw, int pos) {
        addCommands(
                new TrajectoryCommand(() -> {
                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(51, 5.5), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .forward(Math.abs(27.33-5.5), SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                }, drive::isBusy),
                new AutoLiftDown(lift, slide, claw, pos)
        );
        addRequirements(lift, slide);
    }
}
