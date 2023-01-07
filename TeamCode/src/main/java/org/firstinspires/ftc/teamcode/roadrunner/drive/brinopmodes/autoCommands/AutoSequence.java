package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;


public class AutoSequence extends SequentialCommandGroup {
    public AutoSequence(SampleMecanumDrive drive){
        addCommands(
//                new Preload(drive, new Pose2d()),
//                new AdjustAngle(),
//                new DelayedClaw(),
        );
    }
}
