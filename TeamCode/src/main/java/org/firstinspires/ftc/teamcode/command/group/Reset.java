package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveConeGround;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class Reset extends SequentialCommandGroup {
    public Reset(ArmSubsystem arm){
        addCommands(
                new ReleaseCone(arm),
                new ParallelCommandGroup(
                        new SlideIn(arm),
                        new MoveConeGround(arm)
                )
        );
    }
}
