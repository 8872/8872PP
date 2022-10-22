package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
