package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class GrabAndLiftMedium extends SequentialCommandGroup {
    public GrabAndLiftMedium(ArmSubsystem arm){
        addCommands(
                new GrabCone(arm),
                new ParallelCommandGroup(
                        new MoveConeMedium(arm),
                        new SlideOut(arm)
                )
        );
    }
}
