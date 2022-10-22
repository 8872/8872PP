package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class GrabAndLiftLow extends SequentialCommandGroup {
    public GrabAndLiftLow(ArmSubsystem arm){
        addCommands(
                new GrabCone(arm),
                new ParallelCommandGroup(
                        new MoveConeLow(arm),
                        new SlideOut(arm)
                )
        );
    }
}
