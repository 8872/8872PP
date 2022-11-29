package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.lift.MoveToLocation;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class LiftHigh extends SequentialCommandGroup {
    public LiftHigh(ArmSubsystem arm){
        addCommands(
                new SetJunction(arm, ArmSubsystem.Junction.HIGH),
                new MoveToLocation(arm),
                new SlideOut(arm)
        );
    }
}
