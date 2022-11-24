package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveToLocation;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.slide.DelayedSlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveConeGround;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class ResetGround extends SequentialCommandGroup {
    public ResetGround(ArmSubsystem arm){

        addCommands(
                new ReleaseCone(arm),
                new GrabCone(arm)
        );
        if(arm.getSlidePos() > 0.5)
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.NONE)
            );
        else if(arm.getLeftEncoderValue() > -250)
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.LOW),
                    new MoveToLocation(arm),
                    new DelayedSlideIn(arm),
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new MoveToLocation(arm)
            );
        else if(arm.getLeftEncoderValue() > -800)
            addCommands(
                    new DelayedSlideIn(arm),
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new MoveToLocation(arm)
            );
        else
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new SlideIn(arm)
            );
        addCommands(
                new ReleaseCone(arm)
        );

    }
}
