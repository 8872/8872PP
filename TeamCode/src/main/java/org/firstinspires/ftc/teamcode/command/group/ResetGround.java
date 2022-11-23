package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
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
        if(arm.getSlidePos() < 0.5)
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.NONE)
            );
        else if(arm.getLeftEncoderValue() < 250)
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.LOW),
                    new SlideIn(arm),
                    new SetJunction(arm, ArmSubsystem.Junction.NONE)
            );
        else if(arm.getLeftEncoderValue() < 800)
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new SlideIn(arm)
            );
        else
            addCommands(
                    //new ParallelCommandGroup(
                            new SetJunction(arm, ArmSubsystem.Junction.NONE),
                            new SlideIn(arm)

        //)
            );
        addCommands(
                new ReleaseCone(arm)
        );

    }
}
//problem: how to make the low junction sequential
//TODO: make sure left encoder reset works