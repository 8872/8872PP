package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveLiftPID;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;


public class LiftDown extends SequentialCommandGroup {

    public LiftDown(LiftSubsystem lift, SlideSubsystem slide, ClawSubsystem claw){
        addCommands(
                new ReleaseCone(claw),
                new ConditionalCommand(new SetJunction(lift, Junction.MEDIUM),
                        new SetJunction(lift, Junction.NONE), lift::isSlideIncompatible),
                new ParallelCommandGroup(
                        new MoveToJunction(lift),
                        new SlideIn(slide)
                ),
                new SetJunction(lift, Junction.NONE),
                new MoveToJunction(lift)
        );

        addRequirements(lift, slide, claw);
    }
}
