package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetConeStack;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.ConeStack;

public class MoveToConeStackAuto extends SequentialCommandGroup {

    public MoveToConeStackAuto(LiftSubsystem lift, SlideSubsystem slide, ClawSubsystem claw, ConeStack cone) {
        addCommands(
                new ReleaseCone(claw),
                new SetConeStack(lift,cone),
                new ParallelCommandGroup(
                        new MoveToJunction(lift),
                        new SlideIn(slide)
                )
        );
        addRequirements(lift, slide, claw);
    }
}
