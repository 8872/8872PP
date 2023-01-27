//package org.firstinspires.ftc.teamcode.command.group;
//
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
//import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
//import org.firstinspires.ftc.teamcode.util.Height;
//
//
//public class LiftDown extends SequentialCommandGroup {
//
//    public LiftDown(LiftSubsystem lift, SlideSubsystem slide, ClawSubsystem claw){
//        addCommands(
//                claw.runGrabCommand(),
//                new ConditionalCommand(new SetJunction(lift, Height.MEDIUM),
//                        new SetJunction(lift, Height.NONE), lift::isSlideIncompatible),
//                new ParallelCommandGroup(
//                        new MoveToJunction(lift),
//                        new SlideIn(slide)
//                ),
//                new SetJunction(lift, Height.NONE),
//                new MoveToJunction(lift),
//                claw.runReleaseCommand()
//        );
//
//        addRequirements(lift, slide, claw);
//    }
//}
