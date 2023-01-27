//package org.firstinspires.ftc.teamcode.command.group;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.*;
//import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
//import org.firstinspires.ftc.teamcode.util.DelayedCommand;
//import org.firstinspires.ftc.teamcode.util.Height;
//
//@Config
//public class LiftUp extends SequentialCommandGroup {
//    public static int delay = 500;
//    public static int tick = -600;
//    public LiftUp(LiftSubsystem lift, SlideSubsystem slide, Height junction){
//        addCommands(
//                new ConditionalCommand(new SequentialCommandGroup(
//                        new SetTick(lift, tick),
//                        new InstantCommand(() -> lift.setCurrentGoal(Height.LOW)),
//                        new ParallelCommandGroup(
//                                new MoveToJunction(lift),
//                                new DelayedCommand(new SlideOut(slide), 300)),
//                        new SetJunction(lift, Height.LOW),
//                        new MoveToJunction(lift)
//                ), new SequentialCommandGroup(
//                        new SetJunction(lift, junction),
//                        new ParallelCommandGroup(
//                                new MoveToJunction(lift),
//                                new DelayedCommand(new SlideOut(slide), 500)
//                        )
//                ), () -> junction == Height.LOW)
//        );
//
//        addRequirements(lift, slide);
//    }
//}
