package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.EmptyCommand;

@Config
public class GrabAndLift extends SequentialCommandGroup {
    public static int goal = -100;
    public GrabAndLift(LiftSubsystem lift, ClawSubsystem claw) {
        addCommands(
                claw.runGrabCommand(),
                new ConditionalCommand(new EmptyCommand(),
                        new SequentialCommandGroup(
                                lift.goTo(goal)
                        ), lift::isGrabAndLiftIncompatible)
        );
        addRequirements(lift, claw);
    }
}
