package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetTick;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.EmptyCommand;

public class GrabAndLift extends SequentialCommandGroup {

    public GrabAndLift(LiftSubsystem lift, ClawSubsystem claw, int goal) {
        addCommands(
                new GrabCone(claw),
                new ConditionalCommand(new EmptyCommand(),
                        new SequentialCommandGroup(
                                new SetTick(lift, goal),
                                new MoveToJunction(lift)
                        ), lift::isGrabAndLiftIncompatible)
        );
        addRequirements(lift, claw);
    }
}
