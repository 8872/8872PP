package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

public class MoveToHighAuto extends SequentialCommandGroup {
    public static int delay = 500;

    public MoveToHighAuto(LiftSubsystem lift, ClawSubsystem claw) {
        addCommands(
                new GrabCone(claw),
                new SetJunction(lift, Junction.LOW),
                new MoveToJunction(lift),
                new WaitCommand(delay),
                new SetJunction(lift, Junction.HIGH),
                new MoveToJunction(lift)
        );
        addRequirements(lift, claw);
    }
}
