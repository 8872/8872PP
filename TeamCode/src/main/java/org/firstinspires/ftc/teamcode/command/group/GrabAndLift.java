package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetTick;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class GrabAndLift extends SequentialCommandGroup {

    public GrabAndLift(LiftSubsystem lift, ClawSubsystem claw, int goal) {
        addCommands(
                new ReleaseCone(claw),
                new SetTick(lift, goal),
                new MoveToJunction(lift)
        );
        addRequirements(lift, claw);
    }
}
