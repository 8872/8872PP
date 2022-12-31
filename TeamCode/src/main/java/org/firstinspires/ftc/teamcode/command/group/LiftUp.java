package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.command.lift.MoveLiftPID;
import org.firstinspires.ftc.teamcode.command.lift.MoveToJunction;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.Junction;

@Config
public class LiftUp extends SequentialCommandGroup {
    public static int delay = 500;

    public LiftUp(LiftSubsystem lift, SlideSubsystem slide, Junction junction){
        addCommands(
                new SetJunction(lift, junction),
                new ParallelCommandGroup(
                        new MoveToJunction(lift),
                        new DelayedCommand(new SlideOut(slide), 500)
                )
        );

        addRequirements(lift, slide);
    }
}
