package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Height;

public final class DownSequence extends SequentialCommandGroup {

    public DownSequence(LiftSubsystem lift, TurretSubsystem turret, ArmSubsystem arm) {
        addCommands(
                arm.goTo(90),
                turret.goTo(0),
                new ParallelCommandGroup(
                        lift.goTo(Height.NONE),
                        arm.goTo(0)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
