package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public final class DownSequence extends SequentialCommandGroup {

    public DownSequence(LiftSys lift, TurretSys turret, ArmSys arm) {
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
