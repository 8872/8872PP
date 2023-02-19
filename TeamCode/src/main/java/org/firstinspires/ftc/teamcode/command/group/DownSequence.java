package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

@Config
public final class DownSequence extends SequentialCommandGroup {
    public DownSequence(LiftSys lift, TurretSys turret, ArmSys arm) {
        addCommands(
                new ParallelCommandGroup(
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(TurretSys.Pose.ZERO), 100),
                        new DelayedCommand(lift.goTo(Height.NONE), 600),
                        new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN), 600)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
