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
public final class HighSequence extends SequentialCommandGroup {
    public HighSequence(LiftSys lift, TurretSys turret, ArmSys arm, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(Height.HIGH),
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(pose), 300),
                        new DelayedCommand(arm.goTo(ArmSys.Pose.HORIZONTAL, 2, 2),
                                700)
                )

        );

        addRequirements(lift, turret, arm);
    }
}
