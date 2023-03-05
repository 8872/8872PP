package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

@Config
public final class DownSequenceWithPosition extends SequentialCommandGroup {

    //command allows me to do the reset command group to a certain height for conestack
    public DownSequenceWithPosition(LiftSys lift, TurretSys turret, ArmSys arm, ClawSys claw, int height) {
        addCommands(
                new ParallelCommandGroup(
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(TurretSys.Pose.ZERO), 80),
                        new DelayedCommand(lift.goTo(height), 200),
                        new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN, 3, 3), 200),
                        new DelayedCommand(claw.release(), 250)
                )
        );

        addRequirements(lift, turret, arm);
    }
}