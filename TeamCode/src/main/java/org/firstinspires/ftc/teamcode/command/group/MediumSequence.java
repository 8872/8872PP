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
public final class MediumSequence extends SequentialCommandGroup {
    public static double armPosition = 0.93;

    public MediumSequence(LiftSys lift, TurretSys turret, ArmSys arm, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(Height.MEDIUM),
                        arm.goTo(ArmSys.Pose.DEPOSIT),
                        new DelayedCommand(turret.goTo(pose), 600),
                        new DelayedCommand(arm.goTo(armPosition), 800)
                )
        );

        addRequirements(lift, turret, arm);
    }
}