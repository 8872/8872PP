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
public final class LowSequence extends SequentialCommandGroup {
    public static double armPosition = 0.85;

    public LowSequence(LiftSys lift, TurretSys turret, ArmSys arm, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        arm.goTo(ArmSys.Pose.DEPOSIT),
                        lift.goTo(Height.LOW),
                        new DelayedCommand(turret.goTo(pose), 200),
                        new DelayedCommand(arm.goTo(armPosition), 800)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
