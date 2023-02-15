package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

@Config
public final class HighSequence extends SequentialCommandGroup {
    public static double armPosition = 0.85;
    private int currentGoal;
    public HighSequence(LiftSys lift, TurretSys turret, ArmSys arm, TurretSys.Pose pose) {
        addCommands(
                new InstantCommand(() -> currentGoal = lift.getCurrentGoal()),
                new ParallelCommandGroup(
                        lift.goTo(Height.HIGH),
                        new ConditionalCommand(arm.goTo(ArmSys.Pose.DEPOSIT), new InstantCommand(),
                                () -> (currentGoal == Height.NONE.getHeight())),
                        new DelayedCommand(turret.goTo(pose), 300),
                        new DelayedCommand(arm.goTo(armPosition), 700)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
