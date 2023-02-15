package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

@Config
public final class MediumSequence extends SequentialCommandGroup {
    public static double armPosition = 0.85;
    private int currentGoal;
    public MediumSequence(LiftSys lift, TurretSys turret, ArmSys arm, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> currentGoal = lift.getCurrentGoal()),
                        lift.goTo(Height.MEDIUM),
                        new ConditionalCommand(arm.goTo(ArmSys.Pose.DEPOSIT), new InstantCommand(),
                                () -> (currentGoal == Height.NONE.getHeight())),

                        new DelayedCommand(turret.goTo(pose), 600),
                        new DelayedCommand(arm.goTo(armPosition), 800)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
