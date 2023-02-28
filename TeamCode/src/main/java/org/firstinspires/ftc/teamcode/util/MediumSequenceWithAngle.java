package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

import java.util.function.DoubleSupplier;

@Config
public final class MediumSequenceWithAngle extends SequentialCommandGroup {
    //same as medium sequence but with a given turret set position
    public MediumSequenceWithAngle(LiftSys lift, TurretSys turret, ArmSys arm, double pos) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(Height.MEDIUM.getHeight()-30),
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(pos), 350),
                        new DelayedCommand(arm.goTo(ArmSys.Pose.HORIZONTAL, 2, 2),
                                700)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
